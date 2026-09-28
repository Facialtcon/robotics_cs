"""UR7e RTDE connection, state reads, motion execution, and stop operations."""

from __future__ import annotations

import time

import numpy as np

from core.models import RobotState


class RobotError(RuntimeError):
    pass


class SearchLimitReached(RobotError):
    """No motion sent: a fresh execution read exhausted search clearance."""


# Engineering thresholds, not an RTDE noise model. Keep the original 20%
# envelope; only its small overrun band gets two nominal 100 Hz cycles.
CONTINUOUS_TRIP_FACTOR = 1.2
CONTINUOUS_HARD_FACTOR = 1.5
CONTINUOUS_DEBOUNCE_SEC = .020
CONTINUOUS_DEBOUNCE_PACKETS = 3
SPEED_GUARD_FIELDS = (
    'speed_guard_phase', 'speed_guard_nominal_mps', 'speed_guard_trip_mps',
    'speed_guard_hard_mps', 'speed_guard_state', 'speed_guard_elapsed_sec',
    'speed_guard_count', 'speed_guard_device_timestamp',
)


class ContinuousSpeedLimitError(RobotError):
    """Keep the rejected read in memory; no device reads or I/O on this path."""

    def __init__(self, pose, speed, limit, read_start, device_timestamp, diagnostics=None):
        measured = float(np.linalg.norm(speed[:3]))
        trip_limit = 1.2 * limit
        self.speed_limit_observation = {
            'source': 'rtde_state_read_rejected_by_continuous_speed_guard',
            'tcp_pose': pose.tolist(),
            'tcp_speed': speed.tolist(),
            'tcp_pose_units': 'xyz_m_rotation_vector_rad',
            'tcp_speed_units': 'xyz_mps_angular_radps',
            'measured_xyz_speed_mps': measured,
            'configured_speed_limit_mps': float(limit),
            'tolerance_factor': 1.2,
            'trip_limit_mps': float(trip_limit),
            'host_read_start_monotonic_sec': read_start,
            # Cached freshness check; SDK getters need not share one packet.
            'last_checked_device_timestamp_sec': device_timestamp,
            **(diagnostics or {}),
        }
        super().__init__(
            'continuous measured speed exceeds experimental limit: '
            f'measured_xyz={measured:.9g} m/s, trip_limit={trip_limit:.9g} m/s, '
            f'actual_vxyz={speed[:3].tolist()} m/s, guard={diagnostics or {}}')


def check_continuous_xy(config, point):
    """Execution guard only; never supplies target geometry/directions to policy."""
    bounds = config.get('continuous_xy_limits')
    polygon = config.get('continuous_xy_polygon')
    if bounds is None and polygon is None:
        return  # Original discrete execution is unchanged.
    xy = np.asarray(point, dtype=float)
    margin = float(config['continuous_boundary_margin'])
    if xy.shape != (2,) or not np.isfinite(xy).all() or not np.isfinite(margin) or margin < 0:
        raise RobotError('invalid continuous XY boundary observation')
    if bounds is not None and any(not bounds[f'{axis}_min']+margin <= xy[i] <= bounds[f'{axis}_max']-margin
                                  for i,axis in enumerate('xy')):
        raise RobotError('continuous XY execution boundary reached')
    if polygon is not None:
        vertices = np.asarray(polygon, dtype=float)
        if vertices.shape != (4,2) or not np.isfinite(vertices).all():
            raise RobotError('invalid continuous sandbox boundary')
        edges = np.roll(vertices,-1,axis=0)-vertices
        lengths = np.linalg.norm(edges,axis=1)
        turns = edges[:,0]*np.roll(edges,-1,axis=0)[:,1]-edges[:,1]*np.roll(edges,-1,axis=0)[:,0]
        if np.any(lengths<=1e-12) or not (np.all(turns>0) or np.all(turns<0)):
            raise RobotError('invalid continuous sandbox boundary order')
        offsets = xy-vertices
        distances = np.sign(turns[0])*(edges[:,0]*offsets[:,1]-edges[:,1]*offsets[:,0])/lengths
        if np.any(distances < margin):
            raise RobotError('continuous calibrated sandbox boundary reached')


def _finite_six(values, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise RobotError(f"{name} must contain six finite values")
    return result


def _rotvec_to_matrix(rotvec) -> np.ndarray:
    vector = np.asarray(rotvec, dtype=float)
    theta = float(np.linalg.norm(vector))
    if theta < 1e-12:
        return np.eye(3)
    axis = vector / theta
    skew = np.asarray(
        [[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]]
    )
    return np.eye(3) + np.sin(theta) * skew + (1.0 - np.cos(theta)) * (skew @ skew)


def _orientation_distance(left_rotvec, right_rotvec) -> float:
    relative = _rotvec_to_matrix(left_rotvec).T @ _rotvec_to_matrix(right_rotvec)
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.arccos(cosine))


class WorkspaceGuard:
    def __init__(
        self,
        limits: dict,
        fixed_z: float,
        z_tolerance: float,
        max_tcp_speed: float,
        fixed_orientation=None,
        orientation_tolerance_rad: float = float("inf"),
        workspace_enabled: bool = True,
    ):
        self.workspace_enabled = bool(workspace_enabled)
        self.limits = {name: float(limits[name]) for name in ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")}
        if self.workspace_enabled:
            for axis in "xyz":
                if self.limits[f"{axis}_min"] >= self.limits[f"{axis}_max"]:
                    raise RobotError(f"workspace {axis}_min must be smaller than {axis}_max")
        self.fixed_z = float(fixed_z)
        self.z_tolerance = float(z_tolerance)
        self.max_tcp_speed = float(max_tcp_speed)
        self.fixed_orientation = (
            None if fixed_orientation is None else np.asarray(fixed_orientation, dtype=float)
        )
        if self.fixed_orientation is not None and self.fixed_orientation.shape != (3,):
            raise RobotError("fixed orientation must have three rotation-vector values")
        self.orientation_tolerance_rad = float(orientation_tolerance_rad)

    def check_pose(self, pose) -> None:
        value = _finite_six(pose, "TCP pose")
        self.check_workspace(value)
        if abs(value[2] - self.fixed_z) > self.z_tolerance:
            raise RobotError(
                f"TCP z drift {value[2] - self.fixed_z:+.6f} m exceeds tolerance"
            )
        if self.fixed_orientation is not None:
            drift = _orientation_distance(self.fixed_orientation, value[3:])
            if drift > self.orientation_tolerance_rad:
                raise RobotError(
                    f"TCP orientation drift {drift:.6f} rad exceeds tolerance"
                )

    def check_workspace(self, pose) -> None:
        value = _finite_six(pose, "TCP pose")
        if not self.workspace_enabled:
            return
        for index, axis in enumerate("xyz"):
            if not self.limits[f"{axis}_min"] <= value[index] <= self.limits[f"{axis}_max"]:
                raise RobotError(f"TCP {axis}={value[index]:.6f} is outside configured workspace")

    def check_velocity(self, direction_xy, speed: float) -> np.ndarray:
        direction = np.asarray(direction_xy, dtype=float)
        if direction.shape != (2,) or not np.all(np.isfinite(direction)):
            raise RobotError("planar direction must contain two finite values")
        norm = float(np.linalg.norm(direction))
        if norm < 1e-12:
            raise RobotError("cannot command a zero direction")
        if speed <= 0.0 or speed > self.max_tcp_speed:
            raise RobotError(
                f"commanded speed {speed:.6f} m/s is outside (0, {self.max_tcp_speed:.6f}]"
            )
        return direction / norm

    def check_predicted_pose(self, pose, direction_xy, speed: float, duration: float) -> None:
        predicted = _finite_six(pose, "TCP pose").copy()
        predicted[:2] += np.asarray(direction_xy) * speed * duration
        self.check_pose(predicted)


class SimulatedController:
    """Integrates policy velocities locally and never opens a robot connection."""

    is_dry_run = True

    def __init__(self, start_pose, max_tcp_speed: float):
        self.pose = _finite_six(start_pose, "dry-run start pose")
        self.speed = np.zeros(6, dtype=float)
        self.max_tcp_speed = float(max_tcp_speed)
        self.connected = False

    def connect(self) -> None:
        self.connected = True

    def read_state(self) -> RobotState:
        return RobotState(time.monotonic(), self.pose.copy(), self.speed.copy())

    def read_diagnostic_state(self) -> RobotState:
        return self.read_state()

    def command_planar_velocity(self, direction_xy, speed: float, duration: float) -> None:
        direction = np.asarray(direction_xy, dtype=float)
        norm = float(np.linalg.norm(direction))
        if norm <= 0.0 or not 0.0 < speed <= self.max_tcp_speed:
            raise RobotError("invalid dry-run velocity command")
        direction /= norm
        self.speed[:] = 0.0
        self.speed[:2] = direction * speed
        self.pose[:2] += self.speed[:2] * duration

    def stop(self) -> None:
        self.speed[:] = 0.0

    def close(self) -> None:
        self.stop()
        self.connected = False


class URRTDEController:
    """Direct UR control via the `ur-rtde` Python package.

    Only `speedL` is exposed to the policy, with Z and angular velocities
    forced to zero. Every call re-checks actual pose and predicted workspace.
    """

    is_dry_run = False

    def __init__(self, config: dict, *, receive_factory=None, control_factory=None):
        self.config = dict(config)
        self._receive_factory = receive_factory
        self._control_factory = control_factory
        self._control_created = False
        self._closed = False
        self.stop_report = None
        self.stop_history = []
        self._settled_since = self._settled_stamp = None
        self.control = None
        self.receive = None
        self.guard: WorkspaceGuard | None = None
        self.standstill_confirmed = False
        self._stop_method = "speedStop"
        self._return_mode = False
        self.motion_fault = ''
        self._stop_pending = False
        self._stop_started = None
        self._stop_primitive_called = False
        self.stop_state = 'RUNNING'
        self.watchdog_active = False
        self._watchdog_last_kick = None
        self._packet_stamp = self._packet_seen_at = None
        self._packet_advanced = False
        self.observation_timing = {}
        self.connection_failed = False
        self.connection_diagnostics = {}
        self.control_script_stop_requested = False
        self.continuous_phase = 'READY'
        self._continuous_envelope = 'CONTINUOUS_TRACKING'
        self._speed_guard_since = self._speed_guard_stamp = None
        self._speed_guard_count = 0
        self.speed_guard_diagnostics = {}

    def set_continuous_phase(self, phase, *, stop_confirmed=False):
        """Select execution limits; braking retains the preceding envelope.

        The policy's held standstill and the controller's measured stop must
        both agree before FIRST_CONTACT can authorize tracking motion.
        """
        limits = self.config.get('continuous_speed_limits')
        if limits is None:
            return
        phase = str(phase)
        if phase not in ('READY', 'TARGET_SEARCH', 'FIRST_CONTACT', 'CONTINUOUS_TRACKING',
                         'DIRECTION_RECONFIRM', 'CONTACT_LOST', 'LOCAL_REACQUIRE', 'STOP'):
            raise RobotError(f'unknown continuous execution phase: {phase}')
        if phase == 'CONTINUOUS_TRACKING' and self.continuous_phase == 'FIRST_CONTACT':
            if not (stop_confirmed and self._stop_pending and self.standstill_confirmed):
                raise RobotError('FIRST_CONTACT requires measured standstill before tracking')
        if phase in limits:
            self._continuous_envelope = phase
        self.continuous_phase = phase
        self.config['continuous_boundary_margin'] = (
            self.config['continuous_search_boundary_margin']
            if self._continuous_envelope == 'TARGET_SEARCH'
            else self.config['continuous_tracking_boundary_margin'])
        # Do not reset a pending speed episode on state changes or stop requests.

    def _check_continuous_speed(self, pose, speed, now):
        limits = self.config.get('continuous_speed_limits')
        if limits is None:
            # Compatibility for older callers of this shared controller.
            limit = self.config.get('continuous_speed_limit')
            if limit and np.linalg.norm(speed[:3]) > CONTINUOUS_TRIP_FACTOR * limit:
                raise ContinuousSpeedLimitError(pose, speed, limit, now, self._packet_stamp)
            return
        limit = float(limits[self._continuous_envelope])
        measured = float(np.linalg.norm(speed[:3]))
        trip = CONTINUOUS_TRIP_FACTOR * limit
        hard = min(CONTINUOUS_HARD_FACTOR * limit, 1.2 * float(self.config['max_tcp_speed']))
        fresh = self._packet_stamp is not None and self._packet_stamp != self._speed_guard_stamp
        # Re-reading cached packets neither confirms nor clears an excursion.
        # The host deadline still runs, even when no new packet is available.
        elapsed = 0. if self._speed_guard_since is None else now-self._speed_guard_since
        expired = self._speed_guard_since is not None and elapsed + 1e-12 >= CONTINUOUS_DEBOUNCE_SEC
        if fresh:
            self._speed_guard_stamp = self._packet_stamp
            if measured > trip:
                if self._speed_guard_since is None:
                    self._speed_guard_since = now
                self._speed_guard_count += 1
            elif not expired:
                self._speed_guard_since = None
                self._speed_guard_count = 0
        # Missing packet provenance cannot authorize a debounced overspeed.
        severe = measured > hard or (measured > trip and self._packet_stamp is None)
        sustained = expired or self._speed_guard_count >= CONTINUOUS_DEBOUNCE_PACKETS
        elapsed = 0. if self._speed_guard_since is None else now-self._speed_guard_since
        self.speed_guard_diagnostics = dict(zip(SPEED_GUARD_FIELDS, (
            self.continuous_phase, limit, trip, hard,
            'HARD_TRIP' if severe else ('SUSTAINED_TRIP' if sustained else
                ('PENDING' if self._speed_guard_since is not None else 'OK')),
            elapsed, self._speed_guard_count, self._packet_stamp)))
        if severe or sustained:
            error = ContinuousSpeedLimitError(pose, speed, limit, now, self._packet_stamp,
                                              self.speed_guard_diagnostics)
            self.motion_fault = str(error)
            # A rejected moving observation revokes any earlier stop belief.
            # Ensure the runner's stop attempts braking even before first speedL.
            self.standstill_confirmed = False
            self._settled_since = self._settled_stamp = None
            self._stop_pending = False
            raise error

    def connect(self, allow_start_away_from_fixed_pose=True):
        """Open Receive only. This never creates or imports a Control instance."""
        if self._closed:
            raise RobotError('controller session already closed')
        if self.receive is not None:
            return
        try:
            factory = self._receive_factory
            if factory is None:
                from rtde_receive import RTDEReceiveInterface
                factory = RTDEReceiveInterface
            self.receive = factory(self.config['robot_ip'])
            pose = _finite_six(self.receive.getActualTCPPose(), 'actual TCP pose')
            self.guard = WorkspaceGuard(
                self.config['workspace_limits'], pose[2] if self.config.get('fixed_z') is None else self.config['fixed_z'],
                self.config['fixed_z_tolerance'], self.config['max_tcp_speed'],
                fixed_orientation=self.config.get('fixed_orientation', pose[3:]),
                orientation_tolerance_rad=self.config['orientation_tolerance_rad'],
                workspace_enabled=self.config.get('workspace_enabled', True))
            self.guard.check_workspace(pose)
            if not allow_start_away_from_fixed_pose:
                self.guard.check_pose(pose)
            check_continuous_xy(self.config, pose[:2])
            self._check_connected()
        except Exception as exc:
            self.connection_failed = True
            self.connection_diagnostics = self._connection_failure_snapshot()
            self.close()
            raise RobotError(f'RTDE Receive initialization failed: {exc}') from exc

    def activate_control(self, *, confirmed):
        """Sole RTDE Control owner; one construction per task, after confirmation."""
        if confirmed is not True:
            raise RobotError('motion confirmation required')
        self._check_connected()
        self.wait_for_standstill()
        if self._control_created:
            raise RobotError('Control already created for this task; reconnect is forbidden')
        self._control_created = True
        factory = self._control_factory
        if factory is None:
            from rtde_control import RTDEControlInterface
            factory = RTDEControlInterface
        self.control = factory(self.config['robot_ip'])
        self._verify_tcp()
        self.wait_for_standstill()

    def _connection_failure_snapshot(self):
        """Read cached receive data before cleanup; never authorizes motion."""
        result = dict(host_monotonic_sec=time.monotonic(), freshness='not_verified', read_errors={})
        fields = {
            'receive_connected': ('isConnected', bool),
            'robot_mode': ('getRobotMode', int), 'safety_mode': ('getSafetyMode', int),
            'runtime_state': ('getRuntimeState', int),
            'safety_status_bits': ('getSafetyStatusBits', int),
            'robot_status_bits': ('getRobotStatus', int),
            'protective_stopped': ('isProtectiveStopped', bool),
            'emergency_stopped': ('isEmergencyStopped', bool),
            'rtde_device_timestamp': ('getTimestamp', float),
            'tcp_pose': ('getActualTCPPose', lambda v: _finite_six(v, 'diagnostic pose').tolist()),
            'tcp_speed': ('getActualTCPSpeed', lambda v: _finite_six(v, 'diagnostic speed').tolist()),
        }
        for key, (method, convert) in fields.items():
            result[key] = None
            try:
                value = getattr(self.receive, method)()
                if value is not None:
                    result[key] = convert(value)
            except Exception as exc:
                result['read_errors'][key] = f'{type(exc).__name__}: {exc}'
        return result

    def _verify_tcp(self) -> None:
        expected = _finite_six(self.config["tcp_offset"], "tcp_offset")
        tcp_source = None
        if hasattr(self.receive, "getTCPOffset"):
            tcp_source = self.receive
        elif hasattr(self.control, "getTCPOffset"):
            # Current ur-rtde wheels expose getTCPOffset on the control
            # interface rather than RTDEReceiveInterface.
            tcp_source = self.control
        if tcp_source is not None:
            actual = _finite_six(tcp_source.getTCPOffset(), "active TCP offset")
            if not np.allclose(actual, expected, atol=float(self.config["tcp_offset_tolerance"]), rtol=0.0):
                raise RobotError(f"active TCP {actual.tolist()} does not match config {expected.tolist()}")
        elif not bool(self.config.get("allow_unverified_active_tcp", False)):
            raise RobotError("this ur-rtde build cannot read the active TCP; refusing unverified motion")

    def _check_connected(self):
        if self.receive is None or self._closed:
            raise RobotError('RTDE Receive is not connected')
        for interface in (self.receive, self.control):
            if interface is not None and hasattr(interface, 'isConnected') and not interface.isConnected():
                raise RobotError('RTDE interface disconnected')
        if self.receive.isEmergencyStopped() or self.receive.isProtectiveStopped():
            raise RobotError('UR emergency/protective stop is active')

    def read_state(self) -> RobotState:
        self._check_connected()
        try:
            observed_start = time.monotonic()
            self._check_packet_freshness(observed_start)
            pose = _finite_six(self.receive.getActualTCPPose(), "actual TCP pose")
            speed = _finite_six(self.receive.getActualTCPSpeed(), "actual TCP speed")
            assert self.guard is not None
            if self._return_mode:
                self.guard.check_workspace(pose)
            else:
                self.guard.check_pose(pose)
            # Braking may consume the reserved clearance, never the raw boundary.
            boundary_config = ({**self.config, 'continuous_boundary_margin': 0.}
                               if self._stop_pending or self.continuous_phase == 'TARGET_SEARCH' else self.config)
            check_continuous_xy(boundary_config, pose[:2])
            if np.linalg.norm(speed[:3]) > float(self.config["max_tcp_speed"]) * 1.20:
                raise RobotError("measured TCP speed exceeds maximum plus tolerance")
            if not self._return_mode:
                self._check_continuous_speed(pose, speed, observed_start)
            observed_end = time.monotonic()
            self._check_observation_latency(observed_start, observed_end)
            self.observation_timing.update(tcp_read_start=observed_start, tcp_read_end=observed_end)
            self._observe_standstill(speed, observed_end)
            return RobotState(observed_end, pose, speed)
        except RobotError:
            raise
        except Exception as exc:
            raise RobotError(f"RTDE state read failed: {exc}") from exc

    def read_diagnostic_state(self) -> RobotState:
        """Best-effort read for emergency reporting; never authorizes motion."""
        if self.receive is None:
            raise RobotError("RTDE receive interface is not connected")
        try:
            observed_start = time.monotonic()
            self._check_packet_freshness(observed_start)
            pose = _finite_six(self.receive.getActualTCPPose(), "actual TCP pose")
            speed = _finite_six(self.receive.getActualTCPSpeed(), "actual TCP speed")
            observed_end = time.monotonic()
            self._check_observation_latency(observed_start, observed_end)
            self.observation_timing.update(tcp_read_start=observed_start, tcp_read_end=observed_end)
            self._observe_standstill(speed, observed_end)
            return RobotState(observed_end, pose, speed)
        except Exception as exc:
            raise RobotError(f"RTDE diagnostic read failed: {exc}") from exc

    def command_planar_velocity(self, direction_xy, speed: float, duration: float) -> None:
        self._check_motion_authorized()
        limits = self.config.get('continuous_speed_limits')
        if limits is not None:
            if self.continuous_phase not in limits:
                raise RobotError(f'new motion forbidden during {self.continuous_phase}')
            if speed > float(limits[self.continuous_phase]) + 1e-12:
                raise RobotError('command exceeds continuous phase speed envelope')
        self._check_connected()
        state = self.read_state()
        # A fresh read may revoke a previous standstill observation.
        self._check_motion_authorized()
        assert self.guard is not None
        direction = self.guard.check_velocity(direction_xy, float(speed))
        predicted = state.pose[:2]+direction*speed*duration
        try:
            for point in (state.pose[:2], predicted):
                check_continuous_xy(self.config, point)
            geometry = self.config.get('continuous_search_geometry')
            if self.continuous_phase == 'TARGET_SEARCH' and geometry is not None:
                if np.linalg.norm(predicted-np.asarray(geometry['origin_xy'])) >= geometry['usable_distance_m']:
                    raise SearchLimitReached('search geometric budget reached before command')
        except RobotError as exc:
            if self.continuous_phase == 'TARGET_SEARCH':
                raise SearchLimitReached(str(exc)) from exc
            raise
        self.guard.check_predicted_pose(state.pose, direction, float(speed), float(duration))
        if self.config.get('continuous_require_watchdog') and not self._packet_advanced:
            raise RobotError('RTDE packet progress not established')
        velocity = [direction[0] * speed, direction[1] * speed, 0.0, 0.0, 0.0, 0.0]
        try:
            # Set before the call so an ambiguous transport failure still
            # causes stop() to attempt speedStop.
            self.standstill_confirmed = False
            self._settled_since = self._settled_stamp = None
            self._stop_pending = False
            self.stop_state = 'RUNNING'
            self._stop_method = "speedStop"
            accepted = self.control.speedL(
                velocity,
                float(self.config["speed_acceleration"]),
                float(duration),
            )
            if accepted is False:
                raise RobotError("UR controller rejected speedL")
        except Exception as exc:
            self.stop()
            raise RobotError(f"speedL failed: {exc}") from exc

    def begin_return_mode(self) -> None:
        self._check_connected()
        self._return_mode = True

    def end_return_mode(self) -> None:
        self._return_mode = False

    def move_linear_async(self, target_pose, speed: float, acceleration: float) -> None:
        """Start one guarded asynchronous Cartesian segment for safe return."""
        self._check_motion_authorized()
        self._check_connected()
        target = _finite_six(target_pose, "return target pose")
        assert self.guard is not None
        self.guard.check_workspace(target)
        check_continuous_xy(self.config, target[:2])
        if self.config.get('continuous_require_watchdog'):
            if not self._return_mode:
                raise RobotError('continuous moveL is only allowed during safe return')
            state = self.read_state()
            self._check_motion_authorized()
            check_continuous_xy(self.config, state.pose[:2])
            if not self._packet_advanced:
                raise RobotError('RTDE packet progress not established')
        if speed <= 0.0 or speed > float(self.config["max_tcp_speed"]):
            raise RobotError("return speed is outside configured TCP speed limit")
        if acceleration <= 0.0:
            raise RobotError("return acceleration must be positive")
        try:
            self.standstill_confirmed = False
            self._settled_since = self._settled_stamp = None
            self._stop_pending = False
            self.stop_state = 'RUNNING'
            self._stop_method = "stopL"
            accepted = self.control.moveL(
                target.tolist(), float(speed), float(acceleration), True
            )
            if accepted is False:
                raise RobotError("UR controller rejected asynchronous moveL")
        except Exception as exc:
            self.request_stop()
            raise RobotError(f"asynchronous return moveL failed: {exc}") from exc

    def _check_packet_freshness(self, now):
        """Compare device timestamps only with themselves; host clock measures stagnation.

        First-seen host time is a receipt proxy, not synchronized acquisition
        time. The continuous mode requires progress before its first motion.
        """
        if not hasattr(self.receive, 'getTimestamp'):
            raise RobotError('RTDE package timestamp unavailable')
        stamp = float(self.receive.getTimestamp())
        if not np.isfinite(stamp) or (self._packet_stamp is not None and stamp < self._packet_stamp):
            raise RobotError('invalid/backwards RTDE package timestamp')
        if self._packet_stamp is None or stamp > self._packet_stamp:
            self._packet_advanced = self._packet_stamp is not None
            self._packet_stamp, self._packet_seen_at = stamp, now
        age = now-self._packet_seen_at
        self.observation_timing.update(rtde_device_timestamp=stamp, rtde_packet_stagnation_sec=age)
        if age > float(self.config.get('continuous_sample_age_sec', .02)):
            raise RobotError('stale RTDE package: host reads do not refresh device data')

    def _check_observation_latency(self, started, ended):
        limit = float(self.config.get('continuous_sample_age_sec', .02))
        if ended-started > limit or ended-self._packet_seen_at > limit:
            raise RobotError('stale RTDE observation: read latency exceeded')

    @staticmethod
    def verified_watchdog_contract():
        # Importing classes/docs is read-only; never instantiate an interface.
        from importlib.metadata import version
        import rtde_control
        import rtde_receive
        installed = version('ur-rtde')
        if installed != '1.6.5':
            raise RobotError(f'continuous watchdog contract unverified for ur-rtde {installed}')
        cls = rtde_control.RTDEControlInterface
        for name in ('speedL', 'speedStop', 'setWatchdog', 'kickWatchdog'):
            if not hasattr(cls, name) or '-> bool' not in (getattr(cls, name).__doc__ or ''):
                raise RobotError(f'unverified SDK contract: {name}')
        if not hasattr(rtde_receive.RTDEReceiveInterface, 'getTimestamp'):
            raise RobotError('RTDE package timestamps unavailable')
        if 'asynchronous: bool = False' not in (cls.stopL.__doc__ or ''):
            raise RobotError('asynchronous stopL contract unavailable')
        return dict(version=installed, speedL_time='function return time, NOT motion expiry',
                    speedStop='synchronous; real-time stop uses speedL zero with stop_deceleration, '
                              'then speedStop once after held standstill; bool is API evidence only',
                    stopL='void/None; asynchronous=True requests braking without waiting for standstill',
                    watchdog='setWatchdog/kickWatchdog bool; default action shuts down control')

    def _check_motion_authorized(self):
        if self.control is None:
            raise RobotError('motion requires confirmed Control activation')
        if self.motion_fault:
            raise RobotError(f'motion fault latched: {self.motion_fault}')
        if self._stop_pending and not self.standstill_confirmed:
            raise RobotError('stop request pending measured standstill')
        self._check_watchdog_health()

    def _check_watchdog_health(self):
        """Monitoring during braking does not authorize a new motion."""
        if self.motion_fault:
            raise RobotError(f'motion fault latched: {self.motion_fault}')
        if self.config.get('continuous_require_watchdog'):
            if not self.watchdog_active or self._watchdog_last_kick is None:
                raise RobotError('continuous motion requires active watchdog and healthy-cycle kick')
            if time.monotonic()-self._watchdog_last_kick >= 1/self._watchdog_frequency:
                self.motion_fault = 'watchdog deadline exceeded'
                raise RobotError(self.motion_fault)

    def request_return_stop(self):
        return self.request_stop()

    def enable_watchdog(self, frequency):
        self._check_connected()
        if self.motion_fault:
            raise RobotError(self.motion_fault)
        try:
            if self.control.setWatchdog(float(frequency)) is not True:
                raise RobotError('setWatchdog not accepted')
            self.watchdog_active = True
            self._watchdog_frequency = float(frequency)
        except Exception as exc:
            self.motion_fault = f'watchdog enable failed: {exc}'
            raise RobotError(self.motion_fault) from exc

    def kick_watchdog(self):
        if not self.watchdog_active or self.motion_fault:
            raise RobotError('watchdog inactive or faulted')
        try:
            if self.control.kickWatchdog() is not True:
                raise RobotError('kickWatchdog not accepted')
            self._watchdog_last_kick = time.monotonic()
        except Exception as exc:
            self.motion_fault = f'watchdog kick failed: {exc}'
            raise RobotError(self.motion_fault) from exc

    def request_stop(self, *, nonblocking=None):
        """Request braking once; real-time callers poll actual speed each cycle.

        ur-rtde 1.6.5 speedStop has no async flag and waits for stopl to finish.
        In a control loop, first set speedL's persistent target to zero using
        the SAME stop deceleration. Only after held standstill do we exit speed
        mode with speedStop. stopL for return moves already supports async.
        All SDK calls stay on one thread; there is no concurrent watchdog call.
        """
        if self.control is None:
            return None
        if self._stop_pending:
            return self.stop_report
        self._stop_pending = True
        self._stop_started = time.monotonic()
        self._stop_primitive_called = False
        self.stop_state = 'STOPPING'
        self.standstill_confirmed = False
        self._settled_since = self._settled_stamp = None
        report = dict(method=self._stop_method, return_value=None, exception=None,
                      api_anomaly=False, physical_stop='unconfirmed')
        self.stop_report = report
        self.stop_history.append(report)
        if nonblocking is None:
            nonblocking = self._return_mode or self.config.get('continuous_require_watchdog', False)
        if nonblocking and self._stop_method == 'speedStop':
            report['braking_method'] = 'speedL_zero'
            try:
                value = self.control.speedL([0.] * 6, float(self.config['stop_deceleration']),
                                            float(self.config.get('observation_period_sec', .01)))
                report['braking_return_value'] = value
                report['api_anomaly'] = value is not True
            except Exception as exc:
                report.update(braking_exception=f'{type(exc).__name__}: {exc}', api_anomaly=True)
        else:
            self._call_stop_primitive()
        return report

    def _call_stop_primitive(self):
        self._stop_primitive_called = True
        report = self.stop_report
        try:
            primitive = getattr(self.control, self._stop_method)
            args = (float(self.config['stop_deceleration']),)
            if self._stop_method == 'stopL':
                args += (True,)
            value = primitive(*args)
            report['return_value'] = value
            report['api_anomaly'] |= not (value is True or (value is None and self._stop_method == 'stopL'))
        except Exception as exc:
            report.update(exception=f'{type(exc).__name__}: {exc}', api_anomaly=True)

    def poll_stop(self):
        """One tick after a fresh state read. Never sleeps or waits for motion."""
        if not self._stop_pending:
            return self.standstill_confirmed
        if self.stop_state == 'FAILED':
            raise RobotError('STOP_MOTION_ERROR: fresh actual TCP speed did not settle')
        if self.stop_state == 'STOPPED' and self.standstill_confirmed:
            return True
        if time.monotonic()-self._stop_started >= float(self.config.get('confirmation_timeout_sec', 1.0)):
            self.stop_state = 'FAILED'
            self.motion_fault = 'STOP_MOTION_ERROR: fresh actual TCP speed did not settle'
            self.stop_report['physical_stop'] = 'failed'
            raise RobotError(self.motion_fault)
        if self.standstill_confirmed:
            if not self._stop_primitive_called:
                self._call_stop_primitive()
                # A False/exception is evidence about the API, not the robot.
                # Re-observe speed after the mode-exit call before accepting it.
                self.read_diagnostic_state()
            if self.standstill_confirmed:
                self.stop_state = 'STOPPED'
                return True
        return False

    def _observe_standstill(self, speed, now):
        stamp = self._packet_stamp
        if stamp is None or stamp == self._settled_stamp:
            return
        self._settled_stamp = stamp
        low = (np.linalg.norm(speed[:3]) <= float(self.config.get('continuous_settle_speed_mps', 1e-4))
               and np.linalg.norm(speed[3:]) <= .005)
        self._settled_since = (now if self._settled_since is None else self._settled_since) if low else None
        self.standstill_confirmed = bool(self._settled_since is not None and
            now-self._settled_since >= float(self.config.get('settle_hold_sec', .08)))
        if self.stop_report is not None and self._stop_pending and self.stop_state != 'FAILED':
            self.stop_report['physical_stop'] = 'confirmed' if self.standstill_confirmed else 'unconfirmed'

    def wait_for_standstill(self, *, observe=None, timeout=None):
        """Fresh device timestamps + held actual TCP speed are the sole authority."""
        self.standstill_confirmed = False
        self._settled_since = self._settled_stamp = None
        deadline = time.monotonic() + float(timeout or self.config.get('confirmation_timeout_sec', 1.0))
        try:
            while time.monotonic() < deadline:
                state = observe() if observe else self.read_diagnostic_state()
                if self.poll_stop():
                    return state
                time.sleep(float(self.config.get('observation_period_sec', .01)))
            raise RobotError('STOP_MOTION_ERROR: fresh actual TCP speed did not settle')
        except BaseException:
            self.standstill_confirmed = False
            if self.stop_report is not None and self.stop_state != 'FAILED':
                self.stop_report['physical_stop'] = 'unconfirmed'
            raise

    def stop(self):
        return self.request_stop()

    def finish_control_script_if_stopped(self):
        if self.control is None or self.control_script_stop_requested or not self.standstill_confirmed:
            return False
        self.read_diagnostic_state()
        if not self.standstill_confirmed:
            return False
        if not self.poll_stop():
            return False
        value = self.control.stopScript()
        if value is not True and value is not None:
            raise RobotError(f'stopScript returned {value!r}')
        self.control_script_stop_requested = True
        self.watchdog_active = False
        self._watchdog_last_kick = None
        return True

    def close(self):
        if self._closed:
            return
        try:
            if self.control is not None:
                if self.stop_state == 'FAILED':
                    raise RobotError(self.motion_fault)
                if not self.standstill_confirmed:
                    self.request_stop()
                    self.wait_for_standstill()
                self.finish_control_script_if_stopped()
        finally:
            self._closed = True
            errors = []
            for interface in (self.control, self.receive):
                if interface is not None:
                    try:
                        interface.disconnect()
                    except Exception as exc:
                        errors.append(exc)
            self.control = self.receive = None
            if errors:
                raise RobotError(f'RTDE disconnect failed: {errors}')


def validate_execution_configuration(config: dict) -> None:
    """Validate required numeric motion configuration without confirmation gates."""
    if bool(config["workspace"].get("enabled", True)):
        limits = config["workspace"]["limits"]
        for axis in "xyz":
            if float(limits[f"{axis}_min"]) >= float(limits[f"{axis}_max"]):
                raise RobotError(f"workspace {axis} limits are invalid")
    _finite_six(config["tcp"]["offset"], "tcp.offset")
    if float(config["robot"]["max_tcp_speed"]) <= 0.0:
        raise RobotError("robot.max_tcp_speed must be positive")

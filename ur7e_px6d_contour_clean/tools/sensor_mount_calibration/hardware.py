"""Bounded calibration adapter over the existing PX6D and UR RTDE owners.

Importing/constructing this module opens no device. Only ``connect`` opens
Receive + PX6D; the caller's Enter confirmation must precede ``activate``.
No zeroing, wrench transforms, TCP/payload writes or scan startup are used.
"""

from __future__ import annotations

from datetime import datetime, timezone
import math
import time

import numpy as np

from config.loader import runtime_robot_config
from robot.rtde_controller import (
    RobotError, URRTDEController, _finite_six, _orientation_distance,
)
from sensor.px6d_reader import PX6DReader
from .plan import interpolate_path


class HardwareError(RobotError):
    """A latched calibration fault; the session cannot resume motion."""


def interpolate_poses(start, target):
    """The entire unblended moveL path, at <=1 mm and <=1 degree steps."""
    start, target = _finite_six(start, 'path start'), _finite_six(target, 'path target')
    yield from (np.array(p) for p in interpolate_path(start, [dict(target=target)])[1:])


class Hardware:
    WATCHDOG_HZ = 10.0
    WRIST_SINGULARITY_MARGIN_RAD = math.radians(10)

    def __init__(self, config, limits, *, controller_factory=URRTDEController,
                 sensor_factory=PX6DReader):
        self.config, self.limits = config, limits
        self._inspect_sdk_contract = controller_factory is URRTDEController
        configured_bounds = [float(config['robot'][key]) for key in
                             ('max_tcp_speed', 'speed_acceleration', 'stop_deceleration')]
        configured_bounds.extend(float(config['sensor'][key]) for key in
                                 ('timeout_sec', 'poll_rate_hz'))
        if any(not math.isfinite(value) or value <= 0 for value in configured_bounds):
            raise ValueError('finite positive configured motion/sensor bounds required')
        runtime = runtime_robot_config(config)
        # These are isolated calibration bounds, never written to scan config.
        runtime.update(fixed_z=None, fixed_z_tolerance=limits.max_translation_m,
                       orientation_tolerance_rad=limits.max_tilt_rad,
                       continuous_sample_age_sec=limits.observation_timeout_sec,
                       observation_period_sec=limits.sample_period_sec,
                       allow_unverified_active_tcp=False)
        self.command_speed = min(float(config['robot']['max_tcp_speed']),
                                 limits.command_speed, limits.max_linear_speed)
        self.acceleration = min(float(config['robot']['speed_acceleration']),
                                limits.acceleration)
        runtime['max_tcp_speed'] = self.command_speed
        # Existing stop deceleration is retained or lowered, never raised.
        runtime['stop_deceleration'] = min(float(config['robot']['stop_deceleration']),
                                           limits.stop_deceleration)
        if min(self.command_speed, self.acceleration, runtime['stop_deceleration']) <= 0:
            raise ValueError('calibration speed/acceleration/deceleration must be positive')
        self.force_limit = float(config['policy']['absolute_raw_force_threshold'])
        self.torque_limit = float(config['policy']['absolute_raw_torque_threshold'])
        if not (0 < self.force_limit < math.inf and 0 < self.torque_limit < math.inf):
            raise ValueError('finite positive raw force/torque backstops required')
        # The shared controller's "continuous" flag imposes scan-return logic.
        # The calibration owner instead gates its own commands on the same
        # robot-side watchdog and on a fresh, healthy sensor/robot pair.
        runtime.pop('continuous_require_watchdog', None)
        self.controller = controller_factory(runtime)
        sensor = config['sensor']
        self.sensor = sensor_factory(
            device=sensor['serial_port'], baudrate=int(sensor['baudrate']),
            timeout_sec=min(float(sensor['timeout_sec']), limits.observation_timeout_sec),
            poll_rate_hz=float(sensor['poll_rate_hz']),
            startup_delay_sec=float(sensor['startup_delay_sec']))
        self.origin_pose = self.origin_q = None
        self._active = self._connected = self._closed = self._stopped = False
        self._last_stamp = self._last_q = self._last_good = None
        self.last_read_diagnostics = None
        self.progress = None
        self._watchdog_kick_time = None
        self._approved_targets = []
        self._approved_joint_endpoints = []
        self._next_target = 0
        self.metadata = dict(watchdog_hz=self.WATCHDOG_HZ,
                             effective_moveL_speed_mps=self.command_speed,
                             actual_angular_speed_limit_rad_s=limits.max_angular_speed,
                             effective_acceleration=self.acceleration,
                             absolute_raw_force_threshold_N=self.force_limit,
                             absolute_raw_torque_threshold_Nm=self.torque_limit,
                             robot_status_reference=(
                                 'https://docs.universal-robots.com/tutorials/'
                                 'communication-protocol-tutorials/rtde-guide.html'),
                             collision_check='UR safety + sampled IK; external obstacles '
                                             'and cable clearance require operator inspection')

    def connect(self):
        if self._closed or self._connected:
            raise HardwareError('calibration adapter is already connected/closed')
        try:
            self.controller.connect()
            self.sensor.connect()
            self._connected = True
            observation = self.read()
            self.origin_pose = np.array(observation['actual_tcp_pose'])
            self.origin_q = np.array(observation['actual_q'])
            self.metadata.update(start_pose=self.origin_pose.tolist(),
                                 start_joints=self.origin_q.tolist(),
                                 sensor_firmware=self.sensor.firmware)
        except BaseException as exc:
            self._cleanup_after_error(exc, close=True)
            raise

    def _cleanup_after_error(self, original, *, close=False):
        """Preserve the triggering failure even if braking/disconnect also fail."""
        operations = [('stop', self.stop)]
        if close:
            operations.append(('close', self.close))
        for name, operation in operations:
            try:
                operation()
            except BaseException as secondary:
                original.add_note(f'Additional calibration {name} failure: '
                                  f'{type(secondary).__name__}: {secondary}')
            finally:
                self._stopped = True

    def _status(self):
        receive = self.controller.receive
        if receive is None or not receive.isConnected():
            raise HardwareError('RTDE Receive disconnected')
        if receive.isEmergencyStopped() or receive.isProtectiveStopped():
            raise HardwareError('UR emergency/protective stop active')
        # UR's documented enums: robot RUNNING=7, safety NORMAL=1/REDUCED=2.
        if receive.getRobotMode() != 7 or receive.getSafetyMode() not in (1, 2):
            raise HardwareError('UR must be running in normal/reduced safety mode')
        status = int(receive.getRobotStatus())
        if not status & 1 or status & 4:
            raise HardwareError('UR power off or teach/freedrive button active')
        if self._active:
            if (receive.getRuntimeState() != 2 or not status & 2 or
                    not self.controller.control.isConnected()):
                raise HardwareError('UR control program stopped/paused or disconnected')

    def _watchdog_deadline(self):
        if self._active and self._watchdog_kick_time is not None:
            if time.monotonic() - self._watchdog_kick_time >= 1 / self.WATCHDOG_HZ:
                raise HardwareError('robot-side watchdog deadline elapsed; refusing to resume')

    def activate(self):
        if not self._connected or self._active or self._stopped:
            raise HardwareError('activation requires a fresh, connected calibration session')
        try:
            self.read()
            self.controller.activate_control(confirmed=True)
            control = self.controller.control
            tcp_owner = (self.controller.receive if hasattr(self.controller.receive, 'getTCPOffset')
                         else control)
            self.metadata['active_tcp_offset'] = _finite_six(
                tcp_owner.getTCPOffset(), 'active TCP offset').tolist()
            for method in ('setWatchdog', 'kickWatchdog', 'stopL',
                           'getInverseKinematics', 'getInverseKinematicsHasSolution',
                           'isPoseWithinSafetyLimits', 'isJointsWithinSafetyLimits'):
                if not callable(getattr(control, method, None)):
                    raise HardwareError(f'required UR RTDE API unavailable: {method}')
            if self._inspect_sdk_contract:
                # Inspect the sole owner's existing instance, before watchdog or
                # motion. No extra SDK constructor or version allowlist.
                for name in ('setWatchdog', 'kickWatchdog', 'getInverseKinematicsHasSolution',
                             'isPoseWithinSafetyLimits', 'isJointsWithinSafetyLimits'):
                    if '-> bool' not in (getattr(control, name).__doc__ or ''):
                        raise HardwareError(f'unsupported SDK return contract: {name}')
                if 'asynchronous: bool = False' not in (control.stopL.__doc__ or ''):
                    raise HardwareError('SDK does not document asynchronous stopL')
            self._status()
            self.controller.enable_watchdog(self.WATCHDOG_HZ)
            self._watchdog_kick_time = time.monotonic()
            self._active = True
            self.read()
        except BaseException as exc:
            self._cleanup_after_error(exc)
            raise

    def _check_pose(self, pose):
        if self.origin_pose is None:
            return
        if np.linalg.norm(pose[:3] - self.origin_pose[:3]) > self.limits.max_translation_m + 1e-9:
            raise HardwareError('calibration translation bound exceeded')
        if _orientation_distance(self.origin_pose[3:], pose[3:]) > self.limits.max_tilt_rad + 1e-9:
            raise HardwareError('calibration tilt bound exceeded')

    def _check_joints(self, joints, *, previous=None):
        if abs(math.sin(joints[4])) < math.sin(self.WRIST_SINGULARITY_MARGIN_RAD):
            raise HardwareError('wrist too close to singularity; reposition manually')
        # Compare actual unwrapped coordinates: never hide a full turn with modulo.
        if self.origin_q is not None and np.max(np.abs(joints - self.origin_q)) > self.limits.joint_excursion_rad:
            raise HardwareError('joint excursion bound exceeded')
        if previous is not None and np.max(np.abs(joints - previous)) > self.limits.joint_step_rad:
            raise HardwareError('joint continuity bound exceeded')

    def _remember_read_timing(self, timing, ended, *, completed):
        """Keep partial timings without another device read, including on failure."""
        timing.update(hardware_read_ended_monotonic=ended,
                      hardware_read_duration_sec=ended - timing['hardware_read_started_monotonic'],
                      read_completed=completed)
        for prefix, duration in [('raw_read', 'sensor_read_duration_sec'),
                                 ('rtde_read', 'rtde_read_duration_sec'),
                                 ('watchdog', 'watchdog_duration_sec')]:
            start_key, end_key = f'{prefix}_started_monotonic', f'{prefix}_ended_monotonic'
            if start_key in timing:
                timing.setdefault(end_key, ended)
                timing[duration] = timing[end_key] - timing[start_key]
        self.last_read_diagnostics = dict(timing)
        self.metadata['last_read_diagnostics'] = dict(timing)

    def read(self):
        if not self._connected or self._closed or self._stopped:
            raise HardwareError('cannot read an inactive/stopped calibration session')
        started = time.monotonic()
        timing = dict(hardware_read_started_monotonic=started,
                      sensor_read_duration_sec=0., rtde_read_duration_sec=0.,
                      watchdog_duration_sec=0., hardware_read_stage='status')
        try:
            self._watchdog_deadline()
            self._status()
            timing['hardware_read_stage'] = 'sensor_read'
            timing['raw_read_started_monotonic'] = time.monotonic()
            raw = _finite_six(self.sensor.read_wrench().array(), 'raw PX6D wrench')
            timing['raw_read_ended_monotonic'] = time.monotonic()
            if np.linalg.norm(raw[:3]) >= self.force_limit or np.linalg.norm(raw[3:]) >= self.torque_limit:
                raise HardwareError('absolute raw force/torque threshold reached')
            timing['hardware_read_stage'] = 'rtde_read'
            timing['rtde_read_started_monotonic'] = time.monotonic()
            while True:
                state = self.controller.read_state()
                stamp = float(self.controller._packet_stamp)
                if self._last_stamp is None or stamp > self._last_stamp:
                    break
                if time.monotonic() - started >= self.limits.observation_timeout_sec:
                    raise HardwareError('RTDE timestamp did not advance; no stale samples allowed')
                self._watchdog_deadline()
                time.sleep(.001)
            receive = self.controller.receive
            joints = _finite_six(receive.getActualQ(), 'actual joint positions')
            joint_speed = _finite_six(receive.getActualQd(), 'actual joint speeds')
            self._status()
            self._check_pose(state.pose)
            self._check_joints(joints, previous=self._last_q)
            if np.linalg.norm(state.tcp_speed[:3]) > self.limits.max_linear_speed:
                raise HardwareError('actual translation speed exceeded calibration limit')
            if np.linalg.norm(state.tcp_speed[3:]) > self.limits.max_angular_speed:
                raise HardwareError('actual angular speed exceeded calibration limit')
            if np.max(np.abs(joint_speed)) > self.limits.max_joint_speed:
                raise HardwareError('actual joint speed exceeded calibration limit')
            ended = time.monotonic()
            timing['rtde_read_ended_monotonic'] = ended
            timing['pair_read_duration_sec'] = ended - started
            timing['host_monotonic'] = ended
            if ended - started > self.limits.observation_timeout_sec:
                raise HardwareError('paired PX6D/RTDE observation exceeded timeout')
            self._watchdog_deadline()
            if self._active:
                timing['hardware_read_stage'] = 'watchdog'
                timing['watchdog_started_monotonic'] = time.monotonic()
                self.controller.kick_watchdog()
                kicked_at = time.monotonic()
                timing['watchdog_ended_monotonic'] = kicked_at
                # A successful but late SDK call cannot refresh an old pair or
                # revive a watchdog interval that already elapsed.
                if kicked_at - started > self.limits.observation_timeout_sec:
                    raise HardwareError('paired PX6D/RTDE observation exceeded timeout during watchdog kick')
                self._watchdog_deadline()
                self._watchdog_kick_time = kicked_at
            timing['hardware_read_stage'] = 'observation_assembly'
            observation = dict(actual_tcp_pose=state.pose.tolist(), tcp_speed=state.tcp_speed.tolist(),
                               actual_q=joints.tolist(), actual_qd=joint_speed.tolist(),
                               raw_wrench=raw.tolist(), host_monotonic=ended,
                               utc_time=datetime.now(timezone.utc).isoformat(), robot_timestamp=stamp)
            finished = time.monotonic()
            if finished - started > self.limits.observation_timeout_sec:
                raise HardwareError('paired PX6D/RTDE observation exceeded timeout before return')
            self._watchdog_deadline()
            timing['hardware_read_stage'] = 'completed'
            self._remember_read_timing(timing, finished, completed=True)
            observation.update(timing)
            self._last_stamp, self._last_q, self._last_good = stamp, joints.copy(), ended
            return observation
        except BaseException as exc:
            self._remember_read_timing(timing, time.monotonic(), completed=False)
            diagnostics = getattr(exc, 'diagnostics', None)
            exc.diagnostics = {**(diagnostics if isinstance(diagnostics, dict) else {}),
                               'hardware_read_timing': dict(timing)}
            self._cleanup_after_error(exc)
            raise

    def preflight(self, poses, heartbeat):
        """Check every interpolated waypoint with UR safety limits and nearby IK."""
        if not self._active or self._stopped:
            raise HardwareError('preflight requires active, healthy calibration control')
        self._approved_targets = []
        self._approved_joint_endpoints = []
        started = time.monotonic()
        try:
            observation = self.read()
            previous_pose = np.array(observation['actual_tcp_pose'])
            joints = np.array(observation['actual_q'])
            targets = [_finite_six(p, 'preflight target') for p in poses]
            if not targets or len(targets) > 128:
                raise HardwareError('preflight requires a finite plan of 1..128 poses')
            path, endpoints = [], []
            control = self.controller.control
            previous = previous_pose
            estimated_points = 0
            for target in targets:
                estimated_points += max(1, math.ceil(_orientation_distance(previous[3:], target[3:]) / math.radians(1)),
                                        math.ceil(np.linalg.norm(target[:3]-previous[:3]) / .001))
                previous = target
            self.metadata['preflight_progress'] = dict(completed_points=0, estimated_points=estimated_points,
                                                       completed_segments=0, total_segments=len(targets))
            if self.progress is not None:
                self.progress.emit(f'路径预检开始：{len(targets)} 段，约 {estimated_points} 个检查点；机器人保持静止。', force=True)

            def checked(method, *args):
                heartbeat()
                self._watchdog_deadline()
                result = method(*args)
                if time.monotonic() - started > 180:
                    raise HardwareError('preflight time limit exceeded')
                self._watchdog_deadline()
                heartbeat()
                return result

            for segment_index, target in enumerate(targets, 1):
                self._check_pose(target)
                for pose in interpolate_poses(previous_pose, target):
                    if len(path) >= 6000:
                        raise HardwareError('preflight interpolation budget exceeded')
                    self._check_pose(pose)
                    if checked(control.isPoseWithinSafetyLimits, pose.tolist()) is not True:
                        raise HardwareError('UR pose safety limits reject planned path')
                    if checked(control.getInverseKinematicsHasSolution, pose.tolist(), joints.tolist()) is not True:
                        raise HardwareError('planned path has no nearby inverse kinematics solution')
                    candidate = _finite_six(checked(control.getInverseKinematics, pose.tolist(),
                                                    joints.tolist()), 'IK solution')
                    self._check_joints(candidate, previous=joints)
                    if checked(control.isJointsWithinSafetyLimits, candidate.tolist()) is not True:
                        raise HardwareError('UR joint safety limits reject planned path')
                    path.append(candidate.tolist())
                    joints = candidate
                    self.metadata['preflight_progress'].update(completed_points=len(path),
                                                               current_segment=segment_index)
                    if self.progress is not None:
                        self.progress.emit(f'路径预检 {len(path)}/{estimated_points} 点（约 {min(100., len(path)/estimated_points*100):.0f}%），'
                                           f'路段 {segment_index}/{len(targets)}，已用 {time.monotonic()-started:.1f}s。', key='preflight')
                endpoints.append(joints.copy())
                previous_pose = target
                self.metadata['preflight_progress']['completed_segments'] = segment_index
            self._approved_targets = [p.copy() for p in targets]
            self._approved_joint_endpoints = endpoints
            self._next_target = 0
            if self.progress is not None:
                self.progress.emit(f'路径预检通过：{len(path)} 个点，耗时 {time.monotonic()-started:.1f}s。', force=True)
            return dict(joint_path=path, sample_count=len(path),
                        joint_endpoint_path=[q.tolist() for q in endpoints],
                        duration_sec=time.monotonic() - started)
        except BaseException as exc:
            self._cleanup_after_error(exc)
            raise

    def command(self, target_pose):
        try:
            if not self._active or self._stopped:
                raise HardwareError('motion forbidden after stop or before activation')
            target = _finite_six(target_pose, 'command target')
            if self._next_target >= len(self._approved_targets):
                raise HardwareError('no preflight-approved target remains')
            expected = self._approved_targets[self._next_target]
            if (np.linalg.norm(target[:3] - expected[:3]) > 1e-9 or
                    _orientation_distance(target[3:], expected[3:]) > 1e-7):
                raise HardwareError('command does not match preflight-approved path order')
            observation = self.read()
            actual = np.array(observation['actual_tcp_pose'])
            previous_pose = self.origin_pose if self._next_target == 0 else self._approved_targets[self._next_target - 1]
            previous_q = self.origin_q if self._next_target == 0 else self._approved_joint_endpoints[self._next_target - 1]
            if (np.linalg.norm(actual[:3] - previous_pose[:3]) > .0005 or
                    _orientation_distance(actual[3:], previous_pose[3:]) > math.radians(.5)):
                raise HardwareError('actual pose changed from preflight segment start')
            self._check_joints(np.array(observation['actual_q']), previous=previous_q)
            speed = np.array(observation['tcp_speed'])
            if (np.linalg.norm(speed[:3]) > .0001 or np.linalg.norm(speed[3:]) > .005 or
                    np.max(np.abs(observation['actual_qd'])) > .005):
                raise HardwareError('moveL requires actual standstill')
            self._check_pose(target)
            self._watchdog_deadline()
            self.controller.move_linear_async(target, self.command_speed, self.acceleration)
            self._next_target += 1
        except BaseException as exc:
            self._cleanup_after_error(exc)
            raise

    def stop(self):
        """Latch first, request asynchronous stopL immediately; never auto-return."""
        if self._stopped:
            return
        self._stopped = True
        if self.controller.control is not None:
            self.controller._stop_method = 'stopL'
            self.controller.request_stop(nonblocking=True)

    def close(self):
        if self._closed:
            return
        primary = None
        for name, operation in [('stop', self.stop), ('sensor close', self.sensor.close),
                                ('controller close', self.controller.close)]:
            try:
                operation()
            except BaseException as exc:
                if primary is None:
                    primary = exc
                else:
                    primary.add_note(f'Additional calibration {name} failure: '
                                     f'{type(exc).__name__}: {exc}')
        self._closed = self._stopped = True
        self._connected = False
        if primary is not None:
            raise primary

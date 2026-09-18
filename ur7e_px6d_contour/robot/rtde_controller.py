"""UR7e RTDE connection, state reads, motion execution, and stop operations."""

from __future__ import annotations

import time

import numpy as np

from core.models import RobotState


class RobotError(RuntimeError):
    pass


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

    def __init__(self, config: dict):
        self.config = config
        self.control = None
        self.receive = None
        self.guard: WorkspaceGuard | None = None
        self._stopped = True
        self._motion_mode: str | None = None
        self._return_mode = False

    def connect(self, allow_start_away_from_fixed_pose: bool = False) -> None:
        try:
            import rtde_control
            import rtde_receive

            self.receive = rtde_receive.RTDEReceiveInterface(self.config["robot_ip"])
            self.control = rtde_control.RTDEControlInterface(self.config["robot_ip"])
            pose = _finite_six(self.receive.getActualTCPPose(), "actual TCP pose")
            fixed_z = self.config.get("fixed_z")
            if fixed_z is None:
                fixed_z = float(pose[2])
            fixed_orientation = self.config.get("fixed_orientation")
            if fixed_orientation is None:
                fixed_orientation = pose[3:].copy()
            self.guard = WorkspaceGuard(
                self.config["workspace_limits"],
                fixed_z,
                float(self.config["fixed_z_tolerance"]),
                float(self.config["max_tcp_speed"]),
                fixed_orientation=fixed_orientation,
                orientation_tolerance_rad=float(self.config["orientation_tolerance_rad"]),
                workspace_enabled=bool(self.config.get("workspace_enabled", True)),
            )
            if allow_start_away_from_fixed_pose:
                # Startup may intentionally begin at P1 or another stopped
                # pose. SafeReturnExecutor moves to calibrated P0 before SEARCH.
                self.guard.check_workspace(pose)
            else:
                self.guard.check_pose(pose)
            self._verify_tcp()
        except Exception as exc:
            self.close()
            raise RobotError(f"RTDE initialization failed: {exc}") from exc

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
            if not np.allclose(actual, expected, atol=float(self.config["tcp_offset_tolerance"])):
                raise RobotError(f"active TCP {actual.tolist()} does not match config {expected.tolist()}")
        elif not bool(self.config.get("allow_unverified_active_tcp", False)):
            raise RobotError("this ur-rtde build cannot read the active TCP; refusing unverified motion")

    def _check_connected(self) -> None:
        if self.receive is None or self.control is None:
            raise RobotError("RTDE is not connected")
        for interface, name in ((self.receive, "receive"), (self.control, "control")):
            if hasattr(interface, "isConnected") and not interface.isConnected():
                raise RobotError(f"RTDE {name} interface disconnected")
        if hasattr(self.receive, "isEmergencyStopped") and self.receive.isEmergencyStopped():
            raise RobotError("UR emergency stop is active")
        if hasattr(self.receive, "isProtectiveStopped") and self.receive.isProtectiveStopped():
            raise RobotError("UR protective stop is active")

    def read_state(self) -> RobotState:
        self._check_connected()
        try:
            pose = _finite_six(self.receive.getActualTCPPose(), "actual TCP pose")
            speed = _finite_six(self.receive.getActualTCPSpeed(), "actual TCP speed")
            assert self.guard is not None
            if self._return_mode:
                self.guard.check_workspace(pose)
            else:
                self.guard.check_pose(pose)
            if np.linalg.norm(speed[:3]) > float(self.config["max_tcp_speed"]) * 1.20:
                raise RobotError("measured TCP speed exceeds maximum plus tolerance")
            return RobotState(time.monotonic(), pose, speed)
        except RobotError:
            raise
        except Exception as exc:
            raise RobotError(f"RTDE state read failed: {exc}") from exc

    def read_diagnostic_state(self) -> RobotState:
        """Best-effort read for emergency reporting; never authorizes motion."""
        if self.receive is None:
            raise RobotError("RTDE receive interface is not connected")
        try:
            pose = _finite_six(self.receive.getActualTCPPose(), "actual TCP pose")
            speed = _finite_six(self.receive.getActualTCPSpeed(), "actual TCP speed")
            return RobotState(time.monotonic(), pose, speed)
        except Exception as exc:
            raise RobotError(f"RTDE diagnostic read failed: {exc}") from exc

    def command_planar_velocity(self, direction_xy, speed: float, duration: float) -> None:
        self._check_connected()
        state = self.read_state()
        assert self.guard is not None
        direction = self.guard.check_velocity(direction_xy, float(speed))
        self.guard.check_predicted_pose(state.pose, direction, float(speed), float(duration))
        velocity = [direction[0] * speed, direction[1] * speed, 0.0, 0.0, 0.0, 0.0]
        try:
            # Set before the call so an ambiguous transport failure still
            # causes stop() to attempt speedStop.
            self._stopped = False
            self._motion_mode = "speed"
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
        self._check_connected()
        target = _finite_six(target_pose, "return target pose")
        assert self.guard is not None
        self.guard.check_workspace(target)
        if speed <= 0.0 or speed > float(self.config["max_tcp_speed"]):
            raise RobotError("return speed is outside configured TCP speed limit")
        if acceleration <= 0.0:
            raise RobotError("return acceleration must be positive")
        try:
            self._stopped = False
            self._motion_mode = "linear"
            accepted = self.control.moveL(
                target.tolist(), float(speed), float(acceleration), True
            )
            if accepted is False:
                raise RobotError("UR controller rejected asynchronous moveL")
        except Exception as exc:
            self.safe_stop_motion()
            raise RobotError(f"asynchronous return moveL failed: {exc}") from exc

    def safe_stop_motion(self, force: bool = False) -> None:
        """Stop current motion without disconnecting or preventing later return."""
        if self.control is None or (self._stopped and not force):
            return
        if force and self._motion_mode is None:
            # A newly connected manual-recovery process does not know the
            # primitive that may have been active before it started.  At least
            # one synchronous stop primitive must be accepted.
            accepted_stop = False
            for method_name in ("stopL", "speedStop"):
                try:
                    result = getattr(self.control, method_name)(
                        float(self.config["stop_deceleration"])
                    )
                    accepted_stop = accepted_stop or result is not False
                except Exception:
                    pass
            self._stopped = True
            self._motion_mode = None
            if not accepted_stop:
                raise RobotError("unable to confirm forced RTDE motion stop")
            return
        try:
            if self._motion_mode == "linear":
                self.control.stopL(float(self.config["stop_deceleration"]))
            else:
                self.control.speedStop(float(self.config["stop_deceleration"]))
        except Exception:
            # If the motion type is ambiguous after a transport error, attempt
            # both stop primitives. Either call may itself fail on disconnect.
            for method_name in ("speedStop", "stopL"):
                try:
                    getattr(self.control, method_name)(float(self.config["stop_deceleration"]))
                except Exception:
                    pass
        finally:
            self._stopped = True
            self._motion_mode = None

    def stop(self) -> None:
        self.safe_stop_motion()

    def close(self) -> None:
        self.stop()
        for interface in (self.control, self.receive):
            if interface is not None and hasattr(interface, "disconnect"):
                try:
                    interface.disconnect()
                except Exception:
                    pass
        self.control = None
        self.receive = None


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

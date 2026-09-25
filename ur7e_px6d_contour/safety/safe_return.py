"""Force-monitored return behavior, separate from contour exploration."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from core.models import PolicyCommand
from robot.rtde_controller import RobotError, _orientation_distance


class ReturnAborted(RuntimeError):
    pass


@dataclass(frozen=True)
class ReturnResult:
    status: str
    abort_reason: str
    stop_pose: list[float]
    final_pose: list[float]


def calculate_safe_return_z(
    stop_z: float, start_z: float, lift_distance: float, position_tolerance: float
) -> float:
    """Choose a +Z transit height valid from either side of the return target."""
    nominal_safe_z = float(start_z + lift_distance)
    if stop_z >= nominal_safe_z - position_tolerance:
        # Resuming at an already-safe height must not stack another lift.
        return float(max(stop_z, nominal_safe_z))
    # From a point below the target, merely adding lift_distance to stop_z can
    # leave the horizontal segment below the target.  Clear both endpoints.
    return float(max(stop_z + lift_distance, nominal_safe_z))


def validate_return_configuration(config: dict, start_pose) -> None:
    settings = config["safe_return"]
    lift_distance = settings.get("return_lift_distance")
    if lift_distance is None or not np.isfinite(float(lift_distance)):
        raise RobotError("safe_return.return_lift_distance must be configured")
    if float(lift_distance) <= 0.0:
        raise RobotError("safe_return.return_lift_distance must be positive")
    p0 = np.asarray(start_pose, dtype=float)
    if p0.shape != (6,) or not np.all(np.isfinite(p0)):
        raise RobotError("start_tcp_pose must contain six finite values")
    if bool(config["workspace"].get("enabled", True)):
        workspace = config["workspace"]["limits"]
        for index, axis in enumerate("xyz"):
            if not float(workspace[f"{axis}_min"]) <= p0[index] <= float(workspace[f"{axis}_max"]):
                raise RobotError(f"calibrated P0 {axis} is outside the configured workspace")
    positive = (
        "return_speed",
        "return_vertical_speed",
        "return_acceleration",
        "return_force_limit",
        "return_torque_limit",
        "return_position_tolerance",
        "return_orientation_tolerance",
        "return_segment_timeout_sec",
    )
    for name in positive:
        if float(settings[name]) <= 0.0:
            raise RobotError(f"safe_return.{name} must be positive")
    max_speed = float(config["robot"]["max_tcp_speed"])
    if max(float(settings["return_speed"]), float(settings["return_vertical_speed"])) > max_speed:
        raise RobotError("safe return speed exceeds robot.max_tcp_speed")


class SafeReturnExecutor:
    def __init__(
        self,
        config: dict,
        start_pose,
        controller,
        reader,
        preprocessor,
        logger=None,
        policy=None,
        target_label: str = "P0",
    ):
        validate_return_configuration(config, start_pose)
        self.config = config
        self.settings = config["safe_return"]
        self.start_pose = np.asarray(start_pose, dtype=float)
        self.controller = controller
        self.reader = reader
        self.preprocessor = preprocessor
        self.logger = logger
        self.policy = policy
        self.termination = getattr(logger, "termination", None) or getattr(policy, "termination", None)
        self.target_label = str(target_label)
        self.period = 1.0 / float(config["policy"]["control_rate_hz"])
        self._segment_deadline = None

    def execute(self) -> ReturnResult:
        self._observe(phase="SAFE_RETURN_BEGIN", return_target=self.start_pose)
        self.controller.begin_return_mode()
        stop_pose = self.start_pose.copy()
        try:
            # Force a stop even for a newly connected startup/manual-return
            # controller, whose local state cannot describe earlier motion.
            self.controller.safe_stop_motion(force=True)
            stop_state = self.controller.read_state()
            self._observe(robot=stop_state, phase="SAFE_RETURN_START_POSE")
            stop_pose = stop_state.pose.copy()
            lift_distance = float(self.settings["return_lift_distance"])
            safe_z = calculate_safe_return_z(
                float(stop_pose[2]),
                float(self.start_pose[2]),
                lift_distance,
                float(self.settings["return_position_tolerance"]),
            )

            p_safe = stop_pose.copy()
            p_safe[2] = safe_z
            above_start = self.start_pose.copy()
            above_start[2] = safe_z
            phases = []
            if safe_z - float(stop_pose[2]) > float(
                self.settings["return_position_tolerance"]
            ):
                phases.append(
                    ("VERTICAL_RETREAT", p_safe, float(self.settings["return_vertical_speed"]))
                )
            else:
                print("RETURN_TO_START VERTICAL_RETREAT: already at safe height; skipped")
            phases.extend(
                (
                    ("MOVE_ABOVE_START", above_start, float(self.settings["return_speed"])),
                    ("DESCEND_TO_START", self.start_pose, float(self.settings["return_vertical_speed"])),
                )
            )

            print(f"P_stop={stop_pose.tolist()}")
            print(f"{self.target_label}={self.start_pose.tolist()}")
            print(f"return_lift_distance={lift_distance:.6f} m, safe_z={safe_z:.6f} m")
            for phase, target, speed in phases:
                print(f"RETURN_TO_START {phase}: target={target.tolist()}")
                self._run_segment(phase, target, speed)
            final_state = self.controller.read_state()
            self._observe(robot=final_state, phase="SAFE_RETURN_FINAL_POSE")
            position_error = float(np.linalg.norm(final_state.pose[:3] - self.start_pose[:3]))
            orientation_error = _orientation_distance(final_state.pose[3:], self.start_pose[3:])
            if position_error > float(self.settings["return_position_tolerance"]):
                raise ReturnAborted(
                    f"final {self.target_label} position error {position_error:.6f} m"
                )
            if orientation_error > float(self.settings["return_orientation_tolerance"]):
                raise ReturnAborted(
                    f"final {self.target_label} orientation error {orientation_error:.6f} rad"
                )
            self.controller.safe_stop_motion()
            print("RETURN TO START COMPLETE")
            result = ReturnResult("complete", "", stop_pose.tolist(), final_state.pose.tolist())
            self._save_status(result)
            return result
        except Exception as exc:
            if self.termination is not None:
                self.termination.set_stop_reason(
                    detail=f"{type(exc).__name__}: {exc}", exception=exc,
                    source="safe_return.execute", terminal=self.termination.record is None,
                    policy=self.policy)
            self.controller.safe_stop_motion()
            self._after_abort_stop()
            reason = f"{type(exc).__name__}: {exc}"
            print(f"SAFE RETURN ABORTED: {reason}")
            final_pose = stop_pose
            try:
                final_pose = self.controller.read_state().pose
            except Exception:
                pass
            result = ReturnResult("aborted", reason, stop_pose.tolist(), np.asarray(final_pose).tolist())
            self._save_status(result)
            return result
        finally:
            self.controller.end_return_mode()

    def _run_segment(self, phase: str, target: np.ndarray, speed: float) -> None:
        position_tolerance = self._segment_position_tolerance(phase)
        # Check the stationary stopped condition before enabling each segment.
        # A return must never begin by moving against an already excessive load.
        self._observe(phase=f"{phase}_PRECHECK_SENSOR_READ", return_target=target)
        raw = self.reader.read_wrench()
        self._observe(raw=raw, phase=f"{phase}_PRECHECK_PROCESSING")
        processed = self.preprocessor.process(raw) if self.preprocessor is not None else raw
        self._observe(processed=processed, phase=f"{phase}_PRECHECK_ROBOT_READ")
        state = self.controller.read_state()
        self._observe(robot=state, phase=f"{phase}_PRECHECK_FORCE_CHECK")
        self._observe_command(state, target, 0.0, f"{phase}_PRECHECK")
        self._log_cycle(time.monotonic(), raw, processed, state, target, 0.0, f"{phase}_PRECHECK")
        self._check_force(raw, processed)
        self._observe_command(state, target, speed, phase)
        self.controller.move_linear_async(
            target, speed, float(self.settings["return_acceleration"])
        )
        deadline = time.monotonic() + float(self.settings["return_segment_timeout_sec"])
        self._segment_deadline = deadline
        while True:
            cycle_started = time.monotonic()
            self._observe(timestamp=cycle_started, phase=f"{phase}_SENSOR_READ")
            raw = self.reader.read_wrench()
            self._observe(raw=raw, phase=f"{phase}_PROCESSING")
            processed = self.preprocessor.process(raw) if self.preprocessor is not None else raw
            self._observe(processed=processed, phase=f"{phase}_FORCE_CHECK")
            self._check_force(raw, processed)
            self._observe(phase=f"{phase}_ROBOT_READ")
            state = self.controller.read_state()
            self._observe(robot=state, phase=phase)
            position_error = float(np.linalg.norm(state.pose[:3] - target[:3]))
            orientation_error = _orientation_distance(state.pose[3:], target[3:])
            self._log_cycle(cycle_started, raw, processed, state, target, speed, phase)
            if (
                position_error <= position_tolerance
                and orientation_error <= float(self.settings["return_orientation_tolerance"])
            ):
                # End every segment at rest.  In particular, do not blend the
                # vertical retreat into the horizontal transit.
                settled = self._stop_at_segment_end(phase, target)
                self._observe(robot=settled, phase=f"{phase}_SETTLED_POSE")
                settled_position_error = float(
                    np.linalg.norm(settled.pose[:3] - target[:3])
                )
                settled_orientation_error = _orientation_distance(
                    settled.pose[3:], target[3:]
                )
                if settled_position_error > position_tolerance:
                    raise ReturnAborted(
                        f"{phase} settled position error "
                        f"{settled_position_error:.6f} m"
                    )
                if settled_orientation_error > float(
                    self.settings["return_orientation_tolerance"]
                ):
                    raise ReturnAborted(
                        f"{phase} settled orientation error "
                        f"{settled_orientation_error:.6f} rad"
                    )
                return
            if cycle_started >= deadline:
                raise ReturnAborted(f"{phase} timed out")
            remaining = self.period - (time.monotonic() - cycle_started)
            if remaining > 0.0:
                time.sleep(remaining)

    def _segment_position_tolerance(self, phase):
        return float(self.settings["return_position_tolerance"])

    def _stop_at_segment_end(self, phase, target):
        """Wait for the existing controller stop criterion within this segment's budget.

        A returned stop command and an in-tolerance pose do not establish zero
        measured speed. Keep observing force and motion during residual braking;
        never clear the controller's pending-stop gate to permit the next move.
        ContinuousStartupReturn retains its separate asynchronous override.
        """
        self.controller.safe_stop_motion()
        speed_limit = float(getattr(self.controller, "config", {}).get(
            "continuous_settle_speed_mps", 1e-4
        ))  # Same measured XYZ speed criterion as URRTDEController.read_state.
        if self._segment_deadline is None:
            raise ReturnAborted(f"{phase} has no active segment deadline")
        while True:
            cycle_started = time.monotonic()
            if cycle_started >= self._segment_deadline:
                raise ReturnAborted(f"{phase} standstill confirmation timed out")
            self._observe(phase=f"{phase}_SETTLE_SENSOR_READ", return_target=target)
            raw = self.reader.read_wrench()
            self._observe(raw=raw, phase=f"{phase}_SETTLE_PROCESSING")
            processed = self.preprocessor.process(raw) if self.preprocessor is not None else raw
            self._check_force(raw, processed)
            state = self.controller.read_state()
            self._observe(robot=state, processed=processed, phase=f"{phase}_SETTLE")
            self._log_cycle(cycle_started, raw, processed, state, target, 0., f"{phase}_SETTLE")
            if time.monotonic() >= self._segment_deadline:
                raise ReturnAborted(f"{phase} standstill confirmation timed out")
            if np.linalg.norm(state.tcp_speed[:3]) <= speed_limit:
                return state
            remaining = self.period - (time.monotonic() - cycle_started)
            if remaining > 0.:
                time.sleep(remaining)

    def _after_abort_stop(self):
        """Optional terminal cleanup; successful return keeps its controller."""

    def _observe(self, **context) -> None:
        """Keep return diagnostics in memory without changing device ordering."""
        if self.termination is not None:
            self.termination.observe(policy=self.policy, state="RETURN_TO_START", **context)

    def _observe_command(self, robot, target, speed, phase) -> None:
        if self.termination is None:
            return
        # Match the existing return telemetry command, observing it before the
        # move call so a rejected moveL retains the attempted target and speed.
        delta = target[:2] - robot.pose[:2]
        norm = float(np.linalg.norm(delta))
        command = PolicyCommand(
            state="RETURN_TO_START", move=speed > 0.0,
            direction_xy=np.zeros(2) if norm < 1e-12 else delta / norm,
            speed=speed, target_pose=target.copy(), contact_flag=False,
            possible_corner=False, reason=phase,
        )
        self._observe(command=command, phase=phase)

    def _check_force(self, raw, processed) -> None:
        if np.linalg.norm(processed.force) > float(self.settings["return_force_limit"]):
            raise ReturnAborted("return force limit exceeded")
        if np.linalg.norm(processed.torque) > float(self.settings["return_torque_limit"]):
            raise ReturnAborted("return torque limit exceeded")
        policy = self.config["policy"]
        if np.linalg.norm(raw.force) > float(policy["absolute_raw_force_threshold"]):
            raise ReturnAborted("absolute raw force threshold exceeded during return")
        if np.linalg.norm(raw.torque) > float(policy["absolute_raw_torque_threshold"]):
            raise ReturnAborted("absolute raw torque threshold exceeded during return")

    def _log_cycle(self, now, raw, processed, robot, target, speed, phase) -> None:
        if self.logger is None:
            return
        delta = target[:2] - robot.pose[:2]
        norm = float(np.linalg.norm(delta))
        direction = np.zeros(2) if norm < 1e-12 else delta / norm
        command = PolicyCommand(
            state="RETURN_TO_START",
            move=speed > 0.0,
            direction_xy=direction,
            speed=speed,
            target_pose=target.copy(),
            contact_flag=False,
            possible_corner=False,
            reason=phase,
        )
        normal = None if self.policy is None else self.policy.current_target_direction
        tangent = None if self.policy is None else self.policy.current_tangent
        self.logger.log_sample(now, raw, processed, robot, command, normal, tangent)

    def _aborted(self, reason: str, stop_pose: np.ndarray) -> ReturnResult:
        if self.termination is not None:
            self.termination.set_stop_reason(detail=reason, source="safe_return._aborted",
                                             terminal=self.termination.record is None)
        self.controller.safe_stop_motion()
        self._after_abort_stop()
        print(f"SAFE RETURN ABORTED: {reason}")
        result = ReturnResult("aborted", reason, stop_pose.tolist(), stop_pose.tolist())
        self._save_status(result)
        return result

    def _save_status(self, result: ReturnResult) -> None:
        if self.logger is None:
            return
        payload = {
            "return_status": result.status,
            "return_abort_reason": result.abort_reason,
            "stop_pose": result.stop_pose,
            "p0": self.start_pose.tolist(),
            "return_target_label": self.target_label,
            "return_target": self.start_pose.tolist(),
            "final_pose": result.final_pose,
        }
        write_json = getattr(self.logger, 'write_json', None)
        if write_json is not None:
            write_json('return_status.json', payload)
        else:
            path = Path(self.logger.run_dir) / "return_status.json"
            with path.open("w", encoding="utf-8") as handle:
                json.dump(payload, handle, ensure_ascii=False, indent=2)

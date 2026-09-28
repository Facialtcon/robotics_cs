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
        if not np.isfinite(float(settings[name])) or float(settings[name]) <= 0.0:
            raise RobotError(f"safe_return.{name} must be positive")
    max_speed = float(config["robot"]["max_tcp_speed"])
    if max(float(settings["return_speed"]), float(settings["return_vertical_speed"])) > max_speed:
        raise RobotError("safe return speed exceeds robot.max_tcp_speed")


def return_trajectory(config, current, target):
    """Plan and validate all segments before the first movement."""
    validate_return_configuration(config, target)
    current, target = np.asarray(current, dtype=float), np.asarray(target, dtype=float)
    if current.shape != (6,) or not np.isfinite(current).all():
        raise RobotError('invalid current return pose')
    s = config['safe_return']
    z = calculate_safe_return_z(current[2], target[2], s['return_lift_distance'], s['return_position_tolerance'])
    lifted, above = current.copy(), target.copy()
    lifted[2] = above[2] = z
    segments = [('VERTICAL_RETREAT', lifted, s['return_vertical_speed']),
                ('MOVE_ABOVE_START', above, s['return_speed']),
                ('DESCEND_TO_START', target.copy(), s['return_vertical_speed'])]
    from robot.rtde_controller import check_continuous_xy
    from config.loader import runtime_robot_config
    robot_config = runtime_robot_config(config)
    # Continuous runtime passes its extra XY polygon through the controller as well.
    for pose in [current] + [point for _, point, _ in segments]:
        if config['workspace'].get('enabled', True):
            for i, axis in enumerate('xyz'):
                limits = config['workspace']['limits']
                if not limits[f'{axis}_min'] <= pose[i] <= limits[f'{axis}_max']:
                    raise RobotError(f'return trajectory outside workspace: {axis}')
        check_continuous_xy(robot_config, pose[:2])
    return segments


class PX6DForceMonitor:
    """Optional force input to the one return executor, also used while braking."""
    def __init__(self, config, reader, preprocessor):
        self.config, self.reader, self.preprocessor = config, reader, preprocessor
        self.raw = self.processed = None

    def sample(self):
        self.raw = self.reader.read_wrench()
        self.processed = self.preprocessor.process(self.raw)
        if not np.isfinite(self.raw.array()).all() or not np.isfinite(self.processed.array()).all():
            raise ReturnAborted('nonfinite return wrench')
        s, p = self.config['safe_return'], self.config['policy']
        for value, limit, label in (
            (self.processed.force, s['return_force_limit'], 'return force'),
            (self.processed.torque, s['return_torque_limit'], 'return torque'),
            (self.raw.force, p['absolute_raw_force_threshold'], 'absolute raw force'),
            (self.raw.torque, p['absolute_raw_torque_threshold'], 'absolute raw torque')):
            if np.linalg.norm(value) > float(limit):
                raise ReturnAborted(f'{label} limit exceeded')


class SafeReturnExecutor:
    """Shared UR trajectory and held standstill; no sensor is needed by default."""
    def __init__(self, config, start_pose, controller, *, force_monitor=None,
                 logger=None, poll=None, target_label='P0'):
        validate_return_configuration(config, start_pose)
        self.config, self.settings = config, config['safe_return']
        self.start_pose = np.asarray(start_pose, dtype=float)
        self.controller, self.force_monitor = controller, force_monitor
        self.logger, self.poll, self.target_label = logger, poll or (lambda: None), target_label
        self.period = 1 / float(config['policy']['control_rate_hz'])
        self.previous_cycle = None
        self.phase = 'RETURN_PREFLIGHT'
        self.target = self.start_pose
        self.command_speed = 0.

    def observe(self):
        started = time.monotonic()
        if self.poll() in ('Q', 'ESC'):
            raise KeyboardInterrupt('operator stopped return')
        if self.force_monitor is not None:
            self.force_monitor.sample()
        state = self.controller.read_state()
        if self.logger is not None:
            if self.force_monitor is None:
                self.logger.log_return(state, self.phase, self.target)
            else:
                delta = self.target[:2]-state.pose[:2]
                norm = np.linalg.norm(delta)
                command = PolicyCommand(state='RETURN_TO_START', move=self.command_speed > 0,
                    direction_xy=np.zeros(2) if norm < 1e-12 else delta/norm,
                    speed=self.command_speed, target_pose=self.target.copy(),
                    contact_flag=False, possible_corner=False, reason=self.phase)
                self.logger.log_sample(started, self.force_monitor.raw, self.force_monitor.processed,
                                       state, command, None, None)
            if hasattr(self.logger, 'check_health'):
                self.logger.check_health()
        if self.controller.watchdog_active or self.config.get('continuous_real_execution'):
            c = self.config['continuous_tracking']
            now = time.monotonic()
            if now-started > float(c['cycle_timeout_sec']) or now-state.timestamp > float(c['max_observation_age_sec']):
                self._timing_diagnostic('return cycle/observation timeout')
            if self.previous_cycle is not None and started-self.previous_cycle > float(c['max_sample_gap_sec']):
                self._timing_diagnostic('return sample gap exceeded')
        if self.controller.config.get('continuous_require_watchdog') and self.controller.watchdog_active:
            if self.controller._watchdog_last_kick is not None:
                self.controller._check_watchdog_health()
            self.controller.kick_watchdog()
            if time.monotonic()-started > float(c['cycle_timeout_sec']):
                self._timing_diagnostic('return watchdog call exceeded cycle budget')
        self.previous_cycle = started
        return state

    def _timing_diagnostic(self, message):
        if self.config.get('continuous_real_execution'):
            self.controller.diagnostics['return_timing'] = f'WARNING: {message}'
        else:
            raise RobotError(message)

    def _at_target(self, state):
        tolerance = float(self.settings['return_position_tolerance'])
        if self.force_monitor is not None:
            tolerance = min(tolerance, float(self.config['policy']['position_tolerance']))
        return (np.linalg.norm(state.pose[:3]-self.target[:3]) <= tolerance and
                _orientation_distance(state.pose[3:], self.target[3:]) <= float(self.settings['return_orientation_tolerance']))

    def _settle(self, *, diagnostic=False):
        """Drive the shared stop monitor, one observation/guard/log cycle at a time."""
        self.command_speed = 0.
        requested = False
        while True:
            started = time.monotonic()
            if not requested:
                self.controller.request_stop(nonblocking=True)
                requested = True
            state = self.controller.read_diagnostic_state() if diagnostic else self.observe()
            stopped = self.controller.poll_stop()
            if time.monotonic()-started > float(self.config['continuous_tracking']['cycle_timeout_sec']):
                self._timing_diagnostic('return stopping cycle timeout')
            if stopped:
                return state
            time.sleep(max(0., self.period-(time.monotonic()-started)))

    def execute(self):
        self.controller.begin_return_mode()
        initial = final = self.start_pose.copy()
        try:
            self._settle()
            initial = self.observe().pose.copy()
            segments = return_trajectory(self.config, initial, self.start_pose)
            # Validate every endpoint against execution-specific guards before moving.
            from robot.rtde_controller import check_continuous_xy
            for _, target, _ in segments:
                self.controller.guard.check_workspace(target)
                check_continuous_xy(self.controller.config, target[:2])
            for self.phase, self.target, speed in segments:
                self.command_speed = 0.
                state = self.observe()
                if self._at_target(state):
                    continue
                self.command_speed = float(speed)
                self.controller.move_linear_async(self.target, speed, self.settings['return_acceleration'])
                deadline = time.monotonic() + float(self.settings['return_segment_timeout_sec'])
                while True:
                    started = time.monotonic()
                    state = self.observe()
                    if started >= deadline:
                        raise ReturnAborted(f'{self.phase} timed out')
                    if self._at_target(state):
                        break
                    time.sleep(max(0., self.period-(time.monotonic()-started)))
                self.phase += '_STOPPING'
                state = self._settle()
                if not self._at_target(state):
                    raise ReturnAborted(f'{self.phase} settled pose outside tolerance')
            final = self._settle().pose
            result = ReturnResult('complete', '', initial.tolist(), final.tolist())
        except BaseException as exc:
            self.controller.request_stop(nonblocking=True)
            stop_error = ''
            try:
                # Faulted sensor/logger must not prevent fresh physical stop verification.
                final = self._settle(diagnostic=True).pose
            except Exception as stopping:
                stop_error = f'; {stopping}'
            result = ReturnResult('aborted', f'{type(exc).__name__}: {exc}{stop_error}',
                                  initial.tolist(), final.tolist())
        finally:
            self.controller.end_return_mode()
        if self.logger is not None:
            payload = dict(return_status=result.status, return_abort_reason=result.abort_reason,
                stop_pose=result.stop_pose, final_pose=result.final_pose, return_target=self.start_pose.tolist(),
                return_target_label=self.target_label, force_monitor=self.force_monitor is not None,
                stop_requests=self.controller.stop_history)
            if hasattr(self.logger, 'write_json'):
                self.logger.write_json('return_status.json', payload)
            else:
                (Path(self.logger.run_dir)/'return_status.json').write_text(json.dumps(payload, indent=2), encoding='utf-8')
        return result

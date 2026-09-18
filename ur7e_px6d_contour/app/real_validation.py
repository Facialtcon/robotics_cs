"""Bounded real-UR air validation of the shared policy; no physical contact test.

The controller and PX6D are real. Only the wrench presented to the policy is a
deterministic fixture. Real sensor protection runs independently on every read.
"""
from __future__ import annotations

import argparse
import copy
import csv
from dataclasses import dataclass
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

from app.operator_input import OperatorKeyboard
from calibration.scan_calibration import (
    load_scan_calibration, resolve_calibration_path, validate_calibration_constraints,
)
from config.loader import load_config, runtime_robot_config
from core.models import Wrench
from policy.rule_policy import RuleBasedPolicy, State
from robot.rtde_controller import RobotError, URRTDEController, _orientation_distance, validate_execution_configuration
from robot.tcp_identity import tcp_offsets_match
from safety.force_guard import ForceRateGuard, force_safety_reason
from sensor.force_preprocess import WrenchPreprocessor
from sensor.px6d_reader import PX6DReader

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ZERO = Wrench(0, 0, 0, 0, 0, 0)


@dataclass(frozen=True)
class AirValidationOptions:
    speed_cap: float = 0.001
    half_extent: float = 0.030
    timeout_sec: float = 300.0
    settle_timeout_sec: float = 3.0
    settled_speed: float = 0.0001
    settle_samples: int = 3

    def __post_init__(self):
        for name in ("speed_cap", "half_extent", "timeout_sec", "settle_timeout_sec", "settled_speed"):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"air {name} must be finite and positive")
        if self.speed_cap > 0.001 or self.half_extent > 0.030 or self.timeout_sec > 300:
            raise ValueError("air limits: speed <= 1 mm/s, XY half-extent <= 30 mm, duration <= 300 s")
        if isinstance(self.settle_samples, bool) or not isinstance(self.settle_samples, int) or self.settle_samples < 2:
            raise ValueError("air settle_samples must be an integer >= 2")


def load_validation_config(config_path):
    """Use the formal real scan's calibrated direction, without going to P0."""
    config = load_config(config_path)
    validate_execution_configuration(config)
    calibration = load_scan_calibration(
        resolve_calibration_path(config_path, config["calibration"]["file"]), require_tcp_offset=True,
    )
    if calibration["robot_ip"] != config["robot"]["robot_ip"]:
        raise RobotError("scan calibration robot_ip does not match configuration")
    if not tcp_offsets_match(calibration["active_tcp_offset"], config["tcp"]["offset"],
                             float(config["tcp"]["offset_tolerance"])):
        raise RobotError("scan calibration TCP does not match configuration")
    validate_calibration_constraints(
        calibration, float(config["calibration"]["min_direction_calibration_distance"]),
        float(config["calibration"]["max_direction_calibration_z_difference"]),
    )
    policy_config = copy.deepcopy(config["policy"])
    policy_config["search_direction_xy"] = list(calibration["scan_direction_xy"])
    return config, policy_config, calibration


class ScriptedAirContact:
    """Input fixture only: inclined plane, two tracking contacts, then absence.

    No robot positions, policy states, targets or episode outcomes are assigned
    by this fixture. Geometry is used solely to generate labelled test wrench.
    """
    def __init__(self, policy_config, origin_pose):
        direction = np.asarray(policy_config["search_direction_xy"], dtype=float)
        direction = direction / np.linalg.norm(direction)
        angle = np.deg2rad(30)
        self.normal = np.asarray([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]) @ direction
        self.plane_point = np.asarray(origin_pose, dtype=float)[:2] + direction * 0.009
        self.threshold = float(policy_config["contact_threshold"])
        self.tracking_probe_ids = []

    def wrench(self, policy, pose):
        episode = policy.active_episode
        if episode is None or episode.phase not in {"PROBE", "HOLD"}:
            return ZERO
        if episode.purpose == "TRACKING" and episode.probe_id not in self.tracking_probe_ids:
            self.tracking_probe_ids.append(episode.probe_id)
        permitted = episode.purpose in {"ACQUISITION", "INITIALIZATION"} or (
            episode.purpose == "TRACKING" and episode.probe_id in self.tracking_probe_ids[:2]
        )
        if not permitted:
            return ZERO
        penetration = float(np.dot(np.asarray(pose)[:2] - self.plane_point, self.normal))
        force = min(1.2 * self.threshold, max(0.0, penetration * 1000.0))
        return Wrench(force * self.normal[0], force * self.normal[1], 0, 0, 0, 0)

    def complete(self, policy):
        return sum(e.purpose == "RECOVERY" and e.return_completed for e in policy.probe_episodes) >= 2


class AirValidationRunner:
    """Injectable hardware runner; never calls moveL or a calibrated return."""
    def __init__(self, config, policy_config, controller, reader, *, options=None,
                 clock=time.monotonic, sleep=time.sleep, output_dir=None, print_fn=print):
        self.config, self.policy_config = copy.deepcopy(config), copy.deepcopy(policy_config)
        self.controller, self.reader = controller, reader
        self.options = options or AirValidationOptions()
        self.clock, self.sleep, self.print = clock, sleep, print_fn
        self.output_dir = Path(output_dir or PROJECT_ROOT / "data" / ("air_validation_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")))
        self.policy = RuleBasedPolicy(self.policy_config)
        self.termination = self.policy.termination
        self.preprocessor = WrenchPreprocessor.from_config(config["preprocessing"])
        self.force_rate_guard = ForceRateGuard()
        self.period = 1.0 / float(self.policy_config["control_rate_hz"])
        self.origin = None
        self.last_sample = None
        self._bias_ready = False
        self._baseline_first = None
        self._previous_pose = None
        self._previous_command = None
        self._executed_probes = set()
        self._moving_events = set()
        self._initialization_alignment_verified = False
        self._settled_stops = 0
        self._last_print_time = -float("inf")
        self._last_phase = None
        self._jsonl = self._csv_file = self._csv = None

    def _json(self, name, value):
        (self.output_dir / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def _sample(self, *, baseline=False):
        started = self.clock()
        self.termination.observe(policy=self.policy, operation="PX6D_READ", timestamp=started)
        raw = self.reader.read_wrench()
        self.termination.observe(raw=raw, actual_raw_wrench=raw.array(), operation="RTDE_READ")
        robot = self.controller.read_state()
        self.termination.observe(robot=robot, operation="WRENCH_PREPROCESS")
        processed = ZERO if baseline else self.preprocessor.process(raw)
        self.last_sample = (raw, processed, robot, self.clock() - started)
        self.termination.observe(raw=raw, processed=processed, robot=robot,
                                 actual_raw_wrench=raw.array(), actual_processed_wrench=processed.array(),
                                 force_input_mode="synthetic_policy_fixture", operation="AIR_SAMPLE_GUARDS")
        if not np.all(np.isfinite(np.r_[raw.array(), processed.array(), robot.pose, robot.tcp_speed])):
            raise RobotError("air validation: nonfinite real sensor/robot sample")
        reason = force_safety_reason(raw, processed, self.policy_config)
        if reason:
            raise RobotError("real PX6D: " + reason)
        if self.origin is not None:
            delta = robot.pose - self.origin
            if np.any(np.abs(delta[:2]) > self.options.half_extent):
                raise RobotError("air validation: actual TCP outside local XY envelope")
            if abs(delta[2]) > float(self.config["robot"]["fixed_z_tolerance"]):
                raise RobotError("air validation: actual TCP departed latched air Z")
            if _orientation_distance(robot.pose[3:], self.origin[3:]) > float(self.config["robot"]["orientation_tolerance_rad"]):
                raise RobotError("air validation: actual TCP departed latched air orientation")
        if np.linalg.norm(robot.tcp_speed[:3]) > self.options.speed_cap * 1.2:
            raise RobotError("air validation: measured TCP speed exceeds slow cap plus tolerance")
        reference = self._baseline_first if baseline else self.preprocessor.zero_bias_sensor
        if reference is not None and np.linalg.norm(raw.force - reference[:3]) >= float(self.policy_config["contact_threshold"]):
            raise RobotError("air validation: real unfiltered force increment reached contact threshold")
        if not baseline:
            rate = self.force_rate_guard.update(self.clock(), float(np.linalg.norm(processed.force[:2])))
            if rate > float(self.policy_config["force_rate_limit"]):
                raise RobotError("air validation: real PX6D force rate exceeded")
        return raw, processed, robot

    def _record(self, phase, command=None, synthetic=ZERO, *, executed_speed=0.0, cycle_duration=0.0):
        if self.last_sample is None:
            return
        raw, processed, robot, read_duration = self.last_sample
        episode = self.policy.active_episode
        previous = self._previous_command
        anchor = None if episode is None else episode.anchor_pose.tolist()
        movement = np.zeros(6) if self._previous_pose is None else robot.pose - self._previous_pose
        row = {
            "timestamp": self.clock(), "sample_timestamp": robot.timestamp, "phase": phase,
            "state": self.policy.state.value, "policy_sub_state": self.policy.sub_state,
            "probe_id": None if episode is None else episode.probe_id,
            "probe_purpose": None if episode is None else episode.purpose,
            "current_tcp_pose": robot.pose.tolist(), "actual_tcp_speed": robot.tcp_speed.tolist(),
            "command_target_pose": robot.pose.tolist() if command is None else command.target_pose.tolist(),
            "episode_anchor_pose": anchor,
            "target_anchor": anchor if anchor is not None else (robot.pose.tolist() if command is None else command.target_pose.tolist()),
            "command_direction": [0.0, 0.0] if command is None else command.direction_xy.tolist(),
            "requested_speed": 0.0 if command is None else command.speed,
            "executed_speed": executed_speed, "actual_movement": movement.tolist(),
            "movement_corresponds_to": "previous_command_interval",
            "previous_command_direction": [0.0, 0.0] if previous is None else previous["direction"],
            "previous_command_phase": None if previous is None else previous["phase"],
            "previous_executed_speed": 0.0 if previous is None else previous["speed"],
            "actual_raw_wrench": raw.array().tolist(), "actual_processed_wrench": processed.array().tolist(),
            "bias_ready": self._bias_ready, "synthetic_policy_wrench": synthetic.array().tolist(),
            "return_position_error": None if episode is None else float(np.linalg.norm(robot.pose[:2] - episode.anchor_pose[:2])),
            "read_duration_sec": read_duration, "cycle_duration_sec": cycle_duration,
        }
        self._jsonl.write(json.dumps(row, allow_nan=False) + "\n")
        if self._csv is None:
            self._csv = csv.DictWriter(self._csv_file, fieldnames=list(row))
            self._csv.writeheader()
        self._csv.writerow({key: json.dumps(value) if isinstance(value, (list, dict)) else value for key, value in row.items()})
        self._jsonl.flush()
        self._csv_file.flush()
        self._previous_pose = robot.pose.copy()
        self._previous_command = {"direction": row["command_direction"], "phase": phase, "speed": executed_speed}
        if phase != self._last_phase or self.clock() - self._last_print_time >= 0.25:
            self.print(f"AIR {phase}: TCP={np.round(robot.pose, 6).tolist()} target_anchor={row['target_anchor']} "
                       f"target={np.round(row['command_target_pose'], 6).tolist()} "
                       f"direction={row['command_direction']} actual_movement_mm={np.round(movement[:3]*1000, 4).tolist()} "
                       f"speed={executed_speed*1000:.3f} mm/s")
            self._last_print_time, self._last_phase = self.clock(), phase

    def _pace(self, started):
        remaining = self.period - (self.clock() - started)
        if remaining > 0:
            self.sleep(remaining)

    def _capture_bias(self):
        samples = []
        self.termination.observe(phase="BIAS", source="air._capture_bias")
        count = int(self.config["preprocessing"]["baseline"]["sample_count"])
        if count <= 0:
            raise ValueError("air software bias sample count must be positive")
        for _ in range(count):
            started = self.clock()
            raw, _, robot = self._sample(baseline=True)
            if self._baseline_first is None:
                self._baseline_first = raw.array()
            if (np.linalg.norm(robot.tcp_speed[:3]) > self.options.settled_speed
                    or np.linalg.norm(robot.pose[:3] - self.origin[:3]) > float(self.policy_config["position_tolerance"])
                    or _orientation_distance(robot.pose[3:], self.origin[3:]) > 0.001):
                raise RobotError("air validation: TCP must stay stationary during software bias capture")
            samples.append(raw)
            self._record("BIAS", cycle_duration=self.clock() - started)
            self._pace(started)
        self.preprocessor.set_zero_bias(samples)
        self._bias_ready = True

    def _stop_and_settle(self, poll_key):
        self.termination.observe(phase="SETTLING", operation="STOP_COMMAND")
        self.controller.stop()
        deadline, consecutive = self.clock() + self.options.settle_timeout_sec, 0
        retried_stop = False
        while self.clock() < deadline:
            started = self.clock()
            key = poll_key()
            if key in {"Q", "ESC"}:
                self.termination.observe(operator_key=key)
                raise KeyboardInterrupt("operator stop while settling")
            _, _, robot = self._sample()
            self._record("SETTLING", cycle_duration=self.clock() - started)
            consecutive = consecutive + 1 if np.linalg.norm(robot.tcp_speed[:3]) <= self.options.settled_speed else 0
            if consecutive == 0 and not retried_stop and hasattr(self.controller, "safe_stop_motion"):
                # Reuse the existing forced stop once if its cached stop flag
                # disagrees with measured motion; never change controller code.
                self.controller.safe_stop_motion(force=True)
                retried_stop = True
            if consecutive >= self.options.settle_samples:
                self._settled_stops += 1
                return
            self._pace(started)
        raise RobotError("air validation: measured TCP speed did not settle before timeout")

    def _execute(self, command, robot):
        self.termination.observe(policy=self.policy, command=command, robot=robot,
                                 phase=command.policy_sub_state, operation="AIR_MOTION_GUARDS")
        speed = min(command.speed, self.options.speed_cap)
        predicted = robot.pose[:2] + command.direction_xy * speed * self.period
        # Preserve formal policy distances, including its long SEARCH endpoint;
        # only bounded increments can execute. A missing fixture aborts locally.
        margin = speed * speed / (2 * float(self.config["robot"]["stop_deceleration"]))
        if np.any(np.abs(predicted - self.origin[:2]) + margin > self.options.half_extent):
            raise RobotError("air validation: next increment reaches local XY envelope")
        if self.policy.active_episode is None and np.any(np.abs(command.target_pose[:2] - self.origin[:2]) > self.options.half_extent):
            raise RobotError("air validation: transfer target outside local XY envelope")
        self.termination.observe(operation="RTDE_SPEED_COMMAND", executed_speed=speed)
        self.controller.command_planar_velocity(command.direction_xy, speed, self.period)
        if self.policy.active_episode is not None:
            self._executed_probes.add(self.policy.active_episode.probe_id)
        self._moving_events.add(command.policy_sub_state)
        return speed

    def run(self, *, confirm=lambda: False, poll_key=lambda: None):
        """Keep diagnostics for setup/final-output failures without changing their propagation."""
        try:
            return self._run(confirm=confirm, poll_key=poll_key)
        except BaseException as exc:
            self.termination.set_stop_reason(detail=f"{type(exc).__name__}: {exc}",
                                             exception=exc, source="air.run")
            self.termination.flush(emit=True)
            exc.termination_recorder = self.termination
            raise

    def _run(self, *, confirm, poll_key):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        if any((self.output_dir / name).exists() for name in ("summary.json", "trajectory.jsonl", "config_snapshot.json")):
            raise FileExistsError("air validation output directory already contains run artifacts")
        self.termination.bind(self.output_dir)
        source_dir = Path(sys.modules[RuleBasedPolicy.__module__].__file__).resolve().parent
        self._json("config_snapshot.json", {"config": self.config, "effective_policy": self.policy_config,
                    "air_options": vars(self.options), "policy_parameters_overridden_by_air_mode": []})
        self._json("policy_source_snapshot.json", {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in sorted(source_dir.glob("*.py"))})
        status, reason, stop_error = "not_armed", "START_AIR not confirmed", ""
        connected = controller_attempted = False
        self._jsonl = (self.output_dir / "trajectory.jsonl").open("w", encoding="utf-8")
        self._csv_file = (self.output_dir / "trajectory.csv").open("w", newline="", encoding="utf-8")
        try:
            self.termination.observe(policy=self.policy, phase="CONNECT", operation="PX6D_CONNECT")
            self.reader.connect()
            controller_attempted = True
            self.termination.observe(operation="RTDE_CONNECT")
            self.controller.connect()
            connected = True
            self.termination.observe(operation="INITIAL_TCP_READ")
            initial = self.controller.read_state()
            self.origin = initial.pose.copy()
            self.termination.observe(robot=initial)
            self._capture_bias()
            self.print("AIR ONLY: verify the complete robot and probe sweep is clear of box, grains and target. "
                       "Current air Z/orientation are held; there is no P0 move or automatic return.")
            self.print(f"Air origin={self.origin.tolist()}, XY half-extent={self.options.half_extent*1000:.1f} mm, "
                       f"speed <= {self.options.speed_cap*1000:.3f} mm/s; real PX6D guards remain active.")
            if confirm():
                fixture = ScriptedAirContact(self.policy_config, self.origin)
                self._json("fixture.json", {"kind": "synthetic_policy_input_only", "normal_xy": fixture.normal.tolist(),
                            "plane_point_xy": fixture.plane_point.tolist(), "origin_pose": self.origin.tolist()})
                self._stop_and_settle(poll_key)
                deadline = self.clock() + self.options.timeout_sec
                while True:
                    started = self.clock()
                    key = poll_key()
                    if key in {"Q", "ESC"}:
                        status, reason = "stopped", "operator stop in place"
                        self.termination.set_stop_reason("STOP_USER_REQUEST", detail=reason,
                                                         source="air.operator", operator_key=key)
                        break
                    if started >= deadline:
                        raise RobotError("air validation: overall timeout")
                    raw, actual_processed, robot = self._sample()
                    synthetic = fixture.wrench(self.policy, robot.pose)
                    previous_state = self.policy.state
                    self.termination.observe(policy=self.policy, raw=raw, processed=actual_processed,
                                             synthetic_policy_wrench=synthetic.array(), operation="POLICY_UPDATE")
                    command = self.policy.update(self.clock(), raw, synthetic, robot)
                    self.termination.observe(policy=self.policy, robot=robot, raw=raw, processed=actual_processed,
                                             command=command, phase=command.policy_sub_state,
                                             synthetic_policy_wrench=synthetic.array())
                    if previous_state == State.LOCAL_INITIALIZATION and self.policy.state == State.BOUNDARY_TRACKING:
                        initial_contacts = [e for e in self.policy.probe_episodes
                                            if e.purpose == "INITIALIZATION" and e.accepted_as_boundary][-3:]
                        if initial_contacts:
                            frontier = max(initial_contacts, key=lambda e: np.dot(e.contact_pose[:2], self.policy.current_tangent))
                            self._initialization_alignment_verified = bool(
                                np.linalg.norm(robot.pose[:2] - frontier.anchor_pose[:2])
                                <= float(self.policy_config["position_tolerance"])
                            )
                    if fixture.complete(self.policy):
                        self.termination.set_stop_reason("SUCCESS", detail="two tracking probes and two returned recovery rays exercised",
                                                         source="air.fixture", scope="air_fixture")
                        self.controller.stop()
                        self._record("FIXTURE_COMPLETE", command, synthetic, cycle_duration=self.clock() - started)
                        status, reason = "complete", "two tracking probes and two returned recovery rays exercised"
                        break
                    if self.policy.state in {State.STOP, State.STOP_SCAN, State.LOOP_COMPLETE}:
                        self.termination.set_stop_reason(detail=self.policy.reason,
                                                         source="air.policy_terminal")
                        self.controller.stop()
                        self._record("POLICY_STOP", command, synthetic, cycle_duration=self.clock() - started)
                        raise RobotError("shared policy stopped before fixture coverage: " + self.policy.reason)
                    speed = self._execute(command, robot) if command.move else 0.0
                    if not command.move:
                        # Stop before any console or disk I/O on first contact.
                        self.controller.stop()
                    phase = f"{command.state}/{command.policy_sub_state}"
                    self._record(phase, command, synthetic, executed_speed=speed, cycle_duration=self.clock() - started)
                    if not command.move:
                        self._stop_and_settle(poll_key)
                    self._pace(started)
                self._stop_and_settle(lambda: None)
            else:
                self.termination.set_stop_reason("STOP_USER_REQUEST", detail=reason,
                                                 source="air.confirmation", operator_key="START_AIR_NOT_CONFIRMED")
        except KeyboardInterrupt as exc:
            status, reason = "stopped", "operator interrupt: stop in place, no automatic return"
            self.termination.set_stop_reason("STOP_USER_REQUEST", detail=reason,
                                             exception=exc, source="air.run")
        except Exception as exc:
            status, reason = "failed", f"{type(exc).__name__}: {exc}"
            self.termination.set_stop_reason(detail=reason, exception=exc, source="air.run",
                                             replace=bool(self.termination.record and self.termination.record["reason"] == "SUCCESS"))
        finally:
            pending_exception = sys.exc_info()[1]
            if pending_exception is not None and self.termination.record is None:
                self.termination.set_stop_reason(
                    detail=f"{type(pending_exception).__name__}: {pending_exception}",
                    exception=pending_exception, source="air.pending_exit", policy=self.policy)
            # A queued but unsent next probe remains an honest ABORTED record.
            self.policy.request_stop(reason)
            if controller_attempted:
                try:
                    self._stop_and_settle(lambda: None) if connected and self._bias_ready else self.controller.stop()
                except Exception as exc:
                    stop_error = f"{type(exc).__name__}: {exc}"
                    self.termination.set_stop_reason(detail=stop_error, exception=exc,
                                                     source="air.cleanup_stop",
                                                     replace=bool(self.termination.record and self.termination.record["reason"] == "SUCCESS"))
                    status, reason = "failed", reason + "; final measured stop was not verified: " + stop_error
            for resource in (self.controller, self.reader):
                try:
                    resource.close()
                except Exception as exc:
                    self.termination.set_stop_reason(detail=f"{type(exc).__name__}: {exc}",
                                                     exception=exc, source="air.cleanup_close")
                    stop_error += f"; close: {exc}"
            self.termination.flush(emit=True)
            self._jsonl.close()
            self._csv_file.close()
        episodes = self.policy.probe_episodes
        recovery = [e for e in episodes if e.purpose == "RECOVERY" and e.return_completed]
        tracking = [e for e in episodes if e.purpose == "TRACKING" and e.return_completed and e.outcome == "CONTACT"]
        coverage = {
            "initialization": any(e.purpose == "INITIALIZATION" and e.accepted_as_boundary for e in episodes),
            "initialization_alignment": self._initialization_alignment_verified,
            "anchor_return": any(e.return_completed for e in episodes),
            "tangent_step": "TANGENT_STEP" in self._moving_events,
            "tracking": len(tracking) >= 2, "recovery_path": len(recovery) >= 2,
        }
        if status == "complete" and not all(coverage.values()):
            status, reason = "failed", "required air coverage missing: " + ", ".join(k for k, v in coverage.items() if not v)
            self.termination.set_stop_reason(detail=reason, source="air.coverage",
                                             replace=bool(self.termination.record and self.termination.record["reason"] == "SUCCESS"))
            self.termination.flush(emit=True)
        summary = {"status": status, "reason": reason, "stop_error": stop_error,
                   "hardware_run": connected and isinstance(self.controller, URRTDEController) and isinstance(self.reader, PX6DReader),
                   "executor_type": type(self.controller).__module__ + "." + type(self.controller).__name__,
                   "physical_contact_validated": False, "coverage": coverage,
                   "completed_tracking_contacts": len(tracking), "completed_recovery_rays": len(recovery),
                   "verified_stationary_stops": self._settled_stops, "boundary_points": len(self.policy.boundary_points),
                   "executed_probe_ids": sorted(self._executed_probes),
                   "queued_unexecuted_probe_ids": [e.probe_id for e in episodes if e.probe_id not in self._executed_probes],
                   "origin_pose": None if self.origin is None else self.origin.tolist(),
                   "final_tcp_pose": None if self.last_sample is None else self.last_sample[2].pose.tolist(),
                   "policy_source": str(source_dir / "rule_policy.py"), "output_dir": str(self.output_dir),
                   **self.termination.summary_fields()}
        self._json("probe_episodes.json", [dict(e.to_record(), motion_executed=e.probe_id in self._executed_probes) for e in episodes])
        self._json("summary.json", summary)
        self.print(f"Air validation {status}: {reason}; logs={self.output_dir}")
        return summary


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config.yaml"))
    parser.add_argument("--execute", action="store_true", help="connect real UR7e/PX6D; START_AIR still required")
    parser.add_argument("--speed-mm-s", type=float, default=1.0, help="air execution cap, >0 and <=1 mm/s")
    parser.add_argument("--output-root", default=str(PROJECT_ROOT / "data"))
    return parser.parse_args(argv)


def main(args=None):
    args = parse_args() if args is None else args
    config, policy_config, calibration = load_validation_config(args.config)
    options = AirValidationOptions(speed_cap=float(args.speed_mm_s) / 1000.0)
    print(f"Shared policy: {Path(sys.modules[RuleBasedPolicy.__module__].__file__).resolve()}")
    print(f"Calibrated search direction: {policy_config['search_direction_xy']} ({calibration['source_file']})")
    print("Preview: current manually placed air TCP -> software bias -> search -> initialization -> "
          "two tracking contacts -> forced NO_CONTACT -> two recovery rays -> measured stop in place.")
    print(f"Policy/config unchanged; runtime speed <= {options.speed_cap*1000:g} mm/s, "
          "local XY +/-30 mm, <=300 s. Test wrench is synthetic; real PX6D is an independent stop input.")
    if not args.execute:
        print("PREVIEW ONLY: no UR/PX6D connection, no robot commands. Add --execute for the armed air test.")
        return 0
    if not sys.stdin.isatty():
        raise RobotError("real air execution requires an interactive terminal and START_AIR confirmation")
    robot_config = runtime_robot_config(config)
    robot_config["fixed_z"] = None
    robot_config["fixed_orientation"] = None
    sensor = config["sensor"]
    runner = AirValidationRunner(
        config, policy_config, URRTDEController(robot_config),
        PX6DReader(sensor["serial_port"], sensor["baudrate"], sensor["timeout_sec"], sensor["poll_rate_hz"], sensor["startup_delay_sec"]),
        options=options, output_dir=Path(args.output_root) / ("air_validation_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")),
    )
    with OperatorKeyboard() as keyboard:
        # Temporarily restore canonical input for the explicit arming prompt.
        def confirm():
            keyboard.__exit__()
            try:
                return input("确认探针和机械臂处于箱外无遮挡空气区域，输入 START_AIR 开始：").strip() == "START_AIR"
            finally:
                keyboard.__enter__()
        summary = runner.run(confirm=confirm, poll_key=keyboard.poll)
    return 0 if summary["status"] in {"complete", "not_armed"} else 1

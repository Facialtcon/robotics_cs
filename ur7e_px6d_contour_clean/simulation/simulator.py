"""Orchestration for the hardware-free 2D closed-loop simulator."""

from __future__ import annotations

import csv
import json
import shutil
from copy import deepcopy
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

from calibration.scan_calibration import calculate_scan_direction_xy
from experiment_logging.paths import create_run, read_metadata
from core.models import PolicyCommand, Wrench
from policy.rule_policy import RuleBasedPolicy, State
from sensor.force_features import extract_force_features
from sensor.force_preprocess import WrenchPreprocessor
from simulation.geometry import RectangleTarget, TargetGeometry, create_target
from simulation.loop_completion import generic_loop_report, square_loop_report
from simulation.simulated_force_sensor import SimulatedForceSensor
from simulation.simulated_robot import SimulatedRobot
from simulation.physical_validation import segment_enters_interior, probe_audit


def _deep_merge(base: dict, override: dict) -> dict:
    result = deepcopy(base)
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def load_simulation_config(path: str | Path) -> dict:
    source = Path(path).expanduser().resolve()
    with source.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    if not isinstance(config, dict):
        raise ValueError("simulation configuration must be a mapping")
    base_name = config.pop("base_config", None)
    if base_name:
        with (source.parent / base_name).resolve().open("r", encoding="utf-8") as handle:
            base = yaml.safe_load(handle)
        config = _deep_merge(base, config)
    point_0 = np.asarray(config["calibration_point_0"], dtype=float)
    point_1 = np.asarray(config["calibration_point_1"], dtype=float)
    direction = calculate_scan_direction_xy(point_0, point_1)
    config["scan_direction_xy"] = direction.tolist()
    config["policy"]["search_direction_xy"] = direction.tolist()
    dt = float(config["simulation"]["dt"])
    expected_rate = 1.0 / dt
    if not np.isclose(float(config["policy"]["control_rate_hz"]), expected_rate):
        raise ValueError("policy.control_rate_hz must equal 1 / simulation.dt")
    config["_config_source"] = str(source)
    return config


class ContourSimulator:
    """Connect synthetic sensor -> shared preprocessing/policy -> point robot."""

    def __init__(self, config: dict):
        self.config = deepcopy(config)
        self._policy_loop_closure_default = bool(
            self.config["simulation"].get("interactive_loop_closure_enabled", True)
        )
        self.target: TargetGeometry = create_target(self.config)
        self.square_full_loop = bool(self.config["simulation"].get("square_full_loop", False))
        if self.square_full_loop and not (
            isinstance(self.target, RectangleTarget)
            and np.isclose(self.target.width, self.target.height)
            and np.isclose(self.target.rotation_deg, 0.0)
        ):
            raise ValueError("square_full_loop requires an axis-aligned square")
        if self.square_full_loop:
            self.config["policy"]["loop_closure_enabled"] = False
        self.dt = float(self.config["simulation"]["dt"])
        self.output_dir: Path | None = None
        self.finalized = False
        self.reset()

    @property
    def target_shape(self) -> str:
        shape = str(self.config["target_shape"]).lower()
        return "square" if shape in {"rectangle", "rotated_rectangle"} else shape

    def set_interactive_target(self, shape: str, center=None, *, translate_search: bool = False) -> None:
        """Reconfigure the environment and restart; policy receives no geometry."""
        shape = shape.lower()
        if shape not in {"square", "circle", "triangle"}:
            raise ValueError("interactive shape must be square, circle, or triangle")
        old_center = np.asarray(self.config["target_center"], dtype=float)
        new_center = old_center.copy() if center is None else np.asarray(center, dtype=float)
        if new_center.shape != (2,) or not np.all(np.isfinite(new_center)):
            raise ValueError("target center must be finite XY")
        delta = new_center - old_center
        if translate_search:
            for key in ("start_point", "calibration_point_0", "calibration_point_1"):
                self.config[key] = (np.asarray(self.config[key], dtype=float) + delta).tolist()
        self.config["target_center"] = new_center.tolist()
        self.config["target_shape"] = "rectangle" if shape == "square" else shape
        if shape == "square":
            self.config.update(target_width=0.10, target_height=0.10, target_rotation_deg=0.0)
        elif shape == "triangle":
            self.config["target_rotation_deg"] = 90.0
        self.config["scene_name"] = f"interactive_{shape}"
        self.square_full_loop = shape == "square"
        self.config["simulation"]["square_full_loop"] = self.square_full_loop
        self.config["policy"]["loop_closure_enabled"] = (
            False if self.square_full_loop else self._policy_loop_closure_default
        )
        self.target = create_target(self.config)
        self.reset()

    def reset(self) -> None:
        self.robot = SimulatedRobot(
            self.config["start_point"], self.dt, self.config["container"]
        )
        self.sensor = SimulatedForceSensor(self.target, self.config["force_model"])
        self.preprocessor = WrenchPreprocessor.from_config(self.config["preprocessing"])
        bias_samples = [
            self.sensor.read_wrench(self.robot.pose[:2], np.zeros(2))[0] for _ in range(100)
        ]
        self.preprocessor.set_zero_bias(bias_samples)
        self.policy = RuleBasedPolicy(deepcopy(self.config["policy"]))
        self.termination = self.policy.termination
        self.trajectory: list[np.ndarray] = [self.robot.pose[:2].copy()]
        self.records: list[dict] = []
        self.history: list[dict] = []
        self.states_seen: list[str] = []
        self.logged_boundary_count = 0
        self.last_raw = Wrench(0, 0, 0, 0, 0, 0)
        self.last_processed = Wrench(0, 0, 0, 0, 0, 0)
        self.last_command = PolicyCommand(
            State.SEARCH.value,
            False,
            np.zeros(2),
            0.0,
            self.robot.pose.copy(),
            False,
            False,
            "not started",
        )
        self.last_force_metadata = {}
        self.stopped = False
        self.stop_reason = ""
        self.loop_completed = False
        self.loop_report = {}
        self.physical_failure = ""
        self.interior_entered = False
        self.post_contact_push = False
        self.failure_probe_id = None
        self.final_policy_state = ""
        self.result_image: Path | None = None
        self.finalized = False
        self.output_dir = None
        self.termination.observe(policy=self.policy, robot=self.robot.read_state(),
                                 raw=self.last_raw, processed=self.last_processed,
                                 command=self.last_command, source="simulation.reset")

    def step(self) -> dict:
        self._ensure_run()
        try:
            return self._step()
        except BaseException as exc:
            self.termination.set_stop_reason(detail=f"{type(exc).__name__}: {exc}",
                                             exception=exc, source="simulation.step")
            self.termination.flush(emit=True)
            raise

    def _ensure_run(self):
        if self.output_dir is None:
            self.output_dir = create_run('simulation', 'discrete', self.config.get('_config_source'),
                                         data_root=self.config.get('_data_root'))
            self.termination.bind(self.output_dir)
            (self.output_dir/'simulation_config_snapshot.yaml').write_text(
                yaml.safe_dump(self.config, allow_unicode=True, sort_keys=False), encoding='utf-8')

    def _step(self) -> dict:
        if self.stopped:
            return self.history[-1] if self.history else self.snapshot()
        if not np.allclose(self.trajectory[-1], self.robot.pose[:2], atol=1e-15):
            self.trajectory.append(self.robot.pose[:2].copy())
        robot_state = self.robot.read_state()
        raw, metadata = self.sensor.read_wrench(robot_state.pose[:2], robot_state.tcp_speed[:2])
        processed = self.preprocessor.process(raw)
        self.termination.observe(policy=self.policy, robot=robot_state, raw=raw,
                                 processed=processed, timestamp=robot_state.timestamp,
                                 source="simulation.step")
        command = self.policy.update(robot_state.timestamp, raw, processed, robot_state)
        self.termination.observe(policy=self.policy, robot=robot_state, raw=raw,
                                 processed=processed, command=command)
        if segment_enters_interior(robot_state.pose[:2], robot_state.pose[:2], self.target):
            self.interior_entered = True
            self._physical_failure("TCP is inside object")

        self.last_raw = raw
        self.last_processed = processed
        self.last_command = command
        self.last_force_metadata = metadata
        self.states_seen.append(command.state)
        self._append_record(robot_state, raw, processed, command, metadata)
        snapshot = self.snapshot()
        self.history.append(snapshot)

        if self.square_full_loop:
            self.loop_report = square_loop_report(
                self.policy.boundary_points, self.target, self.policy.follow_hand,
                self.config["simulation"],
            )
            completed_recoveries = sum(
                record["confirmed_contact"] is not None for record in self.policy.recovery_records
            )
            # The square experiment must observe all four recovery cycles too.
            audit = probe_audit(self.policy.probe_episodes, self.config["policy"]["position_tolerance"])
            if not self.stopped and self.loop_report["completed"] and completed_recoveries >= 4 and audit["all_probes_returned"]:
                self.loop_completed = True
                self._finish("Full contour loop completed successfully.")
            elif not self.stopped and self.policy.state in {State.LOST, State.STOP, State.LOOP_COMPLETE}:
                self._finish(command.reason or f"policy entered {self.policy.state.value}")
        else:
            self.loop_report = generic_loop_report(
                self.policy.boundary_points, self.target, self.policy.follow_hand,
                self.config["simulation"],
            )
        time_limit = self.config["simulation"].get("max_time_sec")
        if not self.stopped and time_limit is not None and robot_state.timestamp >= float(time_limit):
            self._finish("simulation time limit exceeded")
        if not self.stopped:
            before = self.robot.pose[:2].copy()
            self.robot.apply_command(command.direction_xy, command.speed, command.move)
            if not np.allclose(self.trajectory[-1], self.robot.pose[:2], atol=1e-15):
                self.trajectory.append(self.robot.pose[:2].copy())
            after = self.robot.pose[:2].copy()
            if segment_enters_interior(before, after, self.target):
                self.interior_entered = True
                self._physical_failure("executed TCP segment entered object interior")
            episode = self.policy.active_episode
            if episode is not None and episode.contact_pose is not None and command.move:
                if np.dot(after - before, episode.probe_direction) > 1e-10:
                    self.post_contact_push = True
                    self._physical_failure("probe continued forward after contact")
        if not self.robot.inside_workspace():
            self.normal_stop("simulated TCP left container/workspace")
        elif not self.stopped and self.policy.state in {State.STOP, State.LOOP_COMPLETE}:
            audit = probe_audit(self.policy.probe_episodes, self.config["policy"]["position_tolerance"])
            self.loop_completed = bool(
                self.policy.state == State.LOOP_COMPLETE
                and self.loop_report.get("completed", False)
                and audit["all_probes_returned"]
            )
            reason = command.reason or "policy STOP"
            if self.policy.state == State.LOOP_COMPLETE and not self.loop_completed:
                reason = "policy closure failed independent geometry/return audit"
            self._finish(reason)
        return self.snapshot()

    def _physical_failure(self, reason):
        self.physical_failure = reason
        self.loop_completed = False
        self._finish("PHYSICAL FAILURE: " + reason)

    def _append_record(self, robot, raw, processed, command, metadata) -> None:
        features = extract_force_features(processed)
        target_direction = self.policy.current_target_direction
        tangent = self.policy.current_tangent
        interaction = features.interaction_direction
        self.records.append(
            {
                "simulation_time_sec": robot.timestamp,
                "raw_fx": raw.fx,
                "raw_fy": raw.fy,
                "raw_fz": raw.fz,
                "raw_tx": raw.tx,
                "raw_ty": raw.ty,
                "raw_tz": raw.tz,
                "dfx": processed.fx,
                "dfy": processed.fy,
                "dfz": processed.fz,
                "dtx": processed.tx,
                "dty": processed.ty,
                "dtz": processed.tz,
                "fxy": features.fxy,
                "force_angle_rad": features.force_angle,
                "interaction_direction_x": np.nan if interaction is None else interaction[0],
                "interaction_direction_y": np.nan if interaction is None else interaction[1],
                "tcp_x": robot.pose[0],
                "tcp_y": robot.pose[1],
                "tcp_vx": robot.tcp_speed[0],
                "tcp_vy": robot.tcp_speed[1],
                "state": command.state,
                "policy_sub_state": command.policy_sub_state,
                "contact_flag": int(command.contact_flag),
                "possible_corner": int(command.possible_corner),
                "target_direction_x": np.nan if target_direction is None else target_direction[0],
                "target_direction_y": np.nan if target_direction is None else target_direction[1],
                "normal_x": np.nan if target_direction is None else target_direction[0],
                "normal_y": np.nan if target_direction is None else target_direction[1],
                "tangent_x": np.nan if tangent is None else tangent[0],
                "tangent_y": np.nan if tangent is None else tangent[1],
                "command_direction_x": command.direction_xy[0],
                "command_direction_y": command.direction_xy[1],
                "command_speed": command.speed,
                "target_x": command.target_pose[0],
                "target_y": command.target_pose[1],
                "penetration": metadata["penetration"],
                "ground_truth_boundary_distance": self.target.distance_to_boundary(robot.pose[:2]),
                "corner_search_angle_deg": command.ray_theta_deg,
                "probe_id": "" if self.policy.active_episode is None else self.policy.active_episode.probe_id,
                "recovery_id": "" if command.recovery_id is None else command.recovery_id,
                "ray_index": "" if command.ray_index is None else command.ray_index,
                "ray_theta_deg": "" if command.ray_theta_deg is None else command.ray_theta_deg,
                "candidate_status": command.candidate_status,
                "rejection_reason": command.rejection_reason,
                "reason": command.reason,
            }
        )

    def snapshot(self) -> dict:
        features = extract_force_features(self.last_processed)
        return {
            "time": self.robot.time,
            "position": self.robot.pose[:2].copy(),
            "force": self.last_processed.force[:2].copy(),
            "fxy": features.fxy,
            "force_angle": features.force_angle,
            "state": self.policy.state.value,
            "normal": None if self.policy.current_target_direction is None else self.policy.current_target_direction.copy(),
            "tangent": None if self.policy.current_tangent is None else self.policy.current_tangent.copy(),
            "boundary_points": [point.pose[:2].copy() for point in self.policy.boundary_points],
            "first_contact": None if not self.policy.probe_episodes or self.policy.probe_episodes[0].contact_pose is None else self.policy.probe_episodes[0].contact_pose[:2].copy(),
            "corner_anchor": None if self.policy.recovery_anchor is None else self.policy.recovery_anchor[:2].copy(),
            "recovery_clear": None,
            "corner_direction": None if self.policy.recovery_direction is None else self.policy.recovery_direction.copy(),
            "corner_attempts": [value.copy() for value in self.policy.recovery_attempted_directions],
            "corner_rays": list(self.policy.boundary_recovery_rays),
            "current_corner_ray": self.policy.current_recovery_ray,
            "policy_waypoints": list(self.policy.policy_waypoints),
            "corner_records": list(self.policy.recovery_records),
            "corner_theta_deg": self.last_command.ray_theta_deg or 0.0,
            "probe_anchor": None if self.policy.active_episode is None else self.policy.active_episode.anchor_pose[:2].copy(),
            "probe_direction": None if self.policy.active_episode is None else self.policy.active_episode.probe_direction.copy(),
            "probe_segments": [(e.anchor_pose[:2].copy(), e.end_pose[:2].copy())
                               for e in self.policy.probe_episodes if e.end_pose is not None],
            "probe_anchors": [e.anchor_pose[:2].copy() for e in self.policy.probe_episodes],
            "candidate_contacts": [] if self.policy.recovery is None else [p[:2].copy() for p in self.policy.recovery.candidates],
            "follow_hand": self.policy.follow_hand,
            "policy_sub_state": self.policy.sub_state,
            "reason": self.last_command.reason,
        }

    def normal_stop(self, reason: str = "manual normal stop") -> None:
        if self.stopped:
            return
        self._finish(reason)
        self.policy.request_stop(reason)

    def _finish(self, reason: str) -> None:
        """Freeze the simulator and retain the actual failure/completion state."""
        self.termination.observe(policy=self.policy, robot=self.robot.read_state(),
                                 source="simulation._finish")
        self.termination.set_stop_reason(detail=reason, source="simulation._finish")
        self.final_policy_state = self.policy.state.value
        self.failure_probe_id = None if self.loop_completed or not self.policy.probe_episodes else self.policy.probe_episodes[-1].probe_id
        self.robot.tcp_speed[:] = 0.0
        if not np.allclose(self.trajectory[-1], self.robot.pose[:2], atol=1e-15):
            self.trajectory.append(self.robot.pose[:2].copy())
        self.stopped = True
        self.stop_reason = reason
        self.termination.flush(emit=True)

    def emergency_stop(self) -> None:
        self.normal_stop("manual emergency stop")

    def run_headless(self, max_steps: int | None = None) -> None:
        limit = max_steps or int(self.config["simulation"]["max_steps_headless"])
        for _ in range(limit):
            if self.stopped:
                break
            self.step()
        if not self.stopped:
            self.normal_stop(f"headless step limit reached ({limit})")

    def boundary_error_mean(self) -> float:
        if not self.policy.boundary_points:
            return float("nan")
        return float(
            np.mean(
                [self.target.distance_to_boundary(point.pose[:2]) for point in self.policy.boundary_points]
            )
        )

    def finalize(self) -> Path:
        try:
            return self._finalize()
        except BaseException as exc:
            self.termination.set_stop_reason(detail=f"{type(exc).__name__}: {exc}",
                                             exception=exc, source="simulation.finalize")
            self.termination.flush(emit=True)
            raise

    def _finalize(self) -> Path:
        if self.finalized and self.output_dir is not None:
            return self.output_dir
        if not self.stopped:
            self.normal_stop("finalized by operator")
        self._ensure_run()
        self.termination.bind(self.output_dir)
        self.termination.flush()
        with (self.output_dir / "simulation_config_snapshot.yaml").open("w", encoding="utf-8") as handle:
            yaml.safe_dump(self.config, handle, allow_unicode=True, sort_keys=False)
        self._write_logs()
        from simulation.top_view import save_animation_replay, save_result_figure
        save_result_figure(self, self.output_dir / "simulation_result.png")
        self.result_image = self.output_dir / "simulation_result.png"
        strategy_debug = None
        corner_debug = []
        if self.config["visualization"].get("debug_plots", False):
            from simulation.visualization import save_boundary_recovery_debug_figures, save_scan_strategy_debug
            strategy_debug = save_scan_strategy_debug(self, self.output_dir / "scan_strategy_debug.png")
            corner_debug = save_boundary_recovery_debug_figures(self, self.output_dir)
        animation_result = "disabled"
        if bool(self.config["visualization"]["save_animation"]):
            try:
                animation_result = str(save_animation_replay(self, self.output_dir))
            except Exception as exc:  # Animation export must never invalidate the run.
                animation_result = f"failed without aborting simulation: {type(exc).__name__}: {exc}"
        with (self.output_dir / "summary.json").open("w", encoding="utf-8") as handle:
            json.dump(
                {
                    **read_metadata(self.output_dir),
                    "scene": self.config["scene_name"],
                    "success": self.loop_completed,
                    "physical_failure": self.physical_failure,
                    "trajectory_entered_interior": self.interior_entered,
                    "post_contact_forward_push": self.post_contact_push,
                    "probe_audit": probe_audit(self.policy.probe_episodes, self.config["policy"]["position_tolerance"]),
                    "failure_probe_id": self.failure_probe_id,
                    "final_policy_state": self.final_policy_state,
                    "completed_recoveries": sum(r["confirmed_contact"] is not None for r in self.policy.recovery_records),
                    "last_recovery_number": len(self.policy.recovery_records),
                    "loop_report": self.loop_report,
                    "stop_reason": self.stop_reason,
                    **self.termination.summary_fields(),
                    "steps": len(self.records),
                    "boundary_points": len(self.policy.boundary_points),
                    "mean_boundary_distance_m": self.boundary_error_mean(),
                    "states_seen": list(dict.fromkeys(self.states_seen)),
                    "animation": animation_result,
                    "scan_strategy_debug": None if strategy_debug is None else str(strategy_debug),
                    "boundary_recovery_debug": [str(path) for path in corner_debug],
                },
                handle,
                ensure_ascii=False,
                indent=2,
            )
        self.finalized = True
        print("FULL LOOP COMPLETED" if self.loop_completed else "LOOP FAILED")
        print("Full contour loop completed successfully." if self.loop_completed else f"Failure reason: {self.stop_reason}")
        audit = probe_audit(self.policy.probe_episodes, self.config["policy"]["position_tolerance"])
        print(f"TCP entered object interior: {self.interior_entered}; post-contact forward push: {self.post_contact_push}")
        print(f"Every probe returned to its anchor: {audit['all_probes_returned']} ({audit['probe_count']} episodes; max error {audit['maximum_return_error_m'] * 1000:.6f} mm)")
        if not self.loop_completed:
            print(f"Failure probe: {self.failure_probe_id}; sub-state: {self.policy.sub_state}")
        print(f"Final policy state: {self.final_policy_state}; boundary points: {len(self.policy.boundary_points)}")
        confirmed = sum(r["confirmed_contact"] is not None for r in self.policy.recovery_records)
        print(f"Completed boundary recoveries: {confirmed}; last recovery number: {len(self.policy.recovery_records)} (1-based, 0 = none)")
        print(f"Result image: {self.result_image}")
        return self.output_dir

    def _write_logs(self) -> None:
        from experiment_logging.probe_log import write_probe_logs
        write_probe_logs(self.output_dir, self.policy.probe_episodes)
        assert self.output_dir is not None
        if self.records:
            with (self.output_dir / "simulation_log.csv").open("w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(self.records[0]))
                writer.writeheader()
                writer.writerows(self.records)
        if self.policy.boundary_points:
            with (self.output_dir / "boundary_points.csv").open("w", encoding="utf-8", newline="") as handle:
                fields = [
                    "point_id", "simulation_time_sec", "x", "y", "dfx", "dfy",
                    "dfz", "dtx", "dty", "dtz", "target_direction_x", "target_direction_y",
                    "normal_x", "normal_y",
                    "tangent_x", "tangent_y", "possible_corner", "ground_truth_distance",
                ]
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                for point in self.policy.boundary_points:
                    writer.writerow(
                        {
                            "point_id": f"P{point.index}",
                            "simulation_time_sec": point.timestamp,
                            "x": point.pose[0],
                            "y": point.pose[1],
                            "dfx": point.wrench.fx,
                            "dfy": point.wrench.fy,
                            "dfz": point.wrench.fz,
                            "dtx": point.wrench.tx,
                            "dty": point.wrench.ty,
                            "dtz": point.wrench.tz,
                            "target_direction_x": point.target_direction_xy[0],
                            "target_direction_y": point.target_direction_xy[1],
                            "normal_x": point.target_direction_xy[0],
                            "normal_y": point.target_direction_xy[1],
                            "tangent_x": point.tangent_xy[0],
                            "tangent_y": point.tangent_xy[1],
                            "possible_corner": int(point.possible_corner),
                            "ground_truth_distance": self.target.distance_to_boundary(point.pose[:2]),
                        }
                    )
        if self.policy.policy_waypoints:
            with (self.output_dir / "policy_waypoints.csv").open("w", encoding="utf-8", newline="") as handle:
                fields = [
                    "timestamp", "state", "event_type", "x", "y", "z", "corner_id", "ray_index",
                    "target_direction_x", "target_direction_y", "tangent_x", "tangent_y",
                ]
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                for item in self.policy.policy_waypoints:
                    target, tangent = item.target_direction_xy, item.tangent_xy
                    writer.writerow({
                        "timestamp": item.timestamp, "state": item.state, "event_type": item.event_type,
                        "x": item.pose[0], "y": item.pose[1], "z": item.pose[2],
                        "corner_id": "" if item.corner_id is None else item.corner_id,
                        "ray_index": "" if item.ray_index is None else item.ray_index,
                        "target_direction_x": "" if target is None else target[0],
                        "target_direction_y": "" if target is None else target[1],
                        "tangent_x": "" if tangent is None else tangent[0],
                        "tangent_y": "" if tangent is None else tangent[1],
                    })
        if self.policy.boundary_recovery_rays:
            with (self.output_dir / "boundary_recovery_rays.csv").open("w", encoding="utf-8", newline="") as handle:
                fields = [
                    "recovery_id", "corner_id", "ray_index", "theta_deg", "anchor_x", "anchor_y",
                    "direction_x", "direction_y", "planned_length", "actual_length",
                    "ray_end_x", "ray_end_y", "contact_x", "contact_y", "result",
                ]
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                for ray in self.policy.boundary_recovery_rays:
                    contact = ray.contact_pose
                    ray_end = ray.ray_end_xy
                    writer.writerow({
                        "recovery_id": ray.recovery_id,
                        "corner_id": ray.corner_id, "ray_index": ray.ray_index,
                        "theta_deg": ray.theta_deg, "anchor_x": ray.anchor_pose[0],
                        "anchor_y": ray.anchor_pose[1], "direction_x": ray.direction_xy[0],
                        "direction_y": ray.direction_xy[1], "planned_length": ray.planned_length,
                        "actual_length": ray.actual_length,
                        "ray_end_x": ray_end[0], "ray_end_y": ray_end[1],
                        "contact_x": "" if contact is None else contact[0],
                        "contact_y": "" if contact is None else contact[1], "result": ray.result,
                    })

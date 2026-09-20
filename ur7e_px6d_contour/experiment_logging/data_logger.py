"""Write experiment data without participating in motion decisions."""

from __future__ import annotations

import csv
import json
from datetime import datetime, timezone
from pathlib import Path

import yaml

from core.models import BoundaryPoint, CornerSearchRay, PolicyCommand, PolicyWaypoint, RobotState, Wrench
from experiment_logging.termination import TerminationRecorder
from sensor.force_features import extract_force_features


SAMPLE_FIELDS = [
    "timestamp_utc", "monotonic_sec",
    "raw_fx", "raw_fy", "raw_fz", "raw_tx", "raw_ty", "raw_tz",
    "dfx", "dfy", "dfz", "dtx", "dty", "dtz",
    "fxy", "force_angle_rad",
    "interaction_direction_x", "interaction_direction_y",
    "target_direction_x", "target_direction_y",
    "estimated_normal_x", "estimated_normal_y", "tangent_x", "tangent_y",
    "tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz",
    "tcp_vx", "tcp_vy", "tcp_vz", "tcp_vrx", "tcp_vry", "tcp_vrz", "tcp_speed_mps",
    "current_state", "policy_sub_state", "contact_flag", "possible_corner",
    "probe_id", "recovery_id", "ray_index", "ray_theta_deg",
    "candidate_status", "rejection_reason",
    "target_x", "target_y", "target_z", "target_rx", "target_ry", "target_rz",
    "commanded_direction_x", "commanded_direction_y", "commanded_speed_mps", "reason",
]

BOUNDARY_FIELDS = [
    "point_id", "monotonic_sec",
    "tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz",
    "dfx", "dfy", "dfz", "dtx", "dty", "dtz",
    "target_direction_x", "target_direction_y",
    "estimated_normal_x", "estimated_normal_y", "tangent_x", "tangent_y",
    "possible_corner",
]

WAYPOINT_FIELDS = [
    "timestamp", "state", "event_type", "x", "y", "z", "corner_id", "ray_index",
    "target_direction_x", "target_direction_y", "tangent_x", "tangent_y",
]

RECOVERY_RAY_FIELDS = [
    "recovery_id", "corner_id", "ray_index", "theta_deg", "anchor_x", "anchor_y",
    "direction_x", "direction_y", "planned_length", "actual_length",
    "ray_end_x", "ray_end_y", "contact_x", "contact_y", "result",
]


class ExperimentLogger:
    def __init__(self, root: str | Path, config: dict, *, extra_sample_fields=(), workspace_logging=True):
        fields = SAMPLE_FIELDS + list(extra_sample_fields)
        if len(set(fields)) != len(fields):
            raise ValueError("sample field names must be unique")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.run_dir = Path(root).expanduser().resolve() / f"run_{stamp}"
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.termination = TerminationRecorder().bind(self.run_dir)
        with (self.run_dir / "config_snapshot.yaml").open("w", encoding="utf-8") as handle:
            yaml.safe_dump(config, handle, allow_unicode=True, sort_keys=False)
        self._sample_file = (self.run_dir / "samples.csv").open("w", encoding="utf-8", newline="")
        self._full_log_file = (self.run_dir / "full_log.csv").open("w", encoding="utf-8", newline="")
        self._boundary_file = (self.run_dir / "boundary_points.csv").open("w", encoding="utf-8", newline="")
        self._waypoint_file = (self.run_dir / "policy_waypoints.csv").open("w", encoding="utf-8", newline="")
        self._recovery_ray_file = (self.run_dir / "boundary_recovery_rays.csv").open("w", encoding="utf-8", newline="")
        self._samples = csv.DictWriter(self._sample_file, fieldnames=fields)
        self._full_log = csv.DictWriter(self._full_log_file, fieldnames=fields)
        self._boundaries = csv.DictWriter(self._boundary_file, fieldnames=BOUNDARY_FIELDS)
        self._waypoints = csv.DictWriter(self._waypoint_file, fieldnames=WAYPOINT_FIELDS)
        self._recovery_rays = csv.DictWriter(self._recovery_ray_file, fieldnames=RECOVERY_RAY_FIELDS)
        self._samples.writeheader()
        self._full_log.writeheader()
        self._boundaries.writeheader()
        self._waypoints.writeheader()
        self._recovery_rays.writeheader()
        self._sample_count = 0
        self._workspace_logger = None
        try:
            workspace_config = Path(__file__).resolve().parents[1] / "workspace/config/workspace_calibration.yaml"
            if workspace_logging and workspace_config.is_file():
                from workspace.workspace_logger import create_optional_workspace_logger
                self._workspace_logger = create_optional_workspace_logger(self.run_dir)
        except Exception as exc:
            try:
                print(f"[workspace] Optional workspace logger unavailable: {type(exc).__name__}: {exc}")
            except Exception:
                pass

    def log_sample(
        self,
        monotonic_sec: float,
        raw: Wrench,
        processed: Wrench,
        robot: RobotState,
        command: PolicyCommand,
        target_direction,
        tangent,
        *,
        extra: dict | None = None,
    ) -> None:
        if self.termination is not None:
            self.termination.observe(robot=robot, raw=raw, processed=processed,
                                     command=command, timestamp=monotonic_sec)
        features = extract_force_features(processed)
        raw_values = raw.array()
        processed_values = processed.array()
        pose = robot.pose
        speed = robot.tcp_speed
        target_direction = (
            (float("nan"), float("nan"))
            if target_direction is None else target_direction
        )
        tangent = (float("nan"), float("nan")) if tangent is None else tangent
        interaction = (
            (float("nan"), float("nan"))
            if features.interaction_direction is None
            else features.interaction_direction
        )
        row = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "monotonic_sec": f"{monotonic_sec:.9f}",
            **dict(zip(("raw_fx", "raw_fy", "raw_fz", "raw_tx", "raw_ty", "raw_tz"), raw_values)),
            **dict(zip(("dfx", "dfy", "dfz", "dtx", "dty", "dtz"), processed_values)),
            "fxy": features.fxy,
            "force_angle_rad": features.force_angle,
            "interaction_direction_x": interaction[0],
            "interaction_direction_y": interaction[1],
            "target_direction_x": target_direction[0],
            "target_direction_y": target_direction[1],
            # Retained for compatibility with older analysis tools.
            "estimated_normal_x": target_direction[0],
            "estimated_normal_y": target_direction[1],
            "tangent_x": tangent[0], "tangent_y": tangent[1],
            **dict(zip(("tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz"), pose)),
            **dict(zip(("tcp_vx", "tcp_vy", "tcp_vz", "tcp_vrx", "tcp_vry", "tcp_vrz"), speed)),
            "tcp_speed_mps": float((speed[:3] @ speed[:3]) ** 0.5),
            "current_state": command.state,
            "policy_sub_state": command.policy_sub_state,
            "contact_flag": int(command.contact_flag),
            "possible_corner": int(command.possible_corner),
            "recovery_id": "" if command.recovery_id is None else command.recovery_id,
            "probe_id": "" if command.probe_id is None else command.probe_id,
            "ray_index": "" if command.ray_index is None else command.ray_index,
            "ray_theta_deg": "" if command.ray_theta_deg is None else command.ray_theta_deg,
            "candidate_status": command.candidate_status,
            "rejection_reason": command.rejection_reason,
            **dict(zip(("target_x", "target_y", "target_z", "target_rx", "target_ry", "target_rz"), command.target_pose)),
            "commanded_direction_x": command.direction_xy[0],
            "commanded_direction_y": command.direction_xy[1],
            "commanded_speed_mps": command.speed,
            "reason": command.reason,
        }
        if extra:
            if set(extra) & set(SAMPLE_FIELDS):
                raise ValueError("extra sample data cannot overwrite standard fields")
            row.update(extra)
        self._samples.writerow(row)
        self._full_log.writerow(row)
        if self._workspace_logger is not None:
            self._workspace_logger.log_sample(row)
        self._sample_count += 1
        if self._sample_count % 20 == 0:
            self._sample_file.flush()
            self._full_log_file.flush()

    def log_boundary(self, point: BoundaryPoint) -> None:
        values = point.wrench.array()
        row = {
            "point_id": f"P{point.index}",
            "monotonic_sec": f"{point.timestamp:.9f}",
            **dict(zip(("tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz"), point.pose)),
            **dict(zip(("dfx", "dfy", "dfz", "dtx", "dty", "dtz"), values)),
            "target_direction_x": point.target_direction_xy[0],
            "target_direction_y": point.target_direction_xy[1],
            "estimated_normal_x": point.target_direction_xy[0],
            "estimated_normal_y": point.target_direction_xy[1],
            "tangent_x": point.tangent_xy[0],
            "tangent_y": point.tangent_xy[1],
            "possible_corner": int(point.possible_corner),
        }
        self._boundaries.writerow(row)
        self._boundary_file.flush()
        if self._workspace_logger is not None:
            self._workspace_logger.log_boundary(row)

    def log_waypoint(self, waypoint: PolicyWaypoint) -> None:
        target = waypoint.target_direction_xy
        tangent = waypoint.tangent_xy
        self._waypoints.writerow({
            "timestamp": f"{waypoint.timestamp:.9f}",
            "state": waypoint.state,
            "event_type": waypoint.event_type,
            "x": waypoint.pose[0], "y": waypoint.pose[1], "z": waypoint.pose[2],
            "corner_id": "" if waypoint.corner_id is None else waypoint.corner_id,
            "ray_index": "" if waypoint.ray_index is None else waypoint.ray_index,
            "target_direction_x": "" if target is None else target[0],
            "target_direction_y": "" if target is None else target[1],
            "tangent_x": "" if tangent is None else tangent[0],
            "tangent_y": "" if tangent is None else tangent[1],
        })
        self._waypoint_file.flush()

    def log_recovery_ray(self, ray: CornerSearchRay) -> None:
        contact = ray.contact_pose
        ray_end = ray.ray_end_xy
        self._recovery_rays.writerow({
            "recovery_id": ray.recovery_id,
            "corner_id": ray.corner_id,
            "ray_index": ray.ray_index,
            "theta_deg": ray.theta_deg,
            "anchor_x": ray.anchor_pose[0], "anchor_y": ray.anchor_pose[1],
            "direction_x": ray.direction_xy[0], "direction_y": ray.direction_xy[1],
            "planned_length": ray.planned_length,
            "actual_length": ray.actual_length,
            "ray_end_x": ray_end[0], "ray_end_y": ray_end[1],
            "contact_x": "" if contact is None else contact[0],
            "contact_y": "" if contact is None else contact[1],
            "result": ray.result,
        })
        self._recovery_ray_file.flush()

    def log_corner_ray(self, ray: CornerSearchRay) -> None:
        """Compatibility method for callers written before recovery renaming."""
        self.log_recovery_ray(ray)

    def write_stop_snapshot(
        self,
        robot: RobotState,
        raw: Wrench | None,
        processed: Wrench | None,
        state: str,
        *,
        extra: dict | None = None,
    ) -> None:
        if self.termination is not None:
            self.termination.observe(robot=robot, raw=raw, processed=processed, state=state)
        with (self.run_dir / "scan_stop_snapshot.json").open("w", encoding="utf-8") as handle:
            json.dump(
                {
                    "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                    "state": state,
                    "tcp_pose": robot.pose.tolist(),
                    "tcp_speed": robot.tcp_speed.tolist(),
                    "raw_wrench": None if raw is None else raw.array().tolist(),
                    "processed_wrench": None if processed is None else processed.array().tolist(),
                    **self._termination_fields(),
                    **(extra or {}),
                },
                handle,
                ensure_ascii=False,
                indent=2,
            )
        self.flush()

    def write_summary(self, state: str, reason: str, boundary_count: int, **extra) -> None:
        if self.termination is not None and self.termination.record is None:
            self.termination.set_stop_reason(detail=reason, source="logger.write_summary", state=state)
        payload = {
            "final_state": state,
            "reason": reason,
            "boundary_point_count": boundary_count,
            **extra,
            **self._termination_fields(),
        }
        with (self.run_dir / "summary.json").open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
        if self.termination is not None:
            self.termination.flush()

    def _termination_fields(self) -> dict:
        return {} if self.termination is None else self.termination.summary_fields()

    def flush(self) -> None:
        self._sample_file.flush()
        self._full_log_file.flush()
        self._boundary_file.flush()
        self._waypoint_file.flush()
        self._recovery_ray_file.flush()

        if self._workspace_logger is not None:
            self._workspace_logger.flush()

    def close(self) -> None:
        self._sample_file.close()
        self._full_log_file.close()
        self._boundary_file.close()
        self._waypoint_file.close()
        self._recovery_ray_file.close()
        if self._workspace_logger is not None:
            self._workspace_logger.close()

    def __enter__(self) -> "ExperimentLogger":
        return self

    def __exit__(self, *_args) -> None:
        self.close()

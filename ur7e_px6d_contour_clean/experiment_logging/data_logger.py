"""Write experiment data without participating in motion decisions."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import yaml

from core.models import BoundaryPoint, CornerSearchRay, PolicyCommand, PolicyWaypoint, RobotState, Wrench
from experiment_logging.termination import TerminationRecorder, continuous_label
from experiment_logging.paths import create_run, read_metadata, wall_time_fields
from sensor.force_features import extract_force_features


SAMPLE_FIELDS = [
    "timestamp_utc", "timestamp_local", "timezone", "monotonic_sec",
    "raw_fx", "raw_fy", "raw_fz", "raw_tx", "raw_ty", "raw_tz",
    "dfx", "dfy", "dfz", "dtx", "dty", "dtz",
    "force_base_fx", "force_base_fy", "force_base_fz",
    "filtered_force_base_fx", "filtered_force_base_fy", "filtered_force_base_fz",
    "fxy", "force_angle_rad", "processed_force_frame",
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
    "reacquire_round", "event_label",
]

RECOVERY_RAY_FIELDS = [
    "recovery_id", "corner_id", "ray_index", "theta_deg", "anchor_x", "anchor_y",
    "direction_x", "direction_y", "planned_length", "actual_length",
    "ray_end_x", "ray_end_y", "contact_x", "contact_y", "result",
]


class LazyCSV:
    def __init__(self, path, fields):
        self.path, self.fields, self.handle, self.writer = path, fields, None, None

    def writerow(self, row):
        if self.writer is None:
            self.handle = self.path.open('w', encoding='utf-8', newline='')
            self.writer = csv.DictWriter(self.handle, fieldnames=self.fields)
            self.writer.writeheader()
        self.writer.writerow(row)

    def flush(self):
        if self.handle is not None:
            self.handle.flush()

    def close(self):
        if self.handle is not None:
            self.handle.close()


class ExperimentLogger:
    def __init__(self, root: str | Path, config: dict, *, mode, strategy, config_source=None, extra_sample_fields=(), workspace_logging=True):
        fields = SAMPLE_FIELDS + list(extra_sample_fields)
        if len(set(fields)) != len(fields):
            raise ValueError("sample field names must be unique")
        self.run_dir = create_run(mode, strategy, config_source or config.get('_config_source'), data_root=root)
        self.metadata = read_metadata(self.run_dir)
        self._reacquire_round = 0
        self._reacquire_event_active = False
        self.processed_force_frame = config.get('force_display', {}).get('frame', 'configured_output_frame')
        self.termination = TerminationRecorder().bind(self.run_dir)
        self.write_config_snapshot(config)
        self._samples = LazyCSV(self.run_dir / 'samples.csv', fields)
        self._full_log = LazyCSV(self.run_dir / 'full_log.csv', fields)
        self._boundaries = LazyCSV(self.run_dir / 'boundary_points.csv', BOUNDARY_FIELDS)
        self._waypoints = LazyCSV(self.run_dir / 'policy_waypoints.csv', WAYPOINT_FIELDS)
        self._recovery_rays = LazyCSV(self.run_dir / 'boundary_recovery_rays.csv', RECOVERY_RAY_FIELDS)
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

    def write_config_snapshot(self, config: dict) -> None:
        """Record effective run settings; never write the source configuration."""
        with (self.run_dir / 'config_snapshot.yaml').open('w', encoding='utf-8') as handle:
            yaml.safe_dump(config, handle, allow_unicode=True, sort_keys=False)

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
                command=command, timestamp=monotonic_sec,
                processed_force_frame=(extra or {}).get('processed_force_frame', self.processed_force_frame))
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
            **wall_time_fields(),
            "monotonic_sec": f"{monotonic_sec:.9f}",
            **dict(zip(("raw_fx", "raw_fy", "raw_fz", "raw_tx", "raw_ty", "raw_tz"), raw_values)),
            **dict(zip(("dfx", "dfy", "dfz", "dtx", "dty", "dtz"), processed_values)),
            # Explicit Base aliases accompany historical dfx/dfy/dfz. Raw
            # columns remain Sensor measurements; return rows can be Sensor.
            **dict(zip(('filtered_force_base_fx', 'filtered_force_base_fy', 'filtered_force_base_fz'),
                       processed_values[:3] if (extra or {}).get('processed_force_frame', self.processed_force_frame) == 'Base'
                       else ('', '', ''))),
            "fxy": features.fxy,
            "force_angle_rad": features.force_angle,
            "processed_force_frame": self.processed_force_frame,
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
            # Return may log raw Sensor coordinates while the scan logs Base.
            # Frame and pre-filter force come from preprocessing. Published
            # raw/filtered measurements, poses and timestamps stay protected.
            if set(extra) & (set(SAMPLE_FIELDS) - {'processed_force_frame',
                                                   'force_base_fx', 'force_base_fy', 'force_base_fz'}):
                raise ValueError("extra sample data cannot overwrite standard fields")
            row.update(extra)
        self._samples.writerow(row)
        self._full_log.writerow(row)
        if self._workspace_logger is not None:
            self._workspace_logger.log_sample(row)
        self._sample_count += 1
        if self._sample_count % 20 == 0:
            self._samples.flush()
            self._full_log.flush()

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
        self._boundaries.flush()
        if self._workspace_logger is not None:
            self._workspace_logger.log_boundary(row)

    def log_waypoint(self, waypoint: PolicyWaypoint) -> None:
        target = waypoint.target_direction_xy
        tangent = waypoint.tangent_xy
        round_number, label = '', waypoint.event_type
        if self.metadata['strategy'] == 'continuous':
            if waypoint.event_type == 'LOCAL_REACQUIRE':
                self._reacquire_round += 1
                self._reacquire_event_active = True
            round_number = self._reacquire_round if self._reacquire_event_active else ''
            label = continuous_label(waypoint.event_type, round_number or 0)
            if waypoint.event_type == 'REACQUIRED' or waypoint.state == 'STOP':
                self._reacquire_event_active = False
        self._waypoints.writerow({
            "timestamp": f"{waypoint.timestamp:.9f}",
            "state": waypoint.state,
            "event_type": waypoint.event_type,
            "reacquire_round": round_number, "event_label": label,
            "x": waypoint.pose[0], "y": waypoint.pose[1], "z": waypoint.pose[2],
            "corner_id": "" if waypoint.corner_id is None else waypoint.corner_id,
            "ray_index": "" if waypoint.ray_index is None else waypoint.ray_index,
            "target_direction_x": "" if target is None else target[0],
            "target_direction_y": "" if target is None else target[1],
            "tangent_x": "" if tangent is None else tangent[0],
            "tangent_y": "" if tangent is None else tangent[1],
        })
        self._waypoints.flush()

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
        self._recovery_rays.flush()

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
                    **wall_time_fields(),
                    "state": state,
                    "tcp_pose": robot.pose.tolist(),
                    "tcp_speed": robot.tcp_speed.tolist(),
                    "raw_wrench": None if raw is None else raw.array().tolist(),
                    "processed_wrench": None if processed is None else processed.array().tolist(),
                    **self._termination_fields(),
            **read_metadata(self.run_dir),
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
            **read_metadata(self.run_dir),
        }
        with (self.run_dir / "summary.json").open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
        if self.termination is not None:
            self.termination.flush()

    def _termination_fields(self) -> dict:
        return {} if self.termination is None else self.termination.summary_fields()

    def flush(self) -> None:
        self._samples.flush()
        self._full_log.flush()
        self._boundaries.flush()
        self._waypoints.flush()
        self._recovery_rays.flush()

        if self._workspace_logger is not None:
            self._workspace_logger.flush()

    def close(self) -> None:
        errors = []
        for resource in (self._samples, self._full_log, self._boundaries, self._waypoints,
                         self._recovery_rays, self._workspace_logger):
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    errors.append(exc)
        if errors:
            raise errors[0]

    def __enter__(self) -> "ExperimentLogger":
        return self

    def __exit__(self, *_args) -> None:
        self.close()

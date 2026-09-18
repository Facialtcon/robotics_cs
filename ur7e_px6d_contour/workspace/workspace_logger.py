"""Optional, read-only projection of existing scan logs into a taught frame.

Nothing in this module supplies commands or configuration to a robot or policy.
The guarded adapter disables only these additional outputs on an error.
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

from workspace.workspace_calibrator import save_calibration
from workspace.workspace_transform import DEFAULT_CALIBRATION_PATH, WorkspaceTransform


FIELDS = [
    "record_type", "sample_index", "timestamp_utc", "monotonic_sec", "time_domain",
    "x_base", "y_base", "z_base", "x_workspace", "y_workspace", "z_workspace",
    "Fx", "Fy", "Fz", "force_frame", "policy_state", "policy_sub_state",
    "contact_state", "boundary_state", "contact_point", "probe_id", "boundary_point_id",
    "contact_source", "inside_workspace", "calibration_id",
]


def _true(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


class WorkspaceLogger:
    """Write samples and accepted-boundary events, retaining their original order.

    Sample contact markers mean first *threshold detection*, not policy acceptance.
    On close, available probe records supply the confirmed episode contact poses.
    """

    def __init__(self, output_dir, calibration_path=DEFAULT_CALIBRATION_PATH, *,
                 simulation=False, overwrite=False):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.transform = WorkspaceTransform.from_file(calibration_path)
        self.csv_path = self.output_dir / "workspace_scan.csv"
        self.snapshot_path = self.output_dir / "workspace_calibration_snapshot.yaml"
        if not overwrite and (self.csv_path.exists() or self.snapshot_path.exists()):
            raise FileExistsError("workspace outputs already exist; use a new directory or --overwrite")
        save_calibration(self.transform.calibration, self.snapshot_path)
        self._file = self.csv_path.open("w", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._file, FIELDS)
        self._writer.writeheader()
        self._file.flush()
        self.simulation = simulation
        self.sample_count = 0
        self.boundary_count = 0
        self.threshold_contact_count = 0
        self.episode_contact_count = 0
        self._seen_contact_probes = set()
        self._last_contact = False
        self._contact_samples = {}
        self._closed = False

    def _project(self, row, record_type):
        z = row.get("tcp_z", row.get("z", 0.0 if self.simulation else None))
        if z is None or z == "":
            raise ValueError("real TCP log is missing z; a measured height is required")
        base = np.asarray([row.get("tcp_x", row.get("x")),
                           row.get("tcp_y", row.get("y")), z], dtype=float)
        if base.shape != (3,) or not np.all(np.isfinite(base)):
            raise ValueError("TCP position must contain three finite coordinates")
        local = self.transform.base_to_workspace(base)
        length, width = self.transform.calibration["length"], self.transform.calibration["width"]
        return {
            "record_type": record_type,
            "timestamp_utc": row.get("timestamp_utc", ""),
            "monotonic_sec": row.get("monotonic_sec", row.get("simulation_time_sec", "")),
            "time_domain": "simulation" if self.simulation else "monotonic",
            **dict(zip(("x_base", "y_base", "z_base"), base)),
            **dict(zip(("x_workspace", "y_workspace", "z_workspace"), local)),
            "Fx": row.get("dfx", ""), "Fy": row.get("dfy", ""), "Fz": row.get("dfz", ""),
            # Retain the existing processed wrench unchanged, never silently rotate it.
            "force_frame": "existing_processed_wrench",
            "policy_state": row.get("current_state", row.get("state", "")),
            "policy_sub_state": row.get("policy_sub_state", ""),
            "probe_id": row.get("probe_id", ""),
            "inside_workspace": int(-1e-9 <= local[0] <= length + 1e-9
                                    and -1e-9 <= local[1] <= width + 1e-9),
            "calibration_id": self.transform.calibration_id,
        }

    def log_sample(self, row):
        output = self._project(row, "sample")
        contact = _true(row.get("contact_flag", 0))
        probe = str(row.get("probe_id", ""))
        first = contact and (probe not in self._seen_contact_probes if probe else not self._last_contact)
        if first:
            self.threshold_contact_count += 1
            if probe:
                self._seen_contact_probes.add(probe)
                self._contact_samples[probe] = dict(row)
        self._last_contact = contact
        self.sample_count += 1
        output.update(sample_index=self.sample_count, contact_state=int(contact),
                      boundary_state="", contact_point=int(first),
                      contact_source="threshold_detection" if first else "")
        self._writer.writerow(output)
        if self.sample_count % 20 == 0:
            self.flush()

    def log_boundary(self, row):
        output = self._project(row, "boundary")
        # Boundary time may precede the current sample after local initialization
        # sorting. Do not attach a later sample's policy state or wall-clock time.
        output.update(boundary_state="ACCEPTED", boundary_point_id=row.get("point_id", ""),
                      contact_state=1, contact_point=0)
        self._writer.writerow(output)
        self.boundary_count += 1
        self.flush()

    def log_probe_contacts(self, probe_json):
        """Append authoritative saved episode contact positions without editing policy.

        Older episode records omit contact time/wrench. Reuse a matching threshold
        sample only when its position agrees; otherwise leave missing fields blank.
        """
        path = Path(probe_json)
        if not path.exists():
            return
        with path.open(encoding="utf-8") as handle:
            episodes = json.load(handle)
        for episode in episodes:
            pose = episode.get("contact_pose")
            # A pose retained during HOLD followed by ABORTED is only a
            # threshold detection. Do not upgrade it to a confirmed contact.
            if pose is None or episode.get("outcome") != "CONTACT":
                continue
            if len(pose) < 3:
                raise ValueError("probe contact_pose requires XYZ")
            probe = str(episode.get("probe_id", ""))
            sample = self._contact_samples.get(probe, {})
            if sample:
                xyz = [sample.get("tcp_x"), sample.get("tcp_y"),
                       sample.get("tcp_z", 0.0 if self.simulation else None)]
                if not np.allclose(np.asarray(xyz, dtype=float), pose[:3], atol=1e-8, rtol=0):
                    sample = {}
            row = {**sample, "tcp_x": pose[0], "tcp_y": pose[1], "tcp_z": pose[2],
                   "current_state": episode.get("policy_state", ""), "probe_id": probe}
            output = self._project(row, "contact")
            output.update(contact_state=1, contact_point=1, contact_source="probe_episode",
                          boundary_state="ACCEPTED" if episode.get("accepted_as_boundary") else "NOT_ACCEPTED")
            self._writer.writerow(output)
            self.episode_contact_count += 1

    def flush(self):
        if not self._closed:
            self._file.flush()

    def close(self, *, render=True, probe_json=None):
        if self._closed:
            return
        try:
            if probe_json is not None:
                self.log_probe_contacts(probe_json)
        finally:
            self._closed = True
            self._file.close()
        with (self.output_dir / "workspace_summary.json").open("w", encoding="utf-8") as handle:
            json.dump({"calibration_id": self.transform.calibration_id,
                       "calibration_snapshot": self.snapshot_path.name,
                       "sample_count": self.sample_count,
                       "threshold_contact_count": self.threshold_contact_count,
                       "episode_contact_count": self.episode_contact_count,
                       "accepted_boundary_count": self.boundary_count,
                       "force_frame": "existing_processed_wrench",
                       "simulation_missing_z": "base_z=0" if self.simulation else None,
                       "contour_order": "boundary acceptance order; no artificial closing segment"},
                      handle, ensure_ascii=False, indent=2)
        if render:
            from workspace.workspace_visualizer import visualize_workspace
            visualize_workspace(self.csv_path, self.snapshot_path)


def export_existing_run(run_dir, calibration_path=DEFAULT_CALIBRATION_PATH, output_dir=None, *,
                        overwrite=False, render=True):
    """Stream either real or simulation records; never substitute waypoints for TCP."""
    run_dir = Path(run_dir)
    samples = run_dir / "samples.csv"
    simulation = False
    if not samples.exists():
        samples = run_dir / "simulation_log.csv"
        simulation = True
    if not samples.exists():
        raise FileNotFoundError(f"no samples.csv or simulation_log.csv in {run_dir}")
    logger = WorkspaceLogger(output_dir or run_dir, calibration_path,
                             simulation=simulation, overwrite=overwrite)
    try:
        with samples.open(encoding="utf-8", newline="") as handle:
            for row in csv.DictReader(handle):
                logger.log_sample(row)
        boundary = run_dir / "boundary_points.csv"
        if boundary.exists():
            with boundary.open(encoding="utf-8", newline="") as handle:
                for row in csv.DictReader(handle):
                    logger.log_boundary(row)
        logger.close(render=render, probe_json=run_dir / "probe_episodes.json")
    except BaseException:
        logger.close(render=False)
        raise
    return logger.csv_path


def _warning(exc):
    # Diagnostic failure must not become a motion/policy failure.
    try:
        print(f"[workspace] Optional workspace output disabled: {type(exc).__name__}: {exc}", file=sys.stderr)
    except Exception:
        pass


class OptionalWorkspaceLogger:
    """Small failure-isolating adapter used by the existing experiment logger."""

    def __init__(self, logger):
        self.logger = logger

    def _call(self, method, *args, **kwargs):
        if self.logger is None:
            return
        try:
            getattr(self.logger, method)(*args, **kwargs)
        except Exception as exc:
            logger, self.logger = self.logger, None
            _warning(exc)
            try:
                logger.close(render=False)
            except Exception:
                pass

    def log_sample(self, row):
        self._call("log_sample", row)

    def log_boundary(self, row):
        self._call("log_boundary", row)

    def flush(self):
        self._call("flush")

    def close(self):
        if self.logger is not None:
            self._call("close", probe_json=self.logger.output_dir / "probe_episodes.json")


def create_optional_workspace_logger(run_dir, calibration_path=None):
    try:
        path = Path(calibration_path) if calibration_path is not None else DEFAULT_CALIBRATION_PATH
        if not path.is_file():
            return None
        return OptionalWorkspaceLogger(WorkspaceLogger(run_dir, path))
    except Exception as exc:
        _warning(exc)
        return None


def export_if_calibrated(run_dir, calibration_path=None):
    try:
        path = Path(calibration_path) if calibration_path is not None else DEFAULT_CALIBRATION_PATH
        if not path.is_file():
            return None
        return export_existing_run(run_dir, path)
    except Exception as exc:
        _warning(exc)
        return None

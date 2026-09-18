import csv
import json
from pathlib import Path

import numpy as np
import pytest

from core.models import BoundaryPoint, PolicyCommand, RobotState, Wrench
from experiment_logging.data_logger import ExperimentLogger
from workspace.workspace_calibrator import fit_workspace, save_calibration
from workspace.workspace_logger import (
    OptionalWorkspaceLogger, WorkspaceLogger, create_optional_workspace_logger,
    export_existing_run, export_if_calibrated,
)


@pytest.fixture
def calibration(tmp_path):
    # Deliberately rotated and translated from base coordinates.
    points = [[1, 2, .1, 0, 0, 0], [1, 2.4, .1, 0, 0, 0],
              [.7, 2.4, .1, 0, 0, 0], [.7, 2, .1, 0, 0, 0]]
    value = fit_workspace(dict(zip(("P0", "P1", "P2", "P3"), points)))
    return save_calibration(value, tmp_path / "calibration.yaml")


def sample(**overrides):
    return dict(tcp_x=.9, tcp_y=2.2, tcp_z=.12, monotonic_sec=12,
                timestamp_utc="2026-09-14T01:00:00+00:00", dfx=1, dfy=2, dfz=3,
                current_state="BOUNDARY_TRACKING", policy_sub_state="PROBE",
                contact_flag=0, probe_id=1, **overrides)


def read_rows(path):
    with Path(path).open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_rows(path, rows):
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def test_project_keep_measured_z_and_original_force(calibration, tmp_path):
    logger = WorkspaceLogger(tmp_path / "out", calibration)
    row = sample()
    untouched = dict(row)
    logger.log_sample(row)
    logger.close(render=False)
    output = read_rows(logger.csv_path)[0]
    assert row == untouched
    np.testing.assert_allclose([float(output[f"{axis}_workspace"]) for axis in "xyz"], [.2, .1, .02])
    assert [output[f] for f in ("Fx", "Fy", "Fz")] == ["1", "2", "3"]
    assert output["force_frame"] == "existing_processed_wrench"
    assert output["inside_workspace"] == "1"
    assert logger.snapshot_path.exists()


def test_first_threshold_per_probe_and_boundary_order(calibration, tmp_path):
    logger = WorkspaceLogger(tmp_path / "out", calibration)
    row = sample()
    for contact, probe in [(0, 1), (1, 1), (1, 1), (0, 1), (1, 1), (1, 2)]:
        logger.log_sample({**row, "contact_flag": contact, "probe_id": probe})
    # Initialization acceptance order is not chronological contact order.
    for point, when in [("P0", 20), ("P1", 10)]:
        logger.log_boundary({**row, "point_id": point, "monotonic_sec": when})
    logger.close(render=False)
    rows = read_rows(logger.csv_path)
    assert [r["contact_point"] for r in rows[:6]] == ["0", "1", "0", "0", "0", "1"]
    assert [r["boundary_point_id"] for r in rows[6:]] == ["P0", "P1"]
    assert [r["monotonic_sec"] for r in rows[6:]] == ["20", "10"]
    assert all(r["boundary_state"] == "ACCEPTED" for r in rows[6:])


def test_outside_points_logged_without_clamping(calibration, tmp_path):
    logger = WorkspaceLogger(tmp_path / "out", calibration)
    logger.log_sample({**sample(), "tcp_y": 3})
    logger.close(render=False)
    row = read_rows(logger.csv_path)[0]
    assert row["inside_workspace"] == "0"
    assert float(row["x_workspace"]) == pytest.approx(1)


def test_real_log_without_z_rejected_but_optional_layer_does_not_raise(calibration, tmp_path, capsys):
    logger = WorkspaceLogger(tmp_path / "out", calibration)
    optional = OptionalWorkspaceLogger(logger)
    row = sample()
    del row["tcp_z"]
    optional.log_sample(row)
    optional.log_sample(sample())
    optional.close()
    assert optional.logger is None
    assert "missing z" in capsys.readouterr().err
    assert logger._file.closed


def test_missing_or_invalid_calibration_disables_only_side_output(tmp_path, capsys):
    assert create_optional_workspace_logger(tmp_path, tmp_path / "absent.yaml") is None
    assert export_if_calibrated(tmp_path, tmp_path / "absent.yaml") is None
    bad = tmp_path / "bad.yaml"
    bad.write_text("rotation: invalid")
    assert create_optional_workspace_logger(tmp_path, bad) is None
    assert "workspace" in capsys.readouterr().err
    assert not (tmp_path / "workspace_scan.csv").exists()


def test_export_real_contacts_uses_episode_pose_without_invented_timestamp(calibration, tmp_path):
    source = tmp_path / "old_run"
    source.mkdir()
    write_rows(source / "samples.csv", [{**sample(), "contact_flag": 1}])
    write_rows(source / "boundary_points.csv", [{**sample(), "point_id": "P0"}])
    (source / "probe_episodes.json").write_text(json.dumps([
        {"probe_id": 1, "contact_pose": [.9, 2.2, .12, 0, 0, 0], "policy_state": "TRACK", "accepted_as_boundary": True, "outcome": "CONTACT"},
        {"probe_id": 2, "contact_pose": [.8, 2.3, .11, 0, 0, 0], "policy_state": "RECOVERY", "outcome": "CONTACT"},
        {"probe_id": 3, "contact_pose": [.8, 2.3, .11, 0, 0, 0], "policy_state": "TRACK", "outcome": "ABORTED"},
    ]))
    original = {p.name: p.read_bytes() for p in source.iterdir()}
    path = export_existing_run(source, calibration, tmp_path / "out", render=False)
    rows = read_rows(path)
    assert [r["record_type"] for r in rows] == ["sample", "boundary", "contact", "contact"]
    assert rows[2]["monotonic_sec"] == "12"
    assert rows[2]["boundary_state"] == "ACCEPTED"
    assert rows[3]["monotonic_sec"] == "" and rows[3]["Fx"] == ""
    assert {p.name: p.read_bytes() for p in source.iterdir()} == original


def test_simulation_log_schema_maps_missing_height_to_its_existing_zero_plane(calibration, tmp_path):
    source = tmp_path / "sim"
    source.mkdir()
    write_rows(source / "simulation_log.csv", [dict(tcp_x=.9, tcp_y=2.2, simulation_time_sec=1,
                                                   state="SEARCH", contact_flag=0, dfx=1, dfy=2, dfz=0)])
    write_rows(source / "boundary_points.csv", [dict(x=.9, y=2.2, simulation_time_sec=.9, point_id="P0")])
    path = export_existing_run(source, calibration, render=False)
    rows = read_rows(path)
    assert len(rows) == 2
    assert rows[0]["policy_state"] == "SEARCH"
    assert rows[0]["time_domain"] == "simulation"
    assert float(rows[0]["z_base"]) == 0
    assert float(rows[0]["z_workspace"]) == pytest.approx(-.1)
    assert rows[1]["boundary_point_id"] == "P0"


def test_legacy_contact_flag_without_probe_id_uses_rising_edges(calibration, tmp_path):
    logger = WorkspaceLogger(tmp_path / "out", calibration)
    for flag in (0, 1, 1, 0, 1):
        logger.log_sample({**sample(), "probe_id": "", "contact_flag": flag})
    logger.close(render=False)
    assert [r["contact_point"] for r in read_rows(logger.csv_path)] == ["0", "1", "0", "0", "1"]


def test_real_logger_optional_hook_retains_original_schema(calibration, tmp_path, monkeypatch):
    import experiment_logging.data_logger as original
    import workspace.workspace_logger as side
    project = tmp_path / "project"
    active = project / "workspace/config/workspace_calibration.yaml"
    active.parent.mkdir(parents=True)
    active.write_bytes(calibration.read_bytes())
    monkeypatch.setattr(original, "__file__", str(project / "experiment_logging/data_logger.py"))
    monkeypatch.setattr(side, "DEFAULT_CALIBRATION_PATH", active)
    logger = ExperimentLogger(tmp_path / "runs", {"test": True})
    assert logger._workspace_logger is not None
    wrench = Wrench(1, 2, 3, 0, 0, 0)
    pose = np.array([.9, 2.2, .12, 0, 0, 0])
    command = PolicyCommand("SEARCH", False, np.zeros(2), 0., pose.copy(), False, False, "test")
    logger.log_sample(12, wrench, wrench, RobotState(12, pose, np.zeros(6)), command, None, None)
    logger.flush()
    logger.close()
    assert list(read_rows(logger.run_dir / "samples.csv")[0]) == original.SAMPLE_FIELDS
    assert read_rows(logger.run_dir / "workspace_scan.csv")[0]["policy_state"] == "SEARCH"
    assert (logger.run_dir / "workspace_contour.png").is_file()


def test_optional_plot_failure_preserves_main_logger(calibration, tmp_path, monkeypatch, capsys):
    import workspace.workspace_visualizer as visualizer
    def fail(*args, **kwargs):
        raise RuntimeError("plot failure")
    monkeypatch.setattr(visualizer, "visualize_workspace", fail)
    logger = WorkspaceLogger(tmp_path / "out", calibration)
    adapter = OptionalWorkspaceLogger(logger)
    adapter.log_sample(sample())
    adapter.close()
    assert logger.csv_path.is_file() and logger._file.closed
    assert "plot failure" in capsys.readouterr().err


def test_reexport_requires_explicit_overwrite(calibration, tmp_path):
    logger = WorkspaceLogger(tmp_path / "out", calibration)
    logger.close(render=False)
    with pytest.raises(FileExistsError):
        WorkspaceLogger(tmp_path / "out", calibration)

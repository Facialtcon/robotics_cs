"""Small offline integration checks for optional simulation workspace exports."""

import csv
import json
from pathlib import Path

import numpy as np
import pytest

from simulation import simulator as simulation_module
from workspace import workspace_logger, workspace_visualizer
from workspace.workspace_calibrator import fit_workspace, save_calibration
from workspace.workspace_transform import WorkspaceTransform


ROOT = Path(__file__).resolve().parents[1]
PRIMARY_OUTPUTS = (
    "simulation_log.csv", "boundary_points.csv", "probe_episodes.json",
    "simulation_config_snapshot.yaml", "simulation_result.png", "summary.json",
    "termination.json", "termination.txt",
)


def read_rows(path):
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def control_snapshot(simulator):
    """Exclude output bookkeeping; capture the existing simulation/control data."""
    return json.dumps({
        "tcp_pose": simulator.robot.pose.tolist(),
        "tcp_speed": simulator.robot.tcp_speed.tolist(),
        "simulation_time": simulator.robot.time,
        "policy_state": simulator.policy.state.value,
        "policy_sub_state": simulator.policy.sub_state,
        "boundary_points": [vars(point) for point in simulator.policy.boundary_points],
        "stopped": simulator.stopped,
        "stop_reason": simulator.stop_reason,
        "final_policy_state": simulator.final_policy_state,
        "physical_failure": simulator.physical_failure,
        "loop_completed": simulator.loop_completed,
        "termination": simulator.termination.record,
        "records": simulator.records,
        "trajectory": simulator.trajectory,
    }, sort_keys=True, default=lambda value: value.tolist())


@pytest.fixture
def short_simulation(tmp_path, monkeypatch):
    project = tmp_path / "project"
    calibration_path = project / "workspace/config/workspace_calibration.yaml"
    points = [[.04, .06, .1, 0, 0, 0], [-.36, .06, .1, 0, 0, 0],
              [-.36, -.19, .1, 0, 0, 0], [.04, -.19, .1, 0, 0, 0]]
    calibration = fit_workspace(dict(zip(("P0", "P1", "P2", "P3"), points)))
    save_calibration(calibration, calibration_path)
    # Enable the same optional-file gate as production without creating or
    # activating a calibration in the actual project.
    monkeypatch.setattr(simulation_module, "__file__", str(project / "simulation/simulator.py"))
    monkeypatch.setattr(workspace_logger, "DEFAULT_CALIBRATION_PATH", calibration_path)
    config = simulation_module.load_simulation_config(
        ROOT / "simulation/scene_1_axis_aligned_rectangle.yaml"
    )
    config["simulation"]["output_root"] = str((tmp_path / "results").resolve())
    config["visualization"]["save_animation"] = False
    config["visualization"]["debug_plots"] = False
    simulator = simulation_module.ContourSimulator(config)
    simulator.termination.bind(tmp_path / "before_finalization")
    start = simulator.robot.pose.copy()
    simulator.run_headless(12)
    assert len(simulator.records) == 12
    assert not np.array_equal(simulator.robot.pose, start)
    assert simulator.stopped
    assert simulator.stop_reason == "headless step limit reached (12)"
    assert simulator.termination.record["reason"] == "STOP_TIME_LIMIT"
    return simulator, WorkspaceTransform(calibration)


def assert_primary_outputs_unchanged(simulator, output, snapshot):
    assert simulator.finalized
    assert control_snapshot(simulator) == snapshot
    assert all((output / filename).is_file() for filename in PRIMARY_OUTPUTS)
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    termination = json.loads((output / "termination.json").read_text(encoding="utf-8"))
    assert summary["stop_reason"] == simulator.stop_reason
    assert summary["final_policy_state"] == simulator.final_policy_state
    assert summary["termination_reason"] == termination["reason"] == "STOP_TIME_LIMIT"
    assert summary["steps"] == 12
    assert summary["animation"] == "disabled"
    assert summary["scan_strategy_debug"] is None
    assert len(read_rows(output / "simulation_log.csv")) == 12


def test_finalization_adds_workspace_outputs_without_changing_simulation(short_simulation):
    simulator, transform = short_simulation
    snapshot = control_snapshot(simulator)
    output = simulator.finalize()
    assert_primary_outputs_unchanged(simulator, output, snapshot)
    expected = ("workspace_scan.csv", "workspace_calibration_snapshot.yaml",
                "workspace_summary.json", "workspace_contour.png")
    assert all((output / filename).is_file() for filename in expected)
    assert (output / "workspace_contour.png").read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    original = read_rows(output / "simulation_log.csv")
    projected = read_rows(output / "workspace_scan.csv")
    samples = [row for row in projected if row["record_type"] == "sample"]
    assert len(samples) == len(original)
    for sample, source in zip(samples, original):
        measured_base = np.array([float(source["tcp_x"]), float(source["tcp_y"]), 0.])
        np.testing.assert_allclose([float(sample[f"{axis}_base"]) for axis in "xyz"], measured_base)
        np.testing.assert_allclose([float(sample[f"{axis}_workspace"]) for axis in "xyz"],
                                   transform.base_to_workspace(measured_base))
        assert sample["calibration_id"] == transform.calibration_id
        assert sample["policy_state"] == source["state"]
        assert sample["monotonic_sec"] == source["simulation_time_sec"]
        assert sample["time_domain"] == "simulation"
        assert [sample[key] for key in ("Fx", "Fy", "Fz")] == [source[key] for key in ("dfx", "dfy", "dfz")]
    # Existing finalization idempotency must not append duplicate side records.
    csv_before = (output / "workspace_scan.csv").read_bytes()
    assert simulator.finalize() == output
    assert (output / "workspace_scan.csv").read_bytes() == csv_before
    assert control_snapshot(simulator) == snapshot


@pytest.mark.parametrize("failure_at", ["exporter", "plot"])
def test_optional_export_failure_preserves_stop_reason_and_primary_output(
    short_simulation, monkeypatch, capsys, failure_at
):
    simulator, _ = short_simulation
    snapshot = control_snapshot(simulator)

    def fail(*args, **kwargs):
        raise RuntimeError(f"optional workspace {failure_at} unavailable")

    if failure_at == "exporter":
        # Exercise the simulator's outer guard as well as the helper's own guard.
        monkeypatch.setattr(workspace_logger, "export_if_calibrated", fail)
    else:
        monkeypatch.setattr(workspace_visualizer, "visualize_workspace", fail)
    output = simulator.finalize()
    assert_primary_outputs_unchanged(simulator, output, snapshot)
    assert not (output / "workspace_contour.png").exists()
    if failure_at == "plot":
        assert len(read_rows(output / "workspace_scan.csv")) == 12
    messages = capsys.readouterr()
    assert f"optional workspace {failure_at} unavailable" in messages.out + messages.err

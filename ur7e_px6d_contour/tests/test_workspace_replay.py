"""Offline replay of an existing experiment; the box here is explicitly synthetic."""
import csv
import hashlib
from pathlib import Path

import numpy as np
import pytest

from workspace.workspace_calibrator import fit_workspace, save_calibration
from workspace.workspace_logger import export_existing_run
from workspace.workspace_transform import WorkspaceTransform


ROOT = Path(__file__).resolve().parents[1]
HISTORICAL_RUN = ROOT / "data/run_20260914_023254_164222"


def demo_calibration():
    """Known 15-degree box with XY/Z/TCP-orientation teaching noise, NOT real teaching."""
    theta = np.deg2rad(15.)
    rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    local = np.array([[0, 0], [.14, 0], [.14, .18], [0, .18]])
    xy = local @ rotation.T + [.57, .075]
    xy += [[.0004, -.0006], [-.0002, .0003], [.0007, -.0002], [-.0004, .0005]]
    poses = np.zeros((4, 6))
    poses[:, :2] = xy
    poses[:, 2] = .0934 + np.array([.002, -.001, .003, -.004])
    poses[:, 3:] = [[1.50, -2.75, .01], [1.54, -2.74, -.02], [1.49, -2.77, .02], [1.51, -2.73, -.01]]
    return fit_workspace(dict(zip(("P0", "P1", "P2", "P3"), poses)), metadata={
        "input_mode": "synthetic_offline_validation",
        "synthetic_demo": True,
        "warning": "Synthetic box enclosing historical TCP positions; not a measured sandbox calibration.",
        "historical_run": HISTORICAL_RUN.name,
        "active_tcp_verified": False,
    })


def test_historical_experiment_projects_without_flip_or_data_changes(tmp_path):
    source = HISTORICAL_RUN / "samples.csv"
    if not source.exists():
        pytest.skip("historical experiment dataset is not distributed with this checkout")
    original_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    calibration = save_calibration(demo_calibration(), tmp_path / "demo_calibration.yaml")
    transform = WorkspaceTransform.from_file(calibration)
    output = export_existing_run(HISTORICAL_RUN, calibration, tmp_path)
    with output.open(newline="", encoding="utf-8") as handle:
        records = list(csv.DictReader(handle))
    trajectory = [r for r in records if r["record_type"] == "sample"]
    with source.open(newline="", encoding="utf-8") as handle:
        base = np.array([[float(r[f"tcp_{a}"]) for a in "xyz"] for r in csv.DictReader(handle)])
    local = np.array([[float(r[f"{a}_workspace"]) for a in "xyz"] for r in trajectory])
    assert len(base) == len(local) > 100
    np.testing.assert_allclose(local, transform.base_to_workspace(base), atol=1e-12)
    np.testing.assert_allclose(transform.workspace_to_base(local), base, atol=1e-12)
    assert np.all(local[:, :2] >= 0)
    assert np.all(local[:, 0] <= transform.calibration["length"])
    assert np.all(local[:, 1] <= transform.calibration["width"])
    assert any(r["record_type"] == "contact" for r in records)
    assert any(r["record_type"] == "boundary" for r in records)
    assert (tmp_path / "workspace_contour.png").stat().st_size > 1000
    assert hashlib.sha256(source.read_bytes()).hexdigest() == original_hash

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest
import yaml

from calibration.scan_calibration import (
    CalibrationError,
    build_scan_calibration,
    calculate_scan_direction_xy,
    load_scan_calibration,
    validate_direction_calibration,
)


P0 = [0.10, -0.20, 0.30, 0.0, 3.14, 0.0]
P1 = [0.13, -0.16, 0.302, 0.1, 3.10, -0.1]
TCP_OFFSET = [0.0, 0.0, 0.25, 0.0, 0.0, 0.0]
ROOT = Path(__file__).resolve().parents[1]


def test_calculate_scan_direction_xy_is_shared_with_two_dimensional_simulation():
    direction = calculate_scan_direction_xy([1.0, 2.0], [4.0, 6.0])
    assert np.allclose(direction, [0.6, 0.8])


def test_build_and_load_complete_two_point_calibration(tmp_path):
    payload = build_scan_calibration(
        P0,
        P1,
        "192.168.1.10",
        "2026-09-07T00:00:00Z",
        active_tcp_offset=TCP_OFFSET,
    )
    path = tmp_path / "scan_calibration.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")

    loaded = load_scan_calibration(path)

    assert loaded["start_tcp_pose"] == P0
    assert loaded["direction_reference_tcp_pose"] == P1
    assert np.allclose(loaded["scan_direction_xy"], [0.6, 0.8])
    assert loaded["fixed_z"] == P0[2]
    assert loaded["fixed_orientation"] == P0[3:]
    assert loaded["active_tcp_offset"] == TCP_OFFSET


def test_execute_gate_rejects_legacy_calibration_without_tcp_binding(tmp_path):
    payload = build_scan_calibration(P0, P1, "192.168.1.10")
    path = tmp_path / "scan_calibration.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")

    with pytest.raises(CalibrationError, match="active_tcp_offset"):
        load_scan_calibration(path, require_tcp_offset=True)


@pytest.mark.parametrize(
    "missing_key",
    ["start_tcp_pose", "direction_reference_tcp_pose", "scan_direction_xy"],
)
def test_execute_calibration_gate_rejects_missing_required_item(tmp_path, missing_key):
    payload = build_scan_calibration(
        P0, P1, "192.168.1.10", active_tcp_offset=TCP_OFFSET
    )
    payload.pop(missing_key)
    path = tmp_path / "scan_calibration.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")

    with pytest.raises(CalibrationError):
        load_scan_calibration(path)


def test_loader_rejects_direction_not_matching_p0_to_p1(tmp_path):
    payload = build_scan_calibration(P0, P1, "192.168.1.10")
    invalid = deepcopy(payload)
    invalid["scan_direction_xy"] = {"dx": 1.0, "dy": 0.0}
    path = tmp_path / "scan_calibration.yaml"
    path.write_text(yaml.safe_dump(invalid, sort_keys=False), encoding="utf-8")

    with pytest.raises(CalibrationError, match="inconsistent"):
        load_scan_calibration(path)


def test_direction_calibration_rejects_short_xy_distance_and_large_z_change():
    with pytest.raises(CalibrationError, match="XY 距离"):
        validate_direction_calibration(P0, [0.101, -0.20, 0.30, 0, 0, 0], 0.02, 0.005)
    with pytest.raises(CalibrationError, match="不应明显改变插入深度"):
        validate_direction_calibration(P0, [0.13, -0.16, 0.31, 0, 0, 0], 0.02, 0.005)


def test_read_only_calibration_preview_tool_generates_png(tmp_path):
    config = yaml.safe_load((ROOT / "config.yaml").read_text(encoding="utf-8"))
    config["calibration"]["file"] = "scan_calibration.yaml"
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    payload = build_scan_calibration(
        P0, P1, "192.168.1.10", active_tcp_offset=TCP_OFFSET
    )
    (tmp_path / "scan_calibration.yaml").write_text(
        yaml.safe_dump(payload, sort_keys=False), encoding="utf-8"
    )
    output = tmp_path / "preview.png"

    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "tools" / "check_scan_calibration.py"),
            "--config",
            str(config_path),
            "--output",
            str(output),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert output.is_file() and output.stat().st_size > 0
    assert "did not connect to RTDE" in result.stdout

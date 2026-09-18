from __future__ import annotations

import pytest
import yaml

from calibration.reset_pose import (
    ResetPoseError,
    build_reset_pose,
    load_reset_pose,
    validate_reset_pose_binding,
)
from calibration.scan_calibration import build_scan_calibration
from return_to_reset import load_return_target


POSE = [0.4, -0.2, 0.3, 0.0, 3.14, 0.0]
TCP = [0.0, 0.0, 0.25, 0.0, 0.0, -0.785]


def test_reset_pose_round_trip_includes_tcp_identity(tmp_path):
    payload = build_reset_pose(POSE, TCP, "192.168.1.10", "2026-09-14T00:00:00Z")
    path = tmp_path / "reset_pose.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")

    loaded = load_reset_pose(path)

    assert loaded["reset_tcp_pose"] == POSE
    assert loaded["active_tcp_offset"] == TCP
    validate_reset_pose_binding(loaded, "192.168.1.10", TCP, 1e-4)


def test_reset_pose_rejects_wrong_active_tcp(tmp_path):
    payload = build_reset_pose(POSE, TCP, "192.168.1.10")
    path = tmp_path / "reset_pose.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    loaded = load_reset_pose(path)

    with pytest.raises(ResetPoseError, match="does not match"):
        validate_reset_pose_binding(
            loaded, "192.168.1.10", [0, 0, 0.20, 0, 0, 0], 1e-4
        )


def test_missing_reset_pose_falls_back_to_saved_scan_p0(tmp_path):
    calibration = build_scan_calibration(
        POSE, [0.43, -0.2, 0.3, 0.0, 3.14, 0.0], "192.168.1.10"
    )
    (tmp_path / "scan_calibration.yaml").write_text(
        yaml.safe_dump(calibration, sort_keys=False), encoding="utf-8"
    )
    config = {
        "robot": {"robot_ip": "192.168.1.10"},
        "tcp": {"offset": TCP},
        "calibration": {"file": "scan_calibration.yaml"},
        "reset": {"file": "reset_pose.yaml"},
    }

    target = load_return_target(str(tmp_path / "config.yaml"), config)

    assert target["label"] == "P0"
    assert target["pose"] == POSE
    assert target["active_tcp_offset"] == TCP
    assert target["legacy_tcp_binding"] is True


def test_explicit_reset_pose_takes_precedence_over_scan_p0(tmp_path):
    reset = build_reset_pose(POSE, TCP, "192.168.1.10")
    (tmp_path / "reset_pose.yaml").write_text(
        yaml.safe_dump(reset, sort_keys=False), encoding="utf-8"
    )
    config = {
        "robot": {"robot_ip": "192.168.1.10"},
        "tcp": {"offset": [0, 0, 0.1, 0, 0, 0]},
        "calibration": {"file": "missing.yaml"},
        "reset": {"file": "reset_pose.yaml"},
    }

    target = load_return_target(str(tmp_path / "config.yaml"), config)

    assert target["label"] == "RESET"
    assert target["active_tcp_offset"] == TCP

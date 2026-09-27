"""TCP-bound reset-pose persistence and strict validation."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

from calibration.scan_calibration import POSE_FIELDS, pose_to_mapping
from robot.tcp_identity import normalize_tcp_offset, tcp_offsets_match


class ResetPoseError(ValueError):
    """Raised when a reset pose is absent or cannot safely identify its TCP."""


def resolve_reset_pose_path(config_path: str | Path, configured_path: str | Path) -> Path:
    path = Path(configured_path).expanduser()
    if path.is_absolute():
        return path
    return Path(config_path).expanduser().resolve().parent / path


def _pose_from_mapping(value: Any, name: str) -> list[float]:
    if not isinstance(value, Mapping):
        raise ResetPoseError(f"{name} must be a mapping")
    try:
        pose = np.asarray([value[field] for field in POSE_FIELDS], dtype=float)
    except (KeyError, TypeError, ValueError) as exc:
        raise ResetPoseError(f"{name} must contain x, y, z, rx, ry, rz") from exc
    if pose.shape != (6,) or not np.all(np.isfinite(pose)):
        raise ResetPoseError(f"{name} must contain six finite values")
    return pose.tolist()


def build_reset_pose(
    reset_tcp_pose: Sequence[float],
    active_tcp_offset: Sequence[float],
    robot_ip: str,
    timestamp: str | None = None,
) -> dict[str, Any]:
    pose = _pose_from_mapping(pose_to_mapping(reset_tcp_pose), "reset_tcp_pose")
    offset = normalize_tcp_offset(active_tcp_offset, "active_tcp_offset")
    return {
        "confirmed": True,
        "robot_ip": str(robot_ip),
        "active_tcp_offset": offset,
        "reset_tcp_pose": pose_to_mapping(pose),
        "saved_at": timestamp or datetime.now(timezone.utc).isoformat(),
    }


def write_reset_pose(path: str | Path, payload: Mapping[str, Any]) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(dict(payload), handle, allow_unicode=True, sort_keys=False)


def load_reset_pose(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    if not source.is_file():
        raise ResetPoseError(
            f"reset pose file not found: {source}; run python3 save_reset_pose.py first"
        )
    with source.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, Mapping):
        raise ResetPoseError("reset pose must be a YAML mapping")
    if data.get("confirmed") is not True:
        raise ResetPoseError("reset pose is not confirmed")
    pose = _pose_from_mapping(data.get("reset_tcp_pose"), "reset_tcp_pose")
    try:
        offset = normalize_tcp_offset(data.get("active_tcp_offset"), "active_tcp_offset")
    except ValueError as exc:
        raise ResetPoseError(str(exc)) from exc
    robot_ip = data.get("robot_ip")
    if not isinstance(robot_ip, str) or not robot_ip.strip():
        raise ResetPoseError("robot_ip must be present")
    saved_at = data.get("saved_at")
    if not isinstance(saved_at, str) or not saved_at.strip():
        raise ResetPoseError("saved_at must be present")
    return {
        "confirmed": True,
        "robot_ip": robot_ip,
        "active_tcp_offset": offset,
        "reset_tcp_pose": pose,
        "saved_at": saved_at,
        "source_file": str(source.resolve()),
    }


def validate_reset_pose_binding(
    reset_pose: Mapping[str, Any],
    robot_ip: str,
    active_tcp_offset: Sequence[float],
    tolerance: float,
) -> None:
    if reset_pose.get("robot_ip") != robot_ip:
        raise ResetPoseError("reset pose robot_ip does not match config.yaml")
    if not tcp_offsets_match(
        reset_pose.get("active_tcp_offset"), active_tcp_offset, tolerance
    ):
        raise ResetPoseError(
            "active TCP does not match the TCP bound to reset_pose.yaml; "
            "select the original TCP or save a new reset pose"
        )

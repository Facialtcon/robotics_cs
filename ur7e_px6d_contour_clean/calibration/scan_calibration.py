"""Two-point scan calibration, validation, persistence, and geometry."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

from robot.tcp_identity import normalize_tcp_offset


POSE_FIELDS = ("x", "y", "z", "rx", "ry", "rz")
DIRECTION_FIELDS = ("dx", "dy")


class CalibrationError(ValueError):
    """Raised when a scan calibration is missing or internally inconsistent."""


def resolve_calibration_path(config_path: str | Path, configured_path: str | Path) -> Path:
    """Resolve a calibration path relative to the configuration file."""
    path = Path(configured_path).expanduser()
    if path.is_absolute():
        return path
    return Path(config_path).expanduser().resolve().parent / path


def _finite_vector(value: Sequence[float], size: int, name: str) -> np.ndarray:
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise CalibrationError(f"{name} must contain {size} finite values") from exc
    if vector.shape != (size,) or not np.all(np.isfinite(vector)):
        raise CalibrationError(f"{name} must contain {size} finite values")
    return vector


def _mapping_vector(value: Any, fields: Sequence[str], name: str) -> np.ndarray:
    if not isinstance(value, Mapping):
        raise CalibrationError(f"{name} must be a mapping containing {', '.join(fields)}")
    missing = [field for field in fields if field not in value]
    if missing:
        raise CalibrationError(f"{name} is missing: {', '.join(missing)}")
    return _finite_vector([value[field] for field in fields], len(fields), name)


def pose_to_mapping(pose: Sequence[float]) -> dict[str, float]:
    vector = _finite_vector(pose, 6, "TCP pose")
    return {field: float(value) for field, value in zip(POSE_FIELDS, vector)}


def direction_to_mapping(direction: Sequence[float]) -> dict[str, float]:
    vector = _finite_vector(direction, 2, "scan direction")
    return {field: float(value) for field, value in zip(DIRECTION_FIELDS, vector)}


def calculate_scan_direction_xy(
    calibration_point_0: Sequence[float], calibration_point_1: Sequence[float]
) -> np.ndarray:
    """Return the normalized XY direction from P0 to P1."""
    try:
        point_0 = np.asarray(calibration_point_0, dtype=float)
        point_1 = np.asarray(calibration_point_1, dtype=float)
    except (TypeError, ValueError) as exc:
        raise CalibrationError(
            "P0 and P1 must each contain at least two finite values"
        ) from exc
    if (
        point_0.ndim != 1
        or point_1.ndim != 1
        or point_0.size < 2
        or point_1.size < 2
        or not np.all(np.isfinite(point_0))
        or not np.all(np.isfinite(point_1))
    ):
        raise CalibrationError("P0 and P1 must each contain at least two finite values")
    delta = point_1[:2] - point_0[:2]
    distance = float(np.linalg.norm(delta))
    if distance <= 0.0:
        raise CalibrationError("P0 and P1 do not define an XY scan direction")
    return delta / distance


def validate_direction_calibration(
    calibration_point_0: Sequence[float],
    calibration_point_1: Sequence[float],
    min_distance: float,
    max_z_difference: float,
) -> tuple[np.ndarray, float, float]:
    """Validate teach points and return direction, XY distance and absolute Z delta."""
    if min_distance <= 0.0:
        raise CalibrationError("min_direction_calibration_distance must be positive")
    if max_z_difference < 0.0:
        raise CalibrationError("max_direction_calibration_z_difference cannot be negative")
    point_0 = _finite_vector(calibration_point_0, 6, "P0")
    point_1 = _finite_vector(calibration_point_1, 6, "P1")
    xy_distance = float(np.linalg.norm(point_1[:2] - point_0[:2]))
    z_difference = float(abs(point_1[2] - point_0[2]))
    if xy_distance < min_distance:
        raise CalibrationError(
            f"P0 与 P1 的 XY 距离 {xy_distance:.6f} m 小于最小值 "
            f"{min_distance:.6f} m，请重新移动 P1。"
        )
    if z_difference > max_z_difference:
        raise CalibrationError(
            "P1 应主要用于标定 XY 扫描方向，不应明显改变插入深度。"
            f" 当前 Z 差值为 {z_difference:.6f} m，允许最大值为 "
            f"{max_z_difference:.6f} m。"
        )
    direction = calculate_scan_direction_xy(point_0, point_1)
    return direction, xy_distance, z_difference


def build_scan_calibration(
    start_tcp_pose: Sequence[float],
    direction_reference_tcp_pose: Sequence[float],
    robot_ip: str,
    timestamp: str | None = None,
    *,
    active_tcp_offset: Sequence[float] | None = None,
) -> dict[str, Any]:
    """Build the on-disk mapping after teach-point validation succeeds."""
    point_0 = _finite_vector(start_tcp_pose, 6, "P0")
    point_1 = _finite_vector(direction_reference_tcp_pose, 6, "P1")
    direction = calculate_scan_direction_xy(point_0, point_1)
    payload = {
        "confirmed": True,
        "robot_ip": str(robot_ip),
        "start_tcp_pose": pose_to_mapping(point_0),
        "direction_reference_tcp_pose": pose_to_mapping(point_1),
        "scan_direction_xy": direction_to_mapping(direction),
        "fixed_z": float(point_0[2]),
        "fixed_orientation": {
            "rx": float(point_0[3]),
            "ry": float(point_0[4]),
            "rz": float(point_0[5]),
        },
        "calibration_timestamp": timestamp or datetime.now(timezone.utc).isoformat(),
    }
    if active_tcp_offset is not None:
        payload["active_tcp_offset"] = normalize_tcp_offset(
            active_tcp_offset, "active_tcp_offset"
        )
    return payload


def write_scan_calibration(path: str | Path, payload: Mapping[str, Any]) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(dict(payload), handle, allow_unicode=True, sort_keys=False)


def load_scan_calibration(
    path: str | Path, *, require_tcp_offset: bool = False
) -> dict[str, Any]:
    """Load, strictly validate and normalize a complete two-point calibration."""
    source = Path(path)
    if not source.is_file():
        raise CalibrationError(f"scan calibration file not found: {source}")
    with source.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, Mapping):
        raise CalibrationError("scan calibration must be a YAML mapping")
    if data.get("confirmed") is not True:
        raise CalibrationError("scan calibration is not confirmed")

    point_0 = _mapping_vector(data.get("start_tcp_pose"), POSE_FIELDS, "start_tcp_pose")
    point_1 = _mapping_vector(
        data.get("direction_reference_tcp_pose"),
        POSE_FIELDS,
        "direction_reference_tcp_pose",
    )
    stored_direction = _mapping_vector(
        data.get("scan_direction_xy"), DIRECTION_FIELDS, "scan_direction_xy"
    )
    stored_norm = float(np.linalg.norm(stored_direction))
    if not np.isclose(stored_norm, 1.0, atol=1e-6):
        raise CalibrationError("scan_direction_xy must be a unit vector")
    calculated_direction = calculate_scan_direction_xy(point_0, point_1)
    if not np.allclose(stored_direction, calculated_direction, atol=1e-6):
        raise CalibrationError("scan_direction_xy is inconsistent with P0 -> P1")

    try:
        fixed_z = float(data["fixed_z"])
    except (KeyError, TypeError, ValueError) as exc:
        raise CalibrationError("fixed_z must be present and finite") from exc
    if not np.isfinite(fixed_z) or not np.isclose(fixed_z, point_0[2], atol=1e-9):
        raise CalibrationError("fixed_z must equal P0.z")
    fixed_orientation = _mapping_vector(
        data.get("fixed_orientation"), ("rx", "ry", "rz"), "fixed_orientation"
    )
    if not np.allclose(fixed_orientation, point_0[3:], atol=1e-9):
        raise CalibrationError("fixed_orientation must equal the P0 orientation")
    timestamp = data.get("calibration_timestamp")
    if not isinstance(timestamp, str) or not timestamp.strip():
        raise CalibrationError("calibration_timestamp must be present")

    active_tcp_offset = data.get("active_tcp_offset")
    if active_tcp_offset is None:
        if require_tcp_offset:
            raise CalibrationError(
                "scan calibration has no active_tcp_offset binding; "
                "select/verify the probe TCP and run python3 run_calibration.py again"
            )
    else:
        try:
            active_tcp_offset = normalize_tcp_offset(
                active_tcp_offset, "active_tcp_offset"
            )
        except ValueError as exc:
            raise CalibrationError(str(exc)) from exc

    return {
        "confirmed": True,
        "robot_ip": data.get("robot_ip"),
        "start_tcp_pose": point_0.tolist(),
        "direction_reference_tcp_pose": point_1.tolist(),
        "scan_direction_xy": stored_direction.tolist(),
        "fixed_z": fixed_z,
        "fixed_orientation": fixed_orientation.tolist(),
        "calibration_timestamp": timestamp,
        "active_tcp_offset": active_tcp_offset,
        "source_file": str(source.resolve()),
    }


def validate_calibration_constraints(
    calibration: Mapping[str, Any], min_distance: float, max_z_difference: float
) -> None:
    """Apply current config constraints to an already loaded calibration."""
    direction, _, _ = validate_direction_calibration(
        calibration["start_tcp_pose"],
        calibration["direction_reference_tcp_pose"],
        min_distance,
        max_z_difference,
    )
    if not np.allclose(direction, calibration["scan_direction_xy"], atol=1e-6):
        raise CalibrationError("scan_direction_xy is inconsistent with P0 -> P1")

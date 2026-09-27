"""Fit a horizontal rectangular workspace from four ordered TCP positions.

The raw corner labels define the axes; fitting never reorders those labels or
uses the TCP orientation to rotate the workspace. Distances are in meters.
"""
from __future__ import annotations

import json
from collections.abc import Mapping
from pathlib import Path
import tempfile

import numpy as np
import yaml


POINT_NAMES = ("P0", "P1", "P2", "P3")
POSE_FIELDS = ("x", "y", "z", "rx", "ry", "rz")
MIN_EDGE_LENGTH_M = 1e-6
MIN_TURN_SINE = 1e-3


class WorkspaceCalibrationError(ValueError):
    """Invalid or internally inconsistent workspace geometry."""


def _pose(value, name):
    if isinstance(value, Mapping):
        if not all(axis in value for axis in ("x", "y", "z")):
            raise WorkspaceCalibrationError(f"{name} must contain x, y and z")
        value = [value.get(field, 0.0) for field in POSE_FIELDS]
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise WorkspaceCalibrationError(f"{name} must contain six finite TCP pose values") from exc
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise WorkspaceCalibrationError(f"{name} must contain six finite TCP pose values")
    return result


def _cross_xy(a, b):
    return a[0] * b[1] - a[1] * b[0]


def fit_workspace(raw_points, *, metadata=None) -> dict:
    """Fit P0→P1→P2→P3 without moving hardware or writing calibration files.

    Values are six-element TCP poses or dictionaries with x/y/z and optional
    rx/ry/rz (default zero). All original orientations are retained as metadata,
    but only XY positions determine axes and side lengths. CW and CCW samples
    both retain positive X toward P1 and positive Y toward P3. The right-handed
    Z axis therefore points down in the base frame for a CW corner order.
    """
    if not isinstance(raw_points, Mapping) or set(raw_points) != set(POINT_NAMES):
        raise WorkspaceCalibrationError("raw_points must contain exactly P0, P1, P2 and P3 in boundary order")
    poses = np.asarray([_pose(raw_points[name], name) for name in POINT_NAMES])
    xy = poses[:, :2]
    for i in range(4):
        for j in range(i + 1, 4):
            if np.linalg.norm(xy[i] - xy[j]) <= MIN_EDGE_LENGTH_M:
                raise WorkspaceCalibrationError(f"{POINT_NAMES[i]} and {POINT_NAMES[j]} are duplicate or too close in XY")
    edges = np.roll(xy, -1, axis=0) - xy
    edge_lengths = np.linalg.norm(edges, axis=1)
    turns = np.asarray([_cross_xy(edges[i], edges[(i + 1) % 4]) /
                        (edge_lengths[i] * edge_lengths[(i + 1) % 4]) for i in range(4)])
    if np.any(np.abs(turns) <= MIN_TURN_SINE):
        raise WorkspaceCalibrationError("workspace corners are collinear or nearly collinear")
    if not (np.all(turns > 0) or np.all(turns < 0)):
        raise WorkspaceCalibrationError("P0/P1/P2/P3 order is crossed or non-convex; keep consecutive boundary corners")

    x_xy = (xy[1] - xy[0]) / edge_lengths[0]
    toward_p3 = xy[3] - xy[0]
    orthogonal_y = toward_p3 - np.dot(toward_p3, x_xy) * x_xy
    perpendicular_length = float(np.linalg.norm(orthogonal_y))
    if perpendicular_length <= MIN_EDGE_LENGTH_M or perpendicular_length / edge_lengths[3] <= MIN_TURN_SINE:
        raise WorkspaceCalibrationError("P0→P1 and P0→P3 do not define independent workspace axes")
    y_xy = orthogonal_y / perpendicular_length
    x_axis = np.r_[x_xy, 0.0]
    y_axis = np.r_[y_xy, 0.0]
    z_axis = np.cross(x_axis, y_axis)
    normal_sign = 1 if z_axis[2] > 0 else -1
    # Explicit ±base Z avoids retaining round-off in the horizontal-plane axis.
    z_axis = np.array([0.0, 0.0, float(normal_sign)])
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    length = float((edge_lengths[0] + edge_lengths[2]) / 2.0)
    width = float((edge_lengths[1] + edge_lengths[3]) / 2.0)
    average_z = float(np.mean(poses[:, 2]))
    origin = np.r_[np.mean(xy, axis=0) - (length * x_xy + width * y_xy) / 2.0, average_z]
    local_corners = np.array([[0., 0., 0.], [length, 0., 0.],
                              [length, width, 0.], [0., width, 0.]])
    rectified = local_corners @ rotation.T + origin
    residuals = np.linalg.norm(rectified[:, :2] - xy, axis=1)
    raw_angle = float(np.degrees(np.arccos(np.clip(np.dot(x_xy, toward_p3 / edge_lengths[3]), -1., 1.))))
    if metadata is None:
        metadata = {}
    if not isinstance(metadata, Mapping):
        raise WorkspaceCalibrationError("metadata must be a mapping")
    try:
        metadata = json.loads(json.dumps(dict(metadata), allow_nan=False))
    except (TypeError, ValueError) as exc:
        raise WorkspaceCalibrationError("metadata must contain finite JSON-compatible values") from exc
    return {
        "schema_version": 1,
        "units": "m",
        "raw_points": {name: dict(zip(POSE_FIELDS, (float(v) for v in pose)))
                       for name, pose in zip(POINT_NAMES, poses)},
        "rectified_points": {f"R{i}": point.tolist() for i, point in enumerate(rectified)},
        "average_z": average_z, "length": length, "width": width,
        "origin": origin.tolist(), "x_axis": x_axis.tolist(),
        "y_axis": y_axis.tolist(), "z_axis": z_axis.tolist(),
        "rotation_workspace_to_base": rotation.tolist(),
        "translation_workspace_to_base": origin.tolist(),
        "rotation_base_to_workspace": rotation.T.tolist(),
        "translation_base_to_workspace": (-rotation.T @ origin).tolist(),
        "winding": "CCW" if normal_sign > 0 else "CW",
        "normal_sign": normal_sign,
        "diagnostics": {
            "fit_rms_xy_m": float(np.sqrt(np.mean(residuals ** 2))),
            "fit_max_xy_m": float(np.max(residuals)),
            "corner_residuals_xy_m": residuals.tolist(),
            "raw_z_range_m": float(np.ptp(poses[:, 2])),
            "raw_z_std_m": float(np.std(poses[:, 2])),
            "orthogonality_error_deg": abs(raw_angle - 90.0),
            "opposite_length_difference_m": float(abs(edge_lengths[0] - edge_lengths[2])),
            "opposite_width_difference_m": float(abs(edge_lengths[1] - edge_lengths[3])),
            "origin_correction_xy_m": float(np.linalg.norm(origin[:2] - xy[0])),
        },
        "metadata": metadata,
    }


def save_calibration(calibration, path) -> Path:
    """Validate and atomically save an explicitly requested calibration file."""
    from workspace.workspace_transform import validate_calibration

    normalized = validate_calibration(calibration)
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=destination.parent,
                                         prefix=f".{destination.name}.", suffix=".tmp", delete=False) as handle:
            temporary = Path(handle.name)
            yaml.safe_dump(normalized, handle, allow_unicode=True, sort_keys=False)
        temporary.replace(destination)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return destination

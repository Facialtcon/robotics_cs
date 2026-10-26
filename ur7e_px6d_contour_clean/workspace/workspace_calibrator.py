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


def fit_workspace(raw_points, *, metadata=None, method='legacy') -> dict:
    """Fit P0→P1→P2→P3 without moving hardware or writing calibration files.

    Values are six-element TCP poses or dictionaries with x/y/z and optional
    rx/ry/rz (default zero). All original orientations are retained as metadata,
    but only XY positions determine axes and side lengths. CW and CCW samples
    both retain positive X toward P1 and positive Y toward P3. The right-handed
    Z axis therefore points down in the base frame for a CW corner order.
    Old callers retain method='legacy'. Sandbox scanning explicitly selects
    method='least_squares' to optimize all four corresponding XY corners.
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

    toward_p3 = xy[3] - xy[0]
    if method == 'legacy':
        # Keep the historical fit and its serialized values for old modes.
        x_xy = (xy[1] - xy[0]) / edge_lengths[0]
        orthogonal_y = toward_p3 - np.dot(toward_p3, x_xy) * x_xy
        perpendicular_length = float(np.linalg.norm(orthogonal_y))
        if perpendicular_length <= MIN_EDGE_LENGTH_M or perpendicular_length / edge_lengths[3] <= MIN_TURN_SINE:
            raise WorkspaceCalibrationError("P0→P1 and P0→P3 do not define independent workspace axes")
        y_xy = orthogonal_y / perpendicular_length
        length = float((edge_lengths[0] + edge_lengths[2]) / 2.0)
        width = float((edge_lengths[1] + edge_lengths[3]) / 2.0)
    elif method == 'least_squares':
        # Corners = center + sx*a + sy*b, a perpendicular to b. The optimal
        # center is mean(XY). Orthogonal sign patterns reduce the remaining
        # objective to 4*(||a-u||^2 + ||b-v||^2) plus a constant. For any unit
        # x, a=(u.x)*x and b=(v.y)*y, with y=handedness*J*x. Maximizing the
        # two squared projections is a 2x2 symmetric eigenproblem: global
        # least squares, with no angle search or first-edge direction latch.
        u = (xy[1]+xy[2]-xy[0]-xy[3])/4.
        v = (xy[2]+xy[3]-xy[0]-xy[1])/4.
        handedness = float(np.sign(turns[0]))
        w = handedness*np.array([v[1], -v[0]])
        _, vectors = np.linalg.eigh(np.outer(u, u)+np.outer(w, w))
        x_xy = vectors[:, -1]
        if np.dot(u, x_xy) < 0:
            x_xy = -x_xy
        y_xy = handedness*np.array([-x_xy[1], x_xy[0]])
        length, width = float(2*np.dot(u, x_xy)), float(2*np.dot(v, y_xy))
        if min(length, width) <= MIN_EDGE_LENGTH_M:
            raise WorkspaceCalibrationError('least-squares rectangle is degenerate')
    else:
        raise WorkspaceCalibrationError(f'unknown rectangle fit method: {method}')
    x_axis = np.r_[x_xy, 0.0]
    y_axis = np.r_[y_xy, 0.0]
    z_axis = np.cross(x_axis, y_axis)
    normal_sign = 1 if z_axis[2] > 0 else -1
    # Explicit ±base Z avoids retaining round-off in the horizontal-plane axis.
    z_axis = np.array([0.0, 0.0, float(normal_sign)])
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    average_z = float(np.mean(poses[:, 2]))
    origin = np.r_[np.mean(xy, axis=0) - (length * x_xy + width * y_xy) / 2.0, average_z]
    local_corners = np.array([[0., 0., 0.], [length, 0., 0.],
                              [length, width, 0.], [0., width, 0.]])
    rectified = local_corners @ rotation.T + origin
    residuals = np.linalg.norm(rectified[:, :2] - xy, axis=1)
    raw_angle = float(np.degrees(np.arccos(np.clip(np.dot(edges[0]/edge_lengths[0], toward_p3 / edge_lengths[3]), -1., 1.))))
    if metadata is None:
        metadata = {}
    if not isinstance(metadata, Mapping):
        raise WorkspaceCalibrationError("metadata must be a mapping")
    try:
        metadata = json.loads(json.dumps(dict(metadata), allow_nan=False))
    except (TypeError, ValueError) as exc:
        raise WorkspaceCalibrationError("metadata must contain finite JSON-compatible values") from exc
    result = {
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
    if method != 'legacy':
        result['fit_method'] = method
    return result


def effective_scan_rectangle(fitted, boundary_margin):
    """Largest centered rectangle in the fitted frame and both inset regions.

    Hold the least-squares center and axes. For each raw polygon edge with
    inward unit normal n, a rectangle's half sizes a,b must satisfy
    |n.x_axis|*a + |n.y_axis|*b <= distance(center, edge)-margin.
    Include the fitted rectangle's inset as two more half-size constraints.
    The maximum of a*b is either an intersection of two supporting lines or
    the stationary point on one line (a=d/2A, b=d/2B). Enumerating those finite
    candidates solves this 2D problem without a generic optimizer or padding.
    """
    margin = float(boundary_margin)
    if not np.isfinite(margin) or margin < 0:
        raise WorkspaceCalibrationError('边界余量 boundary_margin 必须为有限非负数')
    axes = np.column_stack((fitted['x_axis'][:2], fitted['y_axis'][:2]))
    size = np.array([fitted['length'], fitted['width']])
    origin = np.array(fitted['origin'][:2])
    center = origin+axes @ (size/2)
    raw = np.array([[fitted['raw_points'][f'P{i}'][a] for a in 'xy'] for i in range(4)])
    edges = np.roll(raw, -1, axis=0)-raw
    normals = fitted['normal_sign']*np.column_stack((-edges[:, 1], edges[:, 0]))/np.linalg.norm(edges, axis=1)[:, None]
    coefficients = np.vstack((np.abs(normals @ axes), np.eye(2)))
    distances = np.r_[np.sum(normals*(center-raw), axis=1)-margin, size/2-margin]
    if np.any(distances <= 0):
        raise WorkspaceCalibrationError('边界余量导致有效扫描区域为空或退化')
    candidates = []
    for row, distance in zip(coefficients, distances):
        if np.all(row > 1e-12):
            candidates.append(distance/(2*row))
    for i, first in enumerate(coefficients):
        for j in range(i+1, len(coefficients)):
            matrix = np.array([first, coefficients[j]])
            if abs(np.linalg.det(matrix)) > 1e-12:
                candidates.append(np.linalg.solve(matrix, distances[[i, j]]))
    # Only floating-point roundoff is tolerated, never a geometric fit error.
    feasible = [p for p in candidates if np.all(p > MIN_EDGE_LENGTH_M/2) and
                np.all(coefficients @ p <= distances+1e-12)]
    if not feasible:
        raise WorkspaceCalibrationError('边界余量导致有效扫描区域为空或退化')
    half = max(feasible, key=lambda p: float(np.prod(p))).copy()
    # Remove possible roundoff-only constraint violations (typically <1e-16 m).
    half *= min(1., float(np.min(distances/(coefficients @ half))))
    low, high = size/2-half, size/2+half
    local = np.array([[low[0],low[1]], [high[0],low[1]], [high[0],high[1]], [low[0],high[1]]])
    corners = local @ axes.T+origin
    clearance = np.sum(normals[None, :, :]*(corners[:, None, :]-raw[None, :, :]), axis=2)
    return dict(fitted_workspace=fitted, origin_xy=origin.tolist(), axes_xy=axes.tolist(),
        local_min_xy=low.tolist(), local_max_xy=high.tolist(),
        raw_corners_xy=raw.tolist(),
        rectified_corners_xy=[fitted['rectified_points'][f'R{i}'][:2] for i in range(4)],
        effective_corners_xy=corners.tolist(), boundary_margin_m=margin,
        minimum_raw_edge_clearance_m=float(np.min(clearance)),
        effective_area_m2=float(4*np.prod(half)),
        effective_method='maximum_area_at_fitted_center_and_orientation')


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

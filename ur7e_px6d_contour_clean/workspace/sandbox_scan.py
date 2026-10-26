"""Fixed-start coverage planning in the existing rectified workspace frame (meters)."""
from __future__ import annotations

import numpy as np

from workspace.workspace_transform import WorkspaceTransform


def plan_coverage(workspace, p0_pose, *, scan_z, line_spacing, boundary_margin):
    transform = WorkspaceTransform(workspace)
    p0 = np.asarray(p0_pose, dtype=float)
    if p0.shape != (6,) or not np.isfinite(p0).all():
        raise ValueError('P0 必须包含六个有限 TCP 位姿分量')
    if not np.isfinite(scan_z):
        raise ValueError('扫描高度 scan_z 必须为有限值')
    if not np.isfinite(line_spacing) or line_spacing <= 0:
        raise ValueError('扫描线间距 line_spacing 必须为有限正数')
    if not np.isfinite(boundary_margin) or boundary_margin < 0:
        raise ValueError('边界余量 boundary_margin 必须为有限非负数')
    size = np.array([workspace['length'], workspace['width']])
    low, high = np.full(2, boundary_margin), size-boundary_margin
    if np.any(high-low <= 1e-6):
        raise ValueError('边界余量导致有效扫描区域为空或退化：矩形边长必须大于两倍余量')
    def base(points):
        xyz = np.column_stack((np.asarray(points), np.zeros(len(points))))
        return transform.workspace_to_base(xyz)[:, :2]
    local_p0 = transform.base_to_workspace(p0[:3])[:2]
    candidates = []
    for axis in (0, 1):
        cross = 1-axis
        intervals = max(1, int(np.ceil((high[cross]-low[cross])/line_spacing)))
        lines = np.linspace(low[cross], high[cross], intervals+1)
        for reverse_lines in (False, True):
            for reverse_first in (False, True):
                points = []
                for i, offset in enumerate(lines[::-1] if reverse_lines else lines):
                    ends = [low[axis], high[axis]]
                    if bool(i % 2) != reverse_first:
                        ends.reverse()
                    for end in ends:
                        point = np.zeros(2)
                        point[axis], point[cross] = end, offset
                        points.append(point)
                xy = base(points)
                entry = float(np.linalg.norm(xy[0]-p0[:2]))
                length = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
                candidates.append((entry, length, axis, reverse_lines, reverse_first, xy, lines))
    # Entry distance first; equal entry lengths prefer the shorter full route.
    # Rounding avoids choosing another axis solely on floating point noise.
    chosen = min(candidates, key=lambda c: (round(c[0], 12), round(c[1], 12), *c[2:5]))
    entry, length, axis, reverse_lines, reverse_first, xy, lines = chosen
    effective = base([[low[0],low[1]], [high[0],low[1]], [high[0],high[1]], [low[0],high[1]]])
    outside = bool(np.any(local_p0 < low-1e-12) or np.any(local_p0 > high+1e-12))
    direction = (xy[1]-xy[0])/np.linalg.norm(xy[1]-xy[0])
    return dict(schema_version=1, frame='Base', units='m',
        p0_tcp_pose=p0.tolist(), scan_z=float(scan_z), line_spacing_m=float(line_spacing),
        actual_line_spacing_m=float(abs(lines[1]-lines[0])), boundary_margin_m=float(boundary_margin),
        raw_corners_xy=[[workspace['raw_points'][f'P{i}'][a] for a in 'xy'] for i in range(4)],
        rectified_corners_xy=[workspace['rectified_points'][f'R{i}'][:2] for i in range(4)],
        effective_corners_xy=effective.tolist(), coverage_xy=xy.tolist(),
        coverage_start_xy=xy[0].tolist(), coverage_end_xy=xy[-1].tolist(),
        first_direction_xy=direction.tolist(), entry_xy=[p0[:2].tolist(),xy[0].tolist()],
        entry_kind='lift_translate_descend' if outside else 'planar_search',
        entry_distance_m=entry, coverage_length_m=length, scan_axis=axis,
        reverse_lines=reverse_lines, reverse_first=reverse_first, line_count=len(lines),
        selection_rule='minimum P0-to-entry distance, then minimum coverage length')

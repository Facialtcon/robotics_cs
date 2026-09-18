"""Evaluate square-loop completion from accepted samples; never command motion.

Ground truth is used here only to score four-face coverage and winding. Neither
the object geometry nor these results are passed to the shared contour policy.
"""

from __future__ import annotations

import numpy as np

from simulation.geometry import RectangleTarget


def square_loop_report(points, target: RectangleTarget, follow_hand: str, config: dict) -> dict:
    report = {
        "completed": False, "boundary_path_m": 0.0, "faces_visited": 0,
        "winding_turns": 0.0, "distance_to_first_m": None,
        "tangent_alignment": None,
        "all_contacts_near_boundary": False, "maximum_boundary_distance_m": None,
    }
    if len(points) < 2:
        return report
    xy = np.asarray([point.pose[:2] for point in points])
    offsets = xy - target.center
    angles = np.unwrap(np.arctan2(offsets[:, 1], offsets[:, 0]))
    # Clockwise winding is negative in an XY top view.
    turn_sign = -1 if follow_hand == "CLOCKWISE" else 1
    segment_lengths = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    cumulative = np.r_[0.0, np.cumsum(segment_lengths)]
    candidates = []
    for index in range(max(1, len(points) - 3), len(points)):
        candidates.append((
            index,
            float(turn_sign * (angles[index] - angles[0]) / (2 * np.pi)),
            float(np.linalg.norm(xy[index] - xy[0])),
            float(np.dot(points[0].tangent_xy, points[index].tangent_xy)),
        ))
    threshold_winding = float(config.get("min_winding_turns", 0.90))
    threshold_distance = float(config.get("closure_distance", 0.012))
    threshold_alignment = np.cos(np.deg2rad(float(config.get("tangent_tolerance_deg", 35))))
    valid = [item for item in candidates if item[1] >= threshold_winding
             and item[2] <= threshold_distance and item[3] >= threshold_alignment]
    selected = min(valid, key=lambda item: item[2]) if valid else candidates[-1]
    selected_index, winding, distance, alignment = selected
    path = float(cumulative[selected_index])
    faces = set()
    maximum_error = max(target.distance_to_boundary(point) for point in xy)
    near_boundary = maximum_error <= float(config.get("boundary_contact_tolerance", 0.002))
    for point in xy:
        boundary_distance, outward = target.signed_distance_and_outward_normal(point)
        if abs(boundary_distance) <= 0.002:
            local_normal = target.rotation.T @ outward
            axis = int(np.argmax(np.abs(local_normal)))
            faces.add((axis, int(np.sign(local_normal[axis]))))
    report.update(
        boundary_path_m=path, faces_visited=len(faces), winding_turns=winding,
        distance_to_first_m=distance, tangent_alignment=alignment,
        all_contacts_near_boundary=near_boundary, maximum_boundary_distance_m=maximum_error,
        completed=bool(
            len(faces) == 4
            and near_boundary
            and path >= float(config["min_boundary_path"])
            and winding >= float(config["min_winding_turns"])
            and distance <= float(config["closure_distance"])
            and alignment >= np.cos(np.deg2rad(float(config["tangent_tolerance_deg"])))
        ),
    )
    return report


def generic_loop_report(points, target, follow_hand: str, config: dict) -> dict:
    """Ground-truth scoring for interactive non-square scenes; never plans motion."""
    report = {
        "completed": False, "boundary_path_m": 0.0, "winding_turns": 0.0,
        "distance_to_first_m": None, "tangent_alignment": None,
        "all_contacts_near_boundary": False, "maximum_boundary_distance_m": None,
    }
    if len(points) < 2:
        return report
    xy = np.asarray([point.pose[:2] for point in points])
    angles = np.unwrap(np.arctan2(xy[:, 1] - target.center[1], xy[:, 0] - target.center[0]))
    turn_sign = -1 if follow_hand == "CLOCKWISE" else 1
    cumulative = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    candidates = [(
        index,
        float(turn_sign * (angles[index] - angles[0]) / (2 * np.pi)),
        float(np.linalg.norm(xy[index] - xy[0])),
        float(np.dot(points[0].tangent_xy, points[index].tangent_xy)),
    ) for index in range(max(1, len(points) - 3), len(points))]
    minimum_winding = float(config.get("min_winding_turns", 0.90))
    maximum_distance = float(config.get("closure_distance", 0.012))
    minimum_alignment = np.cos(np.deg2rad(float(config.get("tangent_tolerance_deg", 35))))
    valid = [item for item in candidates if item[1] >= minimum_winding
             and item[2] <= maximum_distance and item[3] >= minimum_alignment]
    selected = min(valid, key=lambda item: item[2]) if valid else candidates[-1]
    selected_index, winding, distance, alignment = selected
    path = float(cumulative[selected_index])
    maximum_error = max(target.distance_to_boundary(point) for point in xy)
    near = maximum_error <= float(config.get("boundary_contact_tolerance", 0.002))
    report.update(
        boundary_path_m=path, winding_turns=winding, distance_to_first_m=distance,
        tangent_alignment=alignment, all_contacts_near_boundary=near,
        maximum_boundary_distance_m=maximum_error,
        completed=bool(
            near
            and bool(valid)
        ),
    )
    return report

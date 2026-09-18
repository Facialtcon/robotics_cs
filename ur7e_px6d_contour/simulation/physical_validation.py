"""Independent physical audit. Ground truth stays OUTSIDE the policy."""
import numpy as np
from simulation.geometry import RectangleTarget, CircleTarget, TriangleTarget


def segment_enters_interior(start, end, target, tolerance=1e-9):
    """Continuous segment intersection, including jumps across the whole target."""
    start, end = np.asarray(start), np.asarray(end)
    if isinstance(target, RectangleTarget):
        origin = target.rotation.T @ (start - target.center)
        delta = target.rotation.T @ (end - start)
        half = np.asarray([target.width, target.height]) / 2 - tolerance
        lower, upper = 0.0, 1.0
        for index in range(2):
            if abs(delta[index]) < 1e-15:
                if abs(origin[index]) >= half[index]:
                    return False
            else:
                bounds = sorted(((-half[index] - origin[index]) / delta[index],
                                 (half[index] - origin[index]) / delta[index]))
                lower, upper = max(lower, bounds[0]), min(upper, bounds[1])
                if lower >= upper:
                    return False
        return lower < upper
    if isinstance(target, CircleTarget):
        delta = end - start
        fraction = np.clip(np.dot(target.center - start, delta) / max(np.dot(delta, delta), 1e-30), 0, 1)
        return np.linalg.norm(start + fraction * delta - target.center) < target.radius - tolerance
    if isinstance(target, TriangleTarget):
        # Intersect segment parameter t with every strict CCW half-plane.
        lower, upper = 0.0, 1.0
        delta = end - start
        vertices = target.vertices
        for vertex, following in zip(vertices, np.roll(vertices, -1, axis=0)):
            edge = following - vertex
            value = edge[0] * (start[1] - vertex[1]) - edge[1] * (start[0] - vertex[0])
            slope = edge[0] * delta[1] - edge[1] * delta[0]
            threshold = tolerance * np.linalg.norm(edge)
            if abs(slope) < 1e-15:
                if value <= threshold:
                    return False
                continue
            boundary_t = (threshold - value) / slope
            if slope > 0:
                lower = max(lower, boundary_t)
            else:
                upper = min(upper, boundary_t)
            if lower >= upper:
                return False
        return lower < upper and upper >= 0.0 and lower <= 1.0
    raise TypeError("physical audit requires a supported geometry")


def probe_audit(episodes, tolerance):
    def return_target(episode):
        # Only the explicitly logged successful acquisition may stop locally.
        # Old records and every ordinary probe still require the original anchor.
        target = getattr(episode, "return_target_pose", None)
        if (getattr(episode, "purpose", None) == "ACQUISITION"
                and getattr(episode, "outcome", None) == "CONTACT" and target is not None):
            return np.asarray(target, dtype=float)
        return episode.anchor_pose

    incomplete = [e.probe_id for e in episodes if not e.return_completed]
    wrong = [e.probe_id for e in episodes if e.return_completed and
             (e.returned_pose is None or np.linalg.norm(e.returned_pose[:2] - return_target(e)[:2]) > tolerance + 1e-12)]
    return {"probe_count": len(episodes), "all_probes_returned": not incomplete and not wrong,
            "incomplete_probe_ids": incomplete, "wrong_anchor_probe_ids": wrong,
            "maximum_return_error_m": max((float(np.linalg.norm(e.returned_pose[:2] - return_target(e)[:2]))
                for e in episodes if e.returned_pose is not None), default=0.0)}

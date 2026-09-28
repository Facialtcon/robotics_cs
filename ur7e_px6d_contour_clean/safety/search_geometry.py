"""Distances from the scan origin to the saved convex sandbox's raw edges."""
import numpy as np

from robot.rtde_controller import RobotError


def ray_polygon_distance(origin, direction, corners):
    origin = np.asarray(origin, dtype=float)
    direction = np.asarray(direction, dtype=float)
    vertices = np.asarray(corners, dtype=float)
    if (origin.shape != (2,) or direction.shape != (2,) or vertices.shape != (4, 2)
            or not all(np.isfinite(v).all() for v in (origin, direction, vertices))):
        raise RobotError('invalid search ray / sandbox polygon')
    norm = np.linalg.norm(direction)
    if norm <= 1e-12:
        raise RobotError('invalid zero search direction')
    direction = direction / norm
    edges = np.roll(vertices, -1, axis=0) - vertices
    cross = lambda a, b: a[..., 0]*b[..., 1] - a[..., 1]*b[..., 0]
    turns = cross(edges, np.roll(edges, -1, axis=0))
    if not (np.all(turns > 1e-12) or np.all(turns < -1e-12)):
        raise RobotError('search requires an ordered convex sandbox quadrilateral')
    if np.any(np.sign(turns[0])*cross(edges, origin-vertices) <= 0):
        raise RobotError('scan P0 must be strictly inside the sandbox polygon')
    distances = []
    for vertex, edge in zip(vertices, edges):
        denominator = cross(direction, edge)
        if abs(denominator) <= 1e-12:
            continue
        distance = cross(vertex-origin, edge) / denominator
        along_edge = cross(vertex-origin, direction) / denominator
        if distance > 1e-12 and -1e-12 <= along_edge <= 1+1e-12:
            distances.append(float(distance))
    if not distances:
        raise RobotError('no valid forward search intersection with sandbox polygon')
    return min(distances)

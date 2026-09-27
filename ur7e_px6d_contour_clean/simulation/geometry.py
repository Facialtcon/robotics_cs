"""Analytic 2D targets for the policy-debugging simulator."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


class GeometryError(ValueError):
    pass


class TargetGeometry:
    center: np.ndarray

    def signed_distance_and_outward_normal(self, point) -> tuple[float, np.ndarray]:
        """Return signed boundary distance (negative inside) and outward normal."""
        raise NotImplementedError

    def penetration_and_outward_normal(self, point) -> tuple[float, np.ndarray]:
        distance, normal = self.signed_distance_and_outward_normal(point)
        return max(0.0, -distance), normal

    def distance_to_boundary(self, point) -> float:
        distance, _ = self.signed_distance_and_outward_normal(point)
        return abs(distance)

    def boundary_points(self, count: int = 240) -> np.ndarray:
        raise NotImplementedError


@dataclass(frozen=True)
class RectangleTarget(TargetGeometry):
    center: np.ndarray
    width: float
    height: float
    rotation_deg: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "center", np.asarray(self.center, dtype=float))
        if self.center.shape != (2,) or self.width <= 0.0 or self.height <= 0.0:
            raise GeometryError("rectangle center must be XY and dimensions must be positive")

    @property
    def rotation(self) -> np.ndarray:
        angle = np.deg2rad(self.rotation_deg)
        cosine, sine = np.cos(angle), np.sin(angle)
        return np.asarray(((cosine, -sine), (sine, cosine)))

    def signed_distance_and_outward_normal(self, point) -> tuple[float, np.ndarray]:
        local = self.rotation.T @ (np.asarray(point, dtype=float) - self.center)
        half = np.asarray((self.width / 2.0, self.height / 2.0))
        delta = np.abs(local) - half
        outside = np.maximum(delta, 0.0)
        signed_distance = float(np.linalg.norm(outside) + min(max(delta[0], delta[1]), 0.0))

        if np.any(delta > 0.0):
            closest = np.clip(local, -half, half)
            local_normal = local - closest
            norm = float(np.linalg.norm(local_normal))
            if norm < 1e-12:
                axis = int(np.argmax(delta))
                local_normal = np.zeros(2)
                local_normal[axis] = 1.0 if local[axis] >= 0.0 else -1.0
            else:
                local_normal /= norm
        else:
            clearances = half - np.abs(local)
            axis = int(np.argmin(clearances))
            local_normal = np.zeros(2)
            local_normal[axis] = 1.0 if local[axis] >= 0.0 else -1.0
        return signed_distance, self.rotation @ local_normal

    def boundary_points(self, count: int = 240) -> np.ndarray:
        del count
        half_x, half_y = self.width / 2.0, self.height / 2.0
        local = np.asarray(
            [(-half_x, -half_y), (half_x, -half_y), (half_x, half_y), (-half_x, half_y), (-half_x, -half_y)]
        )
        return local @ self.rotation.T + self.center


@dataclass(frozen=True)
class CircleTarget(TargetGeometry):
    center: np.ndarray
    radius: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "center", np.asarray(self.center, dtype=float))
        if self.center.shape != (2,) or self.radius <= 0.0:
            raise GeometryError("circle center must be XY and radius must be positive")

    def signed_distance_and_outward_normal(self, point) -> tuple[float, np.ndarray]:
        offset = np.asarray(point, dtype=float) - self.center
        radius = float(np.linalg.norm(offset))
        normal = np.asarray((1.0, 0.0)) if radius < 1e-12 else offset / radius
        return radius - self.radius, normal

    def boundary_points(self, count: int = 240) -> np.ndarray:
        angles = np.linspace(0.0, 2.0 * np.pi, count, endpoint=True)
        return self.center + self.radius * np.column_stack((np.cos(angles), np.sin(angles)))


@dataclass(frozen=True)
class TriangleTarget(TargetGeometry):
    """Equilateral convex target described by center, circumradius and rotation."""

    center: np.ndarray
    radius: float
    rotation_deg: float = 90.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "center", np.asarray(self.center, dtype=float))
        if self.center.shape != (2,) or self.radius <= 0.0:
            raise GeometryError("triangle center must be XY and radius must be positive")

    @property
    def vertices(self) -> np.ndarray:
        angles = np.deg2rad(self.rotation_deg) + np.arange(3) * 2.0 * np.pi / 3.0
        # Increasing angles produce counterclockwise vertices.
        return self.center + self.radius * np.column_stack((np.cos(angles), np.sin(angles)))

    def signed_distance_and_outward_normal(self, point) -> tuple[float, np.ndarray]:
        point = np.asarray(point, dtype=float)
        vertices = self.vertices
        candidates = []
        crosses = []
        for start, end in zip(vertices, np.roll(vertices, -1, axis=0)):
            edge = end - start
            edge_length_squared = float(np.dot(edge, edge))
            fraction = float(np.clip(np.dot(point - start, edge) / edge_length_squared, 0.0, 1.0))
            closest = start + fraction * edge
            offset = point - closest
            distance = float(np.linalg.norm(offset))
            outward = np.asarray((edge[1], -edge[0])) / np.linalg.norm(edge)
            cross = float(edge[0] * (point[1] - start[1]) - edge[1] * (point[0] - start[0]))
            crosses.append(cross)
            candidates.append((distance, outward, offset))
        inside = all(cross >= -1e-12 for cross in crosses)
        closest_distance, edge_outward, closest_offset = min(candidates, key=lambda item: item[0])
        closest_normal = (
            edge_outward
            if inside or closest_distance < 1e-12
            else closest_offset / closest_distance
        )
        return (-closest_distance if inside else closest_distance), closest_normal

    def boundary_points(self, count: int = 240) -> np.ndarray:
        del count
        vertices = self.vertices
        return np.vstack((vertices, vertices[0]))


def create_target(config: dict) -> TargetGeometry:
    shape = str(config["target_shape"]).lower()
    center = np.asarray(config["target_center"], dtype=float)
    if shape in {"rectangle", "rotated_rectangle"}:
        return RectangleTarget(
            center,
            float(config["target_width"]),
            float(config["target_height"]),
            float(config.get("target_rotation_deg", 0.0)),
        )
    if shape == "circle":
        return CircleTarget(center, float(config["target_radius"]))
    if shape == "triangle":
        return TriangleTarget(
            center,
            float(config["target_radius"]),
            float(config.get("target_rotation_deg", 90.0)),
        )
    raise GeometryError(f"unsupported target_shape: {shape}")

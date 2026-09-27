"""Progressively expanding local sectors from ONE observed free-space anchor."""
from dataclasses import dataclass, field
import numpy as np
from policy.boundary_estimation import unit


@dataclass
class BoundaryRecovery:
    anchor: np.ndarray
    predicted_direction: np.ndarray
    old_tangent: np.ndarray
    last_contact: np.ndarray
    config: dict
    cursor: int = 0
    attempted_directions: list = field(default_factory=list)
    candidates: list = field(default_factory=list)

    def __post_init__(self):
        self.anchor = self.anchor.copy()
        self.anchor.setflags(write=False)
        # Near prediction first; expand in a consistent follow-hand sense.
        # Sector extents are budgets, not assumed corner/next-face angles.
        self.angles = []
        previous = 0.0
        step = float(self.config.get("recovery_angular_resolution_deg", 10))
        for extent in self.config.get("recovery_sector_extents_deg", [20, 50, 90, 130, 170]):
            count = max(1, int(np.ceil((float(extent) - previous) / step)))
            self.angles.extend(np.linspace(previous, float(extent), count + 1)[1:])
            previous = float(extent)

    def next_direction(self):
        if self.cursor >= len(self.angles):
            return None
        angle = float(self.angles[self.cursor])
        self.cursor += 1
        radians = np.deg2rad(angle)
        direction = unit(np.cos(radians) * self.predicted_direction - np.sin(radians) * self.old_tangent)
        self.attempted_directions.append(direction.copy())
        return angle, direction

    def candidate_rejection(self, pose, recent_points):
        # Only reject observed duplicates here. At a convex turn > 90 degrees
        # a valid NEW face can lie behind the OLD tangent's projection. Its
        # direction must be validated by the existing multi-probe confirmation
        # (new-tangent progress and reversal checks), not this one-point screen.
        # Distant history remains reachable at loop closure.
        tolerance = float(self.config.get("minimum_boundary_point_spacing", 0.001))
        if any(np.linalg.norm(pose[:2] - p.pose[:2]) < tolerance for p in recent_points[-8:]):
            return "PREVIOUSLY_VISITED_BOUNDARY"
        return ""

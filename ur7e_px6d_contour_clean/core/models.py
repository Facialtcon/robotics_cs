"""Shared immutable data models passed between sensor, policy, and robot layers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class Wrench:
    fx: float
    fy: float
    fz: float
    tx: float
    ty: float
    tz: float

    @classmethod
    def from_sequence(cls, values: Sequence[float]) -> "Wrench":
        if len(values) != 6:
            raise ValueError("a wrench must contain exactly six values")
        return cls(*(float(value) for value in values))

    def array(self) -> np.ndarray:
        return np.asarray((self.fx, self.fy, self.fz, self.tx, self.ty, self.tz), dtype=float)

    @property
    def force(self) -> np.ndarray:
        return self.array()[:3]

    @property
    def torque(self) -> np.ndarray:
        return self.array()[3:]


@dataclass(frozen=True)
class ForceFeatures:
    fxy: float
    force_angle: float
    interaction_direction: np.ndarray | None


@dataclass(frozen=True)
class RobotState:
    timestamp: float
    pose: np.ndarray
    tcp_speed: np.ndarray

    def __post_init__(self) -> None:
        if np.asarray(self.pose).shape != (6,) or np.asarray(self.tcp_speed).shape != (6,):
            raise ValueError("pose and tcp_speed must each have six values")


@dataclass(frozen=True)
class PolicyCommand:
    state: str
    move: bool
    direction_xy: np.ndarray
    speed: float
    target_pose: np.ndarray
    contact_flag: bool
    possible_corner: bool
    reason: str = ""
    policy_sub_state: str = ""
    recovery_id: int | None = None
    ray_index: int | None = None
    ray_theta_deg: float | None = None
    candidate_status: str = ""
    rejection_reason: str = ""
    probe_id: int | None = None


@dataclass(frozen=True)
class BoundaryPoint:
    index: int
    timestamp: float
    pose: np.ndarray
    wrench: Wrench
    target_direction_xy: np.ndarray
    tangent_xy: np.ndarray
    possible_corner: bool

    @property
    def normal_xy(self) -> np.ndarray:
        """Compatibility alias for logs/readers created before direction naming was fixed."""
        return self.target_direction_xy


@dataclass(frozen=True)
class PolicyWaypoint:
    timestamp: float
    state: str
    event_type: str
    pose: np.ndarray
    corner_id: int | None = None
    ray_index: int | None = None
    target_direction_xy: np.ndarray | None = None
    tangent_xy: np.ndarray | None = None


@dataclass(frozen=True)
class CornerSearchRay:
    """One guarded recovery ray; legacy field names remain CSV-compatible."""

    corner_id: int
    ray_index: int
    theta_deg: float
    anchor_pose: np.ndarray
    direction_xy: np.ndarray
    planned_length: float
    actual_length: float
    contact_pose: np.ndarray | None
    result: str

    @property
    def ray_end_xy(self) -> np.ndarray:
        """Actual guarded-probe endpoint; planned length remains separately logged."""
        return self.anchor_pose[:2] + self.direction_xy * self.actual_length

    @property
    def recovery_id(self) -> int:
        return self.corner_id


# Public vocabulary for new code.  The legacy name stays available so old
# analysis scripts can still read existing data and imports.
RecoveryRay = CornerSearchRay

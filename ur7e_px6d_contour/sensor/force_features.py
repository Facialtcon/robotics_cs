"""Extract policy-facing force features from a processed wrench."""

from __future__ import annotations

import math

import numpy as np

from core.models import ForceFeatures, Wrench
def normalize_xy(vector, epsilon: float = 1e-9) -> np.ndarray | None:
    value = np.asarray(vector, dtype=float)
    if value.shape != (2,):
        raise ValueError("expected a two-dimensional vector")
    norm = float(np.linalg.norm(value))
    return None if norm < epsilon else value / norm


def angle_between(left, right) -> float:
    a, b = normalize_xy(left), normalize_xy(right)
    if a is None or b is None:
        raise ValueError("angle is undefined for a zero vector")
    return math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0)))


def extract_force_features(wrench: Wrench) -> ForceFeatures:
    force_xy = np.asarray((wrench.fx, wrench.fy), dtype=float)
    return ForceFeatures(
        fxy=float(np.linalg.norm(force_xy)),
        force_angle=math.atan2(wrench.fy, wrench.fx),
        interaction_direction=normalize_xy(force_xy),
    )


def perpendicular(normal_xy, side: str = "LEFT") -> np.ndarray:
    """Diagnostic vector helper; never used by the motion policy."""
    value = normalize_xy(normal_xy)
    if value is None:
        raise ValueError("normal cannot be zero")
    if side.upper() == "LEFT":
        return np.asarray((-value[1], value[0]))
    if side.upper() == "RIGHT":
        return np.asarray((value[1], -value[0]))
    raise ValueError("side must be LEFT or RIGHT")


def estimate_effective_contact_location(_force, _torque):
    """Reserved research hook; not used by the v1 motion policy."""
    return None


def check_tip_contact_consistency(_force, _torque):
    """Reserved research hook; no validated F/T model exists yet."""
    return None

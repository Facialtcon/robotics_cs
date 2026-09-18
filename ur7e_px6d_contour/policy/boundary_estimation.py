"""Local contact geometry only: no force vectors or environment geometry."""
from dataclasses import dataclass
import numpy as np


def unit(vector):
    vector = np.asarray(vector, dtype=float)
    length = np.linalg.norm(vector)
    if not np.isfinite(length) or length < 1e-10:
        raise ValueError("direction must be finite and nonzero")
    return vector / length


def left(vector):
    return np.asarray([-vector[1], vector[0]])


def handed_tangent(target_side, hand):
    return left(target_side) * (1 if hand == "CLOCKWISE" else -1)


@dataclass
class BoundaryEstimate:
    tangent: np.ndarray
    target_side: np.ndarray
    residual: float
    span: float


def estimate_boundary(contacts, target_side, hand, min_span=0.001):
    """PCA line, oriented by historical probe side and fixed follow hand.

    target_side is a successful anchor-to-contact direction, NOT measured force.
    Two points define a line; three or more additionally provide a residual.
    """
    xy = np.asarray([point[:2] for point in contacts])
    if len(xy) < 2:
        return None
    centered = xy - xy.mean(axis=0)
    _, _, basis = np.linalg.svd(centered, full_matrices=False)
    tangent = basis[0]
    span = float(np.ptp(centered @ tangent))
    if span < min_span:
        return None
    normal = left(tangent)
    if np.dot(normal, target_side) < 0:
        normal = -normal
    tangent = handed_tangent(normal, hand)
    residual = float(np.sqrt(np.mean((centered @ normal) ** 2)))
    return BoundaryEstimate(tangent, normal, residual, span)

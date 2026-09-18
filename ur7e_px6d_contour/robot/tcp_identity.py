"""Active-TCP normalization, comparison, and read-only RTDE lookup."""

from __future__ import annotations

from typing import Sequence

import numpy as np


class TCPIdentityError(ValueError):
    """Raised when an active TCP cannot be read or validated."""


def normalize_tcp_offset(values: Sequence[float], name: str = "TCP offset") -> list[float]:
    try:
        offset = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise TCPIdentityError(f"{name} must contain six finite values") from exc
    if offset.shape != (6,) or not np.all(np.isfinite(offset)):
        raise TCPIdentityError(f"{name} must contain six finite values")
    return offset.tolist()


def tcp_offsets_match(left, right, tolerance: float) -> bool:
    if float(tolerance) < 0.0:
        raise TCPIdentityError("TCP offset tolerance cannot be negative")
    return bool(
        np.allclose(
            normalize_tcp_offset(left, "left TCP offset"),
            normalize_tcp_offset(right, "right TCP offset"),
            atol=float(tolerance),
            rtol=0.0,
        )
    )


def read_active_tcp_offset(control) -> list[float]:
    """Read getTCPOffset from an already-connected RTDE Control interface."""
    if not hasattr(control, "getTCPOffset"):
        raise TCPIdentityError(
            "this ur-rtde build cannot read the active TCP; refusing unverified motion"
        )
    return normalize_tcp_offset(control.getTCPOffset(), "active TCP offset")

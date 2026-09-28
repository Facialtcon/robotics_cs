"""Pure force/torque safety checks shared by guarded policy motions.

This module evaluates limits only.  Robot stopping remains the controller and
application's responsibility, keeping low-level stop behavior out of policy
geometry and edge-search code.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from core.models import Wrench


def force_safety_reason(raw: Wrench, processed: Wrench, config: dict) -> str | None:
    checks = (
        (np.linalg.norm(processed.force), config["safety_force_threshold"], "processed force safety threshold exceeded"),
        (np.linalg.norm(processed.torque), config["safety_torque_threshold"], "processed torque safety threshold exceeded"),
        (np.linalg.norm(raw.force), config["absolute_raw_force_threshold"], "absolute raw force safety threshold exceeded"),
        (np.linalg.norm(raw.torque), config["absolute_raw_torque_threshold"], "absolute raw torque safety threshold exceeded"),
    )
    return next((reason for value, limit, reason in checks if value >= float(limit)), None)


def raw_safety_reason(raw: Wrench, config: dict) -> str | None:
    """Existing raw backstops, usable before zero bias has been established."""
    if not np.isfinite(raw.array()).all():
        return 'nonfinite raw wrench'
    if np.linalg.norm(raw.force) >= float(config['absolute_raw_force_threshold']):
        return 'absolute raw force safety threshold exceeded'
    if np.linalg.norm(raw.torque) >= float(config['absolute_raw_torque_threshold']):
        return 'absolute raw torque safety threshold exceeded'
    return None


def continuous_force_reason(raw, processed, config, diagnostics):
    """Real continuous mode: processed limits are diagnostics, raw limits stop."""
    for value, limit, key in (
        (np.linalg.norm(processed.force), config['safety_force_threshold'], 'processed_force'),
        (np.linalg.norm(processed.torque), config['safety_torque_threshold'], 'processed_torque'),
    ):
        if value >= float(limit):
            diagnostics[key] = f'WARNING: {value:g} >= {limit:g}; diagnostic only'
    return raw_safety_reason(raw, config)


@dataclass
class ForceRateGuard:
    previous: tuple[float, float] | None = None

    def update(self, timestamp: float, fxy: float) -> float:
        rate = 0.0
        if self.previous is not None:
            old_time, old_force = self.previous
            if timestamp > old_time:
                rate = (fxy - old_force) / (timestamp - old_time)
        self.previous = (timestamp, fxy)
        return rate

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

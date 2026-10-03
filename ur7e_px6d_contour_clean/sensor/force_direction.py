"""Force convention shared by control and read-only checks.

R_BS maps sensor components directly to Base (R_BT @ R_TS if independently
calibrated), updated from actual TCP orientation. TCP pose alone does not
identify R_TS; unknown sensor components cannot authorize Base motion.
n points toward increasing
contact compression; unloading is -n. Fxy remains a processed resultant,
including friction/background, not an isolated target normal force.
"""
import numpy as np

from policy.boundary_estimation import handed_tangent


def control_directions(force_base, sign, hand, *, minimum_force=0.):
    xy = np.asarray(force_base, dtype=float)[:2]
    magnitude = float(np.linalg.norm(xy))
    n = sign*xy/magnitude if magnitude > 1e-12 and magnitude >= minimum_force else np.zeros(2)
    return n, handed_tangent(n, hand), -n


def normal_feedback_speed(fxy, config):
    error = float(config['force_reference'])-fxy
    dead = np.sign(error)*max(abs(error)-float(config['force_deadband']), 0.)
    return float(np.clip(float(config['force_gain'])*dead,
                         -float(config['normal_speed_limit']), float(config['normal_speed_limit'])))

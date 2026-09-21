"""Small explainable synthetic wrench model; not a granular physics model."""

from __future__ import annotations

import numpy as np

from core.models import Wrench
from simulation.geometry import TargetGeometry

PHYSICAL_FORCE_METADATA = dict(schema_version=1, frame='Base',
    force_convention='legacy_inward_normal_to_outward_physical_v1',
    force_source='synthetic_contact_and_drag_model', estimate_method='quasistatic_planar_balance',
    conversion='normal_physical=-object_force; friction/background unchanged; noise excluded',
    other_known_planar='none in this model', physical_available=True)


class SimulatedForceSensor:
    def __init__(self, target: TargetGeometry, config: dict):
        self.target = target
        self.config = config
        self.rng = np.random.default_rng(int(config["random_seed"]))

    def reset(self) -> None:
        self.rng = np.random.default_rng(int(self.config["random_seed"]))

    def read_wrench(self, position_xy, velocity_xy) -> tuple[Wrench, dict]:
        position = np.asarray(position_xy, dtype=float)
        velocity = np.asarray(velocity_xy, dtype=float)
        speed = float(np.linalg.norm(velocity))
        background = np.zeros(2)
        if speed > 1e-12:
            background = -float(self.config["granular_drag_force"]) * velocity / speed

        signed_distance, outward_normal = self.target.signed_distance_and_outward_normal(position)
        penetration = max(0.0, -signed_distance)
        # Explicit finite-radius compliant sensing envelope. TCP remains the
        # measured center, never projected/snapped onto the object boundary.
        tip_radius = float(self.config.get("probe_tip_radius", 0.0))
        compression = max(0.0, tip_radius - signed_distance)
        object_force = np.zeros(2)
        friction = np.zeros(2)
        if compression > 0.0:
            magnitude = float(self.config["contact_stiffness"]) * compression
            # Synthetic interaction direction is targetward (-outward), matching
            # target_direction when probe_direction_sign=+1. This is
            # an explicit simulator convention, not a claim about real PX6D signs.
            interaction_direction = -outward_normal
            object_force = magnitude * interaction_direction
            tangent_velocity = velocity - np.dot(velocity, outward_normal) * outward_normal
            tangent_speed = float(np.linalg.norm(tangent_velocity))
            if tangent_speed > 1e-12:
                friction = (
                    -float(self.config["friction_coefficient"])
                    * magnitude
                    * tangent_velocity
                    / tangent_speed
                )

        noise = self.rng.normal(0.0, float(self.config["noise_std"]), size=2)
        force = background + object_force + friction + noise
        wrench = Wrench(float(force[0]), float(force[1]), 0.0, 0.0, 0.0, 0.0)
        # Display/logging diagnostics ONLY. Preserve the historical synthetic
        # control signal above, including its inward normal and measurement noise.
        normal_physical = -object_force
        boundary_physical = normal_physical + friction
        environment_physical = boundary_physical + background
        return wrench, {
            "normal_physical": normal_physical,
            "boundary_physical": boundary_physical,
            "environment_physical": environment_physical,
            "robot_estimate": -environment_physical,
            "penetration": penetration,
            "tip_compression": compression,
            "signed_distance": signed_distance,
            "outward_normal": outward_normal,
            "background_force": background,
            "object_force": object_force,
            "friction_force": friction,
            "noise_force": noise,
        }

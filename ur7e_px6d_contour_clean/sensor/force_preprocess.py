"""Bias, filtering, compensation, and frame transforms for PX6D wrench data."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from core.models import Wrench


def _six(values, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain six finite numbers")
    return result


@dataclass
class WrenchPreprocessor:
    """Apply bias, EMA, gravity, frame transform, then granular baseline.

    `filter_alpha` weights the previous filtered sample. A larger value is
    smoother but adds more delay. Gravity is a configured constant wrench in
    the sensor frame because this experiment keeps tool orientation fixed.
    """

    filter_alpha: float
    rotation_sensor_to_output: np.ndarray
    sensor_origin_in_output_m: np.ndarray
    gravity_wrench_sensor: np.ndarray
    granular_baseline_output: np.ndarray

    def __post_init__(self) -> None:
        if not 0.0 <= self.filter_alpha < 1.0:
            raise ValueError("filter_alpha must be in [0, 1)")
        self.rotation_sensor_to_output = np.asarray(self.rotation_sensor_to_output, dtype=float)
        if self.rotation_sensor_to_output.shape != (3, 3):
            raise ValueError("rotation_sensor_to_output must be 3x3")
        should_be_identity = self.rotation_sensor_to_output.T @ self.rotation_sensor_to_output
        if not np.allclose(should_be_identity, np.eye(3), atol=1e-6) or not np.isclose(
            np.linalg.det(self.rotation_sensor_to_output), 1.0, atol=1e-6
        ):
            raise ValueError("rotation_sensor_to_output must be a proper rotation matrix")
        self.sensor_origin_in_output_m = np.asarray(self.sensor_origin_in_output_m, dtype=float)
        if self.sensor_origin_in_output_m.shape != (3,):
            raise ValueError("sensor_origin_in_output_m must have three values")
        self.gravity_wrench_sensor = _six(self.gravity_wrench_sensor, "gravity_wrench_sensor")
        self.granular_baseline_output = _six(
            self.granular_baseline_output, "granular_baseline_output"
        )
        self.zero_bias_sensor = np.zeros(6, dtype=float)
        self._filtered_sensor: np.ndarray | None = None

    @classmethod
    def from_config(cls, config: dict) -> "WrenchPreprocessor":
        transform = config["coordinate_transform"]
        return cls(
            filter_alpha=float(config["filter_alpha"]),
            rotation_sensor_to_output=np.asarray(transform["rotation_sensor_to_base"], dtype=float),
            sensor_origin_in_output_m=np.asarray(
                transform["sensor_origin_in_base_m"], dtype=float
            ),
            gravity_wrench_sensor=np.asarray(config["gravity_wrench_sensor"], dtype=float),
            granular_baseline_output=np.asarray(config["granular_baseline_output"], dtype=float),
        )

    def set_zero_bias(self, samples: Iterable[Wrench]) -> None:
        rows = [sample.array() for sample in samples]
        if not rows:
            raise ValueError("at least one sample is required for zero bias")
        self.zero_bias_sensor = np.mean(np.stack(rows), axis=0)
        self._filtered_sensor = None

    def set_granular_baseline(self, wrench_output: Wrench) -> None:
        self.granular_baseline_output = wrench_output.array().copy()

    def process(self, raw: Wrench) -> Wrench:
        corrected = raw.array() - self.zero_bias_sensor
        if self._filtered_sensor is None:
            self._filtered_sensor = corrected
        else:
            self._filtered_sensor = (
                self.filter_alpha * self._filtered_sensor
                + (1.0 - self.filter_alpha) * corrected
            )
        sensor = self._filtered_sensor - self.gravity_wrench_sensor
        rotation = self.rotation_sensor_to_output
        force_output = rotation @ sensor[:3]
        torque_output = (
            rotation @ sensor[3:]
            + np.cross(self.sensor_origin_in_output_m, force_output)
        )
        output = np.concatenate((force_output, torque_output)) - self.granular_baseline_output
        return Wrench.from_sequence(output)

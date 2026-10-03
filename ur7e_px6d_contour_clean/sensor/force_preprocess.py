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


def _rotation(values, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if (result.shape != (3, 3) or not np.isfinite(result).all() or
        not np.allclose(result.T @ result, np.eye(3), atol=1e-6) or
        not np.isclose(np.linalg.det(result), 1., atol=1e-6)):
        raise ValueError(f"{name} must be a finite proper 3x3 rotation matrix")
    return result


def _tool_rotation(rotvec) -> np.ndarray:
    """UR rotation vector maps tool components to Base; no robot connection."""
    vector = np.asarray(rotvec, dtype=float)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError('tool_orientation must contain three finite rotation-vector values')
    angle = float(np.linalg.norm(vector))
    if not np.isfinite(angle):
        raise ValueError('tool_orientation rotation-vector magnitude is not finite')
    if angle < 1e-12:
        return np.eye(3)
    x, y, z = vector / angle
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + np.sin(angle)*skew + (1.-np.cos(angle))*(skew @ skew)


@dataclass
class WrenchPreprocessor:
    """Apply bias, EMA, gravity, frame transform, then granular baseline.

    `filter_alpha` weights the previous filtered sample. A larger value is
    smoother but adds more delay. Gravity is a configured constant wrench in
    the sensor frame and must match the stationary scan orientation. Pose
    updates rotate force using an explicitly supplied installation; they do
    not calibrate the mounting or estimate gravity at another orientation.
    """

    filter_alpha: float
    rotation_sensor_to_output: np.ndarray
    sensor_origin_in_output_m: np.ndarray
    gravity_wrench_sensor: np.ndarray
    granular_baseline_output: np.ndarray

    def __post_init__(self) -> None:
        if not 0.0 <= self.filter_alpha < 1.0:
            raise ValueError("filter_alpha must be in [0, 1)")
        self.rotation_sensor_to_output = _rotation(
            self.rotation_sensor_to_output, 'rotation_sensor_to_output')
        self.sensor_origin_in_output_m = np.asarray(self.sensor_origin_in_output_m, dtype=float)
        if self.sensor_origin_in_output_m.shape != (3,):
            raise ValueError("sensor_origin_in_output_m must have three values")
        self.gravity_wrench_sensor = _six(self.gravity_wrench_sensor, "gravity_wrench_sensor")
        self.granular_baseline_output = _six(
            self.granular_baseline_output, "granular_baseline_output"
        )
        self.zero_bias_sensor = np.zeros(6, dtype=float)
        self._filtered_sensor: np.ndarray | None = None
        # A direct constructor supplies an explicit mathematical transform.
        # Config loading below separately distinguishes physical calibration.
        self._rotation_sensor_to_tool = None
        self._origin_in_tool = None
        self._tool_orientation = None
        self._reference_tool_orientation = None
        self._transform_status = dict(available=True, source='explicit_transform',
                                      output_frame='Base', reason=None)

    @classmethod
    def from_config(cls, config: dict, *, tool_orientation=None,
                    synthetic=False) -> "WrenchPreprocessor":
        """Resolve a measured rigid mount, never infer it from TCP orientation.

        ``reference_tool_orientation`` is the measured UR rotation vector at
        which the explicitly supplied R_BS was valid. Legacy R_BS alone is
        unverified; its presence (including identity) supplies no such fact.
        ``synthetic`` is solely for simulation with a known generated frame.
        """
        transform = config["coordinate_transform"]
        base_values = transform.get('rotation_sensor_to_base')
        # A direct installation R_TS needs no obsolete fixed-pose R_BS. An
        # internal identity here is not promoted to a calibrated output below.
        configured = _rotation(np.eye(3) if base_values is None else base_values,
                               'rotation_sensor_to_base')
        instance = cls(
            filter_alpha=float(config["filter_alpha"]),
            rotation_sensor_to_output=configured,
            sensor_origin_in_output_m=np.asarray(
                transform["sensor_origin_in_base_m"], dtype=float
            ),
            gravity_wrench_sensor=np.asarray(config["gravity_wrench_sensor"], dtype=float),
            granular_baseline_output=np.asarray(config["granular_baseline_output"], dtype=float),
        )
        if synthetic:
            instance._transform_status = dict(available=True, source='synthetic_config',
                                             output_frame='Base', reason=None)
            return instance
        if transform.get('rotation_sensor_to_tool') is not None:
            instance._rotation_sensor_to_tool = _rotation(
                transform['rotation_sensor_to_tool'], 'rotation_sensor_to_tool')
            source = 'configured_sensor_to_tool'
        elif transform.get('reference_tool_orientation') is not None:
            if base_values is None:
                raise ValueError('reference_tool_orientation requires rotation_sensor_to_base')
            reference = _tool_rotation(transform['reference_tool_orientation'])
            instance._reference_tool_orientation = np.asarray(transform['reference_tool_orientation'], dtype=float).copy()
            instance._rotation_sensor_to_tool = reference.T @ configured
            instance._origin_in_tool = reference.T @ instance.sensor_origin_in_output_m
            source = 'configured_base_at_reference_pose'
        else:
            source = 'unknown_sensor_installation'
        if transform.get('sensor_origin_in_tool_m') is not None:
            origin = np.asarray(transform['sensor_origin_in_tool_m'], dtype=float)
            if origin.shape != (3,) or not np.isfinite(origin).all():
                raise ValueError('sensor_origin_in_tool_m must contain three finite numbers')
            instance._origin_in_tool = origin
        elif instance._rotation_sensor_to_tool is not None and instance._origin_in_tool is None:
            if transform.get('reference_tool_orientation') is not None:
                instance._origin_in_tool = (_tool_rotation(transform['reference_tool_orientation']).T
                                            @ instance.sensor_origin_in_output_m)
            elif np.any(instance.sensor_origin_in_output_m):
                raise ValueError('nonzero sensor_origin_in_base_m requires reference_tool_orientation '
                                 'or sensor_origin_in_tool_m when updating tool orientation')
            else:
                instance._origin_in_tool = np.zeros(3)
        instance._transform_status = dict(available=False, source=source,
            output_frame='sensor_uncalibrated', reason='sensor installation rotation is unknown')
        if tool_orientation is not None:
            instance.set_tool_orientation(tool_orientation)
        elif instance._rotation_sensor_to_tool is not None:
            instance._transform_status['reason'] = 'actual tool orientation is unavailable'
        return instance

    @property
    def force_transform_status(self) -> dict:
        return dict(self._transform_status,
                    rotation_sensor_to_tool=(None if self._rotation_sensor_to_tool is None
                                              else self._rotation_sensor_to_tool.tolist()),
                    reference_tool_orientation=(None if self._reference_tool_orientation is None
                                                  else self._reference_tool_orientation.tolist()),
                    rotation_sensor_to_base=(self.rotation_sensor_to_output.tolist()
                        if self._transform_status['available'] else None),
                    tool_orientation=(None if self._tool_orientation is None
                                             else self._tool_orientation.tolist()))

    def set_tool_orientation(self, rotvec) -> dict:
        rotation = _tool_rotation(rotvec)
        self._tool_orientation = np.asarray(rotvec, dtype=float).copy()
        if self._rotation_sensor_to_tool is not None:
            self.rotation_sensor_to_output = rotation @ self._rotation_sensor_to_tool
            if self._origin_in_tool is not None:
                self.sensor_origin_in_output_m = rotation @ self._origin_in_tool
            self._transform_status.update(available=True, output_frame='Base', reason=None)
        # TCP orientation alone cannot turn an unmeasured installation into R_BS.
        return self.force_transform_status

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
        if not self._transform_status['available']:
            # Preserve observable sensor data without mislabelling it as Base.
            # A Base baseline/lever arm has no meaning in this unknown frame.
            return Wrench.from_sequence(sensor)
        rotation = self.rotation_sensor_to_output
        force_output = rotation @ sensor[:3]
        torque_output = (
            rotation @ sensor[3:]
            + np.cross(self.sensor_origin_in_output_m, force_output)
        )
        output = np.concatenate((force_output, torque_output)) - self.granular_baseline_output
        return Wrench.from_sequence(output)

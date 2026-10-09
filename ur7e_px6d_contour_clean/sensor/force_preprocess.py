"""Bias, filtering, compensation, and frame transforms for PX6D wrench data."""

from __future__ import annotations

from dataclasses import dataclass, field
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


class IndependentForceKalman:
    """Independent random-walk axes; Q/R/P are variances in N^2 per sample.

    Defaults are starting tuning values, not an identified PX6D noise model.
    Missing/nonfinite measurements raise before changing either x or P. The
    caller keeps the existing stop-on-acquisition-failure behavior.
    """
    def __init__(self, config=None):
        config = {} if config is None else config
        if not isinstance(config, dict):
            raise ValueError('kalman must be a mapping')
        self.enabled = config.get('enabled', True)
        if not isinstance(self.enabled, bool):
            raise ValueError('kalman.enabled must be true or false')
        self.process_noise = self._axes(config.get('process_noise', .01), 'process_noise')
        self.measurement_noise = self._axes(config.get('measurement_noise', .25), 'measurement_noise')
        self.initial_covariance = self._axes(config.get('initial_covariance', 1.), 'initial_covariance')
        self.reset()

    @staticmethod
    def _axes(value, name):
        values = np.asarray(value, dtype=float)
        if values.shape == ():
            values = np.full(3, values)
        if values.shape != (3,) or not np.isfinite(values).all() or np.any(values <= 0):
            raise ValueError(f'kalman.{name} must be positive finite scalar or three-axis array')
        return values.copy()

    def reset(self):
        self.state = None
        self.covariance = self.initial_covariance.copy()

    def update(self, measurement):
        force_base = np.asarray(measurement, dtype=float)
        if force_base.shape != (3,) or not np.isfinite(force_base).all():
            raise ValueError('Kalman measurement must contain three finite Base force values')
        if not self.enabled:
            return force_base.copy()
        if self.state is None:
            # Seed from the first actual sample, never an artificial zero.
            self.state = force_base.copy()
        else:
            predicted_covariance = self.covariance + self.process_noise
            gain = predicted_covariance / (predicted_covariance + self.measurement_noise)
            self.state = self.state + gain * (force_base - self.state)
            self.covariance = (1. - gain) * predicted_covariance
        return self.state.copy()

    def settling_time(self, rate_hz):
        """Conservative 1% step settling bound for existing unload validation.

        Use the smaller of initial and steady-state gains on the slowest axis.
        This is a filter bound, not a change to the unloading strategy/budgets.
        """
        if not self.enabled:
            return 0.
        q, r = self.process_noise, self.measurement_noise
        steady_prediction = .5 * (q + np.sqrt(q*q + 4*q*r))
        prediction = np.minimum(self.initial_covariance + q, steady_prediction)
        gain = prediction / (prediction + r)
        return float(np.max(np.log(.01) / np.log1p(-gain)) / rate_hz)


@dataclass
class WrenchPreprocessor:
    """Bias/gravity in Sensor -> Base transform/baseline -> force Kalman.

    `filter_alpha` is accepted only for old constructor/config compatibility;
    it no longer filters wrench samples. Gravity is a constant wrench in
    the sensor frame and must match the stationary scan orientation. Pose
    updates rotate force using an explicitly supplied installation; they do
    not calibrate the mounting or estimate gravity at another orientation.
    """

    filter_alpha: float
    rotation_sensor_to_output: np.ndarray
    sensor_origin_in_output_m: np.ndarray
    gravity_wrench_sensor: np.ndarray
    granular_baseline_output: np.ndarray
    kalman: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not 0.0 <= self.filter_alpha < 1.0:
            raise ValueError("filter_alpha must be in [0, 1)")
        self.rotation_sensor_to_output = _rotation(
            self.rotation_sensor_to_output, 'rotation_sensor_to_output')
        self.sensor_origin_in_output_m = np.asarray(self.sensor_origin_in_output_m, dtype=float)
        if self.sensor_origin_in_output_m.shape != (3,) or not np.isfinite(self.sensor_origin_in_output_m).all():
            raise ValueError("sensor_origin_in_output_m must have three finite values")
        self.gravity_wrench_sensor = _six(self.gravity_wrench_sensor, "gravity_wrench_sensor")
        self.granular_baseline_output = _six(
            self.granular_baseline_output, "granular_baseline_output"
        )
        self.zero_bias_sensor = np.zeros(6, dtype=float)
        self.force_kalman = IndependentForceKalman(self.kalman)
        self.raw_sensor_wrench = self.force_base = self.filtered_force_base = None
        self._torque_reference_point_known = True  # explicit direct constructor
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
            filter_alpha=float(config.get("filter_alpha", .8)),
            rotation_sensor_to_output=configured,
            sensor_origin_in_output_m=np.asarray(
                transform["sensor_origin_in_base_m"], dtype=float
            ),
            gravity_wrench_sensor=np.asarray(config["gravity_wrench_sensor"], dtype=float),
            granular_baseline_output=np.asarray(config["granular_baseline_output"], dtype=float),
            kalman=config.get('kalman', {}),
        )
        # Legacy zero offset is a placeholder, not evidence of coincident origins.
        instance._torque_reference_point_known = False
        if synthetic:
            instance._torque_reference_point_known = True  # explicitly generated geometry
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
            instance._torque_reference_point_known = True
        elif instance._rotation_sensor_to_tool is not None and instance._origin_in_tool is None:
            if transform.get('reference_tool_orientation') is not None:
                instance._origin_in_tool = (_tool_rotation(transform['reference_tool_orientation']).T
                                            @ instance.sensor_origin_in_output_m)
            elif np.any(instance.sensor_origin_in_output_m):
                raise ValueError('nonzero sensor_origin_in_base_m requires reference_tool_orientation '
                                 'or sensor_origin_in_tool_m when updating tool orientation')
            else:
                instance._origin_in_tool = np.zeros(3)
        if (instance._origin_in_tool is not None and
                np.any(instance.sensor_origin_in_output_m) and
                transform.get('reference_tool_orientation') is not None):
            instance._torque_reference_point_known = True
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
                    torque_reference_point=('output_origin' if self._torque_reference_point_known
                                            else 'sensor_origin_rotation_only'),
                    wrench_reference_point_transform_complete=(
                        self._transform_status['available'] and self._torque_reference_point_known),
                    sensor_origin_in_base_m=(self.sensor_origin_in_output_m.tolist()
                        if self._transform_status['available'] and self._torque_reference_point_known else None),
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
        rows = [_six(sample.array(), 'bias sample') for sample in samples]
        if not rows:
            raise ValueError("at least one sample is required for zero bias")
        self.zero_bias_sensor = np.mean(np.stack(rows), axis=0)
        self.force_kalman.reset()
        self.raw_sensor_wrench = self.force_base = self.filtered_force_base = None

    def set_granular_baseline(self, wrench_output: Wrench) -> None:
        self.granular_baseline_output = _six(wrench_output.array(), 'granular baseline').copy()
        self.force_kalman.reset()
        self.raw_sensor_wrench = self.force_base = self.filtered_force_base = None

    @property
    def force_log_fields(self) -> dict:
        """Pre-filter Base force; raw_* and d* columns retain their old names."""
        if self.force_base is None:
            return {}
        rotation = self.rotation_sensor_to_output
        raw_force = rotation @ self.raw_sensor_wrench[:3]
        raw_torque = rotation @ self.raw_sensor_wrench[3:]
        if self._torque_reference_point_known:
            raw_torque += np.cross(self.sensor_origin_in_output_m, raw_force)
        torque = self._air_compensated_base(self.raw_sensor_wrench)[3:]-self.granular_baseline_output[3:]
        return {
            **dict(zip(('force_base_fx', 'force_base_fy', 'force_base_fz'), self.force_base)),
            **dict(zip(('force_base_tx', 'force_base_ty', 'force_base_tz'), torque)),
            **dict(zip(('raw_base_fx', 'raw_base_fy', 'raw_base_fz', 'raw_base_tx', 'raw_base_ty', 'raw_base_tz'),
                       np.r_[raw_force, raw_torque])),
        }

    def air_compensated_wrench_base(self, raw: Wrench) -> Wrench:
        """Base wrench after air bias/gravity, before granular baseline/Kalman.

        Used for independent background calibration and insertion protection;
        neither call mutates the air zero or the persistent control filter.
        """
        if raw is None:
            raise ValueError('raw sensor wrench is missing')
        values = _six(raw.array(), 'raw sensor wrench')
        if not self._transform_status['available']:
            raise ValueError('Base transform unavailable for compensated wrench')
        return Wrench.from_sequence(self._air_compensated_base(values))

    def _air_compensated_base(self, raw_sensor_wrench):
        corrected = raw_sensor_wrench-self.zero_bias_sensor-self.gravity_wrench_sensor
        rotation = self.rotation_sensor_to_output
        force_sensor = corrected[:3]
        rotated_force_base = rotation @ force_sensor
        torque_base = rotation @ corrected[3:]
        if self._torque_reference_point_known:
            torque_base += np.cross(self.sensor_origin_in_output_m, rotated_force_base)
        return _six(np.r_[rotated_force_base, torque_base], 'air-compensated Base wrench')

    def process(self, raw: Wrench) -> Wrench:
        if raw is None:
            raise ValueError('raw sensor wrench is missing')
        raw_sensor_wrench = _six(raw.array(), 'raw sensor wrench')
        corrected = raw_sensor_wrench - self.zero_bias_sensor - self.gravity_wrench_sensor
        if not self._transform_status['available']:
            # Read-only/return diagnostics only. Never update a Base Kalman with
            # unknown-frame data; scanning runtimes must reject this status.
            self.raw_sensor_wrench = raw_sensor_wrench.copy()
            self.force_base = self.filtered_force_base = None
            return Wrench.from_sequence(corrected)
        compensated_base = self._air_compensated_base(raw_sensor_wrench)
        force_base = compensated_base[:3]-self.granular_baseline_output[:3]
        torque_base = compensated_base[3:]-self.granular_baseline_output[3:]
        # Validate the entire output before touching persistent filter state.
        _six(np.r_[force_base, torque_base], 'transformed wrench')
        filtered_force_base = self.force_kalman.update(force_base)
        self.raw_sensor_wrench = raw_sensor_wrench.copy()
        self.force_base = force_base.copy()
        self.filtered_force_base = filtered_force_base.copy()
        # Torque is deliberately not force-filtered or silently moved to TCP.
        return Wrench.from_sequence(np.r_[filtered_force_base, torque_base])

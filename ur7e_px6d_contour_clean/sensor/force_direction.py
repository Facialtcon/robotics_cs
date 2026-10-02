"""Fixed-orientation force convention shared by control and read-only checks.

R_BS maps sensor components directly to Base (R_BT @ R_TS if independently
calibrated). TCP pose alone does not identify R_TS. n points toward increasing
contact compression; unloading is -n. Fxy remains a processed resultant,
including friction/background, not an isolated target normal force.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

from policy.boundary_estimation import handed_tangent

AXES = {f'{sign}{axis}': np.eye(3)[i]*value
        for i, axis in enumerate('XYZ') for sign, value in [('+', 1), ('-', -1)]}


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


def direction_binding(config, config_path):
    source = Path(config_path).resolve()
    scan = Path(config['calibration']['file'])
    if not scan.is_absolute():
        scan = source.parent/scan
    payload = dict(preprocessing=config['preprocessing'], tcp=config['tcp'],
                   sensor=config['sensor']['serial_port'],
                   force_direction_sign=config['continuous_tracking']['force_direction_sign'],
                   scan_sha256=hashlib.sha256(scan.read_bytes()).hexdigest())
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def evaluate_direction_measurements(config, measurements):
    """Known external force vectors act ON the stationary probe, not 'from' an axis.

    Validate the configured mapping; never fit a rotation or flip a sign.
    Three axes and both polarities distinguish axis/sign errors and bias drift.
    """
    rotation = np.asarray(config['preprocessing']['coordinate_transform']['rotation_sensor_to_base'], float)
    sign = config['continuous_tracking']['force_direction_sign']
    results = {}
    if not isinstance(measurements, dict):
        raise ValueError('direction measurements must be an axis mapping')
    for label, expected in AXES.items():
        item = measurements.get(label, {})
        if not isinstance(item, dict):
            raise ValueError('invalid direction measurement record')
        mean = np.asarray(item.get('sensor_debiased_mean_N', [np.nan]*3), float)
        std = np.asarray(item.get('sensor_std_N', [np.nan]*3), float)
        if mean.shape != (3,) or std.shape != (3,):
            raise ValueError('direction measurement must contain three force axes')
        base = rotation @ mean
        physical = -sign*base  # n=-F_environment_on_probe / |Fxy|.
        norm = float(np.linalg.norm(physical))
        cosine = float(physical @ expected/norm) if norm > 1e-12 else -1.
        angle = float(np.rad2deg(np.arccos(np.clip(cosine, -1., 1.))))
        good = (np.isfinite(np.r_[mean, std, angle]).all() and item.get('sample_count', 0) >= 20
                and .5 <= norm <= 3. and angle <= 15. and np.linalg.norm(std) <= .15)
        results[label] = dict(passed=bool(good), base_reported_force_N=base.tolist(),
            environment_on_probe_N=physical.tolist(), angle_error_deg=angle, magnitude_N=norm)
    return dict(verified=all(item['passed'] for item in results.values()), axes=results)


def verification_path(config, config_path):
    path = Path(config['continuous_tracking'].get('force_direction_verification_file',
                                               'calibration/force_direction_verification.json'))
    return path if path.is_absolute() else Path(config_path).resolve().parent/path


def load_direction_verification(config, config_path):
    result = dict(verified=False, reason='no measured force-direction verification',
                  reference=str(verification_path(config, config_path)))
    try:
        record = json.loads(Path(result['reference']).read_text())
        if not isinstance(record, dict):
            raise ValueError('invalid direction verification record')
        if record.get('configuration_sha256') != direction_binding(config, config_path):
            raise ValueError('verification does not match sensor/frame/sign/TCP/scan configuration')
        if (record.get('schema_version') != 1 or record.get('unloaded_bias_confirmed') is not True
                or not record.get('operator') or not record.get('timestamp_utc')):
            raise ValueError('missing operator/time/unloaded-start evidence')
        evidence = evaluate_direction_measurements(config, record.get('measurements', {}))
        if not evidence['verified'] or record.get('completed') is not True:
            raise ValueError('measured axis/sign checks did not pass')
        result.update(verified=True, reason='six applied-force directions passed at saved fixed orientation',
            reported_force_convention=('environment_on_probe' if config['continuous_tracking']['force_direction_sign'] == -1
                                       else 'probe_on_environment'),
            physical_force_multiplier=-config['continuous_tracking']['force_direction_sign'])
    except (OSError, ValueError, TypeError, KeyError) as exc:
        result['reason'] = str(exc)
    return result

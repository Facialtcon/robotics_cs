"""Hardware-free gravity calibration, using raw samples and actual TCP poses.

Column-vector convention: f_sensor = b + sign * weight_N * R.T @ g_tool,
where R = rotation_sensor_to_tool and g_tool = R_base_tool.T @ [0, 0, -1].
Each record has pose_id, split ('fit' or 'validation'), actual_tcp_pose[6],
raw_wrench[6], host_monotonic, robot_timestamp, utc_time. Whole pose groups
are held out; their data never enter the fit or the selection of force sign.
No filtering, taring, coordinate preprocessing, or gravity subtraction occurs.
"""
from dataclasses import asdict, dataclass
from typing import Any, Iterable

import numpy as np


class CalibrationError(ValueError):
    """Rejected data; diagnostics may be archived, never installed as a result."""

    def __init__(self, message: str, diagnostics: dict | None = None):
        super().__init__(message)
        self.diagnostics = diagnostics or {}


@dataclass(frozen=True)
class CalibrationLimits:
    min_fit_poses: int = 8
    min_validation_poses: int = 3
    min_samples_per_pose: int = 20
    min_pose_duration_s: float = 0.3
    max_position_span_m: float = 0.0006
    max_orientation_span_deg: float = 0.3
    max_force_std_n: float = 0.12
    max_torque_std_nm: float = 0.025
    max_force_peak_deviation_n: float = 0.6
    max_torque_peak_deviation_nm: float = 0.12
    min_gravity_axis_rms: float = 0.008
    max_gravity_condition: float = 60.0
    min_tilt_span_deg: float = 15.0
    min_validation_separation_deg: float = 1.0
    min_force_signal_n: float = 0.15
    min_weak_axis_signal_n: float = 0.025
    min_sign_rmse_gap_n: float = 0.04
    sign_uncertainty_multiplier: float = 3.0
    max_fit_rmse_n: float = 0.12
    max_fit_pose_error_n: float = 0.25
    max_validation_rmse_n: float = 0.15
    max_validation_pose_error_n: float = 0.30
    max_relative_fit_error: float = 0.10
    max_relative_validation_error: float = 0.12
    # Optional model-based precision target. None preserves standard acceptance.
    max_rotation_uncertainty_deg: float | None = None


def rotation_from_rotvec(rotvec: Iterable[float]) -> np.ndarray:
    """UR actual TCP rotation vector to a proper Base-from-tool rotation."""
    vector = np.asarray(rotvec, dtype=float)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise CalibrationError('invalid actual TCP rotation vector')
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        # First-order term keeps tiny actual-pose perturbations, including zero.
        axis = vector
        skew = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]],
                         [-axis[1], axis[0], 0.]])
        return np.eye(3) + skew + 0.5 * skew @ skew
    axis = vector / angle
    skew = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]],
                     [-axis[1], axis[0], 0.]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * skew @ skew


def _angles(vectors: np.ndarray, reference: np.ndarray) -> np.ndarray:
    dots = vectors @ reference.T
    norms = np.linalg.norm(vectors, axis=-1)[:, None] * np.linalg.norm(reference, axis=-1)[None, :]
    return np.degrees(np.arccos(np.clip(dots / norms, -1., 1.)))


def _pose_groups(records: list[dict], limits: CalibrationLimits) -> list[dict]:
    groups: dict[Any, list[dict]] = {}
    previous_time = -np.inf
    for index, record in enumerate(records):
        if not isinstance(record, dict):
            raise CalibrationError(f'record {index}: expected a sample mapping')
        try:
            pose_id = record['pose_id']
            if isinstance(pose_id, bool) or not isinstance(pose_id, (int, str)):
                raise ValueError('pose_id must be an integer or string')
            split = record['split']
            if split not in ('fit', 'validation'):
                raise ValueError('split must be fit or validation')
            pose = np.asarray(record['actual_tcp_pose'], dtype=float)
            wrench = np.asarray(record['raw_wrench'], dtype=float)
            host_time = float(record['host_monotonic'])
            robot_time = float(record['robot_timestamp'])
            utc = record['utc_time']
            if pose.shape != (6,) or wrench.shape != (6,):
                raise ValueError('expected six pose and six raw wrench components')
            if not np.isfinite(pose).all() or not np.isfinite(wrench).all():
                raise ValueError('nonfinite pose or raw wrench')
            if not np.isfinite([host_time, robot_time]).all() or not isinstance(utc, str) or not utc:
                raise ValueError('missing or nonfinite acquisition timestamps')
            if host_time <= previous_time:
                raise ValueError('host acquisition times must be strictly increasing')
            previous_time = host_time
        except (KeyError, ValueError, TypeError) as exc:
            raise CalibrationError(f'invalid record {index}: {exc}') from exc
        groups.setdefault(pose_id, []).append(record)

    poses = []
    for pose_id, samples in groups.items():
        splits = {sample['split'] for sample in samples}
        if len(splits) != 1:
            raise CalibrationError(f'pose {pose_id}: complete-pose validation violated; mixed splits')
        if len(samples) < limits.min_samples_per_pose:
            raise CalibrationError(f'pose {pose_id}: insufficient sample count')
        raw = np.asarray([s['raw_wrench'] for s in samples], dtype=float)
        actual = np.asarray([s['actual_tcp_pose'] for s in samples], dtype=float)
        host_times = np.asarray([s['host_monotonic'] for s in samples], dtype=float)
        robot_times = np.asarray([s['robot_timestamp'] for s in samples], dtype=float)
        if np.any(np.diff(robot_times) < 0):
            raise CalibrationError(f'pose {pose_id}: robot timestamp moved backwards')
        if robot_times[-1] <= robot_times[0]:
            raise CalibrationError(f'pose {pose_id}: stale robot timestamps')
        duration = float(host_times[-1] - host_times[0])
        if duration < limits.min_pose_duration_s:
            raise CalibrationError(f'pose {pose_id}: insufficient sampling duration')
        rotations = np.asarray([rotation_from_rotvec(p[3:]) for p in actual])
        gravity = np.einsum('nji,j->ni', rotations, np.array([0., 0., -1.]))
        # Pairwise distances bound every sample, without hiding drift in a mean.
        positions = actual[:, :3]
        position_span = float(np.max(np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=-1)))
        traces = np.einsum('nij,mij->nm', rotations, rotations)
        orientation_span = float(np.degrees(np.arccos(np.clip((traces.min() - 1.) / 2., -1., 1.))))
        std = np.std(raw, axis=0, ddof=1)
        peaks = np.max(np.abs(raw - raw.mean(axis=0)), axis=0)
        diagnostics = dict(pose_id=pose_id, split=next(iter(splits)), sample_count=len(samples),
                           duration_s=duration, position_span_m=position_span,
                           orientation_span_deg=orientation_span, raw_wrench_std=std.tolist(),
                           raw_wrench_peak_deviation=peaks.tolist(),
                           mean_gravity_tool=gravity.mean(axis=0).tolist(),
                           mean_raw_wrench=raw.mean(axis=0).tolist())
        if position_span > limits.max_position_span_m or orientation_span > limits.max_orientation_span_deg:
            raise CalibrationError(f'pose {pose_id}: actual TCP was not stable', diagnostics)
        if (std[:3].max() > limits.max_force_std_n or std[3:].max() > limits.max_torque_std_nm
                or peaks[:3].max() > limits.max_force_peak_deviation_n
                or peaks[3:].max() > limits.max_torque_peak_deviation_nm):
            raise CalibrationError(f'pose {pose_id}: raw force/torque was not stable', diagnostics)
        poses.append(dict(id=pose_id, split=next(iter(splits)), gravity=gravity.mean(axis=0),
                          force=raw[:, :3].mean(axis=0), raw_force=raw[:, :3],
                          sample_gravity=gravity, diagnostics=diagnostics,
                          force_mean_uncertainty=float(np.sqrt(np.sum(std[:3] ** 2) / len(samples)))))
    return poses


def _candidate(gravity: np.ndarray, force: np.ndarray, sign: int) -> dict:
    x = gravity - gravity.mean(axis=0)
    y = force - force.mean(axis=0)
    cross = x.T @ (sign * y)
    u, _, vt = np.linalg.svd(cross)
    correction = np.diag([1., 1., float(np.linalg.det(u @ vt))])
    rotation = u @ correction @ vt
    weight = max(0., float(np.sum((x @ rotation) * (sign * y)) / np.sum(x * x)))
    bias = force.mean(axis=0) - sign * weight * gravity.mean(axis=0) @ rotation
    residual = force - (bias + sign * weight * gravity @ rotation)
    errors = np.linalg.norm(residual, axis=1)
    return dict(rotation_sensor_to_tool=rotation.tolist(), force_sign=sign,
                weight_N=weight, bias_sensor_N=bias.tolist(),
                fit_rmse_N=float(np.sqrt(np.mean(errors ** 2))),
                fit_max_pose_error_N=float(errors.max()), fit_pose_errors_N=errors.tolist())


def _rotation_uncertainty(gravity: np.ndarray, candidate: dict,
                          force_mean_uncertainty_n: float) -> dict:
    """Local angular uncertainty of the unchanged, seven-parameter fitted model.

    Each complete fit pose contributes three equally weighted observations.
    The held-out residual supplies only a conservative noise scale; held-out
    poses never enter the parameter fit, Jacobian, or force-sign selection.
    This approximation assumes independent, isotropic pose-mean errors and a
    correct fixed-bias gravity model. It cannot bound unobserved systematic
    errors such as incorrect gravity direction or pose-correlated cable loads.
    """
    rotation = np.asarray(candidate['rotation_sensor_to_tool'])
    sign, weight = candidate['force_sign'], candidate['weight_N']
    jacobian = []
    for vector in gravity @ rotation:
        x, y, z = vector
        skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
        # Right perturbation R @ exp([delta_theta]x), bias, positive weight.
        jacobian.append(np.column_stack((sign * weight * skew, np.eye(3), sign * vector)))
    jacobian = np.vstack(jacobian)
    degrees_of_freedom = 3 * len(gravity) - 7
    fit_variance = len(gravity) * candidate['fit_rmse_N'] ** 2 / degrees_of_freedom
    validation_variance = candidate['validation_rmse_N'] ** 2 / 3.
    sampling_variance = force_mean_uncertainty_n ** 2 / 3.
    variance = max(fit_variance, validation_variance, sampling_variance)
    assumptions = ('Local 95% approximation for the fixed-bias gravity model with independent, '
                   'isotropic complete-pose mean errors; not an absolute accuracy guarantee '
                   'or a bound on systematic errors.')
    details = dict(parameter_count=7, fit_observation_count=int(jacobian.shape[0]),
                   residual_degrees_of_freedom=degrees_of_freedom,
                   fit_variance_N2=float(fit_variance),
                   validation_variance_N2=float(validation_variance),
                   sampling_variance_floor_N2=float(sampling_variance),
                   applied_variance_N2=float(variance),
                   confidence_probability=.95, chi_squared_3d_95=7.814727903251179,
                   is_accuracy_guarantee=False,
                   fit_pose_count=len(gravity), heldout_used_only_for_noise_scale=True)
    _, singular, vt = np.linalg.svd(jacobian, full_matrices=False)
    rank_tolerance = np.finfo(float).eps * max(jacobian.shape) * singular[0]
    if degrees_of_freedom <= 0 or singular[-1] <= rank_tolerance:
        details['status'] = 'insufficient_information'
        return dict(estimated_rotation_95_bound_deg=None,
                    rotation_uncertainty_assumptions=assumptions,
                    rotation_uncertainty=details)
    # SVD avoids squaring the Jacobian condition number in a normal-equation
    # inversion. The angular block includes joint bias/weight uncertainty.
    covariance = variance * ((vt.T / singular ** 2) @ vt)
    angular_covariance = covariance[:3, :3]
    largest_variance = max(0., float(np.linalg.eigvalsh(angular_covariance)[-1]))
    bound = float(np.degrees(np.sqrt(details['chi_squared_3d_95'] * largest_variance)))
    details.update(status='estimated', jacobian_condition=float(singular[0] / singular[-1]),
                   angular_covariance_rad2=angular_covariance.tolist())
    return dict(estimated_rotation_95_bound_deg=bound,
                rotation_uncertainty_assumptions=assumptions,
                rotation_uncertainty=details)


def calibrate(records: Iterable[dict], limits: CalibrationLimits | None = None) -> dict:
    """Fit an SO(3) mount and independent action sign; reject unsafe/weak results.

    Returned bias and mass are diagnostic only. A caller must not put them into
    scanning compensation. The model fixes gravity to Base -Z and therefore
    requires a level-base confirmation before data acquisition.
    """
    limits = limits or CalibrationLimits()
    for name, value in asdict(limits).items():
        if name == 'max_rotation_uncertainty_deg' and value is None:
            continue
        if not np.isfinite(value) or value <= 0:
            raise CalibrationError(f'invalid positive calibration limit: {name}')
    poses = _pose_groups(list(records), limits)
    training = [p for p in poses if p['split'] == 'fit']
    validation = [p for p in poses if p['split'] == 'validation']
    if len(training) < limits.min_fit_poses or len(validation) < limits.min_validation_poses:
        raise CalibrationError('insufficient complete fit or validation poses')
    gravity = np.array([p['gravity'] for p in training])
    force = np.array([p['force'] for p in training])
    held_gravity = np.array([p['gravity'] for p in validation])
    held_force = np.array([p['force'] for p in validation])
    centered = gravity - gravity.mean(axis=0)
    singular = np.linalg.svd(centered, compute_uv=False) / np.sqrt(len(training))
    condition = float(singular[0] / max(singular[-1], np.finfo(float).eps))
    angular_span = float(_angles(gravity, gravity).max())
    diagnostics = dict(gravity_axis_rms=singular.tolist(), gravity_condition=condition,
                       gravity_angular_span_deg=angular_span,
                       fit_pose_ids=[p['id'] for p in training],
                       validation_pose_ids=[p['id'] for p in validation],
                       poses=[p['diagnostics'] for p in poses], limits=asdict(limits))
    if (singular[-1] < limits.min_gravity_axis_rms or condition > limits.max_gravity_condition
            or angular_span < limits.min_tilt_span_deg):
        raise CalibrationError('insufficient 3D gravity coverage / identifiability; do not expand motion automatically', diagnostics)
    separations = _angles(held_gravity, gravity).min(axis=1)
    diagnostics['validation_nearest_fit_angle_deg'] = separations.tolist()
    if np.any(separations < limits.min_validation_separation_deg):
        raise CalibrationError('validation requires independent complete orientations', diagnostics)
    signal = float(np.sqrt(np.mean(np.sum((force - force.mean(axis=0)) ** 2, axis=1))))
    diagnostics['force_signal_N'] = signal
    if signal < limits.min_force_signal_n:
        raise CalibrationError('insufficient gravity force signal', diagnostics)
    candidates = sorted((_candidate(gravity, force, sign) for sign in (1, -1)), key=lambda c: c['fit_rmse_N'])
    best, alternative = candidates
    # Diagnose BOTH fixed, already-fitted candidates on complete held-out poses.
    # These errors neither select the sign nor change rotation, weight or bias.
    for candidate in candidates:
        predictions = (np.asarray(candidate['bias_sensor_N'])
                       + candidate['force_sign'] * candidate['weight_N']
                       * held_gravity @ np.asarray(candidate['rotation_sensor_to_tool']))
        held_errors = np.linalg.norm(held_force - predictions, axis=1)
        candidate.update(validation_rmse_N=float(np.sqrt(np.mean(held_errors ** 2))),
                         validation_max_pose_error_N=float(held_errors.max()),
                         validation_pose_errors_N=held_errors.tolist())
    diagnostics['sign_candidates'] = candidates
    noise = float(np.sqrt(np.mean([p['force_mean_uncertainty'] ** 2 for p in training])))
    diagnostics['force_mean_uncertainty_N'] = noise
    gap = alternative['fit_rmse_N'] - best['fit_rmse_N']
    required_gap = max(limits.min_sign_rmse_gap_n, limits.sign_uncertainty_multiplier * noise,
                       2. * best['fit_rmse_N'])
    diagnostics.update(sign_rmse_gap_N=gap, required_sign_rmse_gap_N=required_gap)
    weak_signal = float(best['weight_N'] * singular[-1])
    diagnostics['weak_axis_force_signal_N'] = weak_signal
    validation_rmse = best['validation_rmse_N']
    diagnostics.update(validation_rmse_N=validation_rmse,
                       validation_max_pose_error_N=best['validation_max_pose_error_N'],
                       validation_pose_errors_N=best['validation_pose_errors_N'])
    uncertainty = _rotation_uncertainty(gravity, best, noise)
    diagnostics.update(uncertainty)
    failures = []
    # A large model residual can also obscure the action sign. Report the
    # fixed-bias model failure first, instead of inviting a sign override.
    if (best['fit_rmse_N'] > limits.max_fit_rmse_n
            or best['fit_max_pose_error_N'] > limits.max_fit_pose_error_n
            or best['fit_rmse_N'] / signal > limits.max_relative_fit_error):
        failures.append('excessive fit error; fixed-bias gravity model inconsistent '
                        f"(fit RMSE {best['fit_rmse_N']:.4f} N, "
                        f'independent validation RMSE {validation_rmse:.4f} N)')
    if gap <= required_gap:
        failures.append('force action sign is ambiguous; no installation result')
    if weak_signal < max(limits.min_weak_axis_signal_n, 2. * noise):
        failures.append('insufficient force signal for third-axis identifiability')
    if (validation_rmse > limits.max_validation_rmse_n
            or best['validation_max_pose_error_N'] > limits.max_validation_pose_error_n
            or validation_rmse / signal > limits.max_relative_validation_error):
        failures.append('excessive independent complete-pose validation error')
    angular_bound = uncertainty['estimated_rotation_95_bound_deg']
    if limits.max_rotation_uncertainty_deg is not None:
        if angular_bound is None or not np.isfinite(angular_bound):
            failures.append('insufficient information to estimate rotation uncertainty')
        elif angular_bound > limits.max_rotation_uncertainty_deg:
            failures.append('excessive estimated rotation uncertainty '
                            f'({angular_bound:.2f} deg > {limits.max_rotation_uncertainty_deg:g} deg; '
                            'local model estimate, not guaranteed accuracy)')
    diagnostics['failure_reasons'] = failures
    if failures:
        raise CalibrationError('; '.join(failures), diagnostics)
    rotation = np.array(best['rotation_sensor_to_tool'])
    bias = np.array(best['bias_sensor_N'])
    # This assertion is a final guard on the returned convention, not a sign fix.
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-10) or not np.isclose(np.linalg.det(rotation), 1., atol=1e-10):
        raise CalibrationError('computed mounting matrix is not a proper rotation', diagnostics)
    diagnostics['sample_force_rmse_N'] = float(np.sqrt(np.mean([
        np.mean(np.sum((p['raw_force'] - (bias + best['force_sign'] * best['weight_N'] * p['sample_gravity'] @ rotation)) ** 2, axis=1))
        for p in poses])))
    return dict(schema_version=1, accepted=True,
                model='f_sensor = bias_sensor_N + force_sign * weight_N * rotation_sensor_to_tool.T @ gravity_tool',
                gravity_base_unit=[0., 0., -1.],
                rotation_sensor_to_tool=best['rotation_sensor_to_tool'], force_sign=best['force_sign'],
                weight_N=best['weight_N'], diagnostic_mass_kg=best['weight_N'] / 9.80665,
                bias_sensor_N=best['bias_sensor_N'],
                diagnostic_only_parameters=['weight_N', 'diagnostic_mass_kg', 'bias_sensor_N', 'force_sign'],
                fit_rmse_N=best['fit_rmse_N'], validation_rmse_N=validation_rmse,
                estimated_rotation_95_bound_deg=angular_bound,
                rotation_uncertainty_assumptions=uncertainty['rotation_uncertainty_assumptions'],
                diagnostics=diagnostics)

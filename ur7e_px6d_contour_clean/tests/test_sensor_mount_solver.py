"""Synthetic independent-pose checks for the standalone mount solver."""
from dataclasses import replace
import copy
import json

import numpy as np
import pytest

from tools.sensor_mount_calibration.solver import (
    CalibrationError, CalibrationLimits, calibrate, rotation_from_rotvec,
)


def synthetic_records(mount=None, bias=(0.8, -1.3, 2.1), weight=12., sign=1,
                      noise=.015, samples=40, seed=73, actual_jitter=False):
    """13 fit and 4 withheld poses at distinct azimuths; never split samples."""
    rng = np.random.default_rng(seed)
    mount = rotation_from_rotvec([1.4, -.7, 2.1]) if mount is None else mount
    planned = [(np.zeros(3), 'fit')]
    for degrees, azimuths, split in ((12., range(0, 360, 45), 'fit'),
                                   (25., range(0, 360, 90), 'fit'),
                                   (25., range(45, 360, 90), 'validation')):
        for azimuth in azimuths:
            radians = np.radians(azimuth)
            planned.append((np.radians(degrees) * np.array([np.cos(radians), np.sin(radians), 0.]), split))
    records = []
    for pose_id, (rotvec, split) in enumerate(planned):
        for _ in range(samples):
            actual = rotvec + (rng.normal(0., .00015, 3) if actual_jitter else 0.)
            gravity = rotation_from_rotvec(actual).T @ [0., 0., -1.]
            force = np.asarray(bias) + sign * weight * mount.T @ gravity + rng.normal(0., noise, 3)
            t = len(records) * .02
            records.append(dict(pose_id=pose_id, split=split,
                                actual_tcp_pose=[.35, -.2, .4, *actual],
                                raw_wrench=[*force, *rng.normal(0., noise / 20, 3)],
                                host_monotonic=10. + t, robot_timestamp=20. + t,
                                utc_time=f'2026-10-03T00:00:{t:.3f}Z'))
    return records


def rotation_error_deg(a, b):
    return np.degrees(np.arccos(np.clip((np.trace(np.asarray(a).T @ b) - 1.) / 2., -1., 1.)))


@pytest.mark.parametrize('sign', [1, -1])
@pytest.mark.parametrize('rotvec', [[0., 0., 0.], [1.4, -.7, 2.1], [-2.3, .9, -.45], [3.1415926, 0., 0.]])
def test_arbitrary_mount_bias_noise_and_separate_sign(sign, rotvec):
    mount = rotation_from_rotvec(rotvec)
    result = calibrate(synthetic_records(mount=mount, sign=sign, actual_jitter=True))
    recovered = np.asarray(result['rotation_sensor_to_tool'])
    assert rotation_error_deg(recovered, mount) < .2
    assert result['force_sign'] == sign
    np.testing.assert_allclose(recovered.T @ recovered, np.eye(3), atol=1e-12)
    assert np.linalg.det(recovered) == pytest.approx(1., abs=1e-12)
    np.testing.assert_allclose(result['bias_sensor_N'], [.8, -1.3, 2.1], atol=.025)
    assert result['weight_N'] == pytest.approx(12., abs=.02)
    assert result['fit_rmse_N'] < .012
    assert result['validation_rmse_N'] < .012
    assert len(result['diagnostics']['validation_pose_ids']) == 4
    assert result['diagnostic_only_parameters'] == ['weight_N', 'diagnostic_mass_kg', 'bias_sensor_N', 'force_sign']
    json.dumps(result, allow_nan=False)


def test_actual_pose_overrides_any_nominal_pose():
    records = synthetic_records(noise=0., actual_jitter=True)
    for record in records:
        record['planned_tcp_pose'] = [0.] * 6
    result = calibrate(records)
    assert result['fit_rmse_N'] < 1e-12
    assert result['validation_rmse_N'] < 1e-12


def test_no_force_signal_rejected():
    with pytest.raises(CalibrationError, match='insufficient gravity force signal'):
        calibrate(synthetic_records(weight=0., noise=.01))


def test_ambiguous_force_action_sign_rejected():
    # The raw noise is acceptable but the weak curvature of the measured cone
    # cannot confidently distinguish the reflected-force hypothesis.
    with pytest.raises(CalibrationError, match='sign is ambiguous'):
        calibrate(synthetic_records(weight=.5, noise=.001),
                  replace(CalibrationLimits(), min_force_signal_n=.05))


def test_constant_tilt_ring_has_unidentifiable_sign_and_bias():
    records = synthetic_records(noise=0.)
    mount = rotation_from_rotvec([1.4, -.7, 2.1])
    for record in records:
        angle = record['pose_id'] * 2. * np.pi / 17
        rv = np.radians(25.) * np.array([np.cos(angle), np.sin(angle), 0.])
        record['actual_tcp_pose'][3:] = rv.tolist()
        record['raw_wrench'][:3] = (.8 + 12. * mount.T @ rotation_from_rotvec(rv).T @ [0., 0., -1.]).tolist()
    with pytest.raises(CalibrationError, match='3D gravity coverage'):
        calibrate(records)


def test_vertical_axis_only_rotation_rejected():
    records = synthetic_records()
    for record in records:
        record['actual_tcp_pose'][3:] = [0., 0., record['pose_id'] * .01]
    with pytest.raises(CalibrationError, match='3D gravity coverage'):
        calibrate(records)


def test_insufficient_pose_count_rejected():
    records = [r for r in synthetic_records() if r['pose_id'] != 15 and r['pose_id'] != 16]
    with pytest.raises(CalibrationError, match='insufficient complete fit or validation poses'):
        calibrate(records)


def test_whole_pose_holdout_is_required():
    records = synthetic_records()
    records[0]['split'] = 'validation'
    with pytest.raises(CalibrationError, match='complete-pose validation violated'):
        calibrate(records)


def test_heldout_pose_cannot_duplicate_fit_orientation():
    records = synthetic_records()
    for record in records:
        if record['pose_id'] == 13:
            record['actual_tcp_pose'][3:] = [0., 0., 0.]
    with pytest.raises(CalibrationError, match='independent complete orientations'):
        calibrate(records)


def test_bad_heldout_data_are_rejected_instead_of_refitted():
    records = synthetic_records()
    baseline = calibrate(records)
    for record in records:
        if record['split'] == 'validation':
            record['raw_wrench'][0] += .7
    with pytest.raises(CalibrationError, match='independent complete-pose validation error') as caught:
        calibrate(records)
    fit = caught.value.diagnostics['sign_candidates'][0]
    assert fit['rotation_sensor_to_tool'] == baseline['rotation_sensor_to_tool']
    assert fit['bias_sensor_N'] == baseline['bias_sensor_N']


def test_acceptable_heldout_change_does_not_change_fit():
    records = synthetic_records()
    baseline = calibrate(records)
    for record in records:
        if record['split'] == 'validation':
            record['raw_wrench'][0] += .025
    result = calibrate(records)
    assert result['rotation_sensor_to_tool'] == baseline['rotation_sensor_to_tool']
    assert result['bias_sensor_N'] == baseline['bias_sensor_N']
    assert result['weight_N'] == baseline['weight_N']
    assert result['validation_rmse_N'] > baseline['validation_rmse_N']


@pytest.mark.parametrize('field,index,value,message', [
    ('actual_tcp_pose', 0, .02, 'actual TCP was not stable'),
    ('actual_tcp_pose', 3, .03, 'actual TCP was not stable'),
    ('raw_wrench', 0, 3., 'raw force/torque was not stable'),
    ('raw_wrench', 4, .4, 'raw force/torque was not stable'),
])
def test_unstable_samples_rejected(field, index, value, message):
    records = synthetic_records()
    records[0][field][index] += value
    with pytest.raises(CalibrationError, match=message):
        calibrate(records)


def test_pose_dependent_force_bias_rejected():
    records = synthetic_records()
    for record in records:
        if record['pose_id'] == 3:
            record['raw_wrench'][1] += .6
    with pytest.raises(CalibrationError, match='excessive fit error'):
        calibrate(records)


@pytest.mark.parametrize('corruption,message', [
    ('nonfinite', 'nonfinite'), ('duplicate_time', 'strictly increasing'),
    ('robot_reverse', 'moved backwards'), ('few_samples', 'sample count'),
    ('missing_timestamp', 'invalid record'), ('stale_robot', 'stale robot'),
])
def test_invalid_raw_data_rejected(corruption, message):
    records = synthetic_records()
    if corruption == 'nonfinite':
        records[0]['raw_wrench'][0] = float('nan')
    elif corruption == 'duplicate_time':
        records[1]['host_monotonic'] = records[0]['host_monotonic']
    elif corruption == 'robot_reverse':
        records[1]['robot_timestamp'] = records[0]['robot_timestamp'] - 1.
    elif corruption == 'few_samples':
        records = records[30:]
    elif corruption == 'stale_robot':
        for record in records:
            if record['pose_id'] == 0:
                record['robot_timestamp'] = 20.
    else:
        del records[0]['utc_time']
    with pytest.raises(CalibrationError, match=message):
        calibrate(records)


def test_solver_does_not_mutate_raw_records():
    records = synthetic_records()
    original = copy.deepcopy(records)
    calibrate(records)
    assert records == original


def test_invalid_limits_rejected():
    with pytest.raises(CalibrationError, match='invalid positive calibration limit'):
        calibrate(synthetic_records(), replace(CalibrationLimits(), min_gravity_axis_rms=0.))


@pytest.mark.parametrize('sign', [1, -1])
@pytest.mark.parametrize('start_rotvec', [
    [1.1, -.4, .9], [np.pi, 0., 0.], [0., np.pi / 2., 0.], [1.8, 2., -1.5],
])
def test_actual_motion_plan_identifies_mount_from_arbitrary_start(start_rotvec, sign):
    """Base-horizontal tilts preserve full gravity coverage for any starting TCP."""
    from tools.sensor_mount_calibration.plan import make_plan

    rng = np.random.default_rng(193)
    mount = rotation_from_rotvec([-.9, 2.4, 1.3])
    bias = np.array([.7, -.4, 1.2])
    records = []
    plan = make_plan([.35, -.2, .4, *start_rotvec])
    for point in plan:
        if not point['acquire']:
            continue
        for _ in range(40):
            actual = np.array(point['target'])
            actual[3:] += rng.normal(0., .0001, 3)
            gravity = rotation_from_rotvec(actual[3:]).T @ [0., 0., -1.]
            force = bias + sign * 8. * mount.T @ gravity + rng.normal(0., .025, 3)
            stamp = len(records) * .02
            records.append(dict(pose_id=point['pose_id'], split=point['split'],
                                actual_tcp_pose=actual.tolist(), raw_wrench=[*force, 0., 0., 0.],
                                host_monotonic=10. + stamp, robot_timestamp=20. + stamp,
                                utc_time='2026-10-03T00:00:00Z'))
    result = calibrate(records)
    assert result['force_sign'] == sign
    assert result['weight_N'] == pytest.approx(8., abs=.025)
    assert rotation_error_deg(result['rotation_sensor_to_tool'], mount) < .35
    np.testing.assert_allclose(result['bias_sensor_N'], bias, atol=.035)
    assert len(result['diagnostics']['fit_pose_ids']) == 13
    assert len(result['diagnostics']['validation_pose_ids']) == 4
    assert result['diagnostics']['gravity_condition'] < 6.
    assert result['validation_rmse_N'] < .015


def test_small_weight_noisy_sign_does_not_pass_from_noise_fit():
    # Repeat independent noise realizations: acceptable within-pose noise alone
    # is insufficient evidence for the weak gravity-curvature direction.
    for seed in range(12):
        records = synthetic_records(weight=.8, noise=.08, samples=20, seed=seed)
        with pytest.raises(CalibrationError, match='sign is ambiguous|insufficient'):
            calibrate(records)


def test_temporal_bias_drift_reports_model_mismatch_before_sign_ambiguity():
    records = synthetic_records(weight=2.4)
    first = records[0]['host_monotonic']
    duration = records[-1]['host_monotonic'] - first
    for record in records:
        record['raw_wrench'][2] -= 1.3 * (record['host_monotonic']-first) / duration
    with pytest.raises(CalibrationError, match='excessive fit error; fixed-bias gravity model inconsistent') as caught:
        calibrate(records)
    diagnostics = caught.value.diagnostics
    assert diagnostics['sign_candidates'][0]['fit_rmse_N'] > .12
    assert diagnostics['validation_rmse_N'] > .15
    assert any('sign is ambiguous' in reason for reason in diagnostics['failure_reasons'])
    assert any('independent complete-pose validation' in reason for reason in diagnostics['failure_reasons'])
    assert all('validation_pose_errors_N' in candidate for candidate in diagnostics['sign_candidates'])


def test_ambiguous_sign_keeps_independent_validation_diagnostics_without_selecting_sign():
    records = synthetic_records(weight=.5, noise=.001)
    limits = replace(CalibrationLimits(), min_force_signal_n=.05)
    with pytest.raises(CalibrationError, match='sign is ambiguous') as baseline:
        calibrate(records, limits)
    candidates = baseline.value.diagnostics['sign_candidates']
    for record in records:
        if record['split'] == 'validation':
            record['raw_wrench'][0] += .7
    with pytest.raises(CalibrationError, match='sign is ambiguous') as changed:
        calibrate(records, limits)
    changed_candidates = changed.value.diagnostics['sign_candidates']
    for original, altered in zip(candidates, changed_candidates):
        assert original['force_sign'] == altered['force_sign']
        assert original['rotation_sensor_to_tool'] == altered['rotation_sensor_to_tool']
        assert original['bias_sensor_N'] == altered['bias_sensor_N']
        assert original['weight_N'] == altered['weight_N']
        assert altered['validation_rmse_N'] > original['validation_rmse_N']
    assert len(changed_candidates[0]['validation_pose_errors_N']) == 4


def wide_records(*, mount=None, sign=1, weight=2., pose_noise=.025,
                 sample_noise=.015, seed=73, angles=(20., 45.)):
    """Whole-pose errors remain after averaging: small tools need real coverage."""
    mount = rotation_from_rotvec([1.4, -.7, 2.1]) if mount is None else mount
    bias = np.array([.8, -1.3, 2.1])
    records = synthetic_records(mount=mount, sign=sign, weight=weight,
                                noise=sample_noise, seed=seed)
    rng = np.random.default_rng(seed + 500)
    offsets = rng.normal(0., pose_noise, (17, 3))
    for record in records:
        vector = np.asarray(record['actual_tcp_pose'][3:])
        old_gravity = rotation_from_rotvec(vector).T @ [0., 0., -1.]
        if np.linalg.norm(vector) > 1e-12:
            inner = np.isclose(np.linalg.norm(vector), np.radians(12.))
            vector *= np.radians(angles[0] if inner else angles[1]) / np.linalg.norm(vector)
        gravity = rotation_from_rotvec(vector).T @ [0., 0., -1.]
        record['actual_tcp_pose'][3:] = vector.tolist()
        force = (np.asarray(record['raw_wrench'][:3])
                 + sign * weight * mount.T @ (gravity - old_gravity)
                 + offsets[record['pose_id']])
        record['raw_wrench'][:3] = force.tolist()
    return records


def wide_limits(**changes):
    limits = replace(CalibrationLimits(), max_fit_rmse_n=.20,
                     max_fit_pose_error_n=.40, max_relative_fit_error=.20,
                     max_validation_rmse_n=.25, max_validation_pose_error_n=.50,
                     max_relative_validation_error=.25,
                     max_rotation_uncertainty_deg=5.)
    return replace(limits, **changes)


@pytest.mark.parametrize('sign', [1, -1])
@pytest.mark.parametrize('rotvec', [[0., 0., 0.], [1.4, -.7, 2.1],
                                  [-2.3, .9, -.45], [np.pi, 0., 0.]])
def test_wide_plan_small_weight_arbitrary_mount_and_pose_errors(sign, rotvec):
    mount = rotation_from_rotvec(rotvec)
    records = wide_records(mount=mount, sign=sign)
    untouched = copy.deepcopy(records)
    result = calibrate(records, wide_limits())
    assert result['force_sign'] == sign
    assert result['weight_N'] == pytest.approx(2., abs=.07)
    assert rotation_error_deg(result['rotation_sensor_to_tool'], mount) < 5.
    assert 0. < result['estimated_rotation_95_bound_deg'] < 5.
    assert result['estimated_rotation_95_bound_deg'] == result['diagnostics']['estimated_rotation_95_bound_deg']
    assert 'not an absolute accuracy guarantee' in result['rotation_uncertainty_assumptions']
    np.testing.assert_allclose(result['bias_sensor_N'], [.8, -1.3, 2.1], atol=.07)
    np.testing.assert_allclose(np.linalg.det(result['rotation_sensor_to_tool']), 1., atol=1e-12)
    assert len(result['diagnostics']['validation_pose_ids']) == 4
    assert records == untouched
    json.dumps(result, allow_nan=False)


def test_wider_angles_increase_weak_signal_and_reduce_rotation_uncertainty():
    narrow = calibrate(wide_records(angles=(12., 25.), pose_noise=0.), wide_limits())
    wide = calibrate(wide_records(pose_noise=0.), wide_limits())
    assert (wide['diagnostics']['weak_axis_force_signal_N']
            > 3.1 * narrow['diagnostics']['weak_axis_force_signal_N'])
    assert wide['estimated_rotation_95_bound_deg'] < .65 * narrow['estimated_rotation_95_bound_deg']
    assert wide['diagnostics']['sign_rmse_gap_N'] > 3. * narrow['diagnostics']['sign_rmse_gap_N']


def test_rotation_precision_target_rejects_otherwise_acceptable_wide_data():
    records = wide_records(pose_noise=.075, seed=0)
    unconstrained = calibrate(records, wide_limits(max_rotation_uncertainty_deg=None))
    assert unconstrained['estimated_rotation_95_bound_deg'] > 5.
    with pytest.raises(CalibrationError, match='excessive estimated rotation uncertainty') as caught:
        calibrate(records, wide_limits())
    assert caught.value.diagnostics['failure_reasons'] == [
        'excessive estimated rotation uncertainty '
        f"({unconstrained['estimated_rotation_95_bound_deg']:.2f} deg > 5 deg; "
        'local model estimate, not guaranteed accuracy)']
    candidate = caught.value.diagnostics['sign_candidates'][0]
    assert candidate['rotation_sensor_to_tool'] == unconstrained['rotation_sensor_to_tool']
    assert candidate['force_sign'] == unconstrained['force_sign']


def test_wide_quality_limits_do_not_disable_force_sign_identification():
    records = wide_records(weight=.5, pose_noise=.035, seed=19)
    with pytest.raises(CalibrationError, match='sign is ambiguous'):
        calibrate(records, wide_limits())


def test_wide_validation_corruption_cannot_refit_or_reselect_sign():
    records = wide_records()
    result = calibrate(records, wide_limits())
    baseline_uncertainty = result['estimated_rotation_95_bound_deg']
    for record in records:
        if record['split'] == 'validation':
            record['raw_wrench'][1] += .45
    with pytest.raises(CalibrationError, match='independent complete-pose validation error') as caught:
        calibrate(records, wide_limits())
    candidate = caught.value.diagnostics['sign_candidates'][0]
    for field in ('rotation_sensor_to_tool', 'bias_sensor_N', 'weight_N', 'force_sign'):
        assert candidate[field] == result[field]
    assert caught.value.diagnostics['estimated_rotation_95_bound_deg'] > baseline_uncertainty


def test_uncertainty_uses_joint_parameter_covariance_and_complete_pose_noise():
    result = calibrate(wide_records(), wide_limits())
    detail = result['diagnostics']['rotation_uncertainty']
    assert detail['parameter_count'] == 7
    assert detail['fit_observation_count'] == 39
    assert detail['residual_degrees_of_freedom'] == 32
    assert detail['applied_variance_N2'] == max(detail['fit_variance_N2'],
                                             detail['validation_variance_N2'],
                                             detail['sampling_variance_floor_N2'])
    covariance = np.asarray(detail['angular_covariance_rad2'])
    assert np.linalg.eigvalsh(covariance).min() > 0.
    np.testing.assert_allclose(covariance, covariance.T, atol=1e-15)
    assert detail['is_accuracy_guarantee'] is False


def test_nonzero_sampling_noise_prevents_zero_uncertainty_from_exact_pose_means():
    records = wide_records(pose_noise=0., sample_noise=0.)
    for index, record in enumerate(records):
        # Each pose has an exactly zero-mean perturbation, yet its measured
        # sample variance still supplies uncertainty for that pose's mean.
        delta = np.array([.03, -.04, .02]) * (-1. if index % 2 else 1.)
        record['raw_wrench'][:3] = (np.asarray(record['raw_wrench'][:3]) + delta).tolist()
    result = calibrate(records, wide_limits())
    detail = result['diagnostics']['rotation_uncertainty']
    assert result['fit_rmse_N'] < 1e-12
    assert result['validation_rmse_N'] < 1e-12
    assert detail['sampling_variance_floor_N2'] > 0.
    assert detail['applied_variance_N2'] == detail['sampling_variance_floor_N2']
    assert result['estimated_rotation_95_bound_deg'] > 0.


@pytest.mark.parametrize('invalid', [0., -1., np.inf, np.nan])
def test_invalid_rotation_precision_target_is_rejected(invalid):
    with pytest.raises(CalibrationError, match='invalid positive calibration limit: max_rotation_uncertainty_deg'):
        calibrate(wide_records(), wide_limits(max_rotation_uncertainty_deg=invalid))

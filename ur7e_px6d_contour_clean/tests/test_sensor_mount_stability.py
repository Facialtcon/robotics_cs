"""Only raw stationary evidence may diagnose drift; it never corrects samples."""
import copy
import json

import numpy as np
import pytest

from tools.sensor_mount_calibration.solver import CalibrationError, rotation_from_rotvec
from tools.sensor_mount_calibration.stability import (
    check_recorded_stability, check_reference, check_stationary_trend, make_reference,
)


def stationary_records(duration=.8, count=80, force=(.8, -.08, 1.98), torque=(.01, .02, -.03),
                       slope=(0., 0., 0.), noise=.015, seed=91, start=20.):
    rng = np.random.default_rng(seed)
    times = np.linspace(0., duration, count)
    result = []
    for t in times:
        wrench = np.r_[np.asarray(force) + t*np.asarray(slope), torque]
        wrench += rng.normal(0., np.r_[np.full(3, noise), np.full(3, noise/20)])
        result.append(dict(actual_tcp_pose=[.6, .2, .3, 1.2, -.5, .3],
                           raw_wrench=wrench.tolist(), host_monotonic=start+float(t),
                           utc_time='2026-10-03T08:47:28Z', split='reference', pose_id=-1))
    return result


def test_noise_and_nonzero_bias_leave_reference_unchanged():
    initial = stationary_records(force=(15., -23., 41.))
    current = stationary_records(force=(15., -23., 41.), seed=35, start=120.)
    original = copy.deepcopy([initial, current])
    reference = make_reference(initial)
    frozen_reference = copy.deepcopy(reference)
    result = check_reference(reference, current)
    assert result['status'] == 'stable'
    assert result['force_delta_norm_N'] < .01
    assert reference == frozen_reference
    assert [initial, current] == original
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('axis', range(3))
def test_slow_bias_drift_across_quiet_poses_is_rejected(axis):
    initial = stationary_records()
    current = stationary_records(seed=35, start=120.)
    for sample in current:
        sample['raw_wrench'][axis] += .14
    with pytest.raises(CalibrationError, match='同姿态原始力/矩漂移') as caught:
        check_reference(make_reference(initial), current)
    assert caught.value.diagnostics['force_delta_norm_N'] > .13
    assert caught.value.diagnostics['current_reference']['raw_wrench_std'][axis] < .03


def test_torque_change_is_checked_without_a_force_change():
    initial = stationary_records(noise=0.)
    current = stationary_records(noise=0., torque=(.04, .02, -.03))
    with pytest.raises(CalibrationError, match='原始力/矩漂移') as caught:
        check_reference(make_reference(initial), current)
    assert caught.value.diagnostics['force_delta_norm_N'] == 0.
    assert caught.value.diagnostics['torque_delta_norm_Nm'] == pytest.approx(.03)


@pytest.mark.parametrize('component,change', [(0, .0004), (3, .003)])
def test_different_actual_reference_pose_is_not_misreported_as_bias_drift(component, change):
    initial = stationary_records()
    current = stationary_records(force=(1., 0., 1.5))
    for sample in current:
        sample['actual_tcp_pose'][component] += change
    with pytest.raises(CalibrationError, match='实际姿态与初始参考不一致'):
        check_reference(make_reference(initial), current)


@pytest.mark.parametrize('component,change', [(0, .0004), (3, .003)])
def test_movement_within_reference_visit_is_rejected(component, change):
    records = stationary_records()
    records[-1]['actual_tcp_pose'][component] += change
    with pytest.raises(CalibrationError, match='实际 TCP 姿态变化过大'):
        make_reference(records)


def test_equivalent_rotvec_branches_are_the_same_reference_pose():
    initial = stationary_records()
    current = stationary_records(seed=33)
    for i, sample in enumerate(initial + current):
        sample['actual_tcp_pose'][3:] = [np.pi if i % 2 else -np.pi, 0., 0.]
    reference = make_reference(initial)
    result = check_reference(reference, current)
    assert result['orientation_delta_deg'] < 1e-5
    np.testing.assert_allclose(reference['mean_rotation_base_tool'], rotation_from_rotvec([np.pi, 0., 0.]), atol=1e-10)


@pytest.mark.parametrize('change', ['short', 'few', 'noisy', 'spike', 'nan', 'time'])
def test_unusable_reference_data_rejected(change):
    records = stationary_records()
    if change == 'short':
        records = stationary_records(duration=.2)
    elif change == 'few':
        records = stationary_records(count=15)
    elif change == 'noisy':
        records = stationary_records(noise=.3)
    elif change == 'spike':
        records[-1]['raw_wrench'][3] += .2
    elif change == 'nan':
        records[0]['raw_wrench'][0] = np.nan
    else:
        records[-1]['host_monotonic'] = records[0]['host_monotonic']
    with pytest.raises(CalibrationError):
        make_reference(records)


def test_stationary_trend_reports_short_preflight_inconclusive():
    assert check_stationary_trend([])['status'] == 'insufficient_duration'
    short = check_stationary_trend(stationary_records(duration=10., count=200, slope=(0., 0., -.02)))
    assert short['status'] == 'insufficient_duration'
    assert check_stationary_trend(stationary_records(duration=30., count=80))['status'] == 'insufficient_duration'


def test_latest_run_scale_drift_is_rejected_before_any_tilt():
    records = stationary_records(duration=30., count=2800, slope=(0., 0., -.12783/60.))
    before = copy.deepcopy(records)
    with pytest.raises(CalibrationError, match='原地原始力持续漂移') as caught:
        check_stationary_trend(records)
    diagnostics = caught.value.diagnostics
    assert diagnostics['force_slope_3sigma_lower_N_per_s'] > .001
    assert diagnostics['endpoint_force_delta_norm_N'] > .04
    assert records == before
    json.dumps(diagnostics, allow_nan=False)


def test_stable_noise_does_not_trigger_trend_or_disguise_large_constant_bias():
    result = check_stationary_trend(stationary_records(duration=30., count=2800, force=(20., -12., 48.)))
    assert result['status'] == 'stable'
    assert result['force_slope_3sigma_lower_N_per_s'] == 0.


def test_trend_needs_slope_and_endpoint_evidence():
    result = check_stationary_trend(stationary_records(duration=20., count=2000, noise=0., slope=(.0015, 0., 0.)))
    assert result['force_slope_3sigma_lower_N_per_s'] > .001
    assert result['endpoint_force_delta_norm_N'] < .03
    assert result['status'] == 'stable'


def test_only_recent_thirty_seconds_are_used_for_preflight_trend():
    old = stationary_records(duration=29., count=2000, slope=(0., 0., -.1), start=20.)
    recent = stationary_records(duration=30., count=2800, start=50.)
    result = check_stationary_trend(old + recent)
    assert result['status'] == 'stable'
    assert result['sample_count'] == len(recent)
    assert result['duration_s'] == 30.


def test_preflight_pose_motion_is_not_classified_as_force_drift():
    records = stationary_records(duration=30., count=300, slope=(0., 0., -.01))
    records[-1]['actual_tcp_pose'][3] += .003
    with pytest.raises(CalibrationError, match='实际 TCP 姿态变化过大'):
        check_stationary_trend(records)


def test_recorded_stability_preserves_legacy_files_without_reference_frames():
    records = stationary_records()
    for record in records:
        record.update(split='fit', pose_id=0)
    result = check_recorded_stability(records)
    assert result['preflight']['status'] == 'insufficient_duration'
    assert result['references'] == []
    assert result['reference_baseline'] is None


def test_recorded_preflight_drift_cannot_be_bypassed_without_reference_frames():
    records = stationary_records(duration=30., count=2800, slope=(0., 0., -.003))
    for record in records:
        record['split'] = 'preflight'
    with pytest.raises(CalibrationError, match='原地原始力持续漂移') as caught:
        check_recorded_stability(records)
    diagnostics = caught.value.diagnostics['recorded_stability']
    assert diagnostics['preflight']['status'] == 'rejected_raw_drift'
    assert diagnostics['reference_count'] == 0


def test_recorded_reference_uses_first_fit_visit_and_never_updates_its_baseline():
    initial = stationary_records(start=20., noise=0.)
    first_return = stationary_records(start=40., force=(.88, -.08, 1.98), noise=0.)
    final_return = stationary_records(start=60., force=(.96, -.08, 1.98), noise=0.)
    for record in initial:
        record.update(split='fit', pose_id=7)
    for record in first_return:
        record.update(split='reference', pose_id=1001)
    for record in final_return:
        record.update(split='reference', pose_id=1002)
    with pytest.raises(CalibrationError, match='同姿态原始力/矩漂移') as caught:
        check_recorded_stability(initial + first_return + final_return)
    diagnostics = caught.value.diagnostics['recorded_stability']
    assert diagnostics['reference_baseline']['pose_id'] == 7
    assert diagnostics['reference_count'] == 1
    assert diagnostics['references'][0]['force_delta_norm_N'] == pytest.approx(.08)
    assert diagnostics['failed_check']['force_delta_norm_N'] == pytest.approx(.16)


def test_recorded_valid_references_are_json_and_motion_samples_are_never_guessed():
    initial = stationary_records(start=20.)
    motion = stationary_records(start=40., force=(12., 10., 8.))
    current = stationary_records(start=60., seed=35)
    for record in initial:
        record.update(split='fit', pose_id=0)
    for record in motion:
        record.update(split='settling', pose_id=-1)
    for record in current:
        record.update(split='reference', pose_id=1001)
    records = initial + motion + current
    before = copy.deepcopy(records)
    result = check_recorded_stability(records)
    assert result['reference_count'] == 1
    assert result['references'][0]['status'] == 'stable'
    assert records == before
    json.dumps(result, allow_nan=False)


def test_recorded_reference_without_initial_fit_is_rejected():
    with pytest.raises(CalibrationError, match='缺少初始拟合姿态'):
        check_recorded_stability(stationary_records())

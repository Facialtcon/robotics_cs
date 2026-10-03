"""Failure descriptions remain independent from calibration acceptance."""

import copy
from datetime import datetime, timedelta, timezone
import json

import numpy as np
import pytest

from tools.sensor_mount_calibration.diagnostics import analyze_failure_records
from tools.sensor_mount_calibration.solver import rotation_from_rotvec


def motion_records(*, seed=19, weight=12., force_step=None, torque_step=None, pulse=False,
                   noise=.015, duration=12., frequency=100):
    rng = np.random.default_rng(seed)
    mount = rotation_from_rotvec(rng.normal(size=3)*2.)
    bias = rng.uniform(-40., 40., 6)
    lever = rng.uniform(-.04, .04, 3)
    origin = datetime(2026, 10, 3, 9, 0, tzinfo=timezone.utc)
    records = []
    for t in np.arange(0., duration, 1./frequency):
        pose = np.r_[[.6, -.2, .3], [.8+.15*np.sin(.3*t), -.4+.12*np.cos(.5*t), .2+.1*np.sin(.2*t)]]
        gravity = weight*mount.T @ rotation_from_rotvec(pose[3:]).T @ [0., 0., -1.]
        wrench = bias + np.r_[gravity, np.cross(lever, gravity)]
        wrench += rng.normal(0., np.r_[np.full(3, noise), np.full(3, noise/25.)])
        if t >= 6. and (not pulse or t < 6.15):
            wrench += np.r_[force_step or [0., 0., 0.], torque_step or [0., 0., 0.]]
        records.append(dict(host_monotonic=1000.+float(t), utc_time=(origin+timedelta(seconds=float(t))).isoformat(),
                            raw_wrench=wrench.tolist(), actual_tcp_pose=pose.tolist(),
                            actual_q=(np.array([.1, -.7, 1.2, .8, -.5, .6])+.02*np.sin(.2*t)).tolist(),
                            split='settling', pose_id=-1))
    return records


@pytest.mark.parametrize('seed,weight', [(0, .5), (1, 3.), (2, 15.), (3, 50.), (4, 200.)])
def test_arbitrary_mount_bias_weight_smooth_gravity_is_not_a_discontinuity(seed, weight):
    records = motion_records(seed=seed, weight=weight)
    original = copy.deepcopy(records)
    result = analyze_failure_records(records)
    assert result['classification'] == 'unknown'
    assert result['change_candidates'] == []
    assert records == original
    assert result['analysis_policy']['controls_acceptance'] is False
    assert 'rotation_sensor_to_tool' not in result
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('seed,weight', [(6, 1.), (7, 10.), (8, 100.)])
def test_persistent_force_and_torque_step_is_localized_with_actual_pose_evidence(seed, weight):
    records = motion_records(seed=seed, weight=weight, force_step=[.01, -.04, .27], torque_step=[0., -.007, 0.])
    result = analyze_failure_records(records)
    assert result['classification'] == 'abrupt_force_change'
    candidate = result['change_candidates'][0]
    assert candidate['host_monotonic'] == pytest.approx(1006., abs=.06)
    assert candidate['utc_time'].startswith('2026-10-03T09:00:0')
    np.testing.assert_allclose(candidate['force_delta_sensor_N'], [.01, -.04, .27], atol=.04)
    np.testing.assert_allclose(candidate['torque_delta_sensor_Nm'], [0., -.007, 0.], atol=.002)
    assert candidate['force_change_detected'] and candidate['torque_change_detected']
    assert candidate['persistence_sample_count'] >= 15
    assert candidate['adjacent_position_delta_m'] == pytest.approx(0.)
    assert candidate['adjacent_orientation_delta_deg'] < .1
    assert candidate['adjacent_max_joint_delta_rad'] < .001
    assert '不能' in result['summary']
    json.dumps(result, allow_nan=False)


def test_brief_impulse_does_not_claim_a_persistent_step():
    result = analyze_failure_records(motion_records(force_step=[0., 0., .4], pulse=True))
    assert result['classification'] == 'unknown'
    assert result['change_candidates'] == []


@pytest.mark.parametrize('seed', [31, 92])
def test_large_measurement_noise_is_not_a_significant_discontinuity(seed):
    result = analyze_failure_records(motion_records(seed=seed, noise=.07))
    assert result['classification'] == 'unknown'
    assert result['change_candidates'] == []


def test_torque_only_step_does_not_claim_an_abrupt_force_change():
    result = analyze_failure_records(motion_records(torque_step=[.012, 0., 0.]))
    assert result['classification'] == 'unknown'
    assert result['change_candidates'][0]['torque_change_detected']
    assert not result['change_candidates'][0]['force_change_detected']


def visits(*, delta_per_visit=.06, pose_change=False):
    records = []
    for visit in range(5):
        for t in np.arange(0., .8, .01):
            records.append(dict(host_monotonic=float(1000.+visit*30.+t), utc_time='2026-10-03T09:00:00+00:00',
                                raw_wrench=[7., -4., 3.+visit*delta_per_visit, .1, .2, .3],
                                actual_tcp_pose=[.6, .2, .3, .8+.01*visit*pose_change, -.4, .2],
                                split='fit' if visit == 0 else 'reference', pose_id=visit))
    return records


def test_continuous_drift_requires_multiple_matching_actual_pose_visits():
    result = analyze_failure_records(visits())
    assert result['classification'] == 'continuous_drift'
    assert len(result['reference_history']) == 5
    assert result['reference_history'][-1]['force_delta_norm_N'] == pytest.approx(.24)
    assert result['reference_trend']['force_slope_N_per_s'][2] == pytest.approx(.002)
    assert result['change_candidates'] == []
    json.dumps(result, allow_nan=False)


def test_different_actual_reference_pose_cannot_establish_a_drift():
    result = analyze_failure_records(visits(pose_change=True))
    assert result['classification'] == 'unknown'
    assert not result['reference_history'][-1]['comparable_actual_pose']


def test_nonmonotonic_reference_changes_are_not_labeled_continuous_drift():
    records = visits()
    offsets = [0., .07, -.02, .04, .24]
    for record in records:
        record['raw_wrench'][2] = 3.+offsets[record['pose_id']]
    result = analyze_failure_records(records)
    assert result['classification'] == 'unknown'
    assert result['reference_trend'] is None


def test_missing_fields_short_and_invalid_records_remain_json_diagnostics():
    records = [None, {}, {'host_monotonic': float('nan'), 'raw_wrench': [0.]*6},
               {'host_monotonic': 1., 'raw_wrench': [0.]*6},
               {'host_monotonic': 1., 'raw_wrench': [0.]*6},
               {'host_monotonic': 2., 'raw_wrench': [0.]*6, 'split': 'fit', 'pose_id': 0}]
    result = analyze_failure_records(records)
    assert result['classification'] == 'unknown'
    assert result['data_quality']['invalid_record_count'] == 3
    assert result['data_quality']['nonmonotonic_record_count'] == 1
    assert result['data_quality']['missing_actual_pose_count'] == 2
    assert result['data_quality']['gap_over_80ms_count'] == 1
    json.dumps(result, allow_nan=False)
    assert analyze_failure_records([])['classification'] == 'unknown'


def test_missing_joints_do_not_hide_force_evidence():
    records = motion_records(force_step=[0., 0., .3])
    for record in records:
        del record['actual_q']
    result = analyze_failure_records(records)
    assert result['classification'] == 'abrupt_force_change'
    assert result['data_quality']['missing_actual_q_count'] == len(records)
    assert 'adjacent_joint_delta_rad' not in result['change_candidates'][0]


def test_numeric_string_fields_are_normalized_without_changing_input():
    records = motion_records(force_step=[0., 0., .3])
    for record in records:
        record['host_monotonic'] = str(record['host_monotonic'])
        record['raw_wrench'] = [str(value) for value in record['raw_wrench']]
    original = copy.deepcopy(records)
    result = analyze_failure_records(records)
    assert result['classification'] == 'abrupt_force_change'
    assert records == original
    json.dumps(result, allow_nan=False)


def test_gap_around_force_change_does_not_invent_a_precise_step_time():
    records = motion_records(force_step=[0., 0., .3])
    records = [r for r in records if not 1005.8 < r['host_monotonic'] < 1006.2]
    result = analyze_failure_records(records)
    assert result['classification'] == 'unknown'
    assert result['change_candidates'] == []
    assert result['data_quality']['gap_over_80ms_count'] == 1


def test_stationary_check_evidence_is_used_but_error_wording_is_not():
    result = analyze_failure_records([], {'reason': '原始力漂移'})
    assert result['classification'] == 'unknown'
    result = analyze_failure_records([], {'force_slope_3sigma_lower_N_per_s': .002,
                                          'endpoint_force_delta_norm_N': .05})
    assert result['classification'] == 'continuous_drift'


def test_old_discontinuity_is_outside_bounded_search():
    records = motion_records(force_step=[0., 0., .3], duration=30.)
    result = analyze_failure_records(records)
    assert result['classification'] == 'unknown'
    assert result['change_candidates'] == []
    assert result['candidate_search']['searched_candidate_count'] < 600

"""An explicit wider motion plan buys noise tolerance, never unlimited acceptance."""
from dataclasses import asdict, replace
import json

import numpy as np
import pytest

from tools.sensor_mount_calibration.plan import Limits, make_plan
from tools.sensor_mount_calibration.profiles import get_profile
from tools.sensor_mount_calibration.solver import CalibrationError, CalibrationLimits, calibrate
from tools.sensor_mount_calibration.stability import (
    StabilityLimits, check_reference, check_stationary_trend, make_reference,
)
from test_sensor_mount_stability import stationary_records
from test_sensor_mount_session import fixture_session


def test_wide_changes_only_explicit_motion_and_quality_budgets():
    standard, wide = get_profile(), get_profile('wide')
    assert standard.motion == Limits() and standard.fit == CalibrationLimits()
    changed_motion = {key for key in asdict(standard.motion)
                      if asdict(standard.motion)[key] != asdict(wide.motion)[key]}
    assert changed_motion == {'inner_tilt_deg', 'outer_tilt_deg', 'interleave_tilts',
                              'max_tilt_rad', 'joint_excursion_rad'}
    assert wide.fit.max_rotation_uncertainty_deg == 5.
    assert wide.fit.min_tilt_span_deg >= 70.
    for key in ('min_sign_rmse_gap_n', 'sign_uncertainty_multiplier', 'min_weak_axis_signal_n',
                'max_force_std_n', 'max_torque_std_nm', 'min_validation_poses'):
        assert getattr(standard.fit, key) == getattr(wide.fit, key)
    assert wide.stability.reference_force_limit_n == .25
    assert wide.stability.reference_torque_limit_nm == standard.stability.reference_torque_limit_nm
    json.dumps(wide.as_dict(), allow_nan=False)
    assert '5°' in wide.describe() and '不是绝对精度保证' in wide.describe()
    with pytest.raises(ValueError, match='unknown calibration profile'):
        get_profile('unbounded')


def test_wide_reference_warns_but_does_not_change_original_bias_or_samples():
    initial = stationary_records(noise=0.)
    current = stationary_records(force=(.8, -.08, 2.16), noise=0., start=100.)
    before = json.dumps([initial, current])
    reference = make_reference(initial)
    with pytest.raises(CalibrationError):
        check_reference(reference, current)
    check = check_reference(reference, current, get_profile('wide').stability)
    assert check['status'] == 'within_coarse_budget' and check['warnings']
    assert check['force_delta_norm_N'] == pytest.approx(.18)
    assert reference['mean_raw_wrench'][2] == pytest.approx(1.98)
    assert json.dumps([initial, current]) == before


@pytest.mark.parametrize('force,torque', [((.8, -.08, 2.24), (.01, .02, -.03)),
                                         ((.8, -.08, 1.98), (.031, .02, -.03))])
def test_wide_still_rejects_large_reference_change(force, torque):
    reference = make_reference(stationary_records(noise=0.))
    current = stationary_records(noise=0., force=force, torque=torque, start=100.)
    with pytest.raises(CalibrationError, match='停止标定'):
        check_reference(reference, current, get_profile('wide').stability)


def test_wide_preflight_can_collect_latest_scale_trend_with_explicit_warning():
    records = stationary_records(duration=30., count=2800, slope=(0., 0., .004))
    with pytest.raises(CalibrationError, match='持续漂移'):
        check_stationary_trend(records)
    check = check_stationary_trend(records, get_profile('wide').stability)
    assert check['status'] == 'within_coarse_budget' and check['warnings']
    assert check['endpoint_force_delta_norm_N'] > .09
    assert check['force_slope_limit_N_per_s'] == .005
    assert check['endpoint_force_delta_limit_N'] == .15


def test_wide_preflight_still_rejects_excessive_drift():
    records = stationary_records(duration=30., count=2800, slope=(0., 0., .008))
    with pytest.raises(CalibrationError, match='持续漂移'):
        check_stationary_trend(records, get_profile('wide').stability)


@pytest.mark.parametrize('changes', [dict(reference_force_limit_n=0.),
                                    dict(force_slope_limit_n_per_s=float('inf')),
                                    dict(warning_reference_force_n=.3)])
def test_quality_limits_cannot_be_disabled_or_inverted(changes):
    with pytest.raises(ValueError):
        replace(StabilityLimits(), **changes)


def test_full_wide_session_uses_actual_poses_and_keeps_complete_holdout():
    session, hardware, store, clock, start = fixture_session()
    profile = get_profile('wide')
    session.limits = replace(profile.motion, settle_hold_sec=.03, samples_per_pose=25)
    session.stability_limits = profile.stability
    records = session.run(start, make_plan(start, session.limits))
    result = calibrate(records, profile.fit)
    assert len(hardware.commands) == 32 and hardware.stops == 0
    assert len({r['pose_id'] for r in records if r['split'] == 'validation'}) == 4
    np.testing.assert_allclose(result['rotation_sensor_to_tool'], hardware.mount, atol=1e-9)
    assert result['estimated_rotation_95_bound_deg'] < 5.
    assert np.allclose(hardware.pose, start)


@pytest.mark.parametrize('change,accepted', [(.18, True), (.26, False)])
def test_live_wide_quality_uses_one_fixed_reference_and_stops_above_budget(change, accepted):
    from types import SimpleNamespace
    session, hardware, store, clock, start = fixture_session()
    profile = get_profile('wide')
    session.limits = replace(profile.motion, settle_hold_sec=.03, samples_per_pose=25)
    session.stability_limits = profile.stability
    read = hardware.read
    messages = []
    session.progress = SimpleNamespace(emit=lambda text, **kwargs: messages.append(text))

    def shifted():
        record = read()
        if len(hardware.commands) >= 2:
            record['raw_wrench'][2] += change
        return record

    hardware.read = shifted
    if accepted:
        session.run(start, make_plan(start, session.limits))
        assert len(hardware.commands) == 32 and hardware.stops == 0
        assert len(session.reference_checks) == 16
        assert all(check['force_delta_norm_N'] == pytest.approx(change) for check in session.reference_checks)
        assert any('质量提示' in message for message in messages)
    else:
        with pytest.raises(CalibrationError, match='同姿态原始力/矩漂移'):
            session.run(start, make_plan(start, session.limits))
        assert len(hardware.commands) == 2 and hardware.stops == 1
        assert session.failed

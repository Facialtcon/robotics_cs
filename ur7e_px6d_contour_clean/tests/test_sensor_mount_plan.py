"""Bounded wide-angle trajectories, independent holdouts and displayed limits."""

from dataclasses import replace
import math

import numpy as np
import pytest

from robot.rtde_controller import _orientation_distance, _rotvec_to_matrix
from tools.sensor_mount_calibration.plan import (
    Limits, describe_plan, interpolate_path, make_plan,
)


def wide_limits():
    return replace(Limits(), inner_tilt_deg=20., outer_tilt_deg=45.,
                   interleave_tilts=True, max_tilt_rad=math.radians(46.),
                   joint_excursion_rad=1.5)


def test_default_plan_retains_original_order_and_complete_pose_holdouts():
    start = [.4, -.2, .3, .2, -.7, 1.1]
    plan = make_plan(start)
    assert plan == make_plan(start, Limits())
    acquired = [point for point in plan if point['acquire']]
    assert [(p['tilt_deg'], p['azimuth_deg']) for p in acquired[1:]] == [
        (tilt, azimuth) for tilt in (12., 25.) for azimuth in range(0, 360, 45)]
    assert [p['pose_id'] for p in acquired if p['split'] == 'validation'] == [10, 12, 14, 16]


def test_wide_plan_interleaves_amplitudes_and_opposite_directions_without_losing_holdouts():
    start = [.4, -.2, .3, .2, -.7, 1.1]
    limits = wide_limits()
    plan = make_plan(start, limits)
    assert plan == make_plan(start, limits)
    acquired = [point for point in plan if point['acquire']]
    assert len(acquired) == 17 and len(plan) - 1 == 32
    assert [(p['tilt_deg'], p['azimuth_deg']) for p in acquired[1:]] == [
        (tilt, azimuth) for azimuth in (0, 180, 90, 270, 45, 225, 135, 315)
        for tilt in (20., 45.)]
    held_out = [p for p in acquired if p['split'] == 'validation']
    assert len(held_out) == 4
    assert len([p for p in acquired if p['split'] == 'fit']) == 13
    assert {(p['tilt_deg'], p['azimuth_deg']) for p in held_out} == {
        (45., 45.), (45., 135.), (45., 225.), (45., 315.)}
    for index in range(1, len(plan), 2):
        assert plan[index]['acquire'] is True
        assert plan[index + 1]['acquire'] is False
        assert plan[index + 1]['target'] == start


@pytest.mark.parametrize('rotation', [
    [0., 0., 0.], [0., math.pi, 0.], [math.pi - 1e-8, 0., 0.],
    [1.4, -.7, 2.1], [-1.47664, 2.28422, -.31186],
])
def test_wide_path_has_fixed_tcp_bounded_tilt_and_at_most_one_degree_interpolation(rotation):
    start = [.64947, .17920, .32894, *rotation]
    limits = wide_limits()
    plan = make_plan(start, limits)
    path = interpolate_path(start, plan[1:])
    assert 1000 < len(path) < 1200
    for actual in path:
        np.testing.assert_array_equal(actual[:3], start[:3])
        assert _orientation_distance(actual[3:], start[3:]) <= math.radians(45.) + 1e-7
    for previous, actual in zip(path, path[1:]):
        assert _orientation_distance(previous[3:], actual[3:]) <= math.radians(1.) + 1e-7
    np.testing.assert_allclose(path[-1], start, atol=1e-7)
    start_rotation = _rotvec_to_matrix(start[3:])
    for point in plan:
        if point['acquire'] and point['tilt_deg']:
            azimuth = math.radians(point['azimuth_deg'])
            axis = np.array([math.cos(azimuth), math.sin(azimuth), 0.])
            expected = _rotvec_to_matrix(axis * math.radians(point['tilt_deg'])) @ start_rotation
            np.testing.assert_allclose(_rotvec_to_matrix(point['target'][3:]), expected, atol=1e-7)


@pytest.mark.parametrize('updates', [
    {'inner_tilt_deg': 0.}, {'inner_tilt_deg': -1.},
    {'inner_tilt_deg': 25.}, {'outer_tilt_deg': 12.},
    {'inner_tilt_deg': math.nan}, {'outer_tilt_deg': math.inf},
    {'outer_tilt_deg': 45.1, 'max_tilt_rad': math.radians(46.)},
    {'outer_tilt_deg': 45.},
    {'max_tilt_rad': math.radians(25.9)},
    {'max_tilt_rad': math.radians(46.1)}, {'max_tilt_rad': math.nan},
    {'interleave_tilts': 'yes'},
])
def test_invalid_plan_limits_fail_before_any_plan_or_device_can_be_created(updates):
    with pytest.raises(ValueError):
        replace(Limits(), **updates)


def test_wide_description_reports_actual_limits_and_entire_tool_clearance():
    start = [.4, -.2, .3, 0., math.pi, 0.]
    limits = wide_limits()
    text = describe_plan(start, make_plan(start, limits), limits,
                         [-.00324324, -.0000626687, .25230436, 0., 0., 0.])
    assert '20° / 45°' in text and '倾角最大 45°' in text
    assert '转角硬限 46°' in text and '1.5rad（85.9°）' in text
    assert '197.2 mm' in text
    assert '整套工具、法兰、各机械臂连杆和线缆' in text
    assert 'IK/安全限位预检不等于碰撞检测' in text
    assert '每段 ≤120s，总执行 ≤2700s' in text
    assert 'moveL speed=0.005，acceleration=0.03' in text

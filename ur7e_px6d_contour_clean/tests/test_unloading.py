"""Finite unloading and recovery without a force-direction verification gate."""
from copy import deepcopy

import numpy as np
import pytest

from core.models import RobotState, Wrench
from policy.continuous_tracking import ContinuousTrackingPolicy, State
from sensor.force_direction import control_directions, normal_feedback_speed


def exercise(config, force, *, moving=True, seconds=3.5, motion_scale=1.):
    c = deepcopy(config)
    c['continuous_real_execution'] = True
    policy = ContinuousTrackingPolicy(c)
    pose, speed = np.zeros(6), np.zeros(6)
    rows = []
    for i in range(int(seconds*100)):
        now = i*.01
        w = Wrench(float(force(now)), 0., 0., 0., 0., 0.)
        command = policy.update(now, w, w, RobotState(now, pose.copy(), speed.copy()), execution_settled=True)
        rows.append((now, command, policy.telemetry(command)))
        if policy.state == State.STOP:
            break
        speed[:2] = command.direction_xy*command.speed*motion_scale if moving else 0.
        pose += speed*.01
    return policy, rows


def test_high_force_entry_suppresses_tangent_then_finite_no_motion_stop(config):
    policy, rows = exercise(config, lambda t: 3., moving=False)
    assert policy.stop_reason.value == 'STOP_UNLOAD_NO_MOTION'
    assert rows[-1][0] >= 1.5 and rows[-1][0] < 1.7
    assert all(abs(row['v_t']) < 1e-10 for _, _, row in rows)
    assert any(row['v_n'] == pytest.approx(-.0005) for _, _, row in rows)
    assert not rows[-1][1].move


def test_movement_without_force_improvement_has_distinct_stop(config):
    policy, rows = exercise(config, lambda t: 3.)
    assert policy.stop_reason.value == 'STOP_UNLOAD_INEFFECTIVE'
    assert policy.unload_projected >= config['continuous_tracking']['unload_min_displacement_m']
    assert not rows[-1][1].move
    assert 'TCP moved' in policy.reason


def test_effective_unloading_recovers_tangent_smoothly_and_clears_current_limit(config):
    policy, rows = exercise(config, lambda t: max(1.5, 3.-max(0., t-.3)*1.5))
    assert policy.state == State.CONTINUOUS_TRACKING
    events = [e.event_type for e in policy.events]
    assert 'UNLOAD_STARTED' in events and 'TANGENT_RESUME_COMPLETED' in events
    assert not policy.unload_active and policy.tangent_limit_reason == ''
    assert policy.v_t == pytest.approx(.001) and policy.v_n == pytest.approx(0.)
    speeds = [cmd.direction_xy*cmd.speed for _, cmd, _ in rows]
    assert np.max(np.linalg.norm(np.diff(speeds, axis=0), axis=1)) <= .01*.01+1e-10
    assert 'overload_stall' not in policy.diagnostics


def test_improving_but_never_released_force_cannot_reset_total_budget(config):
    policy, rows = exercise(config, lambda t: max(2.1, 3.-t*.35), seconds=7.)
    assert policy.stop_reason.value == 'STOP_UNLOAD_BUDGET'
    assert 3.11 < rows[-1][0] <= 6.11
    assert not rows[-1][1].move


def test_hysteresis_prevents_repeated_restart_around_pause_threshold(config):
    policy, rows = exercise(config, lambda t: 3. if t < .3 else 2.22+.06*np.sin(t*60), seconds=1.3)
    assert policy.unload_active and policy.state == State.CONTINUOUS_TRACKING
    assert [e.event_type for e in policy.events].count('UNLOAD_STARTED') == 1
    assert all(abs(row['v_t']) < 1e-10 for _, _, row in rows)


def test_high_force_enters_unloading_without_any_direction_record(config):
    policy, rows = exercise(config, lambda t: 3., seconds=.5)
    assert policy.state == State.CONTINUOUS_TRACKING
    assert policy.unload_active and policy.stop_reason is None
    assert any(command.move for _, command, _ in rows)
    assert all(row['v_t'] == 0. for _, _, row in rows)


@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('hand,orientation', [('CLOCKWISE', 1), ('COUNTERCLOCKWISE', -1)])
def test_signed_normal_unload_and_handed_tangent_stay_consistent(config, sign, hand, orientation):
    n, tangent, unload = control_directions([3., 4., 0.], sign, hand)
    np.testing.assert_allclose(n, sign*np.array([.6, .8]))
    assert n[0]*tangent[1]-n[1]*tangent[0] == pytest.approx(orientation)
    assert normal_feedback_speed(5., config['continuous_tracking']) < 0
    np.testing.assert_allclose(unload, -n)
    # Known external reaction = -sign * reported force: unloading follows it.
    assert unload @ (-sign*np.array([3., 4.])) > 0


def test_unload_hard_force_limit_retains_priority(config):
    policy, rows = exercise(config, lambda t: 3. if t < .5 else 61.)
    assert policy.stop_reason.value == 'STOP_FORCE_LIMIT' and not rows[-1][1].move


def test_extended_budget_releases_high_first_contact_load_beyond_one_mm(config):
    policy, rows = exercise(config, lambda t: max(1.5, 4.894-max(0., t-.3)*.8), seconds=5.)
    assert config['continuous_tracking']['unload_max_displacement_m'] == .002
    assert config['continuous_tracking']['unload_max_time_sec'] == 6.
    assert config['continuous_tracking']['normal_speed_limit'] == .0005
    assert policy.stop_reason is None and not policy.unload_active
    assert .001 < policy.unload_displacement < .002
    assert 3. < policy.unload_elapsed < 6.
    events = [event.event_type for event in policy.events]
    assert 'UNLOAD_FORCE_RELEASED' in events and 'TANGENT_RESUME_COMPLETED' in events
    active = [row for _, _, row in rows if row['unload_active']]
    assert all(abs(row['v_n']) <= .0005+1e-12 and abs(row['v_t']) < 1e-12 for row in active)
    assert rows[-1][2]['unload_initial_force_N'] == pytest.approx(4.894)
    assert rows[-1][2]['unload_last_force_N'] <= config['continuous_tracking']['unload_resume_force']
    assert policy.unload_improvement > 2.


@pytest.mark.parametrize('motion_scale,limit', [(1., 'displacement'), (.5, 'time')])
def test_unload_new_budgets_remain_finite(config, motion_scale, limit):
    policy, rows = exercise(config, lambda t: 4.-t*.15, seconds=7., motion_scale=motion_scale)
    assert policy.stop_reason.value == 'STOP_UNLOAD_BUDGET'
    if limit == 'displacement':
        assert .002 <= policy.unload_displacement < .00201
        assert 4. < policy.unload_elapsed < 6.
    else:
        assert 6. <= policy.unload_elapsed < 6.02
        assert policy.unload_displacement < .002
    assert not rows[-1][1].move


def test_first_contact_braking_metrics_use_actual_tcp_and_freeze_at_low_speed(config):
    policy = ContinuousTrackingPolicy(config)
    pose = np.zeros(6)
    for i, (force, distance, speed) in enumerate([
        (1.210, 0., .018), (4., .0005, .010), (4.894, .00103, 0.)]):
        pose[0] = distance
        wrench = Wrench(force, 0., 0., 0., 0., 0.)
        command = policy.update(i*.01, wrench, wrench,
            RobotState(i*.01, pose.copy(), np.array([speed, 0., 0., 0., 0., 0.])), execution_settled=True)
        assert not command.move
    for i in range(3, 20):
        command = policy.update(i*.01, wrench, wrench,
            RobotState(i*.01, pose.copy(), np.zeros(6)), execution_settled=True)
        if policy.state == State.CONTINUOUS_TRACKING:
            break
    metrics = policy.telemetry(command)
    assert metrics['first_contact_braking_distance_m'] == pytest.approx(.00103)
    assert metrics['first_contact_braking_elapsed_sec'] == pytest.approx(.02)
    assert metrics['first_contact_trigger_force_N'] == pytest.approx(1.210)
    assert metrics['first_contact_confirm_force_N'] == pytest.approx(4.894)


def test_unload_raw_torque_backstop_retains_priority(config):
    policy, rows = exercise(config, lambda t: 3., seconds=.5)
    assert policy.unload_active
    wrench = Wrench(3., 0., 0., 6., 0., 0.)
    command = policy.update(.5, wrench, wrench, RobotState(.5, policy.tracking_pose.copy(), np.zeros(6)),
                            execution_settled=True)
    assert policy.stop_reason.value == 'STOP_FORCE_LIMIT' and not command.move

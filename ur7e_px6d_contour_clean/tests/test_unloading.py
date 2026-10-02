"""Finite unloading and recovery with explicit synthetic force-direction evidence."""
from copy import deepcopy

import numpy as np
import pytest

from core.models import RobotState, Wrench
from policy.continuous_tracking import ContinuousTrackingPolicy, State
from sensor.force_direction import control_directions, normal_feedback_speed


def exercise(config, force, *, moving=True, seconds=3.5):
    c = deepcopy(config)
    c['continuous_real_execution'] = True
    c['force_direction_status'] = {'verified': True, 'reason': 'synthetic plant'}
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
        speed[:2] = command.direction_xy*command.speed if moving else 0.
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
    policy, rows = exercise(config, lambda t: max(2.1, 3.-t*.35), seconds=4.)
    assert policy.stop_reason.value == 'STOP_UNLOAD_BUDGET'
    assert rows[-1][0] <= 3.11
    assert not rows[-1][1].move


def test_hysteresis_prevents_repeated_restart_around_pause_threshold(config):
    policy, rows = exercise(config, lambda t: 3. if t < .3 else 2.22+.06*np.sin(t*60), seconds=1.3)
    assert policy.unload_active and policy.state == State.CONTINUOUS_TRACKING
    assert [e.event_type for e in policy.events].count('UNLOAD_STARTED') == 1
    assert all(abs(row['v_t']) < 1e-10 for _, _, row in rows)


def test_unverified_direction_stops_after_confirmation_without_unloading(config):
    config['continuous_real_execution'] = True
    config['force_direction_status'] = {'verified': False}
    policy = ContinuousTrackingPolicy(config)
    for i in range(20):
        now = i*.01
        w = Wrench(3., 0., 0., 0., 0., 0.)
        command = policy.update(now, w, w, RobotState(now, np.zeros(6), np.zeros(6)), execution_settled=True)
        assert not command.move
    assert policy.stop_reason.value == 'STOP_FORCE_DIRECTION_UNVERIFIED'


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

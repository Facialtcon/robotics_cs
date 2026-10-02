"""Recovery travel must not count stationary RTDE pose variation as motion."""
from pathlib import Path

import numpy as np
import pytest

from config.loader import load_config
from core.models import RobotState, Wrench
from policy.continuous_tracking import ContinuousTrackingPolicy, State


ROOT = Path(__file__).resolve().parents[1]
ZERO_FORCE = Wrench(0., 0., 0., 0., 0., 0.)


def observation(stamp, xy=(0., 0.), velocity=(0., 0.)):
    return RobotState(stamp, np.r_[xy, np.zeros(4)], np.r_[velocity, np.zeros(4)])


def recovery(initial=None):
    config = load_config(ROOT/'config.yaml')
    config['continuous_tracking']['reacquire_enabled'] = True
    policy = ContinuousTrackingPolicy(config)
    initial = observation(0.) if initial is None else initial
    policy.contact_direction = np.array([1., 0.])
    policy.tangent = np.array([0., 1.])
    policy.direction_confidence = 1.
    policy._remember(0., initial.pose)
    policy._begin_recovery(0., initial.pose, robot=initial)
    assert policy.state == State.LOCAL_REACQUIRE
    return policy


def tick(policy, state):
    return policy.update(state.timestamp, ZERO_FORCE, ZERO_FORCE, state)


def test_stationary_pose_variation_does_not_exhaust_travel_budget():
    policy = recovery()
    # The incident contains tens of micrometres of pose variation even with
    # zero measured speed. The former sum consumes 3.5 mm within 0.9 s here.
    for i in range(1, 241):
        command = tick(policy, observation(i*.01, (20e-6*(-1)**i, 0.)))
        assert policy.state == State.LOCAL_REACQUIRE
    assert policy.reacquire_raw_pose_path_length > policy.c['reacquire_max_path']
    assert policy.reacquire_speed_path_length == 0.
    assert policy.reacquire_path_length == pytest.approx(20e-6)
    telemetry = policy.telemetry(command)
    assert telemetry['reacquire_raw_pose_path_length'] == policy.reacquire_raw_pose_path_length
    assert telemetry['reacquire_speed_path_length'] == 0.
    assert telemetry['reacquire_max_displacement'] == pytest.approx(20e-6)


@pytest.mark.parametrize('trajectory', ['circle', 'back_and_forth'])
def test_closed_or_reversing_motion_still_exhausts_total_path(trajectory):
    radius, speed = 40e-6, .0005
    if trajectory == 'circle':
        initial = observation(0., (radius, 0.), (0., speed))
    else:
        initial = observation(0., velocity=(speed, 0.))
    policy = recovery(initial)
    for i in range(1, 751):
        now = i*.01
        if trajectory == 'circle':
            angle = now*speed/radius
            xy = radius*np.array([np.cos(angle), np.sin(angle)])
            velocity = speed*np.array([-np.sin(angle), np.cos(angle)])
        else:
            angle = now*speed*np.pi/(2*radius)
            xy = [radius*2/np.pi*np.arcsin(np.sin(angle)), 0.]
            velocity = [np.copysign(speed, np.cos(angle)), 0.]
        command = tick(policy, observation(now, xy, velocity))
        if policy.state == State.STOP:
            break
    assert policy.state == State.STOP and not command.move
    assert 'path' in policy.reason
    assert policy.stop_reason.value == 'STOP_RECOVERY_EXHAUSTED'
    assert policy.reacquire_max_displacement <= 2*radius+1e-12
    assert policy.reacquire_speed_path_length == pytest.approx(speed*now)
    assert policy.reacquire_path_length >= .00349
    assert now < policy.c['reacquire_max_time_sec']


def test_real_position_jump_still_stops_with_zero_velocity_feedback():
    policy = recovery()
    command = tick(policy, observation(.01, (.0036, 0.)))
    assert policy.state == State.STOP and not command.move
    assert policy.reason == 'local reacquire displacement budget exhausted'
    assert policy.reacquire_speed_path_length == 0.
    assert policy.reacquire_path_length == pytest.approx(.0036)


def test_frozen_pose_does_not_hide_nonzero_actual_speed():
    policy = recovery(observation(0., velocity=(.0005, 0.)))
    for i in range(1, 751):
        command = tick(policy, observation(i*.01, velocity=(.0005, 0.)))
        if policy.state == State.STOP:
            break
    assert policy.state == State.STOP and not command.move
    assert 'path' in policy.reason
    assert policy.reacquire_max_displacement == 0.
    assert policy.reacquire_raw_pose_path_length == 0.
    assert policy.reacquire_speed_path_length >= .00349


def test_radial_lower_bound_is_not_repeatedly_added_to_integral():
    policy = recovery()
    for i in range(1, 101):
        policy._observe_reacquire_path(observation(i*.01, (.001, 0.)))
    assert policy.reacquire_path_length == pytest.approx(.001)
    assert policy.reacquire_speed_path_length == 0.
    policy._observe_reacquire_path(observation(1.01))
    assert policy.reacquire_max_displacement == pytest.approx(.001)
    assert policy.reacquire_path_length == pytest.approx(.001)


def test_odometry_uses_observation_time_not_policy_start_time():
    policy = recovery(observation(.025, velocity=(.0005, 0.)))
    # _begin_recovery's policy time is 0, but the first RTDE observation was
    # received at .025; counting from policy time would overestimate travel.
    policy._observe_reacquire_path(observation(.035, velocity=(.0005, 0.)))
    assert policy.reacquire_speed_path_length == pytest.approx(5e-6)


def test_duplicate_or_backwards_observation_does_not_duplicate_travel():
    policy = recovery()
    policy._observe_reacquire_path(observation(.1, velocity=(.002, 0.)))
    assert policy.reacquire_speed_path_length == pytest.approx(.0001)
    for stamp in (.1, .05, .1):
        policy._observe_reacquire_path(observation(stamp, velocity=(.02, 0.)))
        assert policy.reacquire_speed_path_length == pytest.approx(.0001)
    policy._observe_reacquire_path(observation(.2, velocity=(.002, 0.)))
    assert policy.reacquire_speed_path_length == pytest.approx(.0003)

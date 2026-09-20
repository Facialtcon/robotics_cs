from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from config.loader import load_config
from core.models import RobotState, Wrench
from policy.continuous_tracking import ContinuousTrackingPolicy, State

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def config():
    return deepcopy(load_config(ROOT / "config.yaml"))


def tick(policy, now, force=(1.5, 0), raw=None, pose=None):
    wrench = Wrench(*force, 0, 0, 0, 0)
    pose = np.zeros(6) if pose is None else np.asarray(pose)
    return policy.update(now, wrench if raw is None else raw, wrench, RobotState(now, pose, np.zeros(6)))


def tracking(config):
    policy = ContinuousTrackingPolicy(config)
    tick(policy, 0)
    tick(policy, .01)
    command = tick(policy, .07)
    assert command.state == "FIRST_CONTACT" and not command.move
    command = tick(policy, .08)
    assert command.state == "CONTINUOUS_TRACKING"
    return policy


def lose(policy):
    tick(policy, .09, (0, 0))
    command = tick(policy, .25, (0, 0))
    assert command.state == "CONTACT_LOST" and not command.move
    command = tick(policy, .26, (0, 0))
    assert command.state == "LOCAL_REACQUIRE" and not command.move
    return command


def test_first_stable_contact_stops_and_directly_tracks(config):
    policy = ContinuousTrackingPolicy(config)
    assert tick(policy, 0, (0, 0)).state == "READY"
    assert tick(policy, .01, (0, 0)).state == "TARGET_SEARCH"
    # Slow ramp preserves force-rate guard.
    tick(policy, .06, (1.0, 0))
    assert not tick(policy, .08, (1.0, 0)).move
    command = tick(policy, .12, (1.0, 0))
    assert command.state == "FIRST_CONTACT" and not command.move
    assert np.array_equal(policy.initial_contact, np.zeros(6))
    assert tick(policy, .13, (1.0, 0)).state == "CONTINUOUS_TRACKING"
    assert [event.event_type for event in policy.events] == ["FIRST_CONTACT"]


@pytest.mark.parametrize("force, expected", [(1.0, 1), (2.0, -1), (1.5, 0), (1.6, 0), (5.0, -1)])
def test_force_feedback_sign_deadband_and_normal_limit(config, force, expected):
    policy = tracking(config)
    command = tick(policy, .3, (force, 0))
    inward_component = np.dot(command.direction_xy * command.speed, policy.contact_direction)
    assert np.sign(inward_component) == expected
    assert abs(policy.v_n) <= config["continuous_tracking"]["normal_speed_limit"]


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("hand, rotation", [("CLOCKWISE", 1), ("COUNTERCLOCKWISE", -1)])
def test_direction_sign_hand_and_filter(config, sign, hand, rotation):
    config["continuous_tracking"]["force_direction_sign"] = sign
    config["policy"]["follow_hand"] = hand
    policy = tracking(config)
    assert np.allclose(policy.contact_direction, [sign, 0])
    assert np.allclose(policy.tangent, [0, rotation * sign])
    tick(policy, .1, (0, 1.5))
    # EMA weights PREVIOUS vector by .8. It must not immediately snap to +Y.
    expected = np.array([1.2, .3])
    expected /= np.linalg.norm(expected)
    assert np.allclose(policy.contact_direction, sign * expected)
    old = policy.tangent.copy()
    tick(policy, .11, (0, 1.5))
    assert not np.allclose(policy.tangent, old)
    assert np.isclose(np.dot(policy.contact_direction, policy.tangent), 0)


def test_combined_velocity_is_capped(config):
    config["robot"]["max_tcp_speed"] = .001
    policy = tracking(config)
    command = tick(policy, .3, (4, 0))
    assert command.speed <= .001
    assert np.isclose(np.hypot(policy.v_t, policy.v_n), command.speed)


def test_single_drop_resets_timer_and_preserves_reliable_context(config):
    policy = tracking(config)
    pose = policy.last_contact_pose.copy()
    tangent = policy.last_reliable_tangent.copy()
    assert tick(policy, .1, (0, 0)).state == "CONTINUOUS_TRACKING"
    assert np.array_equal(policy.last_contact_pose, pose)
    assert np.array_equal(policy.last_reliable_tangent, tangent)
    tick(policy, .2, (1.5, 0))
    assert policy.lost_timer == 0
    assert tick(policy, .3, (0, 0)).state == "CONTINUOUS_TRACKING"


def test_lost_stops_and_reacquire_uses_local_heading(config):
    policy = tracking(config)
    lose(policy)
    command = tick(policy, .27, (0, 0))
    assert command.move
    assert np.dot(command.direction_xy, policy.last_contact_direction) > .99
    assert abs(policy.reacquire_heading_deg) <= 60
    assert policy.v_t == 0
    for t in np.arange(.28, 7.9, .02):
        tick(policy, float(t), (0, 0))
        assert abs(policy.reacquire_heading_deg) <= 60 + 1e-10


def test_reacquire_requires_stable_contact_then_stops_search(config):
    policy = tracking(config)
    lose(policy)
    assert not tick(policy, .4, (0, 1.5)).move
    assert tick(policy, .42, (0, 1.5)).state == "LOCAL_REACQUIRE"
    command = tick(policy, .46, (0, 1.5))
    assert command.state == "CONTINUOUS_TRACKING" and not command.move
    assert policy.events[-1].event_type == "REACQUIRED"
    assert tick(policy, .47, (0, 1.5)).move


def test_reacquire_timeout_and_distance_stop(config):
    policy = tracking(config)
    lose(policy)
    assert tick(policy, 8.27, (0, 0)).state == "STOP"
    policy = tracking(config)
    lose(policy)
    assert tick(policy, .27, (0, 0), pose=[.004, 0, 0, 0, 0, 0]).state == "STOP"


@pytest.mark.parametrize("state", [State.TARGET_SEARCH, State.FIRST_CONTACT, State.CONTINUOUS_TRACKING,
                                   State.CONTACT_LOST, State.LOCAL_REACQUIRE])
@pytest.mark.parametrize("kind", ["processed_force", "processed_torque", "raw_force", "raw_torque", "rate", "nan"])
def test_safety_precedes_every_active_state(config, state, kind):
    policy = tracking(config)
    policy.state = state
    raw = Wrench(1.5, 0, 0, 0, 0, 0)
    processed = raw
    if kind == "processed_force":
        processed = Wrench(0, 0, 12, 0, 0, 0)
    elif kind == "processed_torque":
        processed = Wrench(1.5, 0, 0, 1, 0, 0)
    elif kind == "raw_force":
        raw = Wrench(60, 0, 0, 0, 0, 0)
    elif kind == "raw_torque":
        raw = Wrench(1.5, 0, 0, 0, 0, 5)
    elif kind == "rate":
        processed = Wrench(2, 0, 0, 0, 0, 0)  # 50 N/s, below magnitude threshold
    else:
        raw = Wrench(float("nan"), 0, 0, 0, 0, 0)
    command = policy.update(.09, raw, processed, RobotState(.09, np.zeros(6), np.zeros(6)))
    assert command.state == "STOP" and not command.move
    assert policy.events[-1].event_type == "SAFETY_STOP"
    assert not tick(policy, .1).move  # terminal stop cannot be undone


def test_search_direction_budget_and_nonmonotonic_time(config):
    config["policy"]["search_direction_xy"] = [0, 1]
    policy = ContinuousTrackingPolicy(config)
    tick(policy, 0, (0, 0))
    assert np.allclose(tick(policy, .01, (0, 0)).direction_xy, [0, 1])
    assert tick(policy, .02, (0, 0), pose=[0, .1, 0, 0, 0, 0]).state == "STOP"
    policy = tracking(config)
    assert tick(policy, .08).state == "STOP"


@pytest.mark.parametrize("key,value", [("force_direction_sign", 0), ("force_direction_sign", True),
    ("force_gain", float("nan")), ("force_direction_filter_alpha", 1), ("force_deadband", -1),
    ("reacquire_max_angle_deg", 120), ("search_speed", .03), ("force_reference", 12)])
def test_invalid_parameters_fail_closed(config, key, value):
    config["continuous_tracking"][key] = value
    with pytest.raises(ValueError):
        ContinuousTrackingPolicy(config)

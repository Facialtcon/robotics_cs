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
    c = deepcopy(load_config(ROOT / "config.yaml"))
    c['continuous_tracking']['reacquire_enabled'] = True  # explicit offline unit fixture
    # These legacy synthetic traces report targetward force (+X), as does the
    # analytic simulator; they do not represent environment-on-probe PX6D data.
    c['continuous_tracking']['force_direction_sign'] = 1
    return c


def tick(policy, now, force=(1.5, 0), raw=None, pose=None):
    wrench = Wrench(*force, 0, 0, 0, 0)
    pose = np.zeros(6) if pose is None else np.asarray(pose)
    return policy.update(now, wrench if raw is None else raw, wrench, RobotState(now, pose, np.zeros(6)))


def tracking(config):
    policy = ContinuousTrackingPolicy(config)
    for i in range(101):
        command = tick(policy, i*.01)
    assert command.state == "CONTINUOUS_TRACKING"
    return policy


def lose(policy):
    for i in range(101, 117):
        command = tick(policy, i*.01, (0, 0))
    assert command.state == "CONTACT_LOST" and not command.move
    command = tick(policy, 1.17, (0, 0))
    assert command.state == "LOCAL_REACQUIRE" and not command.move
    return command


def test_first_stable_contact_stops_and_directly_tracks(config):
    policy = ContinuousTrackingPolicy(config)
    assert not tick(policy, 0, (0, 0)).move
    assert tick(policy, .01, (0, 0)).state == "TARGET_SEARCH"
    for i in range(2, 7):
        tick(policy, i*.01, ((i-1)*.2, 0))
    assert policy.state == State.FIRST_CONTACT and policy.initial_contact is None
    for i in range(7, 30):
        command = tick(policy, i*.01, (1.0, 0))
    assert command.state == "CONTINUOUS_TRACKING"
    assert np.array_equal(policy.initial_contact, np.zeros(6))
    assert [event.event_type for event in policy.events] == ["FIRST_THRESHOLD_STOP_REQUEST",
        'FIRST_CONTACT_LOW_SPEED_OBSERVED', 'STANDSTILL_CONFIRMED', "FIRST_CONTACT", 'TRACKING_ENTERED']


@pytest.mark.parametrize("force, expected", [(1.0, 1), (2.0, -1), (1.5, 0), (1.6, 0), (5.0, -1)])
def test_force_feedback_sign_deadband_and_normal_limit(config, force, expected):
    policy = tracking(config)
    # Fresh gradual observations, below the unchanged 30 N/s force-rate limit.
    for i, f in enumerate(np.linspace(1.5, force, 21)[1:], 101):
        command = tick(policy, i*.01, (f, 0))
    assert command.state == "CONTINUOUS_TRACKING"
    inward = np.dot(command.direction_xy * command.speed, policy.contact_direction)
    assert np.sign(inward) == expected
    assert abs(policy.v_n) <= config["continuous_tracking"]["normal_speed_limit"] + 1e-12


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("hand, rotation", [("CLOCKWISE", 1), ("COUNTERCLOCKWISE", -1)])
def test_direction_sign_hand_and_filter(config, sign, hand, rotation):
    config["continuous_tracking"]["force_direction_sign"] = sign
    config["policy"]["follow_hand"] = hand
    policy = tracking(config)
    assert np.allclose(policy.contact_direction, [sign, 0])
    assert np.allclose(policy.tangent, [0, rotation * sign])
    angle = np.deg2rad(5)
    f = 1.5*np.array([np.cos(angle), np.sin(angle)])
    tick(policy, 1.01, f)
    expected = .8*np.array([1.5,0])+.2*f
    assert np.allclose(policy._filtered, expected)
    assert np.linalg.norm(policy.contact_direction - sign*f/1.5) > .01
    old = policy.tangent.copy()
    tick(policy, 1.02, f)
    assert not np.allclose(policy.tangent, old)
    assert np.isclose(np.dot(policy.contact_direction, policy.tangent), 0)


def test_combined_velocity_is_capped(config):
    config["robot"]["max_tcp_speed"] = .001
    config['continuous_tracking']['search_speed'] = .001  # keep this reduced-cap fixture valid
    policy = tracking(config)
    for i in range(101, 121):
        command = tick(policy, i*.01, (1.5+(i-100)*.125, 0))
        assert command.speed <= .001 + 1e-12
        assert np.isclose(np.hypot(policy.v_t, policy.v_n), command.speed)


def test_single_drop_resets_timer_and_preserves_reliable_context(config):
    policy = tracking(config)
    pose, tangent = policy.last_contact_pose.copy(), policy.last_reliable_tangent.copy()
    assert tick(policy, 1.01, (0, 0)).state == "CONTINUOUS_TRACKING"
    assert np.array_equal(policy.last_contact_pose, pose)
    assert np.array_equal(policy.last_reliable_tangent, tangent)
    for i in range(102, 108):
        tick(policy, i*.01, ((i-101)*.25, 0))
    assert policy.lost_timer == 0
    assert tick(policy, 1.08, (0, 0)).state == "CONTINUOUS_TRACKING"


def test_lost_stops_and_reacquire_uses_local_heading(config):
    policy = tracking(config)
    lose(policy)
    command = tick(policy, 1.18, (0, 0))
    assert command.move
    assert np.dot(command.direction_xy, policy.last_contact_direction) > .99
    assert 0 < policy.reacquire_heading_deg <= 60
    assert np.allclose(policy.reacquire_origin, np.zeros(6))
    assert policy.reacquire_reference[1] > 0  # deliberate motion toward -t_mem


def test_reacquire_requires_stable_contact_then_stops_search(config):
    # Isolated state transition only. Geometric proof lives in test_continuous_geometry.
    policy = tracking(config)
    lose(policy)
    for i in range(118, 124):
        tick(policy, i*.01, (0, (i-117)*.25))
    assert policy.state == State.LOCAL_REACQUIRE
    command = None
    for i in range(124, 130):
        command = tick(policy, i*.01, (0, 1.5))
    assert policy.state == State.CONTINUOUS_TRACKING
    assert policy.events[-1].event_type == "REACQUIRED"
    assert tick(policy, 1.30, (0, 1.5)).move


def test_reacquire_timeout_and_distance_stop(config):
    policy = tracking(config)
    lose(policy)
    for i in range(118, 920):
        command = tick(policy, i*.01, (0, 0))
        if policy.state == State.STOP:
            break
    assert command.state == "STOP" and 'time' in policy.reason
    policy = tracking(config)
    lose(policy)
    assert tick(policy, 1.18, (0, 0), pose=[.004, 0, 0, 0, 0, 0]).state == "STOP"


# Phase-specific rate assertions are covered in test_continuous_force_rate.py.
@pytest.mark.parametrize("state", [State.TARGET_SEARCH, State.FIRST_CONTACT, State.CONTINUOUS_TRACKING,
                                   State.CONTACT_LOST, State.LOCAL_REACQUIRE])
@pytest.mark.parametrize("kind", ["processed_force", "processed_torque", "raw_force", "raw_torque", "nan"])
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
    else:
        raw = Wrench(float("nan"), 0, 0, 0, 0, 0)
    command = policy.update(1.01, raw, processed, RobotState(1.01, np.zeros(6), np.zeros(6)))
    assert command.state == "STOP" and not command.move
    assert policy.events[-1].event_type == "SAFETY_STOP"
    assert not tick(policy, 1.02).move  # terminal stop cannot be undone


def test_search_direction_budget_and_nonmonotonic_time(config):
    config["policy"]["search_direction_xy"] = [0, 1]
    policy = ContinuousTrackingPolicy(config)
    tick(policy, 0, (0, 0))
    assert np.allclose(tick(policy, .01, (0, 0)).direction_xy, [0, 1])
    assert tick(policy, .02, (0, 0), pose=[0, .1, 0, 0, 0, 0]).state == "STOP"
    policy = tracking(config)
    assert tick(policy, 1.).state == "STOP"


@pytest.mark.parametrize("key,value", [("force_direction_sign", 0), ("force_direction_sign", True),
    ("force_gain", float("nan")), ("force_direction_filter_alpha", 1), ("force_deadband", -1),
    ("reacquire_max_angle_deg", 120), ("search_speed", .03), ("force_reference", 12)])
def test_invalid_parameters_fail_closed(config, key, value):
    config["continuous_tracking"][key] = value
    with pytest.raises(ValueError):
        ContinuousTrackingPolicy(config)


from experiment_logging.termination import TerminationReason
RATE_POSE = np.zeros(6)

def rate_tick(policy, now, force, speed=0., raw=None):
    processed = Wrench(force, 0, 0, 0, 0, 0)
    return policy.update(now, processed if raw is None else raw, processed,
                         RobotState(now, RATE_POSE.copy(), np.array([speed, 0, 0, 0, 0, 0])))


def in_phase(state):
    config = load_config(ROOT/'config.yaml')
    config['continuous_tracking']['reacquire_enabled'] = True  # offline fixture only
    p = ContinuousTrackingPolicy(config)
    p.state = state
    p._started = p._last_time = 0.
    p._start_pose = RATE_POSE.copy()
    p._rate.update(0., .4)
    p.contact_direction = np.array([1., 0.])
    p.tangent = np.array([0., -1.])
    p.direction_confidence = 1.
    p._remember(0., RATE_POSE)
    if state in (State.FIRST_CONTACT, State.DIRECTION_RECONFIRM, State.CONTACT_LOST):
        p._reset_confirmation(0.)
    if state == State.LOCAL_REACQUIRE:
        p._begin_recovery(0., RATE_POSE)
    return p


def test_high_onset_then_braking_then_confirmed_tracking_and_rate_trip():
    p = ContinuousTrackingPolicy(load_config(ROOT/'config.yaml'))
    for i in range(201):
        command = rate_tick(p, i*.01, .4, speed=.018)
    assert command.speed == pytest.approx(.018)
    command = rate_tick(p, 2.01, 1.2, speed=.018)
    assert p.force_rate == pytest.approx(80.)
    assert p.state == State.FIRST_CONTACT and not command.move
    assert p.stop_requested and p.first_threshold_pose is not None
    for i, speed in zip(range(202, 206), [.018, .014, .008, .003]):
        command = rate_tick(p, i*.01, 1.7, speed=speed)
        assert p.state == State.FIRST_CONTACT and not command.move
        assert not p.force_rate_guard_active and not p.stop_confirmed
    for i in range(206, 214):
        assert not rate_tick(p, i*.01, 1.7).move
        assert p.state == State.FIRST_CONTACT
    command = rate_tick(p, 2.14, 1.7)
    assert p.state == State.CONTINUOUS_TRACKING and p.stop_confirmed
    assert not command.move and not p.force_rate_guard_active
    command = rate_tick(p, 2.15, 1.7)
    assert command.move and p.force_rate_guard_active
    command = rate_tick(p, 2.16, 2.2)
    assert p.state == State.STOP and not command.move
    assert p.stop_reason == TerminationReason.STOP_FORCE_LIMIT
    assert 'force rate exceeded' in p.reason
    assert p.telemetry(command)['force_rate_guard_active'] == 1  # decision for this sample


def test_tracking_positive_rate_guard_still_uses_original_limit():
    p = in_phase(State.CONTINUOUS_TRACKING)
    assert p.p['force_rate_limit'] == 30.
    command = rate_tick(p, .01, 1.2)
    assert not command.move and p.state == State.STOP
    assert p.force_rate == pytest.approx(80.)
    assert p.stop_reason == TerminationReason.STOP_FORCE_LIMIT
    assert p.force_rate_guard_active

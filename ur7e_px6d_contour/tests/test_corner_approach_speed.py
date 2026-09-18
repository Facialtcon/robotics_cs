"""Offline checks for corner-only approach speed; existing stop guards stay first."""
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from config.loader import load_config
from core.models import RobotState, Wrench
from experiment_logging.termination import TerminationReason
from policy.probe_episode import ProbeEpisode
from policy.rule_policy import RuleBasedPolicy, State


ROOT = Path(__file__).resolve().parents[1]
ZERO = Wrench(0, 0, 0, 0, 0, 0)
CORNER_PURPOSES = ("RECOVERY", "CONFIRMATION")


@pytest.fixture
def config():
    return {**load_config(ROOT / "config.yaml")["policy"],
            "corner_approach_force_ratio": .5, "corner_approach_speed": .002}


def probe(purpose, probe_id=0):
    return ProbeEpisode(probe_id, np.array([.4, -.2, .3, 0, 3.14, 0]),
                        [3., 4.], 0., .012,
                        State.BOUNDARY_RECOVERY.value if purpose == "RECOVERY"
                        else State.BOUNDARY_CONFIRMATION.value, purpose)


def force(fxy):
    return Wrench(0, fxy, 0, 0, 0, 0)


def nominal_speed(config, purpose):
    return config["recovery_probe_speed"] if purpose == "RECOVERY" else config["probe_speed"]


@pytest.mark.parametrize("purpose", CORNER_PURPOSES)
@pytest.mark.parametrize("contact_threshold", [1., 2.])
def test_corner_speed_changes_at_ratio_without_changing_probe_geometry(config, purpose, contact_threshold):
    config["contact_threshold"] = contact_threshold
    original = deepcopy(config)
    episode = probe(purpose)
    pose = episode.anchor_pose.copy()
    trigger = config["corner_approach_force_ratio"] * contact_threshold
    expected_target = pose.copy()
    expected_target[:2] += .012 * episode.probe_direction
    target, speed = episode.tick(0, pose, force(trigger - .001), config)
    assert speed == nominal_speed(config, purpose)
    assert np.array_equal(target, expected_target)
    target, speed = episode.tick(.1, pose, force(trigger), config)
    assert speed == .002
    assert np.array_equal(target, expected_target)
    assert np.array_equal(episode.anchor_pose, pose)
    assert np.allclose(episode.probe_direction, [.6, .8])
    assert episode.max_probe_distance == .012 and episode.phase == "PROBE"
    assert episode.contact_pose is None and not episode.return_completed
    assert config == original


@pytest.mark.parametrize("purpose", CORNER_PURPOSES)
def test_force_fall_does_not_speed_up_but_new_episode_starts_at_nominal(config, purpose):
    episode = probe(purpose)
    pose = episode.anchor_pose.copy()
    assert episode.tick(0, pose, force(.8), config)[1] == .002
    for index, fxy in enumerate((.49, .2, 0., .3), start=1):
        assert episode.tick(index * .1, pose, force(fxy), config)[1] == .002
        assert episode.max_force == .8
        assert episode.phase == "PROBE"
    fresh = probe(purpose, probe_id=1)
    assert fresh.tick(1., pose, force(.2), config)[1] == nominal_speed(config, purpose)
    assert fresh.max_force == .2


@pytest.mark.parametrize("purpose", ["ACQUISITION", "INITIALIZATION", "TRACKING"])
def test_noncorner_probe_speed_is_unchanged_even_above_slowdown_ratio(config, purpose):
    episode = probe(purpose)
    expected = config["search_speed"] if purpose == "ACQUISITION" else config["probe_speed"]
    for index, fxy in enumerate((0., .5, .9, .2)):
        target, speed = episode.tick(index * .1, episode.anchor_pose.copy(), force(fxy), config)
        assert speed == expected and episode.phase == "PROBE"
        assert np.allclose(target[:2] - episode.anchor_pose[:2], .012 * episode.probe_direction)


@pytest.mark.parametrize("purpose", CORNER_PURPOSES)
def test_corner_cap_never_increases_a_slower_nominal_speed(config, purpose):
    config.update(probe_speed=.0015, recovery_probe_speed=.001)
    RuleBasedPolicy(config)  # The cap may exceed a slower simulation profile.
    episode = probe(purpose)
    target, speed = episode.tick(0, episode.anchor_pose.copy(), force(.8), config)
    assert speed == nominal_speed(config, purpose)
    assert speed < config["corner_approach_speed"]
    assert np.allclose(target[:2] - episode.anchor_pose[:2], .012 * episode.probe_direction)


@pytest.mark.parametrize("purpose", CORNER_PURPOSES)
def test_old_config_uses_same_defaults_without_inserting_parameters(config, purpose):
    config.pop("corner_approach_force_ratio")
    config.pop("corner_approach_speed")
    original = deepcopy(config)
    policy = RuleBasedPolicy(config)
    episode = probe(purpose)
    assert episode.tick(0, episode.anchor_pose.copy(), force(.49), policy.config)[1] == nominal_speed(config, purpose)
    assert episode.tick(.1, episode.anchor_pose.copy(), force(.5), policy.config)[1] == .002
    assert config == policy.config == original


@pytest.mark.parametrize("purpose", CORNER_PURPOSES)
def test_contact_still_holds_immediately_and_return_keeps_retract_speed(config, purpose):
    episode = probe(purpose)
    pose = episode.anchor_pose.copy()
    pose[:2] += .004 * episode.probe_direction
    assert episode.tick(0, pose, force(.5), config)[1] == .002
    contact = force(config["contact_threshold"])
    assert episode.tick(.1, pose, contact, config) is None
    assert episode.phase == "HOLD" and episode.contact_time == .1
    assert np.array_equal(episode.contact_pose, pose)
    assert episode.tick(.1 + config["contact_hold_time"] + .001, pose, contact, config) is None
    assert episode.phase == "RETURN" and episode.outcome == "CONTACT"
    target, speed = episode.tick(.3, pose, ZERO, config, tcp_speed=np.zeros(6))
    assert speed == config["retract_speed"] == .012
    assert np.array_equal(target, episode.anchor_pose)
    assert not episode.return_completed and not episode.accepted_as_boundary


@pytest.mark.parametrize("purpose", CORNER_PURPOSES)
def test_slowed_corner_probe_still_stops_at_original_no_contact_distance(config, purpose):
    episode = probe(purpose)
    assert episode.tick(0, episode.anchor_pose.copy(), force(.7), config)[1] == .002
    endpoint = episode.anchor_pose.copy()
    endpoint[:2] += episode.max_probe_distance * episode.probe_direction
    assert episode.tick(.2, endpoint, force(.2), config) is None
    assert episode.phase == "RETURN" and episode.outcome == "NO_CONTACT"
    assert episode.contact_pose is None and not episode.return_completed
    target, speed = episode.tick(.3, endpoint, ZERO, config)
    assert np.array_equal(target, episode.anchor_pose)
    assert speed == config["retract_speed"]


@pytest.mark.parametrize("name,value", [
    *(('corner_approach_force_ratio', value)
      for value in (0, -.1, 1., 1.1, float("nan"), float("inf"), -float("inf"), True, False, None, "invalid")),
    *(('corner_approach_speed', value)
      for value in (0, -.1, float("nan"), float("inf"), -float("inf"), True, False, None, "invalid")),
])
def test_invalid_corner_approach_parameters_are_rejected(config, name, value):
    with pytest.raises(ValueError, match=name):
        RuleBasedPolicy({**config, name: value})


def active_corner_policy(config, purpose):
    policy = RuleBasedPolicy(config)
    policy.state = State.BOUNDARY_RECOVERY if purpose == "RECOVERY" else State.BOUNDARY_CONFIRMATION
    episode = probe(purpose)
    policy.active_episode = episode
    policy.probe_episodes.append(episode)
    pose = episode.anchor_pose.copy()
    command = policy.update(0, ZERO, force(.6), RobotState(0, pose, np.zeros(6)))
    assert command.move and command.speed == .002
    return policy, episode, pose


@pytest.mark.parametrize("purpose", CORNER_PURPOSES)
@pytest.mark.parametrize("raw,processed,detail", [
    (ZERO, Wrench(0, .6, 12., 0, 0, 0), "processed force safety threshold exceeded"),
    (ZERO, Wrench(0, .6, 0, 1., 0, 0), "processed torque safety threshold exceeded"),
    (Wrench(0, 0, 60., 0, 0, 0), force(.6), "absolute raw force safety threshold exceeded"),
    (Wrench(0, 0, 0, 5., 0, 0), force(.6), "absolute raw torque safety threshold exceeded"),
])
def test_existing_force_and_torque_guards_still_preempt_slow_probe(config, purpose, raw, processed, detail):
    policy, episode, pose = active_corner_policy(config, purpose)
    observed_count = len(episode.force_samples)
    command = policy.update(.1, raw, processed, RobotState(.1, pose, np.zeros(6)))
    assert command.state == "STOP" and not command.move and command.speed == 0
    assert command.reason == detail
    assert policy.termination.record["reason"] == TerminationReason.STOP_FORCE_LIMIT.value
    assert episode.outcome == "ABORTED" and not episode.return_completed
    assert len(episode.force_samples) == observed_count  # Safety ran before tick.


@pytest.mark.parametrize("purpose", CORNER_PURPOSES)
def test_force_rate_spike_still_stops_before_corner_speed_selection(config, purpose):
    policy, episode, pose = active_corner_policy(config, purpose)
    observed_count = len(episode.force_samples)
    command = policy.update(.005, ZERO, force(.9), RobotState(.005, pose, np.zeros(6)))
    assert config["contact_threshold"] == 1. and config["force_rate_limit"] == 30.
    assert command.state == "STOP" and not command.move
    assert command.reason.startswith("force rate exceeded:")
    assert policy.termination.record["reason"] == TerminationReason.STOP_FORCE_LIMIT.value
    assert episode.outcome == "ABORTED" and len(episode.force_samples) == observed_count

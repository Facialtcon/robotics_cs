"""Recorded corner inputs with counterfactual commands, never synthetic success.

The fixture contains the real TCP motion and wrench through the historical
safety stop. Replaying that motion can establish when a different command
would be issued; it cannot establish the motion or wrench that command would
produce on hardware. The separate readiness checks use explicit synthetic
samples and make no physical contour-completion claim.
"""
import json
from pathlib import Path

import numpy as np
import pytest

from core.models import RobotState, Wrench
from experiment_logging.termination import TerminationReason
from policy.probe_episode import ProbeEpisode
from policy.rule_policy import RuleBasedPolicy, State
from sensor.force_preprocess import WrenchPreprocessor


FIXTURE = Path(__file__).parent / "fixtures/corner_contact_20260914.json"


@pytest.fixture
def recorded_corner():
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def _episode(fixture, purpose="CONFIRMATION", start=None):
    data = fixture["episode"]
    state = State.BOUNDARY_RECOVERY if purpose == "RECOVERY" else State.BOUNDARY_CONFIRMATION
    return ProbeEpisode(
        data["probe_id"], data["anchor_pose"], data["probe_direction"],
        data["probe_start_time"] if start is None else start,
        data["max_probe_distance"], state.value, purpose,
        recovery_id=data["recovery_id"], angle_deg=data["angle_deg"],
    )


def _seed_policy(fixture, purpose):
    policy = RuleBasedPolicy(fixture["policy_config"])
    policy.state = State.BOUNDARY_RECOVERY if purpose == "RECOVERY" else State.BOUNDARY_CONFIRMATION
    policy.current_tangent = np.array(fixture["initial_context"]["current_tangent"])
    policy.current_target_direction = np.array(fixture["initial_context"]["current_normal"])
    policy.active_episode = _episode(fixture, purpose)
    policy.probe_episodes.append(policy.active_episode)
    policy.sub_state = "PROBE"
    previous = fixture["force_guard_previous"]
    policy._force_rate_guard.previous = (previous["timestamp"], previous["fxy"])
    return policy


@pytest.mark.parametrize("purpose", ["CONFIRMATION", "RECOVERY"])
def test_recorded_inputs_trigger_early_slow_command_and_preserve_hard_stop(recorded_corner, purpose):
    fixture = recorded_corner
    policy = _seed_policy(fixture, purpose)
    episode = policy.active_episode
    commands = []
    for frame in fixture["frames"]:
        command = policy.update(
            frame["timestamp"], Wrench.from_sequence(frame["raw"]),
            Wrench.from_sequence(frame["processed"]),
            RobotState(frame["timestamp"], np.array(frame["pose"]), np.array(frame["speed"])),
        )
        commands.append(command)
        if frame is not fixture["frames"][-1]:
            assert command.move
            assert command.policy_sub_state == "PROBE"
            np.testing.assert_allclose(command.target_pose, frame["recorded_command"]["target_pose"], atol=1e-12)
            np.testing.assert_allclose(command.direction_xy, frame["recorded_command"]["direction_xy"], atol=1e-12)
    first_slow = next(i for i, command in enumerate(commands) if command.move and command.speed < .006)
    onset_frame = fixture["frames"][first_slow]
    assert onset_frame["timestamp"] == pytest.approx(306820.524268365, abs=1e-9)
    assert fixture["expected_stop"]["timestamp"] - onset_frame["timestamp"] == pytest.approx(.2942494099843316)
    assert np.linalg.norm(onset_frame["processed"][:2]) == pytest.approx(.5039348666619308)
    assert all(command.speed == .006 for command in commands[:first_slow])
    assert all(command.speed == .002 for command in commands[first_slow:-1])
    # Measured speed remains the original run's approximately 6 mm/s. This is
    # deliberately not overwritten with the counterfactual 2 mm/s command.
    assert np.linalg.norm(onset_frame["speed"][:3]) > .005
    assert onset_frame["recorded_command"]["speed"] == .006
    assert policy.state == State.STOP and not commands[-1].move
    assert policy.termination.record["reason"] == TerminationReason.STOP_FORCE_LIMIT.value
    assert policy.reason == fixture["expected_stop"]["recorded_detail"]
    assert episode.outcome == "ABORTED" and not episode.return_completed
    assert episode.contact_pose is None and episode.hold_since is None
    assert episode.force_samples[-1][0] == fixture["frames"][-2]["timestamp"]
    # The unchanged guard preempts episode.tick on the final force-rate spike.
    previous, stopped = fixture["frames"][-2:]
    rate = (np.linalg.norm(stopped["processed"][:2]) - np.linalg.norm(previous["processed"][:2])) / (stopped["timestamp"] - previous["timestamp"])
    assert rate == pytest.approx(34.09784318286094)
    assert rate > fixture["policy_config"]["force_rate_limit"]


def test_portable_recording_preserves_the_original_ema_and_bias(recorded_corner):
    fixture = recorded_corner
    preprocessor = WrenchPreprocessor.from_config(fixture["preprocessing_config"])
    preprocessor.zero_bias_sensor = np.array(fixture["inferred_zero_bias_sensor"])
    preprocessor._filtered_sensor = np.array(fixture["initial_context"]["processed"])
    for frame in fixture["frames"]:
        actual = preprocessor.process(Wrench.from_sequence(frame["raw"]))
        np.testing.assert_allclose(actual.array(), frame["processed"], atol=1e-12, rtol=0)
    assert fixture["provenance"]["physical_slow_approach_validated"] is False
    assert fixture["provenance"]["post_stop_samples_available"] is False


@pytest.mark.parametrize("purpose", ["RECOVERY", "CONFIRMATION"])
def test_corner_cap_latches_per_episode_without_reaccelerating_on_force_drop(recorded_corner, purpose):
    cfg = recorded_corner["policy_config"]
    episode = _episode(recorded_corner, purpose, start=0.)
    anchor = episode.anchor_pose.copy()
    expected_target = anchor.copy()
    expected_target[:2] += episode.probe_direction * episode.max_probe_distance
    # Explicit synthetic sub-threshold samples exercise the latch, including
    # a force drop that must not accelerate the same ray again.
    for now, force, expected_speed in [(0., .49, .006), (.01, .5, .002), (.02, .2, .002)]:
        target, speed = episode.tick(now, anchor, Wrench(force, 0., 0., 0., 0., 0.), cfg)
        np.testing.assert_array_equal(target, expected_target)
        assert speed == expected_speed
    fresh = _episode(recorded_corner, purpose, start=.03)
    assert fresh.tick(.03, anchor, Wrench(.2, 0., 0., 0., 0., 0.), cfg)[1] == .006


@pytest.mark.parametrize("purpose", ["RECOVERY", "CONFIRMATION"])
def test_synthetic_threshold_hold_and_settled_return_keep_priority_over_speed_cap(recorded_corner, purpose):
    cfg = recorded_corner["policy_config"]
    episode = _episode(recorded_corner, purpose, start=0.)
    anchor = episode.anchor_pose.copy()
    contact = anchor.copy()
    contact[:2] += .005 * episode.probe_direction
    low = Wrench(.1, 0., 0., 0., 0., 0.)
    threshold = Wrench(1.01, 0., 0., 0., 0., 0.)
    assert episode.tick(0., contact, Wrench(.6, 0., 0., 0., 0., 0.), cfg)[1] == .002
    assert episode.tick(.01, contact, threshold, cfg) is None
    assert episode.phase == "HOLD"
    assert episode.tick(.07, contact, threshold, cfg) is None
    assert episode.phase == "RETURN" and episode.outcome == "CONTACT"
    target, speed = episode.tick(.08, contact, low, cfg, tcp_speed=np.zeros(6))
    np.testing.assert_array_equal(target, anchor)
    assert speed == cfg["retract_speed"]
    # Position alone cannot finish return: measured speed and force must settle.
    assert episode.tick(.09, anchor, low, cfg, tcp_speed=np.array([.006, 0., 0., 0., 0., 0.])) is None
    assert not episode.return_completed
    assert episode.tick(.10, anchor, threshold, cfg, tcp_speed=np.zeros(6)) is None
    assert not episode.return_completed
    for now in (.11, .15):
        assert episode.tick(now, anchor, low, cfg, tcp_speed=np.zeros(6)) is None
        assert not episode.return_completed
    assert episode.tick(.17, anchor, low, cfg, tcp_speed=np.zeros(6)) is None
    assert episode.return_completed


@pytest.mark.parametrize("purpose", ["TRACKING", "INITIALIZATION", "ACQUISITION"])
def test_noncorner_probe_speeds_remain_unchanged(recorded_corner, purpose):
    cfg = recorded_corner["policy_config"]
    episode = _episode(recorded_corner, purpose, start=0.)
    motion = episode.tick(0., episode.anchor_pose.copy(), Wrench(.8, 0., 0., 0., 0., 0.), cfg)
    expected_speed = cfg["search_speed"] if purpose == "ACQUISITION" else cfg["probe_speed"]
    assert motion[1] == expected_speed

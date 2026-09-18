"""Readiness at the immutable return anchor precedes every next decision."""

import numpy as np
import pytest

from core.models import Wrench
from policy.probe_episode import ProbeEpisode, ReturnReadinessError


ZERO = Wrench(0, 0, 0, 0, 0, 0)
CONTACT = Wrench(1.1, 0, 0, 0, 0, 0)
CONFIG = {
    "position_tolerance": .0002, "contact_threshold": 1., "contact_hold_time": .05,
    "retract_speed": .006, "probe_speed": .003, "search_speed": .009,
}


def returning_episode(outcome="CONTACT", anchor=None):
    anchor = np.zeros(6) if anchor is None else np.asarray(anchor, dtype=float)
    episode = ProbeEpisode(4, anchor, [1., 0.], 0., .01, "BOUNDARY_TRACKING", "TRACKING")
    contact = anchor.copy()
    contact[0] += .01
    if outcome == "CONTACT":
        episode.tick(0., contact, CONTACT, CONFIG)
        episode.tick(.051, contact, CONTACT, CONFIG)
    elif outcome == "UNSTABLE_CONTACT":
        episode.tick(0., contact, CONTACT, CONFIG)
        episode.tick(.02, contact, ZERO, CONFIG)
    else:
        episode.tick(0., contact, ZERO, CONFIG)
    assert episode.phase == "RETURN"
    assert episode.outcome == outcome
    assert not episode.return_completed
    return episode


@pytest.mark.parametrize("outcome", ["CONTACT", "NO_CONTACT", "UNSTABLE_CONTACT"])
def test_every_probe_outcome_requires_a_continuous_return_hold(outcome):
    episode = returning_episode(outcome)
    anchor = episode.anchor_pose.copy()
    assert episode.tick(1., anchor, ZERO, CONFIG, np.zeros(6)) is None
    assert not episode.return_completed
    assert episode.phase == "RETURN"
    assert episode.return_end_time is episode.returned_pose is None
    assert episode.tick(1.049, anchor, ZERO, CONFIG, np.zeros(6)) is None
    assert not episode.return_completed
    assert episode.tick(1.051, anchor, ZERO, CONFIG, np.zeros(6)) is None
    assert episode.return_completed and episode.phase == "DONE"
    assert episode.return_end_time == 1.051
    assert episode.outcome == outcome
    assert np.array_equal(episode.returned_pose, episode.anchor_pose)
    assert [sample[2] for sample in episode.force_samples[-3:]] == ["RETURN"] * 3


def test_logged_episode_four_samples_do_not_report_return_ready():
    # Measured 2026-09-14 episode 4 samples, expressed relative to its anchor.
    # The old logic accepted the first pose solely because error < 0.2 mm.
    episode = returning_episode()
    arrived = np.array([.0000659050094693, -.00017881066901651, 0., 0., 0., 0.])
    force = Wrench(-.28732965380885206, .9987334289520211, .25, 0., 0., 0.)
    speed = np.array([-.0025456403813024597, .005830548285794965, 0., 0., 0., 0.])
    assert np.linalg.norm(arrived[:2]) < CONFIG["position_tolerance"]
    assert episode.tick(1., arrived, force, CONFIG, speed) is None
    assert not episode.return_completed
    next_pose = np.array([.0000433664076582, -.00013781394954658, 0., 0., 0., 0.])
    filtered_tail = Wrench(-.29613127637654946, .9682218648886066, .25, 0., 0., 0.)
    almost_still = np.array([.000005811183181248947, .000006824779355221952, 0., 0., 0., 0.])
    assert episode.tick(1.047912867, next_pose, filtered_tail, CONFIG, almost_still) is None
    assert not episode.return_completed
    assert episode.return_settle_since is None
    assert episode.return_end_time is episode.returned_pose is None
    # Later hypothetical unloaded samples can finish; these are not hardware data.
    assert episode.tick(1.10, next_pose, ZERO, CONFIG, almost_still) is None
    assert not episode.return_completed
    assert episode.tick(1.151, next_pose, ZERO, CONFIG, almost_still) is None
    assert episode.return_completed


@pytest.mark.parametrize("disturbance", ["force", "speed", "position"])
def test_any_readiness_violation_restarts_the_whole_hold(disturbance):
    episode = returning_episode()
    anchor = episode.anchor_pose.copy()
    episode.tick(1., anchor, ZERO, CONFIG, np.zeros(6))
    pose, force, speed = anchor.copy(), ZERO, np.zeros(6)
    if disturbance == "force":
        force = Wrench(CONFIG["contact_threshold"], 0., 0., 0., 0., 0.)
    elif disturbance == "speed":
        speed[0] = .000501
    else:
        pose[0] += .000201
    motion = episode.tick(1.04, pose, force, CONFIG, speed)
    assert not episode.return_completed
    assert episode.return_settle_since is None
    assert episode.return_settle_started_at == 1.
    if disturbance == "position":
        assert np.array_equal(motion[0], episode.anchor_pose)
        assert motion[1] == CONFIG["retract_speed"]
    else:
        assert motion is None
    episode.tick(1.07, anchor, ZERO, CONFIG, np.zeros(6))
    episode.tick(1.119, anchor, ZERO, CONFIG, np.zeros(6))
    assert not episode.return_completed
    episode.tick(1.121, anchor, ZERO, CONFIG, np.zeros(6))
    assert episode.return_completed


def test_readiness_measures_xyz_linear_speed_without_using_angular_components():
    episode = returning_episode()
    anchor = episode.anchor_pose.copy()
    episode.tick(1., anchor, ZERO, CONFIG, [0., 0., .0006, 0., 0., 0.])
    episode.tick(1.2, anchor, ZERO, CONFIG, [0., 0., .0006, 0., 0., 0.])
    assert not episode.return_completed
    assert episode.return_settle_since is None
    for time in (1.3, 1.351):
        episode.tick(time, anchor, ZERO, CONFIG, [0., 0., 0., 1., 2., 3.])
    assert episode.return_completed


def test_missing_legacy_velocity_never_bypasses_the_hold():
    episode = returning_episode()
    episode.tick(1., episode.anchor_pose.copy(), ZERO, CONFIG)
    assert not episode.return_completed
    episode.tick(1.049, episode.anchor_pose.copy(), ZERO, CONFIG)
    assert not episode.return_completed
    episode.tick(1.051, episode.anchor_pose.copy(), ZERO, CONFIG)
    assert episode.return_completed


def test_return_travel_does_not_consume_the_readiness_timeout():
    episode = returning_episode()
    away = episode.anchor_pose.copy()
    away[0] += .006
    for now in (1., 5., 10.):
        target, speed = episode.tick(now, away, ZERO, CONFIG, [0.001, 0., 0.])
        assert np.array_equal(target, episode.anchor_pose)
        assert speed == CONFIG["retract_speed"]
        assert episode.return_settle_started_at is None
    episode.tick(11., episode.anchor_pose.copy(), ZERO, CONFIG, np.zeros(6))
    assert not episode.return_completed
    episode.tick(11.051, episode.anchor_pose.copy(), ZERO, CONFIG, np.zeros(6))
    assert episode.return_completed


@pytest.mark.parametrize("blocked_by", ["force", "speed"])
def test_timeout_keeps_episode_unfinished_and_reports_measurements(blocked_by):
    episode = returning_episode()
    force = CONTACT if blocked_by == "force" else ZERO
    speed = np.array([.0008, 0., 0.]) if blocked_by == "speed" else np.zeros(3)
    pose = episode.anchor_pose.copy()
    pose[0] += .00019
    episode.tick(1., pose, force, CONFIG, speed)
    episode.tick(2.99, pose, force, CONFIG, speed)
    with pytest.raises(ReturnReadinessError, match="return readiness timeout") as raised:
        episode.tick(3., pose, force, CONFIG, speed)
    assert not episode.return_completed
    assert episode.phase == "RETURN"
    assert episode.return_end_time is episode.returned_pose is None
    message = str(raised.value)
    for detail in ("probe=4", "position_error=0.190000 mm", "limit=0.200000 mm",
                   "tcp_linear_speed=", "limit=0.500000 mm/s", "Fxy=",
                   "must be <1.000000 N", "elapsed=2.000000/2.000000 s"):
        assert detail in message
    assert episode.force_samples[-1][0] == 3.


def test_drifting_out_of_tolerance_cannot_restart_the_deadline():
    episode = returning_episode()
    anchor = episode.anchor_pose.copy()
    episode.tick(1., anchor, CONTACT, CONFIG, np.zeros(6))
    away = anchor.copy()
    away[0] += .001
    for time, pose in ((1.4, away), (1.7, anchor), (2.4, away), (2.7, anchor)):
        episode.tick(time, pose, CONTACT, CONFIG, np.zeros(6))
        assert episode.return_settle_started_at == 1.
    with pytest.raises(ReturnReadinessError, match="elapsed=2.000000/2.000000 s"):
        episode.tick(3., away, ZERO, CONFIG, np.zeros(6))
    assert np.array_equal(episode.anchor_pose, anchor)
    assert not episode.anchor_pose.flags.writeable
    assert not episode.return_completed


def test_explicit_readiness_parameters_override_defaults():
    config = {**CONFIG, "return_settle_hold_sec": .1,
              "return_settled_speed_mps": .002, "return_settle_timeout_sec": .5}
    episode = returning_episode()
    episode.tick(1., episode.anchor_pose.copy(), ZERO, config, [.001, 0., 0.])
    episode.tick(1.06, episode.anchor_pose.copy(), ZERO, config, [.001, 0., 0.])
    assert not episode.return_completed
    episode.tick(1.101, episode.anchor_pose.copy(), ZERO, config, [.001, 0., 0.])
    assert episode.return_completed
    blocked = returning_episode()
    blocked.tick(1., blocked.anchor_pose.copy(), CONTACT, config, np.zeros(6))
    with pytest.raises(ReturnReadinessError, match="elapsed=0.500000/0.500000 s"):
        blocked.tick(1.5, blocked.anchor_pose.copy(), CONTACT, config, np.zeros(6))


def test_hold_can_finish_exactly_at_the_inclusive_timeout_deadline():
    config = {**CONFIG, "return_settle_hold_sec": .125, "return_settle_timeout_sec": .125}
    episode = returning_episode()
    episode.tick(1., episode.anchor_pose.copy(), ZERO, config, np.zeros(6))
    episode.tick(1.125, episode.anchor_pose.copy(), ZERO, config, np.zeros(6))
    assert episode.return_completed


def test_a_late_sample_cannot_claim_readiness_after_the_deadline():
    episode = returning_episode()
    episode.tick(1., episode.anchor_pose.copy(), ZERO, CONFIG, np.zeros(6))
    with pytest.raises(ReturnReadinessError, match="return readiness timeout"):
        episode.tick(3.01, episode.anchor_pose.copy(), ZERO, CONFIG, np.zeros(6))
    assert not episode.return_completed


@pytest.mark.parametrize("speed", [[float("nan"), 0., 0.], [0., float("inf"), 0.],
                                   [0., 0., float("nan"), 0., 0., 0.], [0., 0.],
                                   ["invalid", 0., 0.]])
def test_invalid_measured_linear_velocity_cannot_complete_return(speed):
    episode = returning_episode()
    with pytest.raises(ReturnReadinessError, match="invalid TCP linear speed"):
        episode.tick(1., episode.anchor_pose.copy(), ZERO, CONFIG, speed)
    assert not episode.return_completed
    assert episode.return_end_time is episode.returned_pose is None


def test_export_keeps_existing_schema_and_logs_wait_as_return():
    episode = returning_episode()
    episode.tick(1., episode.anchor_pose.copy(), CONTACT, CONFIG, np.zeros(6))
    record = episode.to_record()
    assert "return_settle_since" not in record
    assert "return_settle_started_at" not in record
    assert not record["return_completed"]
    assert record["return_end_time"] is record["returned_pose"] is None
    assert record["force_samples"][-1] == (1., 1.1, "RETURN")

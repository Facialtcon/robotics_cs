"""Measured episode 35 geometry with an explicitly synthetic continuation.

The portable fixture ends at the historical STOP_ANCHOR_ERROR. Earlier state
is seeded from recorded episodes and accepted contacts. Stationary transfer
samples and every motion after that stop are synthetic; these tests make no
claim that the physical robot completed the proposed rollback.
"""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from core.models import BoundaryPoint, RobotState, Wrench
from experiment_logging.termination import TerminationReason
from policy.local_tracking import InsufficientClearanceError
from policy.probe_episode import ProbeEpisode
from policy.rule_policy import RuleBasedPolicy, State


FIXTURE = Path(__file__).parent / "fixtures/tracking_clearance_20260914_1150.json"
ZERO = Wrench(0., 0., 0., 0., 0., 0.)


@pytest.fixture
def observed_run():
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def _historical_episode(record):
    episode = ProbeEpisode(
        record["probe_id"], record["anchor_pose"], record["probe_direction"],
        record["probe_start_time"], record["max_probe_distance"],
        record["policy_state"], record["purpose"],
        recovery_id=record["recovery_id"], angle_deg=record["angle_deg"],
    )
    for name in ("contact_pose", "end_pose", "returned_pose", "return_target_pose"):
        value = record[name]
        setattr(episode, name, None if value is None else np.array(value))
    for name in ("contact_time", "probe_end_time", "return_end_time", "outcome",
                 "return_completed", "accepted_as_boundary", "initialization_round",
                 "rejection_reason", "max_force"):
        setattr(episode, name, record[name])
    if record["contact_wrench"] is not None:
        episode.contact_wrench = Wrench.from_sequence(record["contact_wrench"])
    episode.phase = "DONE"
    return episode


def _seed_and_execute_recorded_approach(observed):
    policy = RuleBasedPolicy(observed["policy_config"])
    policy.probe_episodes = [_historical_episode(record) for record in observed["history"]]
    for index, point in enumerate(observed["boundary_points_before_probe35"]):
        policy.boundary_points.append(BoundaryPoint(
            index, point["timestamp"], np.array(point["pose"]),
            Wrench.from_sequence(point["wrench"]), np.array(point["normal"]),
            np.array(point["tangent"]), False,
        ))
    previous = policy.probe_episodes[34]
    estimate = policy.tracker.update(
        [point.pose for point in policy.boundary_points[-3:]],
        previous.probe_direction, reset=True,
    )
    assert estimate is not None
    policy._set_estimate(estimate)
    policy.state = State.BOUNDARY_TRACKING
    start = observed["approach_start"]
    assert np.array_equal(previous.returned_pose, start["pose"])
    policy._resume_tracking(previous, now=start["timestamp"], pose=previous.returned_pose)
    assert [phase for _, phase in policy._motion_queue] == ["RAY_CLEARANCE", "TANGENT_STEP"]
    # Use the real scheduler and callbacks: do not inject an approach binding.
    # The preparatory zero-speed samples below are synthetic holds at the
    # measured Q/A endpoints, not reconstructed physical travel samples.
    hold = policy.config.get("return_settle_hold_sec", policy.config["contact_hold_time"])
    for frame in (observed["clearance_completion"], observed["tracking_entry"]):
        pose = np.array(frame["pose"])
        assert np.linalg.norm(policy._motion_queue[0][0][:2] - pose[:2]) <= policy.config["position_tolerance"]
        policy._execute_transfer(frame["timestamp"] - hold - .001,
                                 Wrench.from_sequence(frame["processed"]), pose,
                                 tcp_speed=np.zeros(6))
        assert policy.active_episode is None
        command = policy._execute_transfer(
            frame["timestamp"], Wrench.from_sequence(frame["processed"]), pose,
            tcp_speed=np.array(frame["speed"]),
        )
        assert not command.move
    episode = policy.active_episode
    assert episode.probe_id == 35 and episode.purpose == "TRACKING"
    assert np.array_equal(episode.anchor_pose, observed["tracking_entry"]["pose"])
    assert policy._tracking_entry[0] == policy._tracking_approach_start[0] == 35
    assert np.array_equal(policy._tracking_entry[1], observed["clearance_completion"]["pose"])
    assert np.array_equal(policy._tracking_approach_start[1], start["pose"])
    assert np.array_equal(policy._tracking_approach_start[2], episode.anchor_pose)
    assert not policy._motion_queue
    return policy, episode


def _replay_recorded_probe(policy, observed):
    for frame in observed["frames"]:
        command = policy.update(
            frame["timestamp"], Wrench.from_sequence(frame["raw"]),
            Wrench.from_sequence(frame["processed"]),
            RobotState(frame["timestamp"], np.array(frame["pose"]), np.array(frame["speed"])),
        )
    return command


def _replay_to_planned_rollback(observed):
    policy, episode = _seed_and_execute_recorded_approach(observed)
    _replay_recorded_probe(policy, observed)
    assert episode.return_completed and episode.accepted_as_boundary
    assert np.array_equal(episode.contact_pose, observed["expected_probe35"]["contact_pose"])
    assert np.array_equal(episode.returned_pose, observed["expected_probe35"]["returned_pose"])
    assert episode.return_end_time == observed["expected_probe35"]["return_end_time"]
    assert policy.state == State.BOUNDARY_TRACKING
    assert policy.termination.record is None
    assert not policy.recovery_records
    return policy, episode


def _synthetic_complete_leg(policy, now, pose):
    """Stationary synthetic endpoint samples; no physical motion is simulated."""
    hold = policy.config.get("return_settle_hold_sec", policy.config["contact_hold_time"])
    pose = np.array(pose)
    assert not policy._execute_transfer(now, ZERO, pose, tcp_speed=np.zeros(6)).move
    completed_at = now + hold + .001
    assert not policy._execute_transfer(completed_at, ZERO, pose, tcp_speed=np.zeros(6)).move
    return completed_at


def test_real_short_clearance_retraces_q_then_approach_start_before_next_anchor(observed_run):
    original_config = deepcopy(observed_run["policy_config"])
    policy, episode = _replay_to_planned_rollback(observed_run)
    assert "available=2.643205 mm" in observed_run["historical_stop_reason"]
    with pytest.raises(InsufficientClearanceError) as ray_error:
        policy.tracker.next_anchor_path(episode)
    assert ray_error.value.available_clearance == pytest.approx(.0027110415526530818)
    q = np.array(observed_run["clearance_completion"]["pose"])
    b = np.array(observed_run["approach_start"]["pose"])
    with pytest.raises(InsufficientClearanceError) as q_error:
        policy.tracker.next_anchor_path(episode, verified_clearance_pose=q)
    assert q_error.value.available_clearance == pytest.approx(.002643205, abs=1e-9)
    assert q_error.value.required_clearance == .003
    assert [phase for _, phase in policy._motion_queue] == [
        "CLEARANCE_ROLLBACK", "CLEARANCE_ROLLBACK", "TANGENT_STEP",
    ]
    rollback_q, rollback_b, next_anchor = [pose for pose, _ in policy._motion_queue]
    assert np.array_equal(rollback_q, q)
    assert np.array_equal(rollback_b, b)
    normal, tangent = policy.current_target_direction, policy.current_tangent
    gap = -(b[:2] - episode.contact_pose[:2]) @ normal
    assert gap == pytest.approx(.004622234499107219)
    assert -(next_anchor[:2] - episode.contact_pose[:2]) @ normal == pytest.approx(gap)
    assert (next_anchor[:2] - episode.contact_pose[:2]) @ tangent == pytest.approx(.003)
    assert not np.allclose(q[:2], b[:2])
    # Q and B must be visited in order; a direct diagonal to B is not allowed.
    command = policy._execute_transfer(episode.return_end_time + .1, ZERO,
                                       episode.returned_pose, tcp_speed=np.zeros(6))
    assert command.move and np.array_equal(command.target_pose, q)
    now = _synthetic_complete_leg(policy, episode.return_end_time + .2, q)
    assert policy.active_episode is None
    assert np.array_equal(policy._motion_queue[0][0], b)
    command = policy._execute_transfer(now + .1, ZERO, q, tcp_speed=np.zeros(6))
    assert command.move and np.array_equal(command.target_pose, b)
    assert policy.config == original_config


@pytest.mark.parametrize("defect", ["missing_b", "wrong_b_id", "wrong_b_anchor", "insufficient_b", "wrong_q_id"])
def test_missing_mismatched_or_insufficient_verified_approach_must_stop(observed_run, defect):
    policy, episode = _seed_and_execute_recorded_approach(observed_run)
    probe_id, b, anchor = policy._tracking_approach_start
    if defect == "missing_b":
        policy._tracking_approach_start = None
    elif defect == "wrong_b_id":
        policy._tracking_approach_start = (probe_id - 1, b, anchor)
    elif defect == "wrong_b_anchor":
        wrong_anchor = anchor.copy()
        wrong_anchor[0] += .001
        policy._tracking_approach_start = (probe_id, b, wrong_anchor)
    elif defect == "insufficient_b":
        policy._tracking_approach_start = (probe_id, policy._tracking_entry[1].copy(), anchor)
    else:
        policy._tracking_entry = (probe_id - 1, policy._tracking_entry[1], anchor)
    command = _replay_recorded_probe(policy, observed_run)
    assert episode.return_completed and episode.accepted_as_boundary
    assert policy.state == State.STOP and not command.move
    assert policy.termination.record["reason"] == TerminationReason.STOP_ANCHOR_ERROR.value
    assert not policy._motion_queue and policy.active_episode is None
    assert not policy.recovery_records


@pytest.mark.parametrize("rollback_leg", [0, 1])
def test_each_reverse_segment_retains_the_contact_guard(observed_run, rollback_leg):
    policy, episode = _replay_to_planned_rollback(observed_run)
    now = episode.return_end_time + .1
    if rollback_leg:
        now = _synthetic_complete_leg(policy, now, policy._motion_queue[0][0]) + .1
    target, phase = policy._motion_queue[0]
    assert phase == "CLEARANCE_ROLLBACK"
    command = policy._execute_transfer(now, Wrench(1.1, 0., 0., 0., 0., 0.),
                                       target.copy(), tcp_speed=np.zeros(6))
    assert not command.move and policy.state == State.STOP
    assert policy.termination.record["reason"] == TerminationReason.STOP_UNEXPECTED_CONTACT.value
    assert policy.active_episode is None and len(policy.probe_episodes) == 36


def test_following_probe_keeps_adjacent_segments_without_compressing_the_polyline(observed_run):
    policy, previous = _replay_to_planned_rollback(observed_run)
    q, b, next_anchor = [pose.copy() for pose, _ in policy._motion_queue]
    now = previous.return_end_time + .1
    for index, pose in enumerate((q, b, next_anchor)):
        now = _synthetic_complete_leg(policy, now, pose) + .1
        if index < 2:
            assert policy.active_episode is None
    following = policy.active_episode
    assert following.probe_id == 36
    assert np.array_equal(policy._tracking_entry[1], b)
    assert np.array_equal(policy._tracking_approach_start[1], q)
    assert not np.array_equal(policy._tracking_approach_start[1], previous.returned_pose)
    assert np.array_equal(policy._tracking_approach_start[2], following.anchor_pose)
    assert policy._pending_tracking_approach_start is None
    assert policy._pending_tracking_clearance is None

    # Synthetic next short contact: if neither adjacent saved waypoint meets
    # the same minimum clearance, do not invent a diagonal to an older point.
    following.contact_pose = following.anchor_pose.copy()
    following.contact_pose[:2] += policy.current_target_direction * .001
    following.returned_pose = following.anchor_pose.copy()
    following.return_completed = True
    following.return_end_time = now
    following.outcome = "CONTACT"
    policy.active_episode = None
    policy._resume_tracking(following, now=now, pose=following.returned_pose)
    assert policy.state == State.STOP and not policy._motion_queue
    assert policy.termination.record["reason"] == TerminationReason.STOP_ANCHOR_ERROR.value


def test_starting_an_unrelated_probe_clears_approach_identity(observed_run):
    policy, episode = _seed_and_execute_recorded_approach(observed_run)
    policy.active_episode = None
    policy._start_probe(episode.probe_start_time + 1., episode.anchor_pose,
                        episode.probe_direction, .012, "RECOVERY")
    assert policy._tracking_entry is None
    assert policy._tracking_approach_start is None

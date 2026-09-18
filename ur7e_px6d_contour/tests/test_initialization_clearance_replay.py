"""Recorded reinitialization geometry with explicit synthetic transfer samples.

The old B/Q/A tracking approach and probe 9 are replayed through the shared
policy to establish entry bindings. Probes 10--12 retain their real wrench,
TCP, HOLD and RETURN records. Arrival holds at recorded initial anchors and
the additional alignment/rollback waypoints are synthetic: no physical
completion of the changed trajectory is claimed.
"""
import json
from pathlib import Path

import numpy as np
import pytest

from core.models import BoundaryPoint, RobotState, Wrench
from experiment_logging.termination import TerminationReason
from policy.local_tracking import InsufficientClearanceError
from policy.probe_episode import ProbeEpisode
from policy.rule_policy import RuleBasedPolicy, State


FIXTURE = Path(__file__).parent / "fixtures/initialization_clearance_20260914_1242.json"
ZERO = Wrench(0., 0., 0., 0., 0., 0.)


@pytest.fixture
def fixture():
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def _completed(record):
    episode = ProbeEpisode(
        record["probe_id"], record["anchor_pose"], record["probe_direction"],
        record["probe_start_time"], record["max_probe_distance"],
        record["policy_state"], record["purpose"],
        recovery_id=record["recovery_id"], angle_deg=record["angle_deg"],
    )
    for name in ("contact_pose", "end_pose", "returned_pose"):
        value = record[name]
        setattr(episode, name, None if value is None else np.array(value))
    for name in ("contact_time", "probe_end_time", "return_end_time", "outcome", "max_force",
                 "return_completed", "accepted_as_boundary", "initialization_round", "rejection_reason"):
        setattr(episode, name, record[name])
    if record["contact_wrench"] is not None:
        episode.contact_wrench = Wrench.from_sequence(record["contact_wrench"])
    episode.phase = "DONE"
    return episode


def _update(policy, frame):
    return policy.update(
        frame["timestamp"], Wrench.from_sequence(frame["raw"]),
        Wrench.from_sequence(frame["processed"]),
        RobotState(frame["timestamp"], np.array(frame["pose"]), np.array(frame["speed"])),
    )


def _replay_source_entry(fixture):
    policy = RuleBasedPolicy(fixture["policy_config"])
    policy.probe_episodes = [_completed(record) for record in fixture["history_before_probe9"]]
    policy.boundary_points = [BoundaryPoint(
        point["point_id"], point["timestamp"], np.array(point["pose"]),
        Wrench.from_sequence(point["wrench"]), np.array(point["normal"]),
        np.array(point["tangent"]), False,
    ) for point in fixture["boundary_before_probe9"]]
    estimate = policy.tracker.update(
        [policy.probe_episodes[index].contact_pose for index in (5, 6, 8)],
        policy.probe_episodes[8].probe_direction, reset=True,
    )
    assert estimate is not None
    policy._set_estimate(estimate)
    policy.state = State.BOUNDARY_TRACKING
    start = fixture["tracking_entry"]["B"]
    policy._force_rate_guard.previous = (start["timestamp"], np.linalg.norm(start["processed"][:2]))
    policy._resume_tracking(policy.probe_episodes[8], now=start["timestamp"], pose=np.array(start["pose"]))
    for frame in fixture["tracking_entry"]["frames"]:
        _update(policy, frame)
    episode = policy.active_episode
    assert episode.probe_id == 9 and episode.purpose == "TRACKING"
    np.testing.assert_array_equal(episode.anchor_pose, fixture["tracking_entry"]["A"]["pose"])
    # No entry tuple is injected: real transfer samples and callbacks produced it.
    assert policy._tracking_entry[0] == policy._tracking_approach_start[0] == 9
    np.testing.assert_array_equal(policy._tracking_entry[1], fixture["tracking_entry"]["Q"]["pose"])
    np.testing.assert_array_equal(policy._tracking_approach_start[1], start["pose"])
    assert not policy._motion_queue
    return policy


def _replay_fit_failure(fixture, defect=None):
    policy = _replay_source_entry(fixture)
    if defect == "missing_entry":
        policy._tracking_entry = None
    elif defect == "wrong_source_anchor":
        probe_id, q, anchor = policy._tracking_entry
        wrong = anchor.copy()
        wrong[0] += .001
        policy._tracking_entry = (probe_id, q, wrong)
    elif defect == "wrong_approach_id":
        probe_id, b, anchor = policy._tracking_approach_start
        policy._tracking_approach_start = (probe_id - 1, b, anchor)
    for frame in fixture["tracking_probe9"]["frames"]:
        _update(policy, frame)
    episode = policy.probe_episodes[9]
    assert episode.outcome == "CONTACT" and episode.return_completed
    assert episode.rejection_reason == "FIT_FAILURE"
    assert policy.state == State.LOCAL_INITIALIZATION and policy.active_episode is None
    assert [phase for _, phase in policy._motion_queue] == ["LATERAL_OFFSET"]
    np.testing.assert_array_equal(policy._initial_base_anchor, fixture["tracking_probe9"]["episode"]["returned_pose"])
    if defect is None:
        np.testing.assert_array_equal(policy._initial_transfer_history, [
            fixture["tracking_entry"]["B"]["pose"], fixture["tracking_entry"]["Q"]["pose"],
            fixture["tracking_entry"]["A"]["pose"], fixture["tracking_probe9"]["episode"]["returned_pose"],
        ])
    return policy


def _synthetic_arrival_at_recorded_initial_anchor(policy, start):
    """Synthetic stationary hold at a real recorded anchor, then its real frame."""
    assert policy._motion_queue[0][1] == "LATERAL_OFFSET"
    target = policy._motion_queue[0][0]
    assert np.linalg.norm(target[:2] - np.array(start["pose"][:2])) <= policy.config["position_tolerance"]
    earlier = dict(start)
    earlier["timestamp"] = start["timestamp"] - policy.config["contact_hold_time"] - .001
    earlier["speed"] = [0.] * 6
    assert not _update(policy, earlier).move
    assert not _update(policy, start).move


def _replay_initialization(fixture, defect=None):
    policy = _replay_fit_failure(fixture, defect)
    for observed in fixture["initialization"]:
        _synthetic_arrival_at_recorded_initial_anchor(policy, observed["start_frame"])
        episode = policy.active_episode
        assert episode.probe_id == observed["episode"]["probe_id"]
        assert episode.purpose == "INITIALIZATION"
        for frame in observed["frames"]:
            _update(policy, frame)
        expected = observed["episode"]
        assert episode.outcome == "CONTACT" and episode.return_completed
        np.testing.assert_array_equal(episode.contact_pose, expected["contact_pose"])
        np.testing.assert_array_equal(episode.returned_pose, expected["returned_pose"])
        assert episode.return_end_time == expected["return_end_time"]
    assert len(policy.boundary_points) == 10
    assert [episode.probe_id for episode in policy._initial_contacts] == [10, 11, 12]
    np.testing.assert_array_equal(
        [point.pose for point in policy.boundary_points[-3:]],
        [policy.probe_episodes[index].contact_pose for index in fixture["expected"]["ordered_probe_ids"]],
    )
    assert policy.tracker.estimate.residual == pytest.approx(fixture["expected"]["initialization_residual_m"])
    assert all(phase == "INITIALIZATION_ALIGN" for _, phase in policy._motion_queue)
    return policy


def _synthetic_complete_leg(policy, now, actual_pose):
    """Explicit synthetic stationary endpoint samples, not simulated UR motion."""
    pose = np.array(actual_pose)
    assert not policy._execute_transfer(now, ZERO, pose, tcp_speed=np.zeros(6)).move
    completed_at = now + policy.config["contact_hold_time"] + .001
    assert not policy._execute_transfer(completed_at, ZERO, pose, tcp_speed=np.zeros(6)).move
    return completed_at


def _complete_new_alignment(policy, fixture):
    now = fixture["initialization"][-1]["episode"]["return_end_time"] + .1
    completed = []
    while policy._motion_queue and policy._motion_queue[0][1] == "INITIALIZATION_ALIGN":
        pose = policy._motion_queue[0][0].copy()
        pose[:2] += [.00002, -.00001]
        completed.append(pose.copy())
        now = _synthetic_complete_leg(policy, now, pose) + .1
    return now, completed


def test_real_fit_failure_keeps_its_scheduler_verified_entry_history(fixture):
    policy = _replay_fit_failure(fixture)
    assert not policy._initial_probe_entries
    assert not policy.recovery_records
    assert policy.termination.record is None


def test_recorded_initialization_keeps_fit_and_retraces_full_path_to_clearance(fixture):
    policy = _replay_initialization(fixture)
    frontier = policy.probe_episodes[10]
    with pytest.raises(InsufficientClearanceError) as error:
        policy.tracker.next_anchor_path(frontier)
    assert error.value.available_clearance == pytest.approx(.0010251769759089617)
    entry_index = policy._initial_entry_index(frontier)
    assert entry_index is not None
    assert policy._initial_transfer_history[entry_index].tolist() == frontier.anchor_pose.tolist()
    # The new alignment retains actual returns, rather than taking a chord
    # between the old initialization anchor samples.
    queued_alignment = [pose.copy() for pose, _ in policy._motion_queue]
    assert any(np.array_equal(pose, policy.probe_episodes[11].returned_pose) for pose in queued_alignment)
    assert any(np.array_equal(pose, policy.probe_episodes[10].returned_pose) for pose in queued_alignment)
    _, reached = _complete_new_alignment(policy, fixture)
    assert np.linalg.norm(reached[-1][:2] - frontier.anchor_pose[:2]) <= policy.config["position_tolerance"]
    assert policy.state == State.BOUNDARY_TRACKING and policy.termination.record is None
    assert [phase for _, phase in policy._motion_queue] == ["CLEARANCE_ROLLBACK"] * 4 + ["TANGENT_STEP"]
    expected_path = [fixture["tracking_probe9"]["episode"]["returned_pose"],
                     fixture["tracking_entry"]["A"]["pose"], fixture["tracking_entry"]["Q"]["pose"],
                     fixture["tracking_entry"]["B"]["pose"]]
    np.testing.assert_array_equal([pose for pose, _ in policy._motion_queue[:-1]], expected_path)
    normal, tangent = policy.current_target_direction, policy.current_tangent
    b = policy._motion_queue[-2][0]
    next_anchor = policy._motion_queue[-1][0]
    assert -np.dot(b[:2] - frontier.contact_pose[:2], normal) == pytest.approx(.004977277546128192)
    assert -np.dot(next_anchor[:2] - frontier.contact_pose[:2], normal) >= .003
    assert np.dot(next_anchor[:2] - frontier.contact_pose[:2], tangent) == pytest.approx(.003)
    np.testing.assert_allclose(next_anchor[:2] * 1000, [591.232729, 107.013149], atol=1e-6, rtol=0)
    assert "frontier probe=10" in policy.reason
    assert not policy.recovery_records and len(policy.boundary_points) == 10


def test_counterfactual_rollback_starts_tracking_and_keeps_only_adjacent_actual_segments(fixture):
    policy = _replay_initialization(fixture)
    now, _ = _complete_new_alignment(policy, fixture)
    actual_endpoints = []
    while policy._motion_queue:
        pose = policy._motion_queue[0][0].copy()
        pose[:2] += [-.000015, .00001]
        actual_endpoints.append(pose.copy())
        now = _synthetic_complete_leg(policy, now, pose) + .1
    episode = policy.active_episode
    assert episode.probe_id == 13 and episode.purpose == "TRACKING"
    assert episode.outcome == "IN_PROGRESS" and episode.contact_pose is None
    np.testing.assert_array_equal(episode.anchor_pose, actual_endpoints[-1])
    np.testing.assert_array_equal(policy._tracking_entry[1], actual_endpoints[-2])
    np.testing.assert_array_equal(policy._tracking_approach_start[1], actual_endpoints[-3])
    assert not policy._initial_transfer_history and not policy._initial_probe_entries
    assert len(policy.boundary_points) == 10
    assert fixture["provenance"]["new_history_rollback_hardware_validated"] is False


@pytest.mark.parametrize("defect", ["missing_entry", "wrong_source_anchor", "wrong_approach_id"])
def test_missing_or_mismatched_source_path_cannot_supply_an_invented_retreat(fixture, defect):
    policy = _replay_initialization(fixture, defect)
    _complete_new_alignment(policy, fixture)
    assert policy.state == State.STOP
    assert policy.termination.record["reason"] == TerminationReason.STOP_ANCHOR_ERROR.value
    assert policy.active_episode is None and not policy._motion_queue
    assert not policy.recovery_records


@pytest.mark.parametrize("leg_index", [0, 3])
def test_new_rollback_retains_contact_guard_at_first_and_last_reverse_leg(fixture, leg_index):
    policy = _replay_initialization(fixture)
    now, _ = _complete_new_alignment(policy, fixture)
    for _ in range(leg_index):
        now = _synthetic_complete_leg(policy, now, policy._motion_queue[0][0]) + .1
    target, phase = policy._motion_queue[0]
    assert phase == "CLEARANCE_ROLLBACK"
    command = policy._execute_transfer(now, Wrench(1.1, 0., 0., 0., 0., 0.),
                                       target.copy(), tcp_speed=np.zeros(6))
    assert not command.move and policy.state == State.STOP
    assert policy.termination.record["reason"] == TerminationReason.STOP_UNEXPECTED_CONTACT.value
    assert policy.active_episode is None and len(policy.probe_episodes) == 13

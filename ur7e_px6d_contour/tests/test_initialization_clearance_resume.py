"""Offline variants of the measured run_20260914_124139_248366 geometry.

The sibling replay module owns the measured subset and source-entry replay.
Mirrored/rotated poses, subsequent arrival holds, forces and retry episodes
below are explicitly synthetic samples, not additional hardware observations.
"""
import json

import numpy as np
import pytest

from core.models import RobotState, Wrench
from experiment_logging.termination import TerminationReason
from policy.probe_episode import ProbeEpisode
from policy.rule_policy import State
from test_initialization_clearance_replay import (
    FIXTURE, _replay_source_entry, _update,
)


ZERO = Wrench(0., 0., 0., 0., 0., 0.)
CONTACT = Wrench(1.1, 0., 0., 0., 0., 0.)


def geometry(hand="COUNTERCLOCKWISE", angle=0.):
    data = json.loads(FIXTURE.read_text(encoding="utf-8"))
    radians = np.deg2rad(angle)
    transform = np.array([[np.cos(radians), -np.sin(radians)],
                          [np.sin(radians), np.cos(radians)]])
    if hand == "CLOCKWISE":
        transform = transform @ np.diag([-1., 1.])
    origin = np.array(data["tracking_entry"]["A"]["pose"][:2])

    def pose(value):
        if value is None:
            return None
        result = np.array(value)
        result[:2] = origin + transform @ (result[:2] - origin)
        return result.tolist()

    def episode(record):
        for key in ("anchor_pose", "return_target_pose", "contact_pose", "end_pose", "returned_pose"):
            if key in record:
                record[key] = pose(record[key])
        record["probe_direction"] = (transform @ record["probe_direction"]).tolist()

    def frame(record):
        record["pose"] = pose(record["pose"])
        record["speed"][:2] = (transform @ record["speed"][:2]).tolist()

    for record in data["history_before_probe9"]:
        episode(record)
    for record in data["boundary_before_probe9"]:
        record["pose"] = pose(record["pose"])
        for key in ("normal", "tangent"):
            record[key] = (transform @ record[key]).tolist()
    for key in ("B", "Q", "A"):
        frame(data["tracking_entry"][key])
    for record in data["tracking_entry"]["frames"]:
        frame(record)
    episode(data["tracking_probe9"]["episode"])
    for record in data["tracking_probe9"]["frames"]:
        frame(record)
    for observed in data["initialization"]:
        episode(observed["episode"])
        frame(observed["start_frame"])
        for record in observed["frames"]:
            frame(record)
    if hand == "CLOCKWISE":
        # The initialization lateral order is always left(direction), so a
        # reflected test must also reverse sample acquisition order.
        data["initialization"].reverse()
    config = data["policy_config"]
    config["follow_hand"] = hand
    config["search_direction_xy"] = (transform @ config["search_direction_xy"]).tolist()
    return data


def source_reinitialization(data, defect=None):
    policy = _replay_source_entry(data)
    entry, approach = policy._tracking_entry, policy._tracking_approach_start
    if defect == "missing_q":
        policy._tracking_entry = None  # The otherwise-valid B alone grants no route.
    elif defect == "wrong_source_id":
        policy._tracking_entry = (entry[0] + 1, entry[1], entry[2])
    elif defect == "wrong_source_anchor":
        wrong = entry[2].copy()
        wrong[0] += .000001  # Immutable identity is stricter than position tolerance.
        policy._tracking_entry = (entry[0], entry[1], wrong)
    elif defect == "wrong_approach_anchor":
        wrong = approach[2].copy()
        wrong[0] += .000001
        policy._tracking_approach_start = (approach[0], approach[1], wrong)
    for record in data["tracking_probe9"]["frames"]:
        _update(policy, record)
    assert policy.probe_episodes[9].rejection_reason == "FIT_FAILURE"
    assert policy.state == State.LOCAL_INITIALIZATION
    assert policy.active_episode is None
    if defect is None:
        np.testing.assert_array_equal(policy._initial_transfer_history, [
            data["tracking_entry"][key]["pose"] for key in ("B", "Q", "A")
        ] + [data["tracking_probe9"]["episode"]["returned_pose"]])
    return policy, data["tracking_probe9"]["frames"][-1]["timestamp"]


def sample(policy, now, pose, force=ZERO):
    return policy.update(now, force, force, RobotState(now, np.array(pose), np.zeros(6)))


def finish_transfer(policy, now, actual_pose=None):
    pending = policy._motion_queue[0]
    pose = pending[0].copy() if actual_pose is None else np.array(actual_pose)
    assert np.linalg.norm(pose[:2] - pending[0][:2]) <= policy.config["position_tolerance"]
    start = now + .1
    hold = policy.config.get("return_settle_hold_sec", policy.config["contact_hold_time"])
    for index in range(int(np.ceil(hold * policy.config["control_rate_hz"])) + 3):
        now = start + index / policy.config["control_rate_hz"]
        assert not sample(policy, now, pose).move
        advanced = not policy._motion_queue or policy._motion_queue[0] is not pending
        if index == 0:
            assert not advanced
        if advanced:
            assert now - start >= hold
            return now, pose
    pytest.fail("transfer did not finish after continuous actual ready samples")


def finish_probe(policy, now, record=None):
    episode = policy.active_episode
    anchor = episode.anchor_pose.copy()
    contact = (anchor.copy() if record is None else np.array(record["contact_pose"]))
    if record is None:
        contact[:2] += episode.probe_direction * episode.max_probe_distance
    now += .1
    assert not sample(policy, now, contact, ZERO if record is None else CONTACT).move
    if record is not None:
        assert episode.phase == "HOLD"
        now += policy.config["contact_hold_time"] + .001
        assert not sample(policy, now, contact, CONTACT).move
    assert episode.phase == "RETURN"
    returned = anchor if record is None else np.array(record["returned_pose"])
    assert np.linalg.norm(returned[:2] - anchor[:2]) <= policy.config["position_tolerance"]
    start = now + .1
    hold = policy.config.get("return_settle_hold_sec", policy.config["contact_hold_time"])
    for index in range(int(np.ceil(hold * policy.config["control_rate_hz"])) + 3):
        now = start + index / policy.config["control_rate_hz"]
        assert not sample(policy, now, returned).move
        if index == 0:
            assert not episode.return_completed
        if episode.return_completed:
            assert now - start >= hold
            np.testing.assert_array_equal(episode.anchor_pose, anchor)
            return now, returned, episode
    pytest.fail("probe did not return after continuous actual ready samples")


def successful_round(policy, now, data):
    for observed in data["initialization"]:
        now, _ = finish_transfer(policy, now, observed["episode"]["anchor_pose"])
        active = policy.active_episode
        index = policy._initial_entry_index(active)
        assert index is not None
        np.testing.assert_array_equal(policy._initial_transfer_history[index], active.anchor_pose)
        now, _, _ = finish_probe(policy, now, observed["episode"])
    assert len(policy._initial_contacts) == 3
    assert all(phase == "INITIALIZATION_ALIGN" for _, phase in policy._motion_queue)
    frontier = max(policy._initial_contacts,
                   key=lambda ep: np.dot(ep.contact_pose[:2], policy.current_tangent))
    np.testing.assert_array_equal(policy.boundary_points[-1].pose, frontier.contact_pose)
    return now, frontier


def complete_alignment(policy, now):
    while policy._motion_queue and policy._motion_queue[0][1] == "INITIALIZATION_ALIGN":
        now, _ = finish_transfer(policy, now)
    return now


@pytest.mark.parametrize("hand", ["CLOCKWISE", "COUNTERCLOCKWISE"])
@pytest.mark.parametrize("angle", [0., 37.])
def test_sorted_frontier_retraces_actual_polyline_then_forward_tangent(hand, angle):
    data = geometry(hand, angle)
    policy, now = source_reinitialization(data)
    now, frontier = successful_round(policy, now, data)
    index = policy._initial_entry_index(frontier)
    history = [pose.copy() for pose in policy._initial_transfer_history]
    np.testing.assert_array_equal([p for p, _ in policy._motion_queue],
                                  list(reversed(history[index:-1])))
    if hand == "COUNTERCLOCKWISE":
        assert frontier is not policy._initial_contacts[-1]
        assert any(np.array_equal(p, policy._initial_contacts[1].returned_pose)
                   for p, _ in policy._motion_queue)
    now = complete_alignment(policy, now)
    expected_reverse = list(reversed(history[:index]))
    assert [phase for _, phase in policy._motion_queue] == (
        ["CLEARANCE_ROLLBACK"] * len(expected_reverse) + ["TANGENT_STEP"])
    np.testing.assert_array_equal([p for p, _ in policy._motion_queue[:-1]], expected_reverse)
    # Every local actual anchor/return before the frontier is too close. Only
    # the original B supplies clearance, reached through base -> source A -> Q.
    normal, tangent = policy.current_target_direction, policy.current_tangent
    gaps = [-np.dot(p[:2] - frontier.contact_pose[:2], normal) for p in expected_reverse]
    assert all(gap < .003 for gap in gaps[:-1]) and gaps[-1] > .0049
    np.testing.assert_array_equal(expected_reverse[-4:], [
        data["tracking_probe9"]["episode"]["returned_pose"],
        data["tracking_entry"]["A"]["pose"], data["tracking_entry"]["Q"]["pose"],
        data["tracking_entry"]["B"]["pose"],
    ])
    next_anchor = policy._motion_queue[-1][0]
    assert np.dot(next_anchor[:2] - frontier.contact_pose[:2], tangent) == pytest.approx(.003)
    assert -np.dot(next_anchor[:2] - frontier.contact_pose[:2], normal) >= .003
    endpoints = []
    while policy._motion_queue:
        assert policy.active_episode is None
        now, pose = finish_transfer(policy, now)
        endpoints.append(pose)
    active = policy.active_episode
    assert active.purpose == "TRACKING"
    np.testing.assert_array_equal(active.anchor_pose, endpoints[-1])
    np.testing.assert_array_equal(policy._tracking_entry[1], endpoints[-2])
    np.testing.assert_array_equal(policy._tracking_approach_start[1], endpoints[-3])
    assert not policy._initial_transfer_history and not policy._initial_probe_entries
    assert not policy.recovery_records and policy.termination.record is None


@pytest.mark.parametrize("defect", ["missing_q", "wrong_source_id", "wrong_source_anchor", "wrong_approach_anchor"])
def test_unbound_source_cannot_contribute_a_shortcut_or_partial_rollback(defect):
    data = geometry()
    policy, now = source_reinitialization(data, defect)
    b = data["tracking_entry"]["B"]["pose"]
    assert not any(np.array_equal(p, b) for p in policy._initial_transfer_history)
    now, frontier = successful_round(policy, now, data)
    index = policy._initial_entry_index(frontier)
    assert all(-np.dot(p[:2] - frontier.contact_pose[:2], policy.current_target_direction) < .003
               for p in policy._initial_transfer_history[:index])
    complete_alignment(policy, now)
    assert policy.state == State.STOP and not policy._motion_queue
    assert policy.active_episode is None
    assert policy.termination.record["reason"] == TerminationReason.STOP_ANCHOR_ERROR.value
    assert not any(w.event_type == "CLEARANCE_ROLLBACK_COMPLETED" for w in policy.policy_waypoints)


@pytest.mark.parametrize("defect", ["missing", "anchor", "index"])
def test_corrupt_frontier_binding_stops_before_any_clearance_transfer(defect):
    data = geometry()
    policy, now = source_reinitialization(data)
    now, frontier = successful_round(policy, now, data)
    anchor, index = policy._initial_probe_entries[frontier.probe_id]
    if defect == "missing":
        del policy._initial_probe_entries[frontier.probe_id]
    elif defect == "anchor":
        wrong = anchor.copy()
        wrong[0] += .000001
        policy._initial_probe_entries[frontier.probe_id] = (wrong, index)
    else:
        policy._initial_probe_entries[frontier.probe_id] = (anchor, index - 1)
    complete_alignment(policy, now)
    assert policy.state == State.STOP and not policy._motion_queue
    assert policy.termination.record["reason"] == TerminationReason.STOP_ANCHOR_ERROR.value
    assert f"frontier probe={frontier.probe_id}" in policy.reason
    assert "no matching initialization entry path" in policy.reason
    assert policy.active_episode is None


@pytest.mark.parametrize("hand", ["CLOCKWISE", "COUNTERCLOCKWISE"])
def test_failed_initial_round_keeps_every_executed_endpoint_for_next_round(hand):
    data = geometry(hand, 37.)
    policy, now = source_reinitialization(data)
    prefix = [p.copy() for p in policy._initial_transfer_history]
    for _ in range(3):
        now, pose = finish_transfer(policy, now)
        prefix.append(pose.copy())
        now, _, episode = finish_probe(policy, now)  # Explicit synthetic NO_CONTACT.
        assert episode.outcome == "NO_CONTACT"
    assert policy._initial_round == 1
    np.testing.assert_array_equal(policy._initial_transfer_history, prefix)
    assert policy._motion_queue[0][1] == "INITIALIZATION_ROLLBACK"
    while policy._motion_queue[0][1] == "INITIALIZATION_ROLLBACK":
        now, pose = finish_transfer(policy, now)
        if not np.array_equal(prefix[-1], pose):
            prefix.append(pose.copy())
    np.testing.assert_array_equal(policy._initial_transfer_history, prefix)
    assert not policy._initial_probe_entries
    now, frontier = successful_round(policy, now, data)
    history = [p.copy() for p in policy._initial_transfer_history]
    np.testing.assert_array_equal(history[:len(prefix)], prefix)
    index = policy._initial_entry_index(frontier)
    complete_alignment(policy, now)
    assert policy.state == State.BOUNDARY_TRACKING
    np.testing.assert_array_equal([p for p, _ in policy._motion_queue[:-1]],
                                  list(reversed(history[:index])))
    assert policy._motion_queue[-1][1] == "TANGENT_STEP"


@pytest.mark.parametrize("transition", ["acquisition", "reinitialization"])
def test_new_initialization_origin_discards_previous_source_bindings(transition):
    data = geometry()
    policy, now = source_reinitialization(data)
    now, _ = finish_transfer(policy, now)
    assert policy._initial_probe_entries
    policy.active_episode = None  # Construct a separate already-returned dispatch context.
    actual = np.array(data["tracking_entry"]["B"]["pose"])
    actual[:2] += [.05, .05]
    episode = ProbeEpisode(90, actual, policy.search_direction, now, .01,
                           State.TARGET_SEARCH.value, "ACQUISITION" if transition == "acquisition" else "TRACKING")
    episode.outcome = "CONTACT"
    episode.contact_pose = actual.copy()
    episode.contact_pose[:2] += .005 * episode.probe_direction
    episode.returned_pose = actual.copy()
    episode.return_completed = True
    if transition == "acquisition":
        policy._acquired(episode, now + .1, actual)
    else:
        policy._reinitialize_tracking(episode, now + .1, actual, "synthetic new local fit failure")
    np.testing.assert_array_equal(policy._initial_transfer_history, [actual])
    assert not policy._initial_probe_entries
    np.testing.assert_array_equal(policy._initial_base_anchor, actual)
    assert policy.state == State.LOCAL_INITIALIZATION


def test_new_history_segment_applies_raw_force_guard_before_motion():
    data = geometry()
    policy, now = source_reinitialization(data)
    now, frontier = successful_round(policy, now, data)
    now = complete_alignment(policy, now)
    assert policy._motion_queue[0][1] == "CLEARANCE_ROLLBACK"
    command = policy.update(now + .1, Wrench(61., 0., 0., 0., 0., 0.), ZERO,
                            RobotState(now + .1, frontier.anchor_pose.copy(), np.zeros(6)))
    assert not command.move and policy.state == State.STOP
    assert policy.termination.record["reason"] == TerminationReason.STOP_FORCE_LIMIT.value
    assert policy.active_episode is None

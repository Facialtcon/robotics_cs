"""Recorded unstable contacts and explicitly counterfactual recovery dispatch.

The short wrench/TCP records are from the old confirmation rays. They establish
that the unchanged hold test still rejects those contacts and that the new
dispatch retries instead of losing the two earlier contacts. New normal/anchor
plans and stationary endpoint samples below are synthetic checks, not recorded
hardware execution of the changed planner.
"""
import json
from pathlib import Path

import numpy as np
import pytest

from core.models import RobotState, Wrench
from policy.boundary_estimation import estimate_boundary
from policy.local_recovery import BoundaryRecovery
from policy.probe_episode import ProbeEpisode
from policy.rule_policy import RuleBasedPolicy, State


FIXTURE = Path(__file__).parent / "fixtures/confirmation_corner_20260914.json"
ZERO = Wrench(0., 0., 0., 0., 0., 0.)


@pytest.fixture
def fixture():
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def _episode(record, completed=False):
    episode = ProbeEpisode(
        record["probe_id"], record["anchor_pose"], record["probe_direction"],
        record["probe_start_time"], record["max_probe_distance"],
        record["policy_state"], record["purpose"],
        recovery_id=record["recovery_id"], angle_deg=record["angle_deg"],
    )
    if completed:
        for name in ("contact_pose", "end_pose", "returned_pose"):
            value = record[name]
            setattr(episode, name, None if value is None else np.array(value))
        for name in ("probe_end_time", "return_end_time", "outcome", "return_completed", "max_force"):
            setattr(episode, name, record[name])
        episode.phase = "DONE"
        # These two records were valid before the third failed. The old run's
        # later group rejection must not be inserted into the earlier seed.
        episode.rejection_reason = ""
    return episode


def _seed(fixture, case):
    policy = RuleBasedPolicy(fixture["policy_config"])
    context = case["recovery"]
    policy.state = State.BOUNDARY_CONFIRMATION
    policy.current_tangent = np.array(context["old_tangent"])
    policy.current_target_direction = np.array(context["old_normal"])
    policy.recovery = BoundaryRecovery(
        np.array(context["anchor_pose"]), policy.current_target_direction,
        policy.current_tangent, np.array(context["last_boundary_contact"]), policy.config,
    )
    for _ in range(context["cursor"]):
        policy.recovery.next_direction()
    # Isolated test history retains only the two relevant contact records.
    # Recovery IDs are bookkeeping; no earlier physical trajectory is invented.
    policy.recovery_records = [{"corner_id": index} for index in range(context["id"] + 1)]
    contacts = [_episode(record, completed=True) for record in case["valid_contacts"]]
    policy.probe_episodes = contacts.copy()
    policy.recovery.candidates.append(contacts[0].contact_pose.copy())
    policy._confirmation_contacts = contacts.copy()
    policy._confirmation_direction = contacts[0].probe_direction.copy()
    policy._confirmation_anchor_path = [
        contacts[0].returned_pose.copy(), contacts[1].anchor_pose.copy(),
        contacts[1].returned_pose.copy(),
    ]
    return policy, contacts


@pytest.mark.parametrize("case_index", [0, 1])
def test_real_unstable_hold_is_not_accepted_and_retries_after_actual_return(fixture, case_index):
    case = fixture["cases"][case_index]
    policy, contacts = _seed(fixture, case)
    episode = _episode(case["unstable_episode"])
    episode.max_force = case["episode_peak_before_frames"]
    policy.active_episode = episode
    policy.probe_episodes.append(episode)
    policy._confirmation_anchor_path.append(episode.anchor_pose.copy())
    policy.sub_state = "PROBE"
    previous = case["force_guard_previous"]
    policy._force_rate_guard.previous = (previous["timestamp"], previous["fxy"])
    cursor = policy.recovery.cursor
    attempted = [direction.copy() for direction in policy.recovery.attempted_directions]
    saved_path = [pose.copy() for pose in policy._confirmation_anchor_path]
    saw_hold = False
    for frame in case["frames"]:
        command = policy.update(
            frame["timestamp"], Wrench.from_sequence(frame["raw"]),
            Wrench.from_sequence(frame["processed"]),
            RobotState(frame["timestamp"], np.array(frame["pose"]), np.array(frame["speed"])),
        )
        if frame["timestamp"] == case["expected"]["first_hold_timestamp"]:
            assert episode.phase == "HOLD" and not command.move
            saw_hold = True
        if frame["timestamp"] == case["expected"]["hold_dropped_timestamp"]:
            assert episode.phase == "RETURN"
            assert episode.outcome == "UNSTABLE_CONTACT"
            assert episode.contact_pose is None
            assert policy.active_episode is episode
    assert saw_hold
    assert episode.return_completed
    assert episode.return_end_time == case["expected"]["return_completed_timestamp"]
    np.testing.assert_array_equal(episode.returned_pose, case["unstable_episode"]["returned_pose"])
    assert episode.outcome == "UNSTABLE_CONTACT" and episode.contact_pose is None
    assert not episode.accepted_as_boundary
    assert episode.rejection_reason == "CONFIRMATION_UNSTABLE_CONTACT"
    assert policy.state == State.BOUNDARY_CONFIRMATION
    assert policy._confirmation_contacts == contacts
    assert all(contact.rejection_reason == "" for contact in contacts)
    assert not policy.boundary_points and policy.termination.record is None
    assert policy._confirmation_retry_count == 1
    assert policy.recovery.cursor == cursor
    np.testing.assert_array_equal(policy.recovery.attempted_directions, attempted)
    np.testing.assert_array_equal(policy._confirmation_anchor_path, saved_path)
    assert not policy._motion_queue
    # This newly scheduled retry is counterfactual. Its future motion and
    # contact outcome are deliberately absent from the real-recording fixture.
    retry = policy.active_episode
    assert retry is not episode and retry.purpose == "CONFIRMATION"
    assert retry.recovery_id == 3 and retry.outcome == "IN_PROGRESS"
    np.testing.assert_array_equal(retry.anchor_pose, episode.anchor_pose)
    np.testing.assert_array_equal(retry.probe_direction, episode.probe_direction)
    assert retry.max_probe_distance == episode.max_probe_distance
    assert retry.max_force == 0. and retry.hold_since is None
    assert not command.move


def _plan_after_second_recorded_contact(fixture, case):
    policy, contacts = _seed(fixture, case)
    policy._confirmation_contacts = [contacts[0]]
    policy._confirmed(contacts[1], contacts[1].return_end_time, contacts[1].returned_pose)
    assert policy.state == State.BOUNDARY_CONFIRMATION
    assert policy._confirmation_contacts == contacts
    assert [phase for _, phase in policy._motion_queue] == [
        "CONFIRMATION_CLEARANCE", "CONFIRMATION_OFFSET",
    ]
    return policy, contacts


@pytest.mark.parametrize("case_index", [0, 1])
def test_two_recorded_contacts_plan_consistent_normal_and_forward_anchor(fixture, case_index):
    case = fixture["cases"][case_index]
    policy, contacts = _plan_after_second_recorded_contact(fixture, case)
    latest = contacts[-1]
    estimate = estimate_boundary([contact.contact_pose for contact in contacts],
                                 contacts[0].probe_direction, policy.follow_hand)
    clearance, anchor = [pose for pose, _ in policy._motion_queue]
    np.testing.assert_allclose(policy._confirmation_direction, estimate.target_side, atol=1e-12)
    assert np.dot(policy._confirmation_direction, estimate.tangent) == pytest.approx(0., abs=1e-12)
    assert np.dot(anchor[:2] - latest.contact_pose[:2], estimate.tangent) == pytest.approx(.003)
    normal_gap = -np.dot(anchor[:2] - latest.contact_pose[:2], estimate.target_side)
    assert normal_gap >= policy.config["retract_distance"]
    assert normal_gap <= policy.config["recovery_ray_length"]
    # The first leg stays inside the latest physically traversed probe ray.
    ray = latest.anchor_pose[:2] - latest.contact_pose[:2]
    retreat = clearance[:2] - latest.contact_pose[:2]
    assert np.dot(ray, retreat) > 0
    assert np.linalg.norm(retreat) <= np.linalg.norm(ray) + 1e-12
    assert abs(ray[0] * retreat[1] - ray[1] * retreat[0]) < 1e-12
    assert not np.allclose(policy._confirmation_direction, contacts[0].probe_direction)
    # This is temporary sampling geometry, with no accepted contour mutation.
    np.testing.assert_array_equal(policy.current_tangent, case["recovery"]["old_tangent"])
    np.testing.assert_array_equal(policy.current_target_direction, case["recovery"]["old_normal"])
    assert not policy.boundary_points and policy.tracker.estimate is None
    assert not policy.recovery_records[-1].get("confirmed_contact")


def test_recorded_151_old_oblique_ray_exceeds_budget_but_new_normal_ray_does_not(fixture):
    case = fixture["cases"][0]
    policy, contacts = _plan_after_second_recorded_contact(fixture, case)
    latest = contacts[-1]
    normal = policy._confirmation_direction
    old_anchor = np.array(case["unstable_episode"]["anchor_pose"])
    old_direction = np.array(case["unstable_episode"]["probe_direction"])
    old_distance = np.dot(latest.contact_pose[:2] - old_anchor[:2], normal) / np.dot(old_direction, normal)
    assert old_distance == pytest.approx(.01227330552817665)
    assert old_distance > policy.config["recovery_ray_length"]
    new_anchor = policy._motion_queue[-1][0]
    new_distance = np.dot(latest.contact_pose[:2] - new_anchor[:2], normal)
    assert new_distance == pytest.approx(.0038)
    assert new_distance < policy.config["recovery_ray_length"]


@pytest.mark.parametrize("case_index", [0, 1])
def test_synthetic_new_transfer_retains_measured_intermediate_for_rollback(fixture, case_index):
    case = fixture["cases"][case_index]
    policy, contacts = _plan_after_second_recorded_contact(fixture, case)
    planned_clearance, planned_anchor = [pose.copy() for pose, _ in policy._motion_queue]
    actual_clearance = planned_clearance.copy()
    actual_clearance[:2] += [0.00002, -0.00001]
    actual_anchor = planned_anchor.copy()
    actual_anchor[:2] += [-0.00001, 0.00002]
    now = contacts[-1].return_end_time + .1
    hold = policy.config["contact_hold_time"]
    # Explicitly synthetic stationary samples at the newly planned endpoints.
    for pose in (actual_clearance, actual_anchor):
        assert not policy._execute_transfer(now, ZERO, pose, tcp_speed=np.zeros(6)).move
        now += hold + .001
        assert not policy._execute_transfer(now, ZERO, pose, tcp_speed=np.zeros(6)).move
        now += .1
    episode = policy.active_episode
    np.testing.assert_array_equal(episode.anchor_pose, actual_anchor)
    np.testing.assert_array_equal(episode.probe_direction, policy._confirmation_direction)
    assert any(np.array_equal(pose, actual_clearance) for pose in policy._confirmation_anchor_path)
    # Synthetic failure after return exercises only the rollback plan. It does
    # not claim the new physical ray produced NO_CONTACT or any other outcome.
    episode.outcome = "NO_CONTACT"
    episode.return_completed = True
    episode.returned_pose = actual_anchor.copy()
    policy.active_episode = None
    saved_path = [pose.copy() for pose in policy._confirmation_anchor_path]
    policy._confirmed(episode, now, actual_anchor)
    np.testing.assert_array_equal([pose for pose, _ in policy._motion_queue], list(reversed(saved_path[:-1])))
    np.testing.assert_array_equal(policy._motion_queue[0][0], actual_clearance)
    assert all(phase == "CONFIRMATION_ROLLBACK" for _, phase in policy._motion_queue)


def test_recorded_ccw_recovery_angles_follow_the_existing_rotation_sense(fixture):
    case = fixture["cases"][0]
    policy, _ = _seed(fixture, case)
    normal = np.array(case["recovery"]["old_normal"])
    assert policy.follow_hand == "COUNTERCLOCKWISE"
    for angle, actual in zip(policy.recovery.angles[:policy.recovery.cursor], policy.recovery.attempted_directions):
        theta = np.deg2rad(angle)
        rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        np.testing.assert_allclose(actual, rotation @ normal, atol=1e-12)
    assert fixture["provenance"]["new_confirmation_trajectory_hardware_validated"] is False

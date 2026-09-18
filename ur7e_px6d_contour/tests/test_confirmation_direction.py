"""Offline confirmation geometry and return-path tests, without robot hardware.

The ep149/150 constants are measured geometry from run_20260914_122407_154132.
Force, speed, timing and any subsequent contacts in these tests are synthetic;
they do not claim that the revised path has already run on the real robot.
"""
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from config.loader import load_config
from core.models import BoundaryPoint, RobotState, Wrench
from policy.boundary_estimation import handed_tangent, unit
from policy.local_recovery import BoundaryRecovery
from policy.rule_policy import RuleBasedPolicy, State


ROOT = Path(__file__).resolve().parents[1]
ZERO = Wrench(0, 0, 0, 0, 0, 0)
CONTACT = Wrench(1.1, 0, 0, 0, 0, 0)
OBSERVED_DIRECTION = np.array([-.8610634562361331, -.5084975165472148])
OBSERVED_149 = {
    "anchor": [.6602211932894451, .10550307146569406, .09337418783053925,
               1.5132905600551854, -2.7528787828422763, .0007017623889392659],
    "contact": [.654722003582906, .10221857158387104, .09335366601173944,
                1.5132957261387403, -2.752860547692763, .0006464386998858223],
    "returned": [.6603777949309486, .10557553646330013, .09339791414708404,
                 1.513110176181413, -2.7529739875857633, .0006909361728475718],
}
OBSERVED_150 = {
    "anchor": [.6587895403974021, .10826928339156704, .09334904237739022,
               1.5131552713610004, -2.752923647896308, .0007038844796009589],
    "contact": [.6484472485579622, .10215790168553694, .0933520445704949,
                1.5133330533625797, -2.7528273048452965, .0006376041559270944],
    "returned": [.6589321864128225, .10838759227218409, .09339450554707601,
                 1.5131951958105478, -2.7529457068411616, .0006591812319223315],
}


def sample(policy, now, pose, force=ZERO):
    return policy.update(now, force, force, RobotState(now, np.array(pose), np.zeros(6)))


def finish_transfer(policy, now, actual_pose=None):
    pending = policy._motion_queue[0]
    pose = pending[0].copy() if actual_pose is None else np.array(actual_pose)
    assert np.linalg.norm(pose[:2] - pending[0][:2]) <= policy.config["position_tolerance"]
    start = now + .1
    hold = policy.config.get("return_settle_hold_sec", policy.config["contact_hold_time"])
    for index in range(int(np.ceil(hold * policy.config["control_rate_hz"])) + 2):
        timestamp = start + index / policy.config["control_rate_hz"]
        assert not sample(policy, timestamp, pose).move
        advanced = not policy._motion_queue or policy._motion_queue[0] is not pending
        if index == 0:
            assert not advanced
        if advanced:
            assert timestamp - start >= hold
            return timestamp, pose
        assert policy.active_episode is None
    pytest.fail("transfer did not complete after continuous ready samples")


def finish_active(policy, now, *, contact=None, returned=None, outcome="CONTACT"):
    episode = policy.active_episode
    anchor = episode.anchor_pose.copy()
    pose = anchor.copy() if contact is None else np.array(contact)
    if contact is None:
        pose[:2] += episode.probe_direction * (.004 if outcome != "NO_CONTACT" else episode.max_probe_distance)
    now += .1
    assert not sample(policy, now, pose, ZERO if outcome == "NO_CONTACT" else CONTACT).move
    if outcome == "CONTACT":
        assert episode.phase == "HOLD"
        now += policy.config["contact_hold_time"] + .001
        assert not sample(policy, now, pose, CONTACT).move
    elif outcome == "UNSTABLE_CONTACT":
        now += .01
        assert not sample(policy, now, pose).move
    assert episode.phase == "RETURN" and not episode.return_completed
    assert episode.outcome == outcome
    pose = anchor if returned is None else np.array(returned)
    assert np.linalg.norm(pose[:2] - anchor[:2]) <= policy.config["position_tolerance"]
    start = now + .1
    hold = policy.config.get("return_settle_hold_sec", policy.config["contact_hold_time"])
    for index in range(int(np.ceil(hold * policy.config["control_rate_hz"])) + 2):
        timestamp = start + index / policy.config["control_rate_hz"]
        assert not sample(policy, timestamp, pose).move
        if index == 0:
            assert not episode.return_completed
        if episode.return_completed:
            assert timestamp - start >= hold
            assert np.array_equal(episode.anchor_pose, anchor)
            return episode, timestamp, pose
    pytest.fail("episode did not complete after continuous ready samples")


def recovery_context(hand, anchor, direction):
    config = load_config(ROOT / "config.yaml")["policy"]
    policy = RuleBasedPolicy({**config, "follow_hand": hand})
    tangent = handed_tangent(direction, hand)
    old_poses = []
    for index in range(3):
        pose = np.array(anchor)
        pose[:2] -= (.03 + (2 - index) * .003) * tangent
        old_poses.append(pose)
        policy.boundary_points.append(BoundaryPoint(index, index, pose, CONTACT,
                                                   direction.copy(), tangent.copy(), False))
    estimate = policy.tracker.update(old_poses, direction, reset=True)
    policy._set_estimate(estimate)
    policy.recovery = BoundaryRecovery(np.array(anchor), direction, tangent, old_poses[-1], policy.config)
    policy.recovery.next_direction()  # Seed a consumed sector; confirmation must not consume another.
    policy.recovery_records.append({"confirmed_contact": None})
    policy.state = State.BOUNDARY_RECOVERY
    policy._start_probe(0., np.array(anchor), direction, policy.config["recovery_ray_length"], "RECOVERY")
    return policy


def face_intersection(episode, point_on_face, normal):
    depth = np.dot(np.array(point_on_face)[:2] - episode.anchor_pose[:2], normal) / np.dot(episode.probe_direction, normal)
    assert 0 < depth <= episode.max_probe_distance + 1e-8
    contact = episode.anchor_pose.copy()
    contact[:2] += depth * episode.probe_direction
    return contact


def two_contacts(hand="COUNTERCLOCKWISE", angle_deg=0., *, observed=False):
    angle = np.deg2rad(angle_deg)
    normal = np.array([np.cos(angle), np.sin(angle)])
    tangent = handed_tangent(normal, hand)
    direction = unit(normal + .8 * tangent)
    anchor = np.array([.4, -.2, .3, 0, 3.14, 0])
    first_contact = anchor.copy()
    first_contact[:2] += .009 * direction
    first_return = anchor.copy()
    first_return[:2] += .00012 * handed_tangent(direction, hand)
    if observed:
        assert hand == "COUNTERCLOCKWISE"
        direction = OBSERVED_DIRECTION.copy()
        anchor = np.array(OBSERVED_149["anchor"])
        first_contact, first_return = (np.array(OBSERVED_149[key]) for key in ("contact", "returned"))
        tangent = unit(np.array(OBSERVED_150["contact"])[:2] - first_contact[:2])
        normal = np.array([-tangent[1], tangent[0]])
        assert np.dot(normal, direction) > 0
    policy = recovery_context(hand, anchor, direction)
    old_tangent, old_normal = policy.current_tangent.copy(), policy.current_target_direction.copy()
    first, now, _ = finish_active(policy, 0., contact=first_contact, returned=first_return)
    assert policy.state == State.BOUNDARY_CONFIRMATION
    assert policy._confirmation_contacts == [first]
    assert [phase for _, phase in policy._motion_queue] == ["CONFIRMATION_OFFSET"]
    expected = first_return.copy()
    expected[:2] += handed_tangent(direction, hand) * policy.config["confirm_step"]
    assert np.array_equal(policy._motion_queue[0][0], expected)
    assert np.array_equal(policy._confirmation_direction, first.probe_direction)
    actual_second_anchor = np.array(OBSERVED_150["anchor"]) if observed else None
    now, _ = finish_transfer(policy, now, actual_second_anchor)
    second = policy.active_episode
    assert np.allclose(second.probe_direction, first.probe_direction, rtol=0, atol=1e-15)
    second_contact = (np.array(OBSERVED_150["contact"]) if observed
                      else face_intersection(second, first_contact, normal))
    second_return = second.anchor_pose.copy()
    second_return[:2] += .0001 * direction
    if observed:
        second_return = np.array(OBSERVED_150["returned"])
    second, now, _ = finish_active(policy, now, contact=second_contact, returned=second_return)
    assert policy.state == State.BOUNDARY_CONFIRMATION
    assert policy._confirmation_contacts == [first, second]
    assert not first.accepted_as_boundary and not second.accepted_as_boundary
    assert len(policy.boundary_points) == 3
    assert np.array_equal(policy.current_tangent, old_tangent)
    assert np.array_equal(policy.current_target_direction, old_normal)
    assert np.array_equal(policy._confirmation_anchor_path[-1], second_return)
    return policy, first, second, normal, tangent, now


def assert_confirmation_geometry(policy, second, normal, tangent):
    assert [phase for _, phase in policy._motion_queue] == ["CONFIRMATION_CLEARANCE", "CONFIRMATION_OFFSET"]
    (clearance, _), (anchor, _) = policy._motion_queue
    assert np.allclose(policy._confirmation_direction, normal, atol=1e-12)
    offset = anchor[:2] - second.contact_pose[:2]
    assert np.dot(offset, tangent) == pytest.approx(policy.config["confirm_step"], abs=1e-12)
    assert -np.dot(offset, normal) >= policy.config["retract_distance"] - 1e-12
    assert np.dot(anchor[:2] - clearance[:2], normal) == pytest.approx(0., abs=1e-12)
    ray = second.contact_pose[:2] - second.anchor_pose[:2]
    from_anchor = clearance[:2] - second.anchor_pose[:2]
    fraction = np.dot(from_anchor, ray) / np.dot(ray, ray)
    assert 0 <= fraction <= 1
    assert np.allclose(from_anchor, fraction * ray, atol=1e-12)
    # A normal-only direction change at the old offset anchor would probe
    # behind the last contact on these oblique approach fixtures.
    old_offset_anchor = second.returned_pose[:2] + tangent * policy.config["confirm_step"]
    assert np.dot(old_offset_anchor - second.contact_pose[:2], tangent) < 0
    return clearance.copy(), anchor.copy()


@pytest.mark.parametrize("hand", ["CLOCKWISE", "COUNTERCLOCKWISE"])
@pytest.mark.parametrize("angle_deg", [0., 37., 123.])
def test_two_contacts_turn_probe_normal_and_preserve_forward_progress(hand, angle_deg):
    policy, _, second, normal, tangent, now = two_contacts(hand, angle_deg)
    _, anchor = assert_confirmation_geometry(policy, second, normal, tangent)
    now, _ = finish_transfer(policy, now)
    assert policy.active_episode is None
    now, _ = finish_transfer(policy, now)
    assert policy.active_episode.purpose == "CONFIRMATION"
    assert np.allclose(policy.active_episode.probe_direction, normal)
    assert np.array_equal(policy.active_episode.anchor_pose, anchor)


def test_measured_ep149_150_geometry_gets_a_forward_normal_probe():
    policy, first, second, normal, tangent, _ = two_contacts(observed=True)
    assert np.array_equal(first.contact_pose, OBSERVED_149["contact"])
    assert np.array_equal(second.contact_pose, OBSERVED_150["contact"])
    assert_confirmation_geometry(policy, second, normal, tangent)
    assert np.dot(policy._confirmation_direction, OBSERVED_DIRECTION) < .6
    assert normal[1] < -.999


def start_third_confirmation(policy, now, tangent):
    actual_clearance = policy._motion_queue[0][0].copy()
    actual_clearance[:2] += .00001 * tangent
    now, _ = finish_transfer(policy, now, actual_clearance)
    assert np.array_equal(policy._confirmation_anchor_path[-1], actual_clearance)
    now, _ = finish_transfer(policy, now)
    return policy.active_episode, actual_clearance, now


@pytest.mark.parametrize("hand", ["CLOCKWISE", "COUNTERCLOCKWISE"])
def test_rejected_confirmation_retraces_actual_clearance_and_prior_anchors(hand):
    policy, first, second, _, tangent, now = two_contacts(hand, 37.)
    fixed_recovery = policy.recovery.anchor.copy()
    cursor = policy.recovery.cursor
    third, actual_clearance, now = start_third_confirmation(policy, now, tangent)
    failed, now, _ = finish_active(policy, now, outcome="NO_CONTACT")
    assert failed is third and failed.rejection_reason == "CONFIRMATION_NO_CONTACT"
    assert policy._confirmation_retry_count == 0
    expected = [actual_clearance, second.returned_pose, second.anchor_pose, first.returned_pose]
    assert [phase for _, phase in policy._motion_queue] == ["CONFIRMATION_ROLLBACK"] * len(expected)
    for (target, _), point in zip(policy._motion_queue, expected, strict=True):
        assert np.array_equal(target, point)
    assert policy.active_episode is None and policy.recovery.cursor == cursor
    for point in expected:
        assert np.array_equal(policy._motion_queue[0][0], point)
        now, _ = finish_transfer(policy, now)
    assert policy.state == State.BOUNDARY_RECOVERY
    assert policy.recovery.cursor == cursor + 1
    assert policy.active_episode.purpose == "RECOVERY"
    assert np.array_equal(policy.active_episode.anchor_pose, fixed_recovery)
    assert np.array_equal(policy.recovery.anchor, fixed_recovery)


def test_unstable_contact_retries_same_ray_twice_then_rejects_without_false_no_contact():
    policy, first, second, _, tangent, now = two_contacts()
    third, _, now = start_third_confirmation(policy, now, tangent)
    anchor, direction = third.anchor_pose.copy(), third.probe_direction.copy()
    path = deepcopy(policy._confirmation_anchor_path)
    cursor = policy.recovery.cursor
    for attempt in range(1, 4):
        failed, now, _ = finish_active(policy, now, outcome="UNSTABLE_CONTACT")
        assert failed.return_completed and failed.contact_pose is None
        assert failed.rejection_reason == "CONFIRMATION_UNSTABLE_CONTACT"
        assert policy._confirmation_retry_count == attempt
        assert policy._confirmation_contacts == [first, second]
        assert all(np.array_equal(a, b) for a, b in zip(policy._confirmation_anchor_path, path, strict=True))
        assert policy.recovery.cursor == cursor
        if attempt <= 2:
            assert not policy._motion_queue
            assert policy.active_episode.purpose == "CONFIRMATION"
            assert np.array_equal(policy.active_episode.anchor_pose, anchor)
            assert np.array_equal(policy.active_episode.probe_direction, direction)
            assert policy.active_episode.probe_start_time >= failed.return_end_time
        else:
            assert policy.active_episode is None
            assert policy._motion_queue[0][1] == "CONFIRMATION_ROLLBACK"
            assert "NO_CONTACT" not in policy.termination.events[-1]["detail"]
    assert policy.termination.record is None and len(policy.boundary_points) == 3


@pytest.mark.parametrize("hand", ["CLOCKWISE", "COUNTERCLOCKWISE"])
def test_third_valid_contact_after_retry_confirms_face_and_resumes_tracking(hand):
    policy, first, second, normal, tangent, now = two_contacts(hand, 37.)
    third, _, now = start_third_confirmation(policy, now, tangent)
    for _ in range(2):
        _, now, _ = finish_active(policy, now, outcome="UNSTABLE_CONTACT")
    third = policy.active_episode
    third_contact = face_intersection(third, second.contact_pose, normal)
    assert np.dot(third_contact[:2] - second.contact_pose[:2], tangent) == pytest.approx(policy.config["confirm_step"])
    completed, now, _ = finish_active(policy, now, contact=third_contact)
    assert completed is third and policy._confirmation_retry_count == 0
    assert policy.state == State.BOUNDARY_TRACKING
    assert all(episode.accepted_as_boundary for episode in (first, second, completed))
    assert len(policy.boundary_points) == 6
    assert np.allclose(policy.current_target_direction, normal)
    assert np.allclose(policy.current_tangent, tangent)
    assert np.array_equal(policy.recovery_records[-1]["confirmed_contact"], third_contact)
    assert [phase for _, phase in policy._motion_queue] == ["RAY_CLEARANCE", "TANGENT_STEP"]
    now, _ = finish_transfer(policy, now)
    assert policy.active_episode is None
    now, _ = finish_transfer(policy, now)
    assert policy.active_episode.purpose == "TRACKING"
    assert np.allclose(policy.active_episode.probe_direction, normal)
    assert policy.termination.record is None

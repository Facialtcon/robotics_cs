"""Acquisition returns locally on its measured ray while preserving search P0."""
import csv
import json
from types import SimpleNamespace

import numpy as np
import pytest

from core.models import Wrench
from experiment_logging.probe_log import write_probe_logs
from experiment_logging.termination import TerminationRecorder, TerminationReason
from policy.probe_episode import ProbeEpisode, ReturnReadinessError
from simulation.physical_validation import probe_audit


CONFIG = {
    "position_tolerance": .0002, "contact_threshold": 1., "contact_hold_time": .05,
    "retract_speed": .006, "probe_speed": .003, "search_speed": .009,
    "initialization_clearance": .009, "retract_distance": .003,
}
ZERO = Wrench(0., 0., 0., 0., 0., 0.)
CONTACT = Wrench(1.1, 0., 0., 0., 0., 0.)


def make_episode(purpose="ACQUISITION"):
    anchor = np.array([.62, .25, .093, 1.51, -2.75, .001])
    episode = ProbeEpisode(0, anchor, [0., -1.], 0., .2, "TARGET_SEARCH", purpose)
    return episode, anchor


def detect_contact(episode, displacement=(.005, -.13), config=CONFIG):
    contact = episode.anchor_pose.copy()
    contact[:2] += displacement
    contact[2:] += [.0001, .0002, -.0001, .0003]
    episode.tick(1., contact, CONTACT, config, np.zeros(6))
    # A transient HOLD must not change its designated return target yet.
    assert np.array_equal(episode.return_target_pose, episode.anchor_pose)
    episode.tick(1.051, contact, CONTACT, config, np.zeros(6))
    assert episode.phase == "RETURN" and episode.outcome == "CONTACT"
    return contact


@pytest.mark.parametrize("displacement", [(.005, -.13), (.003, -.004), (0., 0.)])
def test_acquisition_target_stays_on_finite_observed_ray_and_preserves_search_anchor(displacement):
    episode, original_anchor = make_episode()
    contact = detect_contact(episode, displacement)
    ray = contact[:2] - original_anchor[:2]
    ray_length = np.linalg.norm(ray)
    target = episode.return_target_pose
    assert np.array_equal(episode.anchor_pose, original_anchor)
    assert not episode.anchor_pose.flags.writeable
    assert not target.flags.writeable
    assert not np.shares_memory(target, episode.anchor_pose)
    assert np.array_equal(target[2:], contact[2:])
    assert np.linalg.norm(target[:2] - contact[:2]) == pytest.approx(min(.009, ray_length))
    if ray_length:
        displacement_from_anchor = target[:2] - original_anchor[:2]
        fraction = displacement_from_anchor @ ray / (ray @ ray)
        assert -1e-12 <= fraction <= 1. + 1e-12
        assert np.allclose(displacement_from_anchor, fraction * ray)
    if ray_length < .009:
        assert np.allclose(target[:2], original_anchor[:2])


def test_measured_search_ray_sets_return_direction_instead_of_nominal_direction():
    episode, anchor = make_episode()
    contact = detect_contact(episode, (.03, -.04))
    assert np.allclose(episode.return_target_pose[:2], contact[:2] - np.array([.6, -.8]) * .009)
    assert episode.return_target_pose[0] != pytest.approx(contact[0])
    assert np.array_equal(episode.anchor_pose, anchor)


def test_local_return_commands_and_readiness_use_new_target_not_distant_search_start():
    episode, anchor = make_episode()
    contact = detect_contact(episode)
    target = episode.return_target_pose.copy()
    motion = episode.tick(1.1, contact, ZERO, CONFIG, np.zeros(6))
    assert np.array_equal(motion[0], target)
    assert motion[1] == CONFIG["retract_speed"]
    # Being back at P0 is not completion of this episode's designated return.
    motion = episode.tick(1.2, anchor, ZERO, CONFIG, np.zeros(6))
    assert np.array_equal(motion[0], target)
    assert not episode.return_completed
    assert episode.return_settle_started_at is None
    episode.tick(2., target, CONTACT, CONFIG, np.zeros(6))
    episode.tick(2.1, target, ZERO, CONFIG, [.001, 0., 0.])
    assert not episode.return_completed
    assert episode.return_settle_since is None
    episode.tick(2.2, target, ZERO, CONFIG, np.zeros(6))
    episode.tick(2.249, target, ZERO, CONFIG, np.zeros(6))
    assert not episode.return_completed
    episode.tick(2.251, target, ZERO, CONFIG, np.zeros(6))
    assert episode.return_completed
    assert np.array_equal(episode.returned_pose, target)
    assert np.array_equal(episode.anchor_pose, anchor)


def test_local_return_force_timeout_still_leaves_episode_incomplete():
    episode, _ = make_episode()
    detect_contact(episode)
    target = episode.return_target_pose.copy()
    episode.tick(2., target, CONTACT, CONFIG, np.zeros(6))
    with pytest.raises(ReturnReadinessError, match="return readiness timeout"):
        episode.tick(4., target, CONTACT, CONFIG, np.zeros(6))
    assert not episode.return_completed
    assert episode.return_end_time is None


def test_local_return_stop_reports_actual_target_and_retains_original_search_start():
    episode, start = make_episode()
    detect_contact(episode)
    policy = SimpleNamespace(active_episode=episode, probe_episodes=[episode],
                             state="TARGET_SEARCH", sub_state="RETURN")
    recorder = TerminationRecorder()
    record = recorder.set_stop_reason(TerminationReason.STOP_ANCHOR_ERROR,
                                      detail="local return timeout", policy=policy)
    assert record["anchor_pose"] == episode.return_target_pose.tolist()
    assert record["probe_return_target_pose"] == episode.return_target_pose.tolist()
    assert record["probe_anchor_pose"] == start.tolist()
    assert record["probe_return_completed"] is False


@pytest.mark.parametrize("purpose", ["INITIALIZATION", "TRACKING", "RECOVERY", "CONFIRMATION"])
def test_other_successful_probes_still_return_to_their_original_anchor(purpose):
    episode, anchor = make_episode(purpose)
    contact = detect_contact(episode)
    assert np.array_equal(episode.return_target_pose, anchor)
    motion = episode.tick(1.1, contact, ZERO, CONFIG, np.zeros(6))
    assert np.array_equal(motion[0], anchor)


@pytest.mark.parametrize("outcome", ["NO_CONTACT", "UNSTABLE_CONTACT"])
def test_unsuccessful_acquisition_still_returns_to_original_search_anchor(outcome):
    episode, anchor = make_episode()
    endpoint = anchor.copy()
    endpoint[:2] += episode.probe_direction * episode.max_probe_distance
    if outcome == "UNSTABLE_CONTACT":
        episode.tick(1., endpoint, CONTACT, CONFIG, np.zeros(6))
        episode.tick(1.02, endpoint, ZERO, CONFIG, np.zeros(6))
    else:
        episode.tick(1., endpoint, ZERO, CONFIG, np.zeros(6))
    assert episode.outcome == outcome and episode.phase == "RETURN"
    assert np.array_equal(episode.return_target_pose, anchor)
    assert np.array_equal(episode.tick(1.1, endpoint, ZERO, CONFIG, np.zeros(6))[0], anchor)


def test_local_target_is_recorded_separately_in_json_csv_and_return_audit(tmp_path):
    episode, anchor = make_episode()
    detect_contact(episode)
    target = episode.return_target_pose.copy()
    episode.tick(2., target, ZERO, CONFIG, np.zeros(6))
    episode.tick(2.051, target, ZERO, CONFIG, np.zeros(6))
    write_probe_logs(tmp_path, [episode])
    record = json.loads((tmp_path / "probe_episodes.json").read_text())[0]
    assert record["anchor_pose"] == anchor.tolist()
    assert record["return_target_pose"] == target.tolist()
    csv_record = next(csv.DictReader((tmp_path / "probe_episodes.csv").open()))
    assert json.loads(csv_record["return_target_pose"]) == target.tolist()
    audit = probe_audit([episode], CONFIG["position_tolerance"])
    assert audit["all_probes_returned"]
    assert audit["maximum_return_error_m"] == pytest.approx(0.)
    episode.returned_pose = anchor.copy()
    assert probe_audit([episode], CONFIG["position_tolerance"])["wrong_anchor_probe_ids"] == [0]


@pytest.mark.parametrize("purpose,outcome,explicit_target", [
    ("TRACKING", "CONTACT", True), ("ACQUISITION", "NO_CONTACT", True),
    ("ACQUISITION", "UNSTABLE_CONTACT", True), ("ACQUISITION", "CONTACT", False),
])
def test_audit_local_return_exception_cannot_apply_to_other_or_legacy_episodes(purpose, outcome, explicit_target):
    values = dict(probe_id=7, purpose=purpose, outcome=outcome, anchor_pose=np.zeros(6),
                  return_completed=True, returned_pose=np.array([.05, 0., 0., 0., 0., 0.]))
    if explicit_target:
        values["return_target_pose"] = values["returned_pose"].copy()
    episode = SimpleNamespace(**values)
    assert probe_audit([episode], .0002)["wrong_anchor_probe_ids"] == [7]

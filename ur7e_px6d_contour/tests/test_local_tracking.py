"""Geometric invariants for tracking after normal and oblique contact rays."""
from types import SimpleNamespace

import numpy as np
import pytest

from policy.boundary_estimation import BoundaryEstimate, unit
from policy.local_tracking import InsufficientClearanceError, LocalBoundaryTracker


def setup_path(direction, ray_length=.03, hand="CLOCKWISE"):
    tracker = LocalBoundaryTracker({"retract_distance": .004, "tangent_step": .003}, hand)
    tangent = np.array([0., 1. if hand == "CLOCKWISE" else -1.])
    tracker.estimate = BoundaryEstimate(tangent, np.array([1., 0.]), 0., .01)
    contact = np.array([.02, .01, -.05, 0., 0., 0.])
    direction = unit(direction)
    anchor = contact.copy()
    anchor[:2] -= ray_length * direction
    return tracker, SimpleNamespace(contact_pose=contact, anchor_pose=anchor,
                                    probe_direction=direction)


@pytest.mark.parametrize("hand", ["CLOCKWISE", "COUNTERCLOCKWISE"])
@pytest.mark.parametrize("direction", [[1, 0], [1, 2], [1, -2]])
def test_anchor_progress_survives_oblique_retreat_in_both_follow_directions(hand, direction):
    tracker, episode = setup_path(direction, hand=hand)
    clearance, anchor = tracker.next_anchor_path(episode)
    contact = episode.contact_pose[:2]
    tangent, normal = tracker.estimate.tangent, tracker.estimate.target_side
    # Contact -> clearance is the already traversed probe ray; the second
    # transfer stays at the requested normal distance from the local boundary.
    retreat = clearance[:2] - contact
    assert np.dot(retreat, normal) == pytest.approx(-.004)
    assert np.dot(anchor[:2] - contact, normal) == pytest.approx(-.004)
    assert np.dot(anchor[:2] - contact, tangent) == pytest.approx(.003)
    assert np.dot(retreat, episode.probe_direction) < 0
    assert abs(retreat[0] * episode.probe_direction[1]
               - retreat[1] * episode.probe_direction[0]) < 1e-12
    assert np.array_equal(anchor[2:], episode.contact_pose[2:])


def test_short_shallow_ray_cannot_schedule_an_under_clearance_transfer(capsys):
    tracker, episode = setup_path([1, 8], ray_length=.006)
    with pytest.raises(InsufficientClearanceError) as captured:
        tracker.next_anchor_path(episode)
    assert captured.value.available_clearance == pytest.approx(.006 / np.sqrt(65))
    assert captured.value.required_clearance == pytest.approx(.004)
    assert "clearance_limited=True" in capsys.readouterr().out


@pytest.mark.parametrize("hand", ["CLOCKWISE", "COUNTERCLOCKWISE"])
def test_short_ray_uses_only_explicit_verified_transfer_point(hand, capsys):
    tracker, episode = setup_path([1, 8], ray_length=.006, hand=hand)
    verified_pose = episode.contact_pose.copy()
    verified_pose[:2] += [-.007, -.004]
    clearance, anchor = tracker.next_anchor_path(episode, verified_clearance_pose=verified_pose)
    assert np.array_equal(clearance, verified_pose)
    assert np.dot(anchor[:2] - episode.contact_pose[:2], tracker.estimate.tangent) == pytest.approx(.003)
    assert np.dot(anchor[:2] - episode.contact_pose[:2], tracker.estimate.target_side) == pytest.approx(-.007)
    assert np.array_equal(anchor[2:], verified_pose[2:])
    output = capsys.readouterr().out
    assert "clearance_source=verified_transfer" in output
    assert "retreat_displacement_kind=geometry_offset" in output


@pytest.mark.parametrize("normal_offset", [-.002, 0, .005])
def test_verified_point_is_rejected_when_new_normal_clearance_is_insufficient(normal_offset):
    tracker, episode = setup_path([1, 8], ray_length=.006)
    verified_pose = episode.contact_pose.copy()
    verified_pose[:2] += [normal_offset, -.010]
    with pytest.raises(InsufficientClearanceError) as captured:
        tracker.next_anchor_path(episode, verified_clearance_pose=verified_pose)
    assert captured.value.available_clearance == pytest.approx(-normal_offset)
    assert captured.value.clearance_source == "verified_transfer"


@pytest.mark.parametrize("verified_pose", [[0., 0.], [np.nan, 0., 0., 0., 0., 0.]])
def test_verified_point_requires_a_finite_complete_pose(verified_pose):
    tracker, episode = setup_path([1, 8], ray_length=.006)
    with pytest.raises(ValueError, match="finite contact pose"):
        tracker.next_anchor_path(episode, verified_clearance_pose=verified_pose)


def test_normal_clearance_reserves_position_and_fit_margin_when_ray_allows(capsys):
    tracker, episode = setup_path([1, 2])
    tracker.config.update(position_tolerance=.0002, local_fit_max_residual=.0006)
    clearance, anchor = tracker.next_anchor_path(episode)
    for pose in (clearance, anchor):
        gap = -(pose[:2] - episode.contact_pose[:2]) @ tracker.estimate.target_side
        assert gap == pytest.approx(.0048)
    assert (anchor[:2] - episode.contact_pose[:2]) @ tracker.estimate.tangent == pytest.approx(.003)
    output = capsys.readouterr().out
    assert "planned_normal_clearance_mm=4.8000" in output
    assert "clearance_margin_mm=0.8000" in output


def test_planning_margin_never_extends_past_observed_free_ray():
    tracker, episode = setup_path([1, 0], ray_length=.0044)
    tracker.config.update(position_tolerance=.0002, local_fit_max_residual=.0006)
    clearance, anchor = tracker.next_anchor_path(episode)
    assert np.allclose(clearance, episode.anchor_pose)
    assert -(anchor[:2] - episode.contact_pose[:2]) @ tracker.estimate.target_side == pytest.approx(.0044)


def test_planning_margin_does_not_relax_minimum_clearance():
    tracker, episode = setup_path([1, 0], ray_length=.0039)
    tracker.config.update(position_tolerance=.0002, local_fit_max_residual=.0006)
    with pytest.raises(InsufficientClearanceError) as captured:
        tracker.next_anchor_path(episode)
    assert captured.value.available_clearance == pytest.approx(.0039)
    assert captured.value.required_clearance == pytest.approx(.004)


def test_corrected_anchor_advances_beyond_previous_accepted_contact():
    tracker, episode = setup_path([1, 2])
    previous = episode.contact_pose.copy()
    previous[1] += .005  # latest measured contact was 5 mm behind the accepted one
    _, anchor = tracker.next_anchor_path(episode, tangent_step=.006, reference_contact=previous)
    assert np.dot(anchor[:2] - previous[:2], tracker.estimate.tangent) == pytest.approx(.006)
    assert np.dot(anchor[:2] - episode.contact_pose[:2], tracker.estimate.tangent) == pytest.approx(.011)


def test_tracking_debug_exposes_generated_geometry(capsys):
    tracker, episode = setup_path([1, 2])
    tracker.next_anchor_path(episode)
    output = capsys.readouterr().out
    for field in ("current_contact_point_mm", "old_tangent", "probe_direction",
                  "retreat_displacement_mm", "actual_tangent_progress_mm"):
        assert field in output
    assert "actual_tangent_progress_mm=3.0000" in output


def test_invalid_ray_cannot_generate_a_transfer_at_contact():
    tracker, episode = setup_path([0, 1])
    with pytest.raises(ValueError, match="inward normal"):
        tracker.next_anchor_path(episode)
    episode.anchor_pose = episode.contact_pose.copy()
    with pytest.raises(ValueError, match="verified retreat"):
        tracker.next_anchor_path(episode)

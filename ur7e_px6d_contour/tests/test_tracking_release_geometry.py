"""Real episode 4 geometry from run_20260914_104553_980818, without hardware.

The logged contact is only 0.349482 mm inward from its original anchor in
the newly fitted normal. This must never become an ordinary tangent transfer.
"""
from types import SimpleNamespace

import numpy as np
import pytest

from policy.boundary_estimation import BoundaryEstimate
from policy.local_tracking import InsufficientClearanceError, LocalBoundaryTracker


def real_tracking_case():
    contact = np.array([
        .609045128510668, .11560262375910071, .0934126102426559,
        1.513412619167875, -2.752929910485223, .0007691248011698395,
    ])
    anchor = np.array([
        .6088301659309487, .11595058183417489, .0933782767006518,
        1.5133080379375126, -2.752888750709453, .0006846076753163042,
    ])
    # Measured completion of the previous RAY_CLEARANCE, not its planned target.
    verified_pose = anchor.copy()
    verified_pose[:2] = [.6131727914231192, .12015084543617253]
    tracker = LocalBoundaryTracker({"retract_distance": .003, "tangent_step": .003}, "COUNTERCLOCKWISE")
    tracker.estimate = BoundaryEstimate(
        np.array([-.9999745712790247, -.007131395048032304]),
        np.array([.007131395048032304, -.9999745712790247]),
        .000028335780157446443, .005,
    )
    episode = SimpleNamespace(
        contact_pose=contact, anchor_pose=anchor,
        probe_direction=np.array([.6798717818692449, -.7333310031751951]),
    )
    return tracker, episode, verified_pose


def test_real_short_ray_is_rejected_before_unsafe_next_transfer(capsys):
    tracker, episode, _ = real_tracking_case()
    with pytest.raises(InsufficientClearanceError) as captured:
        tracker.next_anchor_path(episode)
    assert captured.value.available_clearance * 1000 == pytest.approx(.34948221002189883)
    assert captured.value.required_clearance == pytest.approx(.003)
    output = capsys.readouterr().out
    assert "retreat_displacement_mm=[-0.215, 0.348]" in output
    assert "actual_tangent_progress_mm=3.0000" in output
    assert "clearance_limited=True" in output


def test_real_previous_transfer_start_preserves_new_clearance_and_forward_progress():
    tracker, episode, verified_pose = real_tracking_case()
    clearance, next_anchor = tracker.next_anchor_path(episode, verified_clearance_pose=verified_pose)
    assert np.array_equal(clearance, verified_pose)
    offset = next_anchor[:2] - episode.contact_pose[:2]
    assert offset @ tracker.estimate.tangent == pytest.approx(.003)
    expected_gap = -(verified_pose[:2] - episode.contact_pose[:2]) @ tracker.estimate.target_side
    assert -offset @ tracker.estimate.target_side == pytest.approx(expected_gap)
    assert expected_gap > .0045
    # The replacement came from an explicit executed transfer, not an invented
    # extension behind the short original anchor on its observed probe ray.
    ray = episode.contact_pose[:2] - episode.anchor_pose[:2]
    replacement = clearance[:2] - episode.contact_pose[:2]
    cross = ray[0] * replacement[1] - ray[1] * replacement[0]
    assert abs(cross) > 1e-7


def test_real_original_anchor_is_not_an_adequate_verified_fallback():
    tracker, episode, _ = real_tracking_case()
    with pytest.raises(InsufficientClearanceError) as captured:
        tracker.next_anchor_path(episode, verified_clearance_pose=episode.anchor_pose)
    assert captured.value.available_clearance * 1000 == pytest.approx(.34948221002189883)
    assert captured.value.clearance_source == "verified_transfer"

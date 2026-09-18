from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest
from core.models import Wrench
from policy.probe_episode import ProbeEpisode
from simulation.geometry import RectangleTarget, CircleTarget
from simulation.physical_validation import segment_enters_interior, probe_audit
from simulation.simulator import ContourSimulator, load_simulation_config


def config():
    return load_simulation_config(Path(__file__).resolve().parents[1] / "simulation/scene_square.yaml")


@pytest.mark.parametrize("target", [RectangleTarget(np.zeros(2), .1, .1), RectangleTarget(np.zeros(2), .1, .1, 30), CircleTarget(np.zeros(2), .05)])
def test_continuous_audit_detects_entire_object_crossing_with_endpoints_outside(target):
    assert segment_enters_interior([-.1, 0], [.1, 0], target)
    assert segment_enters_interior([0, 0], [0, 0], target)
    assert not segment_enters_interior([-.1, .1], [.1, .1], target)


def test_boundary_touch_is_not_interior():
    target = RectangleTarget(np.zeros(2), .1, .1)
    assert not segment_enters_interior([-.1, .05], [.1, .05], target)
    assert segment_enters_interior([-.1, .049], [.1, .049], target)


def test_wrong_anchor_cannot_pass_even_if_policy_claims_return_completed():
    episode = ProbeEpisode(42, np.zeros(6), [1, 0], 0, .01, "BOUNDARY_TRACKING", "TRACKING")
    episode.return_completed = True
    episode.returned_pose = np.array([.002, 0, 0, 0, 0, 0])
    audit = probe_audit([episode], .0001)
    assert not audit["all_probes_returned"] and audit["wrong_anchor_probe_ids"] == [42]


def test_no_contact_envelope_is_reported_as_physical_failure_not_hidden(tmp_path):
    c = config(); c["force_model"]["probe_tip_radius"] = 0
    c["simulation"]["output_root"] = str(tmp_path)
    sim = ContourSimulator(c); sim.run_headless()
    assert not sim.loop_completed and sim.interior_entered
    assert "PHYSICAL FAILURE" in sim.stop_reason
    assert any(sim.target.signed_distance_and_outward_normal(p)[0] < 0 for p in sim.trajectory)
    output = sim.finalize()
    summary = json.loads((output / "summary.json").read_text())
    assert summary["trajectory_entered_interior"] and not summary["success"]
    assert not summary["probe_audit"]["all_probes_returned"]


def test_injected_continued_probe_after_contact_is_physical_failure():
    sim = ContourSimulator(config())
    for _ in range(500):
        sim.step()
        episode = sim.policy.active_episode
        if episode is not None and episode.phase == "HOLD":
            break
    assert episode.contact_pose is not None
    template = sim.last_command
    sim.policy.update = lambda *args: replace(template, move=True, speed=.0001, direction_xy=episode.probe_direction)
    sim.step()
    assert sim.stopped and sim.post_contact_push and not sim.loop_completed
    assert "PHYSICAL FAILURE" in sim.stop_reason


def test_geometry_estimation_is_invariant_to_sensor_force_direction():
    # Exactly the same Fxy magnitude and pose history, deliberately wrong force
    # direction on every sample. No parameter allows this force angle into PCA.
    c = config(); first = ContourSimulator(c); first.run_headless()
    second = ContourSimulator(c)
    original = second.sensor.read_wrench
    def rotated_force(position, velocity):
        force, metadata = original(position, velocity)
        return Wrench(-force.fy, force.fx, force.fz, force.tx, force.ty, force.tz), metadata
    second.sensor.read_wrench = rotated_force
    second.run_headless()
    assert first.loop_completed and second.loop_completed
    assert np.array_equal(first.trajectory, second.trajectory)
    assert np.array_equal([p.tangent_xy for p in first.policy.boundary_points], [p.tangent_xy for p in second.policy.boundary_points])


def test_counterclockwise_uses_same_policy_without_changing_object():
    c = config(); c["policy"]["follow_hand"] = "COUNTERCLOCKWISE"
    sim = ContourSimulator(c); sim.run_headless()
    assert sim.loop_completed and not sim.interior_entered
    assert sim.loop_report["winding_turns"] >= .95

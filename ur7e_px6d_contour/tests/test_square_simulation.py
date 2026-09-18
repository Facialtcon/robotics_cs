"""Full-loop success must be measured, and failure must preserve executed data."""

from dataclasses import replace
from pathlib import Path
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from policy.rule_policy import RuleBasedPolicy, State
from simulation.loop_completion import square_loop_report
from simulation.simulator import ContourSimulator, load_simulation_config
from simulation.top_view import TopViewPlot
from simulation.physical_validation import probe_audit, segment_enters_interior
from policy.boundary_estimation import handed_tangent

ROOT = Path(__file__).resolve().parents[1]


def square_config():
    return load_simulation_config(ROOT / "simulation" / "scene_square.yaml")


def test_ideal_square_completes_four_recoveries_with_shared_policy(tmp_path):
    config = square_config()
    config["simulation"]["output_root"] = str(tmp_path)
    sim = ContourSimulator(config)
    assert type(sim.policy) is RuleBasedPolicy
    assert not sim.policy.config["loop_closure_enabled"]
    for position in ([-0.08, 0], [0.08, 0]):
        force, _ = sim.sensor.read_wrench(position, [1, 0])
        assert np.array_equal(force.array(), np.zeros(6))
    first_force, _ = sim.sensor.read_wrench([-0.049, 0], [0, 0])
    second_force, _ = sim.sensor.read_wrench([-0.049, 0], [0, 0])
    assert np.array_equal(first_force.array(), second_force.array())
    sim.run_headless()
    assert sim.loop_completed
    assert sim.loop_report["faces_visited"] == 4
    assert sim.loop_report["winding_turns"] >= .95
    assert sim.loop_report["boundary_path_m"] >= .36
    assert sim.loop_report["distance_to_first_m"] <= .012
    assert sum(r["confirmed_contact"] is not None for r in sim.policy.recovery_records) == 4
    assert all(point.tangent_xy @ handed_tangent(point.target_direction_xy, sim.policy.follow_hand) > .999 for point in sim.policy.boundary_points)
    assert not sim.interior_entered and not sim.post_contact_push
    assert all(not segment_enters_interior(a, b, sim.target) for a, b in zip(sim.trajectory, sim.trajectory[1:]))
    assert probe_audit(sim.policy.probe_episodes, config["policy"]["position_tolerance"])["all_probes_returned"]
    assert max(sim.target.distance_to_boundary(p.pose[:2]) for p in sim.policy.boundary_points) < .001
    episodes = sim.policy.probe_episodes
    assert all(previous.return_end_time <= following.probe_start_time for previous, following in zip(episodes, episodes[1:]))
    assert all(not e.accepted_as_boundary for e in episodes if e.outcome != "CONTACT")
    assert sum(e.rejection_reason == "CONFIRMATION_NO_CONTACT" for e in episodes) >= 1
    for record in sim.policy.recovery_records:
        rays = [e for e in episodes if e.purpose == "RECOVERY" and e.recovery_id == record["corner_id"]]
        assert all(np.linalg.norm(e.anchor_pose[:2] - record["p_anchor"][:2]) <= config["policy"]["position_tolerance"] for e in rays)
        accepted = [e for e in episodes if e.recovery_id == record["corner_id"] and e.accepted_as_boundary]
        assert len(accepted) >= 3

    # Neither an immediate return nor repeated local contacts is a full loop.
    points = sim.policy.boundary_points
    assert not square_loop_report(points[:2], sim.target, sim.policy.follow_hand, config["simulation"])["completed"]
    assert not square_loop_report([points[0], points[1]] * 100, sim.target, sim.policy.follow_hand, config["simulation"])["completed"]
    # The scorer considers the last three contacts, because a confirmation
    # batch can cross the start region before its final contact. Invalidate all
    # closure candidates rather than relying on a particular sampling phase.
    reversed_tangent = points[:-3] + [replace(point, tangent_xy=-points[0].tangent_xy)
                                     for point in points[-3:]]
    assert not square_loop_report(reversed_tangent, sim.target, sim.policy.follow_hand, config["simulation"])["completed"]

    run = sim.finalize()
    assert (tmp_path / "full_loop_square.png").is_file()
    # A user's independent workspace calibration may add its optional overlay.
    # The simulator itself still exports exactly its original result figure.
    assert {p.name for p in run.glob("*.png")} - {"workspace_contour.png"} == {"simulation_result.png"}
    assert json.loads((run / "summary.json").read_text())["success"] is True
    records = json.loads((run / "probe_episodes.json").read_text())
    assert len(records) == len(episodes)
    assert all(r["return_completed"] for r in records)
    assert (run / "probe_forces.csv").is_file()
    plot = TopViewPlot(sim)
    plot.draw(sim.snapshot(), sim.trajectory, finished=True)
    assert len(plot.figure.axes) == 1
    assert "SUCCESS" in plot.axis.get_title()
    assert np.allclose(plot.trajectory.get_xdata(), np.asarray(sim.trajectory)[:, 0] * 1000)
    assert len(plot.rays.get_segments()) == 0
    plt.close(plot.figure)


def test_square_failure_preserves_path_and_last_state(tmp_path):
    config = square_config()
    config["simulation"]["output_root"] = str(tmp_path)
    sim = ContourSimulator(config)
    sim.run_headless(10)
    assert not sim.loop_completed
    assert sim.final_policy_state == "TARGET_SEARCH"
    assert "step limit" in sim.stop_reason
    assert len(sim.trajectory) > 1
    run = sim.finalize()
    summary = json.loads((run / "summary.json").read_text())
    assert summary["success"] is False
    assert (run / "simulation_result.png").is_file()
    assert (run / "simulation_log.csv").is_file()


def test_square_time_limit_and_lost_stop_before_another_motion():
    config = square_config()
    config["simulation"]["max_time_sec"] = 0.04
    sim = ContourSimulator(config)
    sim.run_headless()
    assert not sim.loop_completed
    assert sim.stop_reason == "simulation time limit exceeded"

    sim = ContourSimulator(square_config())
    sim.policy.request_stop("injected stop")
    before = sim.robot.pose.copy()
    sim.step()
    assert sim.stopped and not sim.loop_completed
    assert sim.final_policy_state == "STOP"
    assert np.array_equal(before, sim.robot.pose)

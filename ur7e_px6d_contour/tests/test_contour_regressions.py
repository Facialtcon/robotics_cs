"""Full-loop regressions using the existing point-robot/force simulator."""

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest

from policy.boundary_estimation import handed_tangent
from simulation.physical_validation import probe_audit, segment_enters_interior
from simulation.simulator import ContourSimulator, load_simulation_config
from simulation.top_view import TopViewPlot


ROOT = Path(__file__).resolve().parents[1]
CONTOUR_CASES = (
    "square_clockwise",
    "square_counterclockwise",
    "triangle_shifted_down",
    "triangle_shifted_up",
    "rotated_rectangle",
)


def contour_simulator(case):
    """Also usable by an artifact runner; no output is written until finalize."""
    scene = "scene_2_rotated_rectangle.yaml" if case == "rotated_rectangle" else "scene_square.yaml"
    config = load_simulation_config(ROOT / "simulation" / scene)
    if case.startswith("square_"):
        config["policy"].update(
            follow_hand="COUNTERCLOCKWISE" if case.endswith("counterclockwise") else "CLOCKWISE",
            tangent_step=0.003,
            initialization_lateral_offset=0.0015,
            retract_distance=0.003,
        )
        # Three-millimetre sampling requires more returned probes per circuit.
        # Extend only the execution budget, preserving the existing physics.
        config["simulation"].update(max_time_sec=240.0, max_steps_headless=12000)
    simulator = ContourSimulator(config)
    if case.startswith("triangle_"):
        center_y = -0.010 if case.endswith("down") else 0.005
        # Keep P0/P1 fixed: this is a translated object, not a translated run.
        simulator.set_interactive_target("triangle", [0.0, center_y])
    return simulator


@pytest.mark.parametrize("case", CONTOUR_CASES)
def test_simple_contours_complete_a_physical_loop(case, tmp_path):
    simulator = contour_simulator(case)
    simulator.config["simulation"]["output_root"] = str(tmp_path)
    simulator.run_headless()
    diagnostic = f"{case}: {simulator.stop_reason}; {simulator.loop_report}"
    assert simulator.loop_completed, diagnostic
    report = simulator.loop_report
    assert report["completed"] and report["all_contacts_near_boundary"], diagnostic
    assert report["winding_turns"] >= simulator.config["simulation"].get("min_winding_turns", 0.90)
    assert report["distance_to_first_m"] <= simulator.config["simulation"].get("closure_distance", 0.012)
    if case.startswith("square_"):
        assert report["faces_visited"] == 4
        assert report["boundary_path_m"] >= 0.36

    assert not simulator.interior_entered and not simulator.post_contact_push, diagnostic
    assert all(not segment_enters_interior(a, b, simulator.target)
               for a, b in zip(simulator.trajectory, simulator.trajectory[1:]))
    episodes = simulator.policy.probe_episodes
    policy_config = simulator.policy.config
    assert probe_audit(episodes, policy_config["position_tolerance"])["all_probes_returned"]
    assert all(previous.return_end_time <= following.probe_start_time
               for previous, following in zip(episodes, episodes[1:]))
    points = simulator.policy.boundary_points
    assert all(point.tangent_xy @ handed_tangent(point.target_direction_xy, simulator.policy.follow_hand) > 0.999
               for point in points)

    # Accepted tracking contacts must advance from the preceding published point,
    # including the first probe after initialization and corner confirmation.
    tracking_times = {episode.contact_time for episode in episodes
                      if episode.purpose == "TRACKING" and episode.accepted_as_boundary}
    assert tracking_times
    if case.startswith("square_"):
        first_tracking = next(episode for episode in episodes if episode.purpose == "TRACKING")
        assert first_tracking.accepted_as_boundary, first_tracking.rejection_reason
    for previous, point in zip(points, points[1:]):
        if point.timestamp in tracking_times:
            progress = float((point.pose[:2] - previous.pose[:2]) @ previous.tangent_xy)
            assert progress >= policy_config["minimum_boundary_point_spacing"] - 1e-10, (case, progress)

    # Final figures must expose measured data, including recovery contacts after
    # the policy has left recovery. Test the plotted arrays as well as the file.
    plot = TopViewPlot(simulator)
    try:
        plot.draw(simulator.snapshot(), simulator.trajectory, finished=True)
        boundary = simulator.target.boundary_points() * 1000
        assert np.allclose(np.column_stack(plot.object_outline.get_data()), boundary)
        assert np.allclose(np.column_stack(plot.trajectory.get_data()), np.asarray(simulator.trajectory) * 1000)
        assert np.allclose(plot.points.get_offsets(), np.asarray([point.pose[:2] for point in points]) * 1000)
        recovery_contacts = [ray.contact_pose[:2] for ray in simulator.policy.boundary_recovery_rays
                             if ray.contact_pose is not None]
        assert recovery_contacts
        assert np.allclose(plot.candidates.get_offsets(), np.asarray(recovery_contacts) * 1000)
        output = tmp_path / f"{case}.png"
        plot.figure.savefig(output, dpi=100)
        assert output.stat().st_size > 0
    finally:
        plt.close(plot.figure)

import pathlib

import numpy as np

from simulation.geometry import CircleTarget, RectangleTarget
from simulation.simulated_force_sensor import SimulatedForceSensor
from simulation.simulator import ContourSimulator, load_simulation_config
from policy.boundary_estimation import handed_tangent


ROOT = pathlib.Path(__file__).resolve().parents[1]


def test_simulation_exposes_the_same_initial_scan_direction_structure():
    config = load_simulation_config(
        ROOT / "simulation" / "scene_1_axis_aligned_rectangle.yaml"
    )
    assert np.allclose(config["scan_direction_xy"], [1.0, 0.0])
    assert config["scan_direction_xy"] == config["policy"]["search_direction_xy"]


def test_rectangle_and_circle_signed_distance():
    rectangle = RectangleTarget(np.zeros(2), 0.10, 0.06, 30.0)
    assert rectangle.signed_distance_and_outward_normal([0, 0])[0] < 0
    assert rectangle.signed_distance_and_outward_normal([1, 1])[0] > 0
    circle = CircleTarget(np.zeros(2), 0.05)
    assert np.isclose(circle.distance_to_boundary([0.05, 0]), 0.0)
    assert np.isclose(circle.penetration_and_outward_normal([0, 0])[0], 0.05)


def test_synthetic_contact_direction_matches_shared_policy_convention():
    target = RectangleTarget(np.zeros(2), 0.10, 0.06, 0.0)
    sensor = SimulatedForceSensor(
        target,
        {
            "random_seed": 1,
            "granular_drag_force": 0.0,
            "noise_std": 0.0,
            "contact_stiffness": 1000.0,
            "friction_coefficient": 0.0,
        },
    )
    wrench, metadata = sensor.read_wrench([-0.049, 0], [1, 0])
    assert metadata["penetration"] > 0
    # Sensor direction convention is diagnostic only; policy ignores this angle.
    assert wrench.fx > 0 and np.isclose(wrench.fy, 0)


def test_axis_rectangle_uses_shared_policy_and_recovers_first_corner():
    config = load_simulation_config(
        ROOT / "simulation" / "scene_1_axis_aligned_rectangle.yaml"
    )
    simulator = ContourSimulator(config)
    simulator.run_headless(6000)
    states = simulator.states_seen
    corner_index = states.index("BOUNDARY_RECOVERY")
    assert "BOUNDARY_TRACKING" in states[corner_index + 1 :]
    assert "BOUNDARY_CONFIRMATION" in states[corner_index + 1 :]
    assert len(simulator.policy.boundary_points) >= 8
    first_corner_rays = [
        ray for ray in simulator.policy.boundary_recovery_rays if ray.corner_id == 0
    ]
    assert np.all(np.diff([ray.theta_deg for ray in first_corner_rays]) > 0)
    assert any(ray.result == "CONTACT" for ray in first_corner_rays)
    assert all(np.linalg.norm(ray.anchor_pose - first_corner_rays[0].anchor_pose) <= config["policy"]["position_tolerance"] for ray in first_corner_rays)
    assert all(ray.actual_length <= ray.planned_length + 1e-9 for ray in first_corner_rays)


def test_simulation_finalization_writes_strategy_and_corner_diagnostics(tmp_path):
    config = load_simulation_config(
        ROOT / "simulation" / "scene_1_axis_aligned_rectangle.yaml"
    )
    config["simulation"]["output_root"] = str(tmp_path)
    config["visualization"]["debug_plots"] = True
    simulator = ContourSimulator(config)
    # Wait for the event being visualized, not a step count coupled to the
    # number of initialization probes. Keep a bound for failed policies.
    for _ in range(3000):
        simulator.step()
        if simulator.stopped or simulator.policy.boundary_recovery_rays:
            break
    assert simulator.policy.boundary_recovery_rays
    simulator.normal_stop("diagnostic test reached first recovery ray")
    output = simulator.finalize()

    expected = (
        "policy_waypoints.csv",
        "boundary_recovery_rays.csv",
        "scan_strategy_debug.png",
        "boundary_recovery_0.png",
    )
    assert all((output / name).is_file() for name in expected)
    header = (output / "boundary_recovery_rays.csv").read_text(encoding="utf-8").splitlines()[0]
    assert "ray_end_x" in header and "ray_end_y" in header


def test_rectangles_complete_a_loop_with_four_confirmed_recoveries():
    for scene in (
        "scene_1_axis_aligned_rectangle.yaml",
        "scene_2_rotated_rectangle.yaml",
    ):
        simulator = ContourSimulator(load_simulation_config(ROOT / "simulation" / scene))
        simulator.run_headless(6000)
        assert simulator.policy.state == simulator.policy.state.LOOP_COMPLETE
        accepted = [episode for episode in simulator.policy.probe_episodes if episode.recovery_id is not None and episode.accepted_as_boundary]
        assert len({ray.recovery_id for ray in accepted}) >= 4
        assert len(accepted) >= 4
        expected = [
            handed_tangent(point.target_direction_xy, simulator.policy.follow_hand)
            for point in simulator.policy.boundary_points
        ]
        assert all(np.dot(point.tangent_xy, tangent) > 0.999 for point, tangent in zip(simulator.policy.boundary_points, expected))


def test_circle_completes_a_loop_without_boundary_recovery():
    simulator = ContourSimulator(
        load_simulation_config(ROOT / "simulation" / "scene_3_circle.yaml")
    )
    simulator.run_headless(6000)
    assert simulator.policy.state == simulator.policy.state.LOOP_COMPLETE
    assert not simulator.policy.boundary_recovery_rays
    assert len(simulator.policy.boundary_points) >= 40


def test_first_contact_at_square_vertex_recovers_on_one_face_without_penetration():
    config = load_simulation_config(ROOT / "simulation" / "scene_square.yaml")
    config.update(target_rotation_deg=45.0, start_point=[-.1, 0.0],
                  calibration_point_0=[-.1, 0.0], calibration_point_1=[-.07, 0.0])
    config["simulation"]["square_full_loop"] = False
    sim = ContourSimulator(config)
    for _ in range(3000):
        sim.step()
        if sim.stopped or sim.policy.state.value == "BOUNDARY_TRACKING":
            break
    assert sim.policy.state.value == "BOUNDARY_TRACKING"
    assert not sim.stopped and not sim.interior_entered and not sim.post_contact_push
    episodes = sim.policy.probe_episodes
    failed = [e for e in episodes if e.initialization_round in (1, 2)]
    accepted = [e for e in episodes if e.accepted_as_boundary]
    assert len(failed) == 6 and all(e.rejection_reason for e in failed)
    assert not any(e.accepted_as_boundary for e in failed)
    assert len(accepted) == 3 and all(e.initialization_round == 3 for e in accepted)
    assert all(e.return_completed for e in episodes)
    assert sim.policy.tracker.estimate.residual <= config["policy"]["local_fit_max_residual"]

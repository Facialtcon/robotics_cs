from pathlib import Path
from types import SimpleNamespace

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from simulation.geometry import TriangleTarget
from simulation.physical_validation import probe_audit, segment_enters_interior
from simulation.simulator import ContourSimulator, load_simulation_config
from simulation.top_view import SimulationView, TopViewPlot


ROOT = Path(__file__).resolve().parents[1]


def config():
    return load_simulation_config(ROOT / "simulation" / "scene_square.yaml")


def test_triangle_geometry_and_continuous_interior_audit():
    target = TriangleTarget(np.zeros(2), .055, 90)
    assert target.signed_distance_and_outward_normal([0, 0])[0] < 0
    assert target.signed_distance_and_outward_normal([.1, .1])[0] > 0
    assert segment_enters_interior([-.1, 0], [.1, 0], target)
    assert not segment_enters_interior([-.1, .08], [.1, .08], target)
    assert target.boundary_points().shape == (4, 2)


def test_shape_switch_and_layout_translation_restart_the_same_policy():
    sim = ContourSimulator(config())
    original_start = np.asarray(sim.config["start_point"])
    sim.set_interactive_target("circle", [.01, 0])
    assert sim.target_shape == "circle"
    assert np.array_equal(sim.config["start_point"], original_start)
    assert sim.policy.config["loop_closure_enabled"]
    assert "target_shape" not in sim.policy.config and "target_center" not in sim.policy.config
    sim.set_interactive_target("triangle", [.02, .01], translate_search=True)
    assert sim.target_shape == "triangle"
    assert np.allclose(sim.config["start_point"], original_start + [.01, .01])
    assert sim.policy.state.value == "TARGET_SEARCH" and len(sim.trajectory) == 1


def test_triangle_completes_with_shared_policy_and_no_physical_failure():
    sim = ContourSimulator(config())
    sim.set_interactive_target("triangle")
    sim.run_headless(6000)
    audit = probe_audit(sim.policy.probe_episodes, sim.config["policy"]["position_tolerance"])
    assert sim.loop_completed
    assert sim.loop_report["completed"]
    assert not sim.interior_entered and not sim.post_contact_push
    assert audit["all_probes_returned"]
    assert len(sim.policy.recovery_records) == 3


def test_top_view_keeps_one_xy_axes_and_mouse_drag_restarts():
    sim = ContourSimulator(config())
    view = SimulationView(sim)
    assert len(view.plot.figure.axes) == 1
    center = sim.target.center.copy()
    # Pixel coordinates are needed by Text.contains; data coordinates drive drag.
    data_x, data_y = center * 1000
    pixel_x, pixel_y = view.plot.axis.transData.transform((data_x, data_y))
    press = SimpleNamespace(button=1, inaxes=view.plot.axis, xdata=data_x, ydata=data_y,
                            x=pixel_x, y=pixel_y, key=None)
    view._on_press(press)
    assert view._dragging and not view.running
    move = SimpleNamespace(inaxes=view.plot.axis, xdata=data_x + 5, ydata=data_y + 4)
    view._on_motion(move)
    release = SimpleNamespace(inaxes=view.plot.axis, xdata=data_x + 5, ydata=data_y + 4)
    view._on_release(release)
    assert np.allclose(sim.target.center, center + [.005, .004])
    assert view.running and sim.policy.state.value == "TARGET_SEARCH"
    sim.finalized = True  # Closing this test figure must not write a partial run.
    view.animation._draw_was_started = True
    plt.close(view.plot.figure)


def test_clickable_shape_labels_exist_on_the_same_axes():
    sim = ContourSimulator(config())
    plot = TopViewPlot(sim)
    assert set(plot.shape_buttons) == {"square", "circle", "triangle"}
    assert all(label.axes is plot.axis for label in plot.shape_buttons.values())
    assert len(plot.figure.axes) == 1
    plt.close(plot.figure)

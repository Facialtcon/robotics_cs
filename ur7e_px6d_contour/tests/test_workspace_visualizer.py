"""Offline checks that workspace figures preserve recorded observation geometry."""

import csv
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from workspace.workspace_calibrator import fit_workspace, save_calibration
from workspace.workspace_transform import WorkspaceTransform
from workspace import workspace_visualizer as visualizer


FIELDS = [
    "record_type", "timestamp_utc", "monotonic_sec", "x_base", "y_base", "z_base",
    "x_workspace", "y_workspace", "z_workspace", "Fx", "Fy", "Fz", "policy_state",
    "contact_state", "boundary_state", "contact_point", "probe_id", "boundary_point_id",
    "contact_source", "calibration_id",
]


@pytest.fixture
def calibration(tmp_path):
    # A translated, rotated, clockwise frame makes any use of base XY or a
    # plotting-specific axis flip visibly disagree with the recorded coordinates.
    theta = np.deg2rad(37.)
    x_axis = np.array([np.cos(theta), np.sin(theta)])
    y_axis = np.array([x_axis[1], -x_axis[0]])
    origin = np.array([.62, -.24])
    corners = [origin, origin + .4 * x_axis,
               origin + .4 * x_axis + .25 * y_axis, origin + .25 * y_axis]
    raw = {f"P{i}": [*xy, .18, 0., 0., 0.] for i, xy in enumerate(corners)}
    fitted = fit_workspace(raw)
    path = save_calibration(fitted, tmp_path / "calibration.yaml")
    return path, WorkspaceTransform(fitted).calibration_id


def write_scan(path, calibration_id, rows, fields=FIELDS):
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            # These deliberately unusable base fields must have no influence.
            values = {"record_type": "sample", "x_base": "DO NOT PLOT BASE X",
                      "y_base": "DO NOT PLOT BASE Y", "z_base": .18,
                      "x_workspace": .02, "y_workspace": .03, "z_workspace": 0.,
                      "contact_point": 0, "calibration_id": calibration_id, **row}
            writer.writerow({key: value for key, value in values.items() if key in fields})
    return path


@pytest.fixture
def captured_figures(monkeypatch):
    figures = []
    original = visualizer.Figure

    def capture(*args, **kwargs):
        figure = original(*args, **kwargs)
        figures.append(figure)
        return figure

    monkeypatch.setattr(visualizer, "Figure", capture)
    yield figures
    for figure in figures:
        figure.clear()


def test_workspace_axes_samples_and_boundary_order_are_preserved(tmp_path, calibration, captured_figures):
    path, calibration_id = calibration
    rows = [
        {"x_workspace": -.03, "y_workspace": .01},
        {"record_type": "boundary", "boundary_point_id": 8, "monotonic_sec": 9,
         "x_workspace": .1, "y_workspace": .1},
        {"record_type": "contact", "probe_id": 7,
         "x_workspace": .11, "y_workspace": .12},
        {"x_workspace": .46, "y_workspace": .31},
        {"record_type": "boundary", "boundary_point_id": 2, "monotonic_sec": 4,
         "x_workspace": .22, "y_workspace": .1},
        {"record_type": "boundary", "boundary_point_id": 3, "monotonic_sec": 5,
         "x_workspace": .22, "y_workspace": .18},
    ]
    scan = write_scan(tmp_path / "workspace_scan.csv", calibration_id, rows)
    result = visualizer.visualize_workspace(scan, path)
    assert result == tmp_path / "workspace_contour.png"
    assert result.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    axis = captured_figures[0].axes[0]
    lines = {line.get_label(): line for line in axis.lines}
    assert set(lines) == {"Robot trajectory", "Ordered contour"}
    assert np.allclose(lines["Robot trajectory"].get_xydata(), [[-30, 10], [460, 310]])
    ordered = lines["Ordered contour"].get_xydata()
    assert np.allclose(ordered, [[100, 100], [220, 100], [220, 180]])
    assert len(ordered) == 3
    assert not np.array_equal(ordered[-1], ordered[0])
    markers = {item.get_label(): item for item in axis.collections}
    assert np.allclose(markers["Recorded contact points"].get_offsets(), [[110, 120]])
    assert np.allclose(markers["Accepted boundary points"].get_offsets(), ordered)
    rectangle = axis.patches[0]
    assert rectangle.get_xy() == (0., 0.)
    assert rectangle.get_width() == pytest.approx(400.)
    assert rectangle.get_height() == pytest.approx(250.)
    assert rectangle.get_edgecolor() == (0., 0., 0., 1.)
    assert not rectangle.get_fill()
    assert axis.get_xlim()[0] < -30 < 460 < axis.get_xlim()[1]
    assert axis.get_ylim()[0] < 0 < 310 < axis.get_ylim()[1]
    assert not axis.xaxis_inverted()
    assert not axis.yaxis_inverted()
    assert axis.get_aspect() == 1.
    assert axis.get_xlabel() == "Workspace X [mm]"
    assert axis.get_ylabel() == "Workspace Y [mm]"


def test_episode_contact_supersedes_same_probe_threshold(tmp_path, calibration, captured_figures):
    path, calibration_id = calibration
    rows = [
        {"contact_point": 1, "probe_id": 1, "contact_source": "threshold_detection",
         "x_workspace": .1},
        {"contact_point": 1, "probe_id": 2, "contact_source": "threshold_detection",
         "x_workspace": .2},
        {"contact_point": 1, "probe_id": "", "x_workspace": .3},
        {"record_type": "boundary", "probe_id": 1, "x_workspace": .11},
        # Contact rows arrive at the end of the run, after the threshold sample.
        {"record_type": "contact", "contact_point": 1, "probe_id": 1,
         "contact_source": "probe_episode", "x_workspace": .11},
    ]
    scan = write_scan(tmp_path / "workspace_scan.csv", calibration_id, rows)
    visualizer.visualize_workspace(scan, path)
    axis = captured_figures[0].axes[0]
    markers = {item.get_label(): item for item in axis.collections}
    assert np.allclose(markers["Recorded contact points"].get_offsets(), [[110, 30]])
    thresholds = markers["Threshold detections"]
    assert np.allclose(thresholds.get_offsets(), [[200, 30], [300, 30]])
    assert thresholds.get_alpha() < .6
    assert np.allclose(markers["Accepted boundary points"].get_offsets(), [[110, 30]])
    trajectory = next(line for line in axis.lines if line.get_label() == "Robot trajectory")
    assert np.allclose(trajectory.get_xydata(), [[100, 30], [200, 30], [300, 30]])


@pytest.mark.parametrize("record_type", ["sample", "contact", "boundary"])
def test_every_nonempty_calibration_id_must_match(tmp_path, calibration, record_type):
    path, calibration_id = calibration
    scan = write_scan(tmp_path / "workspace_scan.csv", calibration_id,
                      [{}, {"record_type": record_type, "calibration_id": "different-calibration"}])
    with pytest.raises(ValueError, match=r"row 3: calibration_id .* does not match"):
        visualizer.visualize_workspace(scan, path)
    assert not (tmp_path / "workspace_contour.png").exists()


def test_empty_calibration_id_is_allowed_and_explicit_output_is_used(tmp_path, calibration):
    path, calibration_id = calibration
    scan = write_scan(tmp_path / "workspace_scan.csv", calibration_id, [{"calibration_id": ""}])
    output = tmp_path / "figures" / "scan.png"
    assert visualizer.visualize_workspace(scan, path, output) == output
    assert output.is_file()


@pytest.mark.parametrize("row, message", [
    ({"x_workspace": "not a number"}, "workspace XY must contain numbers"),
    ({"x_workspace": "NaN"}, "workspace XY must be finite"),
    ({"y_workspace": "inf"}, "workspace XY must be finite"),
    ({"contact_point": 2}, "contact_point must be 0 or 1"),
    ({"record_type": "waypoint"}, "unsupported record_type"),
])
def test_invalid_observation_is_rejected_without_base_coordinate_fallback(tmp_path, calibration, row, message):
    path, calibration_id = calibration
    scan = write_scan(tmp_path / "workspace_scan.csv", calibration_id, [row])
    with pytest.raises(ValueError, match=message):
        visualizer.visualize_workspace(scan, path)


def test_missing_workspace_columns_are_rejected(tmp_path, calibration):
    path, calibration_id = calibration
    scan = write_scan(tmp_path / "workspace_scan.csv", calibration_id, [{}],
                      fields=[field for field in FIELDS if field != "x_workspace"])
    with pytest.raises(ValueError, match="missing columns: x_workspace"):
        visualizer.visualize_workspace(scan, path)


def test_empty_scan_draws_workspace_without_fabricated_observations(tmp_path, calibration, captured_figures):
    path, calibration_id = calibration
    scan = write_scan(tmp_path / "workspace_scan.csv", calibration_id, [])
    visualizer.visualize_workspace(scan, path)
    axis = captured_figures[0].axes[0]
    assert len(axis.lines) == len(axis.collections) == 0
    assert len(axis.patches) == 1
    assert axis.get_xlim()[0] < 0 < 400 < axis.get_xlim()[1]
    assert axis.get_ylim()[0] < 0 < 250 < axis.get_ylim()[1]


@pytest.mark.parametrize("overwrite", ["csv", "calibration"])
def test_output_cannot_overwrite_its_input(tmp_path, calibration, overwrite):
    path, calibration_id = calibration
    scan = write_scan(tmp_path / "workspace_scan.csv", calibration_id, [{}])
    destination = scan if overwrite == "csv" else path
    before = destination.read_bytes()
    with pytest.raises(ValueError, match="must not overwrite"):
        visualizer.visualize_workspace(scan, path, destination)
    assert destination.read_bytes() == before


def test_import_and_render_keep_callers_matplotlib_backend(tmp_path, calibration):
    path, calibration_id = calibration
    scan = write_scan(tmp_path / "workspace_scan.csv", calibration_id, [{}])
    code = """
import sys
import matplotlib
matplotlib.use('svg')
from workspace.workspace_visualizer import visualize_workspace
assert matplotlib.get_backend().lower() == 'svg'
visualize_workspace(sys.argv[1], sys.argv[2])
assert matplotlib.get_backend().lower() == 'svg'
"""
    result = subprocess.run([sys.executable, "-B", "-c", code, str(scan), str(path)],
                            cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_cli_uses_explicit_csv_and_calibration(tmp_path, calibration, capsys):
    path, calibration_id = calibration
    scan = write_scan(tmp_path / "workspace_scan.csv", calibration_id, [{}])
    output = tmp_path / "cli.png"
    assert visualizer.main(["--csv", str(scan), "--calibration", str(path), "--output", str(output)]) == 0
    assert capsys.readouterr().out.strip() == str(output)
    assert output.is_file()


def test_startup_return_is_not_scan_trajectory_and_hidden_intervals_are_not_bridged(
        tmp_path, calibration, captured_figures):
    path, calibration_id = calibration
    rows = [
        {"policy_state": "RETURN_TO_START", "x_workspace": 9., "y_workspace": 9.},
        {"policy_state": "TARGET_SEARCH", "x_workspace": .1, "y_workspace": .1},
        {"policy_state": "BOUNDARY_TRACKING", "policy_sub_state": "PROBE",
         "x_workspace": .11, "y_workspace": .1},
        {"policy_state": "BOUNDARY_TRACKING", "policy_sub_state": "RETURN",
         "x_workspace": .105, "y_workspace": .1},
        {"policy_state": "RETURN_TO_START", "x_workspace": 8., "y_workspace": 8.,
         "contact_point": 1},
        {"policy_state": "BOUNDARY_RECOVERY", "x_workspace": .2, "y_workspace": .1},
    ]
    scan = write_scan(tmp_path / "workspace_scan.csv", calibration_id, rows,
                      fields=FIELDS + ["policy_sub_state"])
    original = scan.read_bytes()
    visualizer.visualize_workspace(scan, path)
    axis = captured_figures[0].axes[0]
    assert len(axis.lines) == 1
    trajectory = axis.lines[0].get_xydata()
    assert np.allclose(trajectory[:3], [[100, 100], [110, 100], [105, 100]])
    assert np.isnan(trajectory[3]).all()
    assert np.allclose(trajectory[4], [200, 100])
    assert len(trajectory) == 5
    assert not axis.collections  # Hidden positioning force is not a scan contact.
    assert axis.get_xlim()[1] < 500 and axis.get_ylim()[1] < 350
    assert scan.read_bytes() == original


def test_legend_is_outside_plot_and_inside_exported_figure(tmp_path, calibration, captured_figures):
    path, calibration_id = calibration
    rows = [{"policy_state": "TARGET_SEARCH"}, {"record_type": "contact"},
            {"record_type": "boundary"}, {"contact_point": 1}]
    scan = write_scan(tmp_path / "workspace_scan.csv", calibration_id, rows)
    visualizer.visualize_workspace(scan, path)
    figure = captured_figures[0]
    figure.canvas.draw()
    axis = figure.axes[0]
    legend_box = axis.get_legend().get_window_extent(figure.canvas.get_renderer())
    assert legend_box.x0 > axis.get_window_extent().x1
    assert legend_box.x1 <= figure.bbox.x1
    assert legend_box.y1 <= figure.bbox.y1

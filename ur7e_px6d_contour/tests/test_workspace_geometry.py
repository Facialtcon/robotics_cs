"""Offline geometry checks for the independent four-corner workspace frame."""
from copy import deepcopy

import numpy as np
import pytest
import yaml

from workspace.workspace_calibrator import WorkspaceCalibrationError, fit_workspace, save_calibration
from workspace.workspace_transform import WorkspaceTransform, base_to_workspace, load_calibration


def raw_rectangle(angle_deg=37., winding="CCW", length=.4, width=.25):
    angle = np.deg2rad(angle_deg)
    x = np.array([np.cos(angle), np.sin(angle)])
    y = np.array([-x[1], x[0]]) * (1 if winding == "CCW" else -1)
    origin = np.array([.42, -.18, .21])
    xy = np.array([origin[:2], origin[:2] + length * x,
                   origin[:2] + length * x + width * y, origin[:2] + width * y])
    poses = np.column_stack((xy, np.full(4, origin[2]), np.zeros((4, 3))))
    return {f"P{i}": pose.tolist() for i, pose in enumerate(poses)}


@pytest.mark.parametrize("angle", [0., 37., 180., -110.])
@pytest.mark.parametrize("winding", ["CCW", "CW"])
def test_exact_rectangles_keep_labeled_axes_and_right_handed_rotation(angle, winding):
    raw = raw_rectangle(angle, winding)
    fit = fit_workspace(raw)
    transform = WorkspaceTransform(fit)
    rotation = np.asarray(fit["rotation_workspace_to_base"])
    assert fit["length"] == pytest.approx(.4)
    assert fit["width"] == pytest.approx(.25)
    assert fit["origin"] == pytest.approx(raw["P0"][:3])
    assert fit["winding"] == winding
    assert fit["normal_sign"] == (1 if winding == "CCW" else -1)
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1.)
    direction = np.asarray(raw["P1"][:2]) - raw["P0"][:2]
    assert np.dot(fit["x_axis"][:2], direction / np.linalg.norm(direction)) == pytest.approx(1.)
    assert np.dot(fit["y_axis"][:2], np.asarray(raw["P3"][:2]) - raw["P0"][:2]) > 0
    points = np.asarray([fit["rectified_points"][f"R{i}"] for i in range(4)])
    expected = np.array([[0, 0, 0], [.4, 0, 0], [.4, .25, 0], [0, .25, 0]])
    assert np.allclose(transform.base_to_workspace(points), expected, atol=1e-12)
    assert np.allclose(transform.workspace_to_base(expected), points, atol=1e-12)


@pytest.mark.parametrize("winding", ["CCW", "CW"])
def test_transform_preserves_height_and_round_trips_arbitrary_points(winding):
    fit = fit_workspace(raw_rectangle(winding=winding))
    transform = WorkspaceTransform(fit)
    rng = np.random.default_rng(619)
    points = rng.uniform(-.8, .8, (20, 3))
    local = transform.base_to_workspace(points)
    assert np.allclose(transform.workspace_to_base(local), points, atol=1e-12)
    assert np.allclose(local[:, 2], fit["normal_sign"] * (points[:, 2] - fit["average_z"]))
    assert np.allclose(base_to_workspace(points[0], fit), local[0])
    assert np.allclose(base_to_workspace(points[0], transform), local[0])
    stored_inverse = np.asarray(fit["rotation_base_to_workspace"])
    stored_translation = np.asarray(fit["translation_base_to_workspace"])
    assert np.allclose(stored_inverse @ points[0] + stored_translation, local[0])


def test_noisy_corners_produce_mean_z_plane_without_tilting_or_using_tcp_orientation():
    rng = np.random.default_rng(20260914)
    poses = np.asarray(list(raw_rectangle().values()))
    poses[:, :2] += rng.normal(0., .001, (4, 2))
    noisy_z = poses.copy()
    noisy_z[:, 2] += rng.normal(0., .012, 4)
    noisy_z[:, 3:] = rng.uniform(-6., 6., (4, 3))
    fit = fit_workspace({f"P{i}": p for i, p in enumerate(noisy_z)})
    flat = fit_workspace({f"P{i}": p for i, p in enumerate(poses)})
    assert fit["length"] == flat["length"]
    assert fit["width"] == flat["width"]
    assert fit["x_axis"] == flat["x_axis"]
    assert fit["y_axis"] == flat["y_axis"]
    assert fit["x_axis"][2] == fit["y_axis"][2] == 0
    assert fit["average_z"] == pytest.approx(noisy_z[:, 2].mean())
    rectified = np.asarray(list(fit["rectified_points"].values()))
    assert np.allclose(rectified[:, 2], noisy_z[:, 2].mean())
    assert np.dot(fit["x_axis"], fit["y_axis"]) == pytest.approx(0., abs=1e-12)
    assert fit["diagnostics"]["fit_max_xy_m"] < .005
    assert fit["diagnostics"]["raw_z_range_m"] > .001
    assert fit["raw_points"]["P0"]["rx"] == noisy_z[0, 3]


def test_centroid_origin_minimizes_translation_error_instead_of_anchoring_noisy_p0():
    raw = raw_rectangle()
    raw["P0"][0] += .004
    raw["P2"][1] -= .002
    fit = fit_workspace(raw)
    xy = np.asarray(list(raw.values()))[:, :2]
    corners = np.asarray(list(fit["rectified_points"].values()))[:, :2]
    anchored_at_p0 = corners + xy[0] - corners[0]
    assert np.allclose(corners.mean(axis=0), xy.mean(axis=0))
    assert np.sum((corners - xy) ** 2) < np.sum((anchored_at_p0 - xy) ** 2)
    assert fit["diagnostics"]["origin_correction_xy_m"] > 0


def test_moderate_quadrilateral_distortion_is_orthogonalized_without_reordering():
    xy = [[0, 0], [.4, .04], [.43, .32], [-.01, .28]]
    raw = {f"P{i}": [*p, .2, 0, 0, 0] for i, p in enumerate(xy)}
    fit = fit_workspace(raw)
    expected_length = (np.linalg.norm(np.subtract(xy[1], xy[0]))
                       + np.linalg.norm(np.subtract(xy[2], xy[3]))) / 2
    assert fit["length"] == pytest.approx(expected_length)
    assert fit["raw_points"]["P2"]["x"] == .43
    assert np.dot(fit["x_axis"], fit["y_axis"]) == pytest.approx(0., abs=1e-12)
    assert fit["diagnostics"]["orthogonality_error_deg"] > 0
    assert fit["diagnostics"]["fit_rms_xy_m"] > 0


def test_reversing_corner_order_preserves_new_labels_and_swaps_lengths():
    original = raw_rectangle()
    fit = fit_workspace(original)
    reversed_order = fit_workspace({name: original[source] for name, source in
                                    zip(("P0", "P1", "P2", "P3"), ("P0", "P3", "P2", "P1"))})
    assert reversed_order["winding"] == "CW"
    assert reversed_order["length"] == pytest.approx(fit["width"])
    assert reversed_order["width"] == pytest.approx(fit["length"])
    assert reversed_order["x_axis"] == pytest.approx(fit["y_axis"])
    assert reversed_order["y_axis"] == pytest.approx(fit["x_axis"])
    assert reversed_order["origin"] == pytest.approx(fit["origin"])


@pytest.mark.parametrize("xy", [
    [[0, 0], [1, 1], [1, 0], [0, 1]],  # crossed order
    [[0, 0], [1, 0], [.2, .2], [0, 1]],  # concave order
    [[0, 0], [1, 0], [2, 0], [3, 0]],  # collinear
    [[0, 0], [1, 0], [2, .00001], [0, .00001]],  # nearly collinear turn
    [[0, 0], [1, 0], [1, 0], [0, 1]],  # duplicate
])
def test_invalid_corner_geometry_is_rejected_without_silent_sorting(xy):
    with pytest.raises(WorkspaceCalibrationError):
        fit_workspace({f"P{i}": [*point, .2, 0, 0, 0] for i, point in enumerate(xy)})


@pytest.mark.parametrize("field", [0, 2, 5])
def test_nonfinite_positions_and_recorded_orientations_are_rejected(field):
    raw = raw_rectangle()
    raw["P2"][field] = float("nan")
    with pytest.raises(WorkspaceCalibrationError, match="finite"):
        fit_workspace(raw)


def test_xyz_mapping_input_fills_unused_orientation_with_zero():
    raw = raw_rectangle()
    as_mappings = {name: dict(zip(("x", "y", "z"), pose[:3])) for name, pose in raw.items()}
    fit = fit_workspace(as_mappings)
    assert fit == fit_workspace(raw)


def test_save_load_and_metadata_changes_keep_geometry_identifier(tmp_path):
    raw = raw_rectangle()
    fit = fit_workspace(raw, metadata={"robot_ip": "192.0.2.1", "timestamps": [1., 2., 3., 4.]})
    destination = tmp_path / "config" / "workspace.yaml"
    assert save_calibration(fit, destination) == destination
    loaded = load_calibration(destination)
    assert loaded == fit
    original = WorkspaceTransform(fit)
    restored = WorkspaceTransform.from_file(destination)
    assert restored.calibration_id == original.calibration_id
    for pose in raw.values():
        pose[3:] = [1., -2., 3.]
    new_metadata = fit_workspace(raw, metadata={"robot_ip": "192.0.2.2"})
    assert WorkspaceTransform(new_metadata).calibration_id == original.calibration_id
    raw["P2"][0] += .002
    assert WorkspaceTransform(fit_workspace(raw)).calibration_id != original.calibration_id
    assert not list(destination.parent.glob("*.tmp"))


@pytest.mark.parametrize("field", ["length", "origin", "rotation_workspace_to_base",
                                  "translation_base_to_workspace", "rectified_points", "diagnostics",
                                  "normal_sign", "winding", "raw_points", "units", "schema_version"])
def test_loader_rejects_inconsistent_or_tampered_calibration_fields(tmp_path, field):
    calibration = deepcopy(fit_workspace(raw_rectangle()))
    if field == "length":
        calibration[field] *= 1.1
    elif field in {"origin", "translation_base_to_workspace"}:
        calibration[field][0] += .01
    elif field == "rotation_workspace_to_base":
        calibration[field][0][0] *= -1
    elif field == "rectified_points":
        calibration[field]["R2"][2] += .01
    elif field == "diagnostics":
        calibration[field]["fit_rms_xy_m"] += .01
    elif field == "normal_sign":
        calibration[field] *= -1
    elif field == "winding":
        calibration[field] = "CW"
    elif field == "raw_points":
        calibration[field]["P2"]["x"] += .01
    elif field == "units":
        calibration[field] = "mm"
    else:
        calibration[field] = 2
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(calibration), encoding="utf-8")
    with pytest.raises(WorkspaceCalibrationError):
        load_calibration(path)


def test_invalid_save_does_not_create_a_calibration_and_missing_load_is_explicit(tmp_path):
    path = tmp_path / "new" / "calibration.yaml"
    with pytest.raises(WorkspaceCalibrationError):
        save_calibration({"schema_version": 1}, path)
    assert not path.exists()
    with pytest.raises(FileNotFoundError, match="workspace calibration"):
        load_calibration(path)


@pytest.mark.parametrize("points", [[1., 2.], [1., 2., 3., 0., 0., 0.],
                                   [[1., 2.], [3., 4.]], [1., float("nan"), 3.]])
def test_point_transform_rejects_wrong_shapes_and_nonfinite_inputs(points):
    transform = WorkspaceTransform(fit_workspace(raw_rectangle()))
    with pytest.raises(WorkspaceCalibrationError):
        transform.base_to_workspace(points)

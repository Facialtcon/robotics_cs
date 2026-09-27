"""Validated rigid point transforms for a separately calibrated workspace."""
from __future__ import annotations

from collections.abc import Mapping
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from workspace.workspace_calibrator import (
    POINT_NAMES, POSE_FIELDS, WorkspaceCalibrationError, fit_workspace,
)


DEFAULT_CALIBRATION_PATH = Path(__file__).resolve().parent / "config" / "workspace_calibration.yaml"
CONSISTENCY_TOLERANCE = 1e-9


def _same_numeric(actual, expected, name):
    try:
        value, reference = np.asarray(actual, dtype=float), np.asarray(expected, dtype=float)
    except (TypeError, ValueError) as exc:
        raise WorkspaceCalibrationError(f"invalid workspace calibration field: {name}") from exc
    if (value.shape != reference.shape or not np.all(np.isfinite(value))
            or not np.allclose(value, reference, rtol=0., atol=CONSISTENCY_TOLERANCE)):
        raise WorkspaceCalibrationError(f"workspace calibration field {name} is inconsistent with raw_points")


def validate_calibration(calibration) -> dict:
    """Recompute geometry from raw points and reject stale or mixed transforms."""
    if not isinstance(calibration, Mapping):
        raise WorkspaceCalibrationError("workspace calibration must be a mapping")
    if type(calibration.get("schema_version")) is not int or calibration["schema_version"] != 1:
        raise WorkspaceCalibrationError("unsupported workspace calibration schema_version; expected 1")
    if calibration.get("units") != "m":
        raise WorkspaceCalibrationError("workspace calibration units must be m")
    raw = calibration.get("raw_points")
    if not isinstance(raw, Mapping) or set(raw) != set(POINT_NAMES):
        raise WorkspaceCalibrationError("workspace calibration raw_points must contain P0..P3")
    if any(not isinstance(raw[name], Mapping) or not all(field in raw[name] for field in POSE_FIELDS)
           for name in POINT_NAMES):
        raise WorkspaceCalibrationError("saved raw_points must contain all x/y/z/rx/ry/rz pose fields")
    expected = fit_workspace(raw, metadata=calibration.get("metadata", {}))
    for name in ("average_z", "length", "width", "origin", "x_axis", "y_axis", "z_axis",
                 "rotation_workspace_to_base", "translation_workspace_to_base",
                 "rotation_base_to_workspace", "translation_base_to_workspace"):
        _same_numeric(calibration.get(name), expected[name], name)
    if calibration.get("winding") != expected["winding"]:
        raise WorkspaceCalibrationError("workspace winding is inconsistent with raw corner order")
    if type(calibration.get("normal_sign")) is not int or calibration["normal_sign"] != expected["normal_sign"]:
        raise WorkspaceCalibrationError("workspace normal_sign is inconsistent with winding")
    corners = calibration.get("rectified_points")
    if not isinstance(corners, Mapping) or set(corners) != {"R0", "R1", "R2", "R3"}:
        raise WorkspaceCalibrationError("rectified_points must contain R0..R3")
    for name, point in expected["rectified_points"].items():
        _same_numeric(corners[name], point, f"rectified_points.{name}")
    diagnostics = calibration.get("diagnostics")
    if not isinstance(diagnostics, Mapping):
        raise WorkspaceCalibrationError("workspace calibration diagnostics must be a mapping")
    for name, value in expected["diagnostics"].items():
        _same_numeric(diagnostics.get(name), value, f"diagnostics.{name}")
    return expected


def load_calibration(path=DEFAULT_CALIBRATION_PATH) -> dict:
    source = Path(path).expanduser()
    if not source.is_file():
        raise FileNotFoundError(f"workspace calibration file not found: {source}")
    try:
        with source.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except yaml.YAMLError as exc:
        raise WorkspaceCalibrationError(f"invalid workspace calibration YAML: {source}") from exc
    return validate_calibration(data)


class WorkspaceTransform:
    """Transform 3D positions; TCP rotation-vector components are not points.

    R stores workspace axes as columns in base coordinates. The Z coordinate
    retains the signed height from the mean base-Z plane, even though top-view
    plots use only XY. A clockwise calibration has workspace Z pointing down.
    """

    def __init__(self, calibration):
        self.calibration = validate_calibration(calibration)
        self.rotation_workspace_to_base = np.asarray(self.calibration["rotation_workspace_to_base"])
        self.translation_workspace_to_base = np.asarray(self.calibration["translation_workspace_to_base"])
        self.rotation_base_to_workspace = self.rotation_workspace_to_base.T.copy()
        self.translation_base_to_workspace = -self.rotation_base_to_workspace @ self.translation_workspace_to_base
        geometry = {key: self.calibration[key] for key in
                    ("schema_version", "units", "length", "width", "origin", "x_axis", "y_axis", "z_axis")}
        # Ignore acquisition metadata and TCP orientation; equivalent geometry
        # keeps its identifier across save/load and sub-picometer round-off.
        for key in ("length", "width", "origin", "x_axis", "y_axis", "z_axis"):
            values = np.round(np.asarray(geometry[key], dtype=float), 12)
            values = np.where(values == 0., 0., values)  # canonicalize negative zero
            geometry[key] = values.tolist()
        encoded = json.dumps(geometry, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self.calibration_id = hashlib.sha256(encoded).hexdigest()

    @classmethod
    def from_file(cls, path=DEFAULT_CALIBRATION_PATH):
        return cls(load_calibration(path))

    @staticmethod
    def _points(points):
        try:
            values = np.asarray(points, dtype=float)
        except (TypeError, ValueError) as exc:
            raise WorkspaceCalibrationError("points must be finite XYZ or N×3 positions") from exc
        if not ((values.ndim == 1 and values.shape == (3,))
                or (values.ndim == 2 and values.shape[1] == 3)) or not np.all(np.isfinite(values)):
            raise WorkspaceCalibrationError("points must be finite XYZ or N×3 positions, not six-component TCP poses")
        return values

    def base_to_workspace(self, points):
        values = self._points(points)
        return (values - self.translation_workspace_to_base) @ self.rotation_workspace_to_base

    def workspace_to_base(self, points):
        values = self._points(points)
        return values @ self.rotation_workspace_to_base.T + self.translation_workspace_to_base


def base_to_workspace(points, calibration=None):
    transform = (WorkspaceTransform.from_file() if calibration is None else
                 calibration if isinstance(calibration, WorkspaceTransform) else WorkspaceTransform(calibration))
    return transform.base_to_workspace(points)


def workspace_to_base(points, calibration=None):
    transform = (WorkspaceTransform.from_file() if calibration is None else
                 calibration if isinstance(calibration, WorkspaceTransform) else WorkspaceTransform(calibration))
    return transform.workspace_to_base(points)

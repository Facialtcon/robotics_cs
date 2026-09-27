"""Draw recorded workspace coordinates without participating in robot control."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
import numpy as np

from workspace.workspace_transform import WorkspaceTransform


# High-level positioning states only. Probe phase RETURN is part of scanning
# and must remain visible along with search, tracking, and recovery travel.
POSITIONING_STATES = {"RETURN_TO_START", "STARTUP_RETURN", "SAFE_RETURN"}


def _read_coordinates(scan_csv: Path, calibration_id: str) -> dict[str, np.ndarray]:
    groups = {"trajectory": [], "contacts": [], "thresholds": [], "boundary": [], "all": []}
    threshold_candidates = []
    recorded_contact_probes = set()
    trajectory_gap = False
    with scan_csv.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"record_type", "x_workspace", "y_workspace", "contact_point", "calibration_id"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError("workspace scan CSV is missing columns: " + ", ".join(sorted(missing)))
        for row_number, row in enumerate(reader, start=2):
            row_id = (row.get("calibration_id") or "").strip()
            if row_id and row_id != calibration_id:
                raise ValueError(
                    f"CSV row {row_number}: calibration_id {row_id!r} does not match "
                    f"the supplied calibration {calibration_id!r}"
                )
            record_type = (row.get("record_type") or "").strip()
            if record_type not in {"sample", "contact", "boundary"}:
                raise ValueError(f"CSV row {row_number}: unsupported record_type {record_type!r}")
            try:
                xy = np.asarray([float(row["x_workspace"]), float(row["y_workspace"])])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"CSV row {row_number}: workspace XY must contain numbers") from exc
            if not np.all(np.isfinite(xy)):
                raise ValueError(f"CSV row {row_number}: workspace XY must be finite")
            flag_text = (row.get("contact_point") or "0").strip()
            try:
                flag = float(flag_text)
            except ValueError as exc:
                raise ValueError(f"CSV row {row_number}: contact_point must be 0 or 1") from exc
            if flag not in (0.0, 1.0):
                raise ValueError(f"CSV row {row_number}: contact_point must be 0 or 1")
            point_mm = xy * 1000.0
            if record_type == "sample" and (row.get("policy_state") or "").strip() in POSITIONING_STATES:
                # Elevated return/positioning paths are still in the CSV, but
                # their XY projections are not a sandbox scanning trajectory.
                trajectory_gap = True
                continue
            groups["all"].append(point_mm)
            if record_type == "sample":
                if trajectory_gap and groups["trajectory"]:
                    groups["trajectory"].append(np.array([np.nan, np.nan]))
                groups["trajectory"].append(point_mm)
                trajectory_gap = False
            probe_id = (row.get("probe_id") or "").strip()
            if record_type == "contact":
                groups["contacts"].append(point_mm)
                if probe_id:
                    recorded_contact_probes.add(probe_id)
            elif record_type == "sample" and flag == 1.0:
                threshold_candidates.append((probe_id, point_mm))
            if record_type == "boundary":
                groups["boundary"].append(point_mm)
    # Episode contacts are appended when logging closes. Prefer that recorded
    # contact pose over the same probe's earlier force-threshold sample.
    groups["thresholds"] = [point for probe, point in threshold_candidates
                            if not probe or probe not in recorded_contact_probes]
    return {name: np.asarray(points, dtype=float).reshape(-1, 2) for name, points in groups.items()}


def visualize_workspace(scan_csv, calibration_path, output_path=None) -> Path:
    """Save a workspace XY plot from the supplied CSV and matching calibration.

    Coordinates in the CSV are metres. Base coordinates are never read or
    transformed here; the CSV's workspace values are displayed in millimetres.
    Boundary samples retain their recorded acceptance order, without closure.
    Threshold detections remain distinct from saved probe contact positions.
    High-level positioning/return samples are omitted from this scan-only view;
    paths separated by those samples are never joined across the omitted part.
    """
    scan_csv = Path(scan_csv).expanduser().resolve()
    calibration_path = Path(calibration_path).expanduser().resolve()
    transform = WorkspaceTransform.from_file(calibration_path)
    calibration_id = str(transform.calibration_id)
    length_mm = float(transform.calibration["length"]) * 1000.0
    width_mm = float(transform.calibration["width"]) * 1000.0
    if not np.all(np.isfinite([length_mm, width_mm])) or min(length_mm, width_mm) <= 0:
        raise ValueError("workspace calibration length and width must be finite and positive")
    groups = _read_coordinates(scan_csv, calibration_id)
    destination = (scan_csv.parent / "workspace_contour.png" if output_path is None
                   else Path(output_path).expanduser()).resolve()
    if destination in {scan_csv, calibration_path}:
        raise ValueError("visualization output must not overwrite its CSV or calibration input")

    figure = Figure(figsize=(9, 7), layout="constrained", facecolor="white")
    figure.get_layout_engine().set(rect=(0.0, .08, 1.0, .92))
    FigureCanvasAgg(figure)
    axis = figure.subplots()
    axis.add_patch(Rectangle((0.0, 0.0), length_mm, width_mm, fill=False,
                             edgecolor="black", linewidth=1.8, label="Workspace boundary", zorder=2))
    trajectory, contacts, boundary = (groups[name] for name in ("trajectory", "contacts", "boundary"))
    if len(trajectory):
        axis.plot(trajectory[:, 0], trajectory[:, 1], color="#3376a8", linewidth=1.0,
                  alpha=.85, label="Robot trajectory", zorder=1)
    if len(contacts):
        axis.scatter(contacts[:, 0], contacts[:, 1], s=34, marker="x", color="#cf751a",
                     linewidths=1.3, label="Recorded contact points", zorder=4)
    thresholds = groups["thresholds"]
    if len(thresholds):
        axis.scatter(thresholds[:, 0], thresholds[:, 1], s=25, marker="+", color="#cf751a",
                     alpha=.45, linewidths=1.0, label="Threshold detections", zorder=3)
    if len(boundary):
        axis.plot(boundary[:, 0], boundary[:, 1], color="#277443", linewidth=1.5,
                  label="Ordered contour", zorder=3)
        axis.scatter(boundary[:, 0], boundary[:, 1], s=28, color="#277443", edgecolors="white",
                     linewidths=.7, label="Accepted boundary points", zorder=5)

    extent_points = np.vstack((np.asarray([[0.0, 0.0], [length_mm, width_mm]]), groups["all"]))
    lower, upper = extent_points.min(axis=0), extent_points.max(axis=0)
    margin = np.maximum((upper - lower) * .06, 1.0)
    axis.set_xlim(lower[0] - margin[0], upper[0] + margin[0])
    axis.set_ylim(lower[1] - margin[1], upper[1] + margin[1])
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("Workspace X [mm]")
    axis.set_ylabel("Workspace Y [mm]")
    synthetic = transform.calibration.get("metadata", {}).get("synthetic_demo", False)
    axis.set_title("Offline coordinate validation — synthetic workspace" if synthetic
                   else "Workspace scan observations")
    axis.grid(True, color="#dddddd", linewidth=.6, alpha=.65)
    axis.set_axisbelow(True)
    axis.legend(loc="upper left", bbox_to_anchor=(1.03, 1.0), borderaxespad=0,
                framealpha=.95, fontsize=9)
    figure.text(.5, .04, "Scan trajectory only; startup and safe-return positioning omitted",
                ha="center", va="bottom", fontsize=8, color="#555555")
    figure.text(.5, .01, f"Calibration: {calibration_id}  |  Contour follows recorded point order",
                ha="center", va="bottom", fontsize=8, color="#555555")
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=160)
    return destination


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True, type=Path, help="workspace scan CSV")
    parser.add_argument("--calibration", required=True, type=Path, help="workspace calibration file")
    parser.add_argument("--output", type=Path, help="image output; default is workspace_contour.png beside CSV")
    args = parser.parse_args(argv)
    print(visualize_workspace(args.csv, args.calibration, args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

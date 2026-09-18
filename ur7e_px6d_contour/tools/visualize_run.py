#!/usr/bin/env python3
"""Create a static diagnostic figure from one contour experiment run."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
STATE_ORDER = [
    "TARGET_SEARCH",
    "LOCAL_INITIALIZATION",
    "BOUNDARY_TRACKING",
    "BOUNDARY_RECOVERY",
    "BOUNDARY_CONFIRMATION",
    "LOOP_COMPLETE",
    # Retain historical names so archived real runs remain readable.
    "SEARCH",
    "CONTACT",
    "RETRACT",
    "TANGENTIAL_STEP",
    "PROBE",
    "UPDATE",
    "CORNER_SEARCH",
    "NEW_EDGE_CONFIRM",
    "LOST",
    "STOP_SCAN",
    "RETURN_TO_START",
    "STOP",
]
STATE_COLORS = {
    "TARGET_SEARCH": "#4c78a8",
    "LOCAL_INITIALIZATION": "#f2cf5b",
    "BOUNDARY_TRACKING": "#54a24b",
    "BOUNDARY_RECOVERY": "#f28e2b",
    "BOUNDARY_CONFIRMATION": "#2ca02c",
    "LOOP_COMPLETE": "#17becf",
    "SEARCH": "#4c78a8",
    "CONTACT": "#e45756",
    "RETRACT": "#72b7b2",
    "TANGENTIAL_STEP": "#54a24b",
    "PROBE": "#f2cf5b",
    "UPDATE": "#b279a2",
    "CORNER_SEARCH": "#f28e2b",
    "NEW_EDGE_CONFIRM": "#2ca02c",
    "LOST": "#ff9da6",
    "STOP_SCAN": "#9c9c9c",
    "RETURN_TO_START": "#9467bd",
    "STOP": "#79706e",
}


def latest_run(data_root: Path = ROOT / "data") -> Path:
    candidates = sorted(path for path in data_root.glob("run_*") if path.is_dir())
    if not candidates:
        raise FileNotFoundError(f"no run directory found below {data_root}")
    return candidates[-1]


def _read_csv(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise FileNotFoundError(path)
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _number(rows: list[dict[str, str]], field: str) -> np.ndarray:
    return np.asarray([float(row[field]) for row in rows], dtype=float)


def create_visualization(run_dir: Path, output: Path | None = None) -> Path:
    run_dir = run_dir.expanduser().resolve()
    samples = _read_csv(run_dir / "samples.csv")
    boundaries = _read_csv(run_dir / "boundary_points.csv")
    if not samples:
        raise ValueError(f"samples.csv is empty in {run_dir}")

    with (run_dir / "config_snapshot.yaml").open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)

    elapsed = _number(samples, "monotonic_sec")
    elapsed -= elapsed[0]
    x = (_number(samples, "tcp_x") - float(samples[0]["tcp_x"])) * 1000.0
    y = (_number(samples, "tcp_y") - float(samples[0]["tcp_y"])) * 1000.0
    fxy = _number(samples, "fxy")
    commanded_speed = _number(samples, "commanded_speed_mps") * 1000.0
    states = [row["current_state"] for row in samples]
    # Plot only observed states. Preserve unknown future names rather than
    # dropping their samples or failing to save the whole experiment figure.
    state_order = [state for state in STATE_ORDER if state in states]
    state_order.extend(sorted(set(states) - set(state_order)))
    state_colors = {state: STATE_COLORS.get(state, "#777777") for state in state_order}
    state_index = np.asarray([state_order.index(state) for state in states])

    figure, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    path_axis, force_axis, speed_axis, state_axis = axes.flat

    return_mask = np.asarray([value == "RETURN_TO_START" for value in states])
    scan_mask = ~return_mask
    path_axis.plot(x[scan_mask], y[scan_mask], color="#bbbbbb", linewidth=1.0, zorder=1)
    if np.any(return_mask):
        return_indices = np.flatnonzero(return_mask)
        first = max(0, int(return_indices[0]) - 1)
        return_path = np.arange(first, int(return_indices[-1]) + 1)
        path_axis.plot(
            x[return_path],
            y[return_path],
            color=STATE_COLORS["RETURN_TO_START"],
            linestyle="--",
            linewidth=1.2,
            label="safe-return XY path",
            zorder=1,
        )
    for state in state_order:
        mask = np.asarray([value == state for value in states])
        if np.any(mask):
            path_axis.scatter(
                x[mask], y[mask], s=7, color=state_colors[state], label=state, zorder=2
            )
    path_axis.scatter([x[0]], [y[0]], marker="*", s=130, color="black", zorder=5)

    if boundaries:
        bx_absolute = _number(boundaries, "tcp_x")
        by_absolute = _number(boundaries, "tcp_y")
        bx = (bx_absolute - float(samples[0]["tcp_x"])) * 1000.0
        by = (by_absolute - float(samples[0]["tcp_y"])) * 1000.0
        direction_prefix = (
            "target_direction" if "target_direction_x" in boundaries[0]
            else "estimated_normal"
        )
        nx = _number(boundaries, f"{direction_prefix}_x")
        ny = _number(boundaries, f"{direction_prefix}_y")
        tx = _number(boundaries, "tangent_x")
        ty = _number(boundaries, "tangent_y")
        arrow_length_mm = max(
            1.0, float(config["policy"]["tangent_step"]) * 1000.0 * 0.65
        )
        path_axis.scatter(bx, by, s=42, color="black", facecolor="white", zorder=6)
        path_axis.quiver(
            bx, by, nx, ny, angles="xy", scale_units="xy", scale=1.0 / arrow_length_mm,
            color="#d62728", width=0.004, zorder=5,
        )
        path_axis.quiver(
            bx, by, tx, ty, angles="xy", scale_units="xy", scale=1.0 / arrow_length_mm,
            color="#1f77b4", width=0.004, zorder=5,
        )
        for row, px, py in zip(boundaries, bx, by):
            path_axis.annotate(row["point_id"], (px, py), xytext=(4, 4), textcoords="offset points")

    path_axis.set_title("TCP path and boundary estimates")
    path_axis.set_xlabel("X relative to start [mm]")
    path_axis.set_ylabel("Y relative to start [mm]")
    path_axis.axis("equal")
    path_axis.margins(0.15)
    path_axis.grid(True, alpha=0.25)
    handles, labels = path_axis.get_legend_handles_labels()
    handles.extend(
        [
            Line2D([0], [0], color="#d62728", label="target direction"),
            Line2D([0], [0], color="#1f77b4", label="selected tangent"),
        ]
    )
    labels.extend(["target direction", "selected tangent"])
    path_axis.legend(handles, labels, fontsize=7, ncol=2, loc="lower left")

    force_axis.plot(elapsed, fxy, color="#e45756", linewidth=1.1, label="processed Fxy")
    force_axis.axhline(
        float(config["policy"]["contact_threshold"]),
        color="black",
        linestyle="--",
        linewidth=1.0,
        label="contact threshold",
    )
    force_axis.set_title("Processed planar force")
    force_axis.set_xlabel("Time [s]")
    force_axis.set_ylabel("Fxy [N]")
    force_axis.grid(True, alpha=0.25)
    force_axis.legend(fontsize=8)

    speed_axis.plot(elapsed, commanded_speed, color="#4c78a8", linewidth=1.0)
    contact = _number(samples, "contact_flag")
    speed_axis.fill_between(
        elapsed,
        0,
        max(0.1, float(np.max(commanded_speed))) * contact,
        color="#e45756",
        alpha=0.25,
        label="contact flag",
    )
    speed_axis.set_title("Policy command")
    speed_axis.set_xlabel("Time [s]")
    speed_axis.set_ylabel("Commanded XY speed [mm/s]")
    speed_axis.grid(True, alpha=0.25)
    speed_axis.legend(fontsize=8)

    state_axis.step(elapsed, state_index, where="post", color="#333333", linewidth=1.0)
    state_axis.scatter(
        elapsed,
        state_index,
        s=5,
        color=[state_colors[state] for state in states],
    )
    state_axis.set_yticks(range(len(state_order)), state_order)
    state_axis.set_ylim(-0.5, len(state_order) - 0.5)
    state_axis.set_title("Finite-state-machine timeline")
    state_axis.set_xlabel("Time [s]")
    state_axis.grid(True, axis="x", alpha=0.25)

    figure.suptitle(
        f"UR7e + PX6D contour policy visualization\n{run_dir.name}", fontsize=14
    )
    destination = output.expanduser().resolve() if output else run_dir / "visualization.png"
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=170)
    plt.close(figure)
    return destination


def create_strategy_debug(run_dir: Path) -> tuple[Path, list[Path]]:
    """Create real-run strategy and per-corner plots without fake ground truth."""
    run_dir = run_dir.expanduser().resolve()
    samples = _read_csv(run_dir / "samples.csv")
    boundaries = _read_csv(run_dir / "boundary_points.csv")
    waypoints = _read_csv(run_dir / "policy_waypoints.csv")
    recovery_path = run_dir / "boundary_recovery_rays.csv"
    rays = _read_csv(recovery_path if recovery_path.exists() else run_dir / "corner_search_rays.csv")
    if not samples:
        raise ValueError("cannot draw strategy debug without samples")
    with (run_dir / "config_snapshot.yaml").open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)

    trajectory = np.column_stack((_number(samples, "tcp_x"), _number(samples, "tcp_y")))
    figure, axis = plt.subplots(figsize=(11, 8), constrained_layout=True)
    axis.plot(trajectory[:, 0], trajectory[:, 1], color="#4c78a8", linewidth=0.8,
              alpha=0.75, label="actual TCP trajectory")
    if boundaries:
        boundary_xy = np.column_stack((_number(boundaries, "tcp_x"), _number(boundaries, "tcp_y")))
        axis.scatter(boundary_xy[:, 0], boundary_xy[:, 1], s=36, facecolor="white",
                     edgecolor="#d62728", label="confirmed boundary points", zorder=7)
        for row, point in zip(boundaries, boundary_xy):
            axis.annotate(row["point_id"], point, xytext=(2, 2), textcoords="offset points", fontsize=6)
    if waypoints:
        waypoint_xy = np.column_stack((_number(waypoints, "x"), _number(waypoints, "y")))
        axis.scatter(waypoint_xy[:, 0], waypoint_xy[:, 1], marker="+", s=24,
                     color="#17becf", alpha=0.7, label="policy waypoints")
        for row, point in zip(waypoints, waypoint_xy):
            if row["event_type"] in {"CORNER_PC", "P_CLEAR", "CORNER_ANCHOR", "REJECTED_CONTACT", "NEW_EDGE_CONFIRMED"}:
                axis.annotate(row["event_type"], point, xytext=(3, 3), textcoords="offset points", fontsize=7)
    for row in rays:
        start = np.asarray((float(row["anchor_x"]), float(row["anchor_y"])))
        direction = np.asarray((float(row["direction_x"]), float(row["direction_y"])))
        end = start + direction * float(row["planned_length"])
        result = row["result"]
        color = "#2ca02c" if result == "NEW_EDGE_CONFIRMED" else (
            "#d62728" if result.startswith(("REJECT", "CONFIRM_FAILED")) else "#8c8c8c"
        )
        style = "-" if result == "NEW_EDGE_CONFIRMED" else "--" if color == "#d62728" else ":"
        axis.plot([start[0], end[0]], [start[1], end[1]], color=color, linestyle=style,
                  linewidth=2.2 if result == "NEW_EDGE_CONFIRMED" else 1.1)
    start = trajectory[0]
    direction = np.asarray(config["policy"]["search_direction_xy"], dtype=float)
    direction /= np.linalg.norm(direction)
    arrow_length = max(0.01, float(config["policy"]["tangent_step"]) * 4)
    axis.arrow(*start, *(direction * arrow_length), color="#9467bd", width=arrow_length * 0.025,
               head_width=arrow_length * 0.2, length_includes_head=True, label="initial scan direction")
    axis.set_title(f"Real run scan strategy debug — no ground truth\n{run_dir.name}")
    axis.set_xlabel("Base X [m]")
    axis.set_ylabel("Base Y [m]")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.25)
    handles, labels = axis.get_legend_handles_labels()
    handles.extend([
        Line2D([0], [0], color="#8c8c8c", linestyle=":", label="attempted/no-contact fan ray"),
        Line2D([0], [0], color="#d62728", linestyle="--", label="rejected fan ray"),
        Line2D([0], [0], color="#2ca02c", linewidth=2.2, label="confirmed fan ray"),
    ])
    labels.extend(["attempted/no-contact fan ray", "rejected fan ray", "confirmed fan ray"])
    axis.legend(handles, labels, fontsize=8, ncol=2)
    strategy_path = run_dir / "scan_strategy_debug.png"
    figure.savefig(strategy_path, dpi=190)
    plt.close(figure)

    corner_paths = []
    corner_ids = sorted({int(row["corner_id"]) for row in waypoints if row["corner_id"] != ""})
    for corner_id in corner_ids:
        corner_waypoints = [row for row in waypoints if row["corner_id"] == str(corner_id)]
        corner_rays = [row for row in rays if row["corner_id"] == str(corner_id)]
        if not corner_waypoints:
            continue
        figure, axis = plt.subplots(figsize=(9, 8), constrained_layout=True)
        points = np.asarray([(float(row["x"]), float(row["y"])) for row in corner_waypoints])
        span = max(0.012, float(np.ptp(points[:, 0])), float(np.ptp(points[:, 1])))
        center = np.mean(points, axis=0)
        local = (
            (trajectory[:, 0] >= center[0] - span) & (trajectory[:, 0] <= center[0] + span)
            & (trajectory[:, 1] >= center[1] - span) & (trajectory[:, 1] <= center[1] + span)
        )
        axis.plot(trajectory[local, 0], trajectory[local, 1], color="#4c78a8", linewidth=0.8,
                  label="actual local trajectory")
        for row in corner_rays:
            anchor = np.asarray((float(row["anchor_x"]), float(row["anchor_y"])))
            direction = np.asarray((float(row["direction_x"]), float(row["direction_y"])))
            end = anchor + direction * float(row["planned_length"])
            result = row["result"]
            color = "#2ca02c" if result == "NEW_EDGE_CONFIRMED" else (
                "#d62728" if result.startswith(("REJECT", "CONFIRM_FAILED")) else "#8c8c8c"
            )
            axis.plot([anchor[0], end[0]], [anchor[1], end[1]], color=color,
                      linestyle="-" if result == "NEW_EDGE_CONFIRMED" else ":")
            axis.annotate(f"r{row['ray_index']} {float(row['theta_deg']):.0f}°\n{result}",
                          end, xytext=(3, 3), textcoords="offset points", fontsize=7, color=color)
            if row["contact_x"]:
                axis.scatter(float(row["contact_x"]), float(row["contact_y"]), marker="D",
                             s=48, color=color, zorder=8)
        marker_map = {"CORNER_PC": ("s", "#9467bd"), "P_CLEAR": ("^", "#ff7f0e"),
                      "CORNER_ANCHOR": ("*", "#ff00aa"), "REJECTED_CONTACT": ("x", "#d62728"),
                      "NEW_EDGE_CONFIRMED": ("P", "#2ca02c")}
        for row in corner_waypoints:
            if row["event_type"] in marker_map:
                marker, color = marker_map[row["event_type"]]
                point = np.asarray((float(row["x"]), float(row["y"])))
                axis.scatter(*point, marker=marker, s=120, color=color,
                             label=row["event_type"], zorder=9)
                if row["event_type"] == "CORNER_PC" and row["target_direction_x"]:
                    n = np.asarray((float(row["target_direction_x"]), float(row["target_direction_y"])))
                    t = np.asarray((float(row["tangent_x"]), float(row["tangent_y"])))
                    axis.arrow(*point, *(n * span * 0.3), color="#d62728", head_width=span * 0.04)
                    axis.arrow(*point, *(t * span * 0.3), color="#1f77b4", head_width=span * 0.04)
        margin = span * 0.45
        axis.set_xlim(float(np.min(points[:, 0]) - margin), float(np.max(points[:, 0]) + margin))
        axis.set_ylim(float(np.min(points[:, 1]) - margin), float(np.max(points[:, 1]) + margin))
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, alpha=0.25)
        axis.set_xlabel("Base X [m]")
        axis.set_ylabel("Base Y [m]")
        axis.set_title(f"Corner {corner_id}: fixed-anchor fan search — no ground truth")
        handles, labels = axis.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        axis.legend(unique.values(), unique.keys(), fontsize=8)
        path = run_dir / f"boundary_recovery_{corner_id}.png"
        figure.savefig(path, dpi=200)
        plt.close(figure)
        corner_paths.append(path)
    return strategy_path, corner_paths


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, help="run directory; default is latest")
    parser.add_argument("--output", type=Path, help="PNG destination")
    args = parser.parse_args()
    run_dir = args.run_dir or latest_run()
    destination = create_visualization(run_dir, args.output)
    print(f"可视化已生成：{destination}")
    strategy, corners = create_strategy_debug(run_dir)
    print(f"策略调试图已生成：{strategy}")
    for corner in corners:
        print(f"拐角调试图已生成：{corner}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

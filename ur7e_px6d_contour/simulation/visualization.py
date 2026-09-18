"""Live matplotlib animation and post-stop simulation result rendering."""

from __future__ import annotations

from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.widgets import Button


class SimulationView:
    def __init__(self, simulator):
        self.simulator = simulator
        self.config = simulator.config["visualization"]
        self.running = True
        self._finalized = False
        self.figure, self.axis = plt.subplots(figsize=(10, 8))
        self.figure.subplots_adjust(bottom=0.14, right=0.77)
        self._build_scene()
        self.figure.canvas.mpl_connect("key_press_event", self._on_key)
        self.figure.canvas.mpl_connect("close_event", self._on_close)
        self.animation = animation.FuncAnimation(
            self.figure,
            self._update,
            interval=float(self.config["interval_ms"]),
            blit=False,
            cache_frame_data=False,
        )

    def _build_scene(self) -> None:
        self.axis.clear()
        container = self.simulator.config["container"]
        self.axis.set_xlim(container["x_min"], container["x_max"])
        self.axis.set_ylim(container["y_min"], container["y_max"])
        self.axis.set_aspect("equal", adjustable="box")
        self.axis.grid(True, alpha=0.25)
        self.axis.set_xlabel("X [m]")
        self.axis.set_ylabel("Y [m]")
        self.axis.set_title("2D rule-policy simulator (not a physics simulation)")
        workspace_x = [container["x_min"], container["x_max"], container["x_max"], container["x_min"], container["x_min"]]
        workspace_y = [container["y_min"], container["y_min"], container["y_max"], container["y_max"], container["y_min"]]
        self.axis.plot(workspace_x, workspace_y, "--", color="#666666", linewidth=1.2, label="container/workspace")

        target = self.simulator.target.boundary_points()
        self.axis.fill(target[:, 0], target[:, 1], color="#bbbbbb", alpha=0.35, label="target")
        self.axis.plot(target[:, 0], target[:, 1], color="black", linewidth=2.0)
        start = np.asarray(self.simulator.config["start_point"], dtype=float)
        self.axis.scatter(*start, marker="*", s=140, color="black", label="start", zorder=7)
        scan = np.asarray(self.simulator.policy.search_direction)
        self.axis.arrow(
            start[0], start[1], scan[0] * 0.035, scan[1] * 0.035,
            width=0.0008, head_width=0.006, color="#9467bd", length_includes_head=True,
            label="initial scan direction",
        )

        (self.trajectory_line,) = self.axis.plot([], [], color="#4c78a8", linewidth=1.4, label="executed trajectory")
        self.boundary_scatter = self.axis.scatter([], [], s=34, color="#d62728", label="boundary points", zorder=6)
        self.probe_scatter = self.axis.scatter([], [], s=75, color="#ff7f0e", edgecolor="black", label="TCP", zorder=8)
        self.force_arrow = self.axis.quiver([0], [0], [0], [0], color="#e45756", angles="xy", scale_units="xy", scale=1, label="current force")
        self.normal_arrow = self.axis.quiver([0], [0], [0], [0], color="#d62728", angles="xy", scale_units="xy", scale=1, label="target direction")
        self.tangent_arrow = self.axis.quiver([0], [0], [0], [0], color="#1f77b4", angles="xy", scale_units="xy", scale=1, label="current tangent")
        self.attempted_rays = LineCollection([], colors="#aaaaaa", linewidths=1.0, alpha=0.55, label="attempted recovery rays")
        self.successful_rays = LineCollection([], colors="#2ca02c", linewidths=2.4, alpha=0.9, label="successful fan ray")
        self.rejected_rays = LineCollection([], colors="#d62728", linewidths=1.7, linestyles="--", label="rejected fan ray")
        self.current_ray = LineCollection([], colors="#ff00aa", linewidths=2.2, alpha=0.95, label="current recovery ray")
        self.axis.add_collection(self.attempted_rays)
        self.axis.add_collection(self.successful_rays)
        self.axis.add_collection(self.rejected_rays)
        self.axis.add_collection(self.current_ray)
        self.clear_scatter = self.axis.scatter([], [], marker="^", s=75, color="#ff7f0e", label="P_clear", zorder=9)
        self.anchor_scatter = self.axis.scatter([], [], marker="*", s=150, color="#ff00aa", label="P_anchor", zorder=9)
        self.waypoint_scatter = self.axis.scatter([], [], marker="+", s=24, color="#17becf", label="policy waypoints", zorder=5)
        self.candidate_scatter = self.axis.scatter([], [], marker="D", s=42, color="#2ca02c", label="candidate contact", zorder=8)
        self.rejected_scatter = self.axis.scatter([], [], marker="x", s=55, color="#d62728", label="rejected contact", zorder=8)
        self.value_text = self.axis.text(
            1.02,
            0.98,
            "",
            transform=self.axis.transAxes,
            va="top",
            ha="left",
            family="monospace",
            fontsize=10,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9},
        )
        self.help_text = self.axis.text(
            1.02,
            0.38,
            "SPACE  Pause/Continue\nQ      Normal stop\nESC    Emergency stop\nR      Reset",
            transform=self.axis.transAxes,
            va="top",
            fontsize=9,
        )
        self.axis.legend(loc="upper left", fontsize=8)
        self._build_buttons()

    def _build_buttons(self) -> None:
        specs = [
            ("Start", [0.16, 0.035, 0.13, 0.055], self._start),
            ("Pause", [0.31, 0.035, 0.13, 0.055], self._pause),
            ("Stop", [0.46, 0.035, 0.13, 0.055], self._stop),
            ("Reset", [0.61, 0.035, 0.13, 0.055], self._reset),
        ]
        self.buttons = []
        for label, rectangle, callback in specs:
            button = Button(self.figure.add_axes(rectangle), label)
            button.on_clicked(callback)
            self.buttons.append(button)

    def _start(self, _event=None) -> None:
        if not self.simulator.stopped:
            self.running = True

    def _pause(self, _event=None) -> None:
        self.running = not self.running if not self.simulator.stopped else False

    def _stop(self, _event=None) -> None:
        self.simulator.normal_stop("Q/Stop normal stop")
        self.running = False
        self._finalize()

    def _reset(self, _event=None) -> None:
        self.simulator.reset()
        self.running = True
        self._finalized = False
        self._update_artists(self.simulator.snapshot())

    def _on_key(self, event) -> None:
        if event.key == " ":
            self._pause()
        elif event.key and event.key.lower() == "q":
            self._stop()
        elif event.key == "escape":
            self.simulator.emergency_stop()
            self.running = False
            self._finalize()
            plt.close(self.figure)
        elif event.key and event.key.lower() == "r":
            self._reset()

    def _on_close(self, _event) -> None:
        if not self.simulator.stopped:
            self.simulator.normal_stop("animation window closed")
        self._finalize()

    def _finalize(self) -> None:
        if self._finalized:
            return
        output = self.simulator.finalize()
        self._finalized = True
        self.axis.set_title(f"STOPPED — results saved to\n{output}")
        print(f"\n模拟结果已保存：{output}")
        self.figure.canvas.draw_idle()

    def _update(self, _frame):
        if self.running and not self.simulator.stopped:
            snapshot = self.simulator.step()
        else:
            snapshot = self.simulator.snapshot()
        if self.simulator.stopped:
            self.running = False
            self._finalize()
        return self._update_artists(snapshot)

    def _update_artists(self, snapshot):
        trajectory = np.asarray(self.simulator.trajectory)
        max_points = int(self.config["trail_max_points"])
        trajectory = trajectory[-max_points:]
        self.trajectory_line.set_data(trajectory[:, 0], trajectory[:, 1])
        position = np.asarray(snapshot["position"])
        self.probe_scatter.set_offsets(position.reshape(1, 2))
        boundaries = np.asarray(snapshot["boundary_points"])
        self.boundary_scatter.set_offsets(
            boundaries.reshape(-1, 2) if boundaries.size else np.empty((0, 2))
        )

        force_scale = float(self.config["force_arrow_scale_m_per_n"])
        force = np.asarray(snapshot["force"]) * force_scale
        self.force_arrow.set_offsets(position.reshape(1, 2))
        self.force_arrow.set_UVC([force[0]], [force[1]])
        vector_length = float(self.config["vector_display_length"])
        for arrow, vector in (
            (self.normal_arrow, snapshot["normal"]),
            (self.tangent_arrow, snapshot["tangent"]),
        ):
            value = np.zeros(2) if vector is None else np.asarray(vector) * vector_length
            arrow.set_offsets(position.reshape(1, 2))
            arrow.set_UVC([value[0]], [value[1]])

        anchor = snapshot["corner_anchor"]
        self.anchor_scatter.set_offsets(
            np.empty((0, 2)) if anchor is None else np.asarray(anchor).reshape(1, 2)
        )
        clear = snapshot["recovery_clear"]
        self.clear_scatter.set_offsets(
            np.empty((0, 2)) if clear is None else np.asarray(clear).reshape(1, 2)
        )
        ray_groups = {"attempted": [], "success": [], "rejected": []}
        for ray in snapshot["corner_rays"]:
            start = np.asarray(ray.anchor_pose[:2])
            segment = [start, start + np.asarray(ray.direction_xy) * ray.planned_length]
            if ray.result == "NEW_EDGE_CONFIRMED":
                ray_groups["success"].append(segment)
            elif ray.result.startswith("REJECT") or ray.result.startswith("CONFIRM_FAILED"):
                ray_groups["rejected"].append(segment)
            else:
                ray_groups["attempted"].append(segment)
        self.attempted_rays.set_segments(ray_groups["attempted"])
        self.successful_rays.set_segments(ray_groups["success"])
        self.rejected_rays.set_segments(ray_groups["rejected"])
        current = snapshot["current_corner_ray"]
        self.current_ray.set_segments(
            [] if current is None else [[
                np.asarray(current["anchor_pose"][:2]),
                np.asarray(current["anchor_pose"][:2])
                + np.asarray(current["direction_xy"]) * current["planned_length"],
            ]]
        )
        waypoints = snapshot["policy_waypoints"]
        waypoint_xy = np.asarray([item.pose[:2] for item in waypoints])
        self.waypoint_scatter.set_offsets(waypoint_xy if waypoint_xy.size else np.empty((0, 2)))
        candidates = np.asarray([
            item.pose[:2] for item in waypoints
            if item.event_type in {"CANDIDATE_CONTACT", "CONFIRM_CONTACT", "NEW_EDGE_CONFIRMED"}
        ])
        rejected = np.asarray([item.pose[:2] for item in waypoints if item.event_type == "REJECTED_CONTACT"])
        self.candidate_scatter.set_offsets(candidates.reshape(-1, 2) if candidates.size else np.empty((0, 2)))
        self.rejected_scatter.set_offsets(rejected.reshape(-1, 2) if rejected.size else np.empty((0, 2)))

        self.value_text.set_text(
            f"state: {snapshot['state']}\n"
            f"sub:   {snapshot['policy_sub_state'][:20]}\n"
            f"hand:  {snapshot['follow_hand']}\n"
            f"time:  {snapshot['time']:7.2f} s\n"
            f"TCP x: {position[0]:+8.4f} m\n"
            f"TCP y: {position[1]:+8.4f} m\n"
            f"Fx:    {snapshot['force'][0]:+8.3f} N\n"
            f"Fy:    {snapshot['force'][1]:+8.3f} N\n"
            f"Fxy:   {snapshot['fxy']:8.3f} N\n"
            f"angle: {np.degrees(snapshot['force_angle']):+8.2f} deg\n"
            f"theta: {snapshot['corner_theta_deg']:+8.1f} deg\n"
            f"points:{len(snapshot['boundary_points']):8d}\n"
            f"reason: {snapshot['reason'][:30]}"
        )
        return (
            self.trajectory_line,
            self.boundary_scatter,
            self.probe_scatter,
            self.force_arrow,
            self.normal_arrow,
            self.tangent_arrow,
            self.attempted_rays,
            self.successful_rays,
            self.rejected_rays,
            self.current_ray,
            self.clear_scatter,
            self.anchor_scatter,
            self.waypoint_scatter,
            self.candidate_scatter,
            self.rejected_scatter,
            self.value_text,
        )

    def show(self) -> None:
        plt.show()


def save_result_figure(simulator, destination: Path) -> None:
    figure, axis = plt.subplots(figsize=(9, 7), constrained_layout=True)
    target = simulator.target.boundary_points()
    trajectory = np.asarray(simulator.trajectory)
    boundaries = np.asarray([point.pose[:2] for point in simulator.policy.boundary_points])
    axis.fill(target[:, 0], target[:, 1], color="#bbbbbb", alpha=0.25)
    axis.plot(target[:, 0], target[:, 1], color="black", linewidth=2.0, label="target ground truth")
    axis.plot(trajectory[:, 0], trajectory[:, 1], color="#4c78a8", linewidth=1.0, alpha=0.8, label="complete TCP trajectory")
    if boundaries.size:
        axis.scatter(boundaries[:, 0], boundaries[:, 1], s=28, color="#d62728", label="detected boundary points", zorder=5)
        axis.plot(boundaries[:, 0], boundaries[:, 1], color="#f28e2b", linewidth=1.4, label="estimated contour (ordered points)")
        label_stride = max(1, len(boundaries) // 24)
        for point, value in zip(simulator.policy.boundary_points, boundaries):
            if point.index % label_stride == 0 or point.index == len(boundaries) - 1:
                axis.annotate(
                    f"P{point.index}", value, xytext=(3, 3), textcoords="offset points", fontsize=7
                )
    error_mm = simulator.boundary_error_mean() * 1000.0
    axis.set_title(
        f"{simulator.config['scene_name']} — contour result\n"
        f"mean boundary distance = {error_mm:.3f} mm; stop = {simulator.stop_reason}"
    )
    axis.set_xlabel("X [m]")
    axis.set_ylabel("Y [m]")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.25)
    axis.legend(fontsize=8)
    container = simulator.config["container"]
    axis.set_xlim(container["x_min"], container["x_max"])
    axis.set_ylim(container["y_min"], container["y_max"])
    figure.savefig(destination, dpi=180)
    plt.close(figure)


def _ray_style(result: str) -> tuple[str, str, float]:
    if result == "NEW_EDGE_CONFIRMED":
        return "#2ca02c", "-", 2.4
    if result.startswith("REJECT") or result.startswith("CONFIRM_FAILED"):
        return "#d62728", "--", 1.7
    if result == "SAFETY_ABORT":
        return "#000000", "-", 2.5
    return "#8c8c8c", ":", 1.1


def save_scan_strategy_debug(simulator, destination: Path) -> Path:
    """Render every policy point and fixed-anchor fan ray with ground truth."""
    figure, axis = plt.subplots(figsize=(11, 8), constrained_layout=True)
    target = simulator.target.boundary_points()
    trajectory = np.asarray(simulator.trajectory)
    axis.fill(target[:, 0], target[:, 1], color="#bbbbbb", alpha=0.20)
    axis.plot(target[:, 0], target[:, 1], color="black", linewidth=2, label="ground truth target")
    axis.plot(trajectory[:, 0], trajectory[:, 1], color="#4c78a8", linewidth=0.8,
              alpha=0.75, label="actual TCP trajectory")
    boundaries = np.asarray([point.pose[:2] for point in simulator.policy.boundary_points])
    if boundaries.size:
        axis.scatter(boundaries[:, 0], boundaries[:, 1], s=34, facecolor="white",
                     edgecolor="#d62728", linewidth=1.3, label="confirmed boundary points", zorder=7)
        for point in simulator.policy.boundary_points:
            axis.annotate(f"P{point.index}", point.pose[:2], xytext=(2, 2),
                          textcoords="offset points", fontsize=6)
    waypoints = simulator.policy.policy_waypoints
    if waypoints:
        points = np.asarray([item.pose[:2] for item in waypoints])
        axis.scatter(points[:, 0], points[:, 1], marker="+", s=22, color="#17becf",
                     alpha=0.7, label="policy waypoints", zorder=5)
    for ray in simulator.policy.boundary_recovery_rays:
        start = ray.anchor_pose[:2]
        planned_end = start + ray.direction_xy * ray.planned_length
        color, style, width = _ray_style(ray.result)
        axis.plot([start[0], planned_end[0]], [start[1], planned_end[1]],
                  color=color, linestyle=style, linewidth=width, alpha=0.9)
        actual_end = start + ray.direction_xy * ray.actual_length
        axis.scatter(*actual_end, s=15, color=color, zorder=6)
    for record in simulator.policy.recovery_records:
        pc, clear, anchor = record["pc"][:2], record["p_clear"][:2], record["p_anchor"][:2]
        axis.scatter(*pc, marker="s", s=65, color="#9467bd", zorder=8)
        axis.scatter(*clear, marker="^", s=65, color="#ff7f0e", zorder=8)
        axis.scatter(*anchor, marker="*", s=155, color="#ff00aa", zorder=9)
        axis.annotate(f"C{record['corner_id']} Pc", pc, xytext=(4, 4), textcoords="offset points")
    rejected = [item.pose[:2] for item in waypoints if item.event_type == "REJECTED_CONTACT"]
    if rejected:
        rejected = np.asarray(rejected)
        axis.scatter(rejected[:, 0], rejected[:, 1], marker="x", s=65,
                     color="#d62728", label="rejected contact", zorder=9)
    start = np.asarray(simulator.config["start_point"])
    direction = np.asarray(simulator.config["scan_direction_xy"])
    length = float(simulator.config["visualization"]["vector_display_length"]) * 2
    axis.arrow(*start, *(direction * length), width=0.0005, head_width=0.004,
               color="#9467bd", length_includes_head=True, label="initial scan direction")
    axis.set_title(f"{simulator.config['scene_name']} — complete scan strategy debug")
    axis.set_xlabel("X [m]")
    axis.set_ylabel("Y [m]")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.25)
    handles, labels = axis.get_legend_handles_labels()
    handles.extend([
        Line2D([0], [0], color="#8c8c8c", linestyle=":", label="attempted/no-contact fan ray"),
        Line2D([0], [0], color="#d62728", linestyle="--", label="rejected fan ray"),
        Line2D([0], [0], color="#2ca02c", linewidth=2.4, label="confirmed fan ray"),
        Line2D([0], [0], marker="*", color="none", markerfacecolor="#ff00aa",
               markersize=12, label="fixed P_anchor"),
    ])
    labels.extend(["attempted/no-contact fan ray", "rejected fan ray",
                   "confirmed fan ray", "fixed P_anchor"])
    axis.legend(handles, labels, fontsize=8, ncol=2)
    container = simulator.config["container"]
    axis.set_xlim(container["x_min"], container["x_max"])
    axis.set_ylim(container["y_min"], container["y_max"])
    figure.savefig(destination, dpi=190)
    plt.close(figure)
    return destination


def save_boundary_recovery_debug_figures(simulator, output_dir: Path) -> list[Path]:
    """Create one debug proof of fixed-anchor geometry per recovery."""
    destinations = []
    target_boundary = simulator.target.boundary_points()
    trajectory = np.asarray(simulator.trajectory)
    for record in simulator.policy.recovery_records:
        corner_id = int(record["corner_id"])
        rays = [ray for ray in simulator.policy.boundary_recovery_rays if ray.recovery_id == corner_id]
        figure, axis = plt.subplots(figsize=(9, 8), constrained_layout=True)
        axis.fill(target_boundary[:, 0], target_boundary[:, 1], color="#bbbbbb", alpha=0.18)
        axis.plot(target_boundary[:, 0], target_boundary[:, 1], color="black", linewidth=1.5,
                  label="ground truth target")
        pc, clear, anchor = record["pc"][:2], record["p_clear"][:2], record["p_anchor"][:2]
        relevant = [pc, clear, anchor]
        for ray in rays:
            planned_end = ray.anchor_pose[:2] + ray.direction_xy * ray.planned_length
            actual_end = ray.anchor_pose[:2] + ray.direction_xy * ray.actual_length
            relevant.extend((planned_end, actual_end))
            color, style, width = _ray_style(ray.result)
            axis.plot([ray.anchor_pose[0], planned_end[0]],
                      [ray.anchor_pose[1], planned_end[1]], color=color,
                      linestyle=style, linewidth=width)
            axis.scatter(*actual_end, s=24, color=color, zorder=7)
            axis.annotate(
                f"r{ray.ray_index} {ray.theta_deg:.0f}°\n{ray.result}", planned_end,
                xytext=(3, 3), textcoords="offset points", fontsize=7, color=color,
            )
            if ray.contact_pose is not None:
                axis.scatter(*ray.contact_pose[:2], marker="D", s=48, color=color, zorder=9)
                relevant.append(ray.contact_pose[:2])
        relevant_array = np.asarray(relevant)
        span = max(0.012, float(np.ptp(relevant_array[:, 0])), float(np.ptp(relevant_array[:, 1])))
        center = np.mean(relevant_array, axis=0)
        local = (
            (trajectory[:, 0] >= center[0] - span) & (trajectory[:, 0] <= center[0] + span)
            & (trajectory[:, 1] >= center[1] - span) & (trajectory[:, 1] <= center[1] + span)
        )
        if np.any(local):
            axis.plot(trajectory[local, 0], trajectory[local, 1], color="#4c78a8",
                      linewidth=0.8, alpha=0.7, label="actual local trajectory")
        axis.scatter(*pc, marker="s", s=80, color="#9467bd", label="Pc", zorder=10)
        axis.scatter(*clear, marker="^", s=80, color="#ff7f0e", label="P_clear", zorder=10)
        axis.scatter(*anchor, marker="*", s=180, color="#ff00aa", label="P_anchor", zorder=10)
        vector_length = span * 0.35
        old_target, old_tangent = record["old_target_direction"], record["old_tangent"]
        axis.arrow(*pc, *(old_target * vector_length), color="#d62728", width=span * 0.006,
                   head_width=span * 0.04, length_includes_head=True)
        axis.arrow(*pc, *(old_tangent * vector_length), color="#1f77b4", width=span * 0.006,
                   head_width=span * 0.04, length_includes_head=True)
        axis.annotate("n_old target", pc + old_target * vector_length, fontsize=8)
        axis.annotate("t_old", pc + old_tangent * vector_length, fontsize=8)
        confirmed = record.get("confirmed_contact")
        if confirmed is not None:
            axis.scatter(*confirmed[:2], marker="P", s=110, color="#2ca02c",
                         label="NEW EDGE CONFIRMED", zorder=11)
            relevant_array = np.vstack((relevant_array, confirmed[:2]))
        margin = span * 0.35
        axis.set_xlim(float(np.min(relevant_array[:, 0]) - margin), float(np.max(relevant_array[:, 0]) + margin))
        axis.set_ylim(float(np.min(relevant_array[:, 1]) - margin), float(np.max(relevant_array[:, 1]) + margin))
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, alpha=0.25)
        axis.set_xlabel("X [m]")
        axis.set_ylabel("Y [m]")
        axis.set_title(f"Corner {corner_id}: fixed P_anchor, independent monotonic fan rays")
        axis.legend(fontsize=8, loc="best")
        destination = output_dir / f"boundary_recovery_{corner_id}.png"
        figure.savefig(destination, dpi=200)
        plt.close(figure)
        destinations.append(destination)
    return destinations


def save_corner_debug_figures(simulator, output_dir: Path) -> list[Path]:
    """Compatibility alias for external scripts using the old function name."""
    return save_boundary_recovery_debug_figures(simulator, output_dir)


def save_animation_replay(simulator, output_dir: Path) -> Path:
    stride = max(1, int(simulator.config["visualization"]["animation_stride"]))
    frames = simulator.history[::stride]
    if not frames:
        raise ValueError("no simulation frames available for animation")
    figure, axis = plt.subplots(figsize=(8, 6))
    target = simulator.target.boundary_points()
    axis.fill(target[:, 0], target[:, 1], color="#bbbbbb", alpha=0.3)
    axis.plot(target[:, 0], target[:, 1], color="black", linewidth=2.0)
    (trail,) = axis.plot([], [], color="#4c78a8", linewidth=1.2)
    probe = axis.scatter([], [], s=65, color="#ff7f0e", edgecolor="black", zorder=5)
    points = axis.scatter([], [], s=28, color="#d62728", zorder=4)
    anchor_marker = axis.scatter([], [], marker="*", s=130, color="#ff00aa", zorder=7)
    attempted_fan = LineCollection([], colors="#8c8c8c", linewidths=1.0, linestyles=":")
    successful_fan = LineCollection([], colors="#2ca02c", linewidths=2.2)
    rejected_fan = LineCollection([], colors="#d62728", linewidths=1.6, linestyles="--")
    current_fan = LineCollection([], colors="#ff00aa", linewidths=2.0)
    axis.add_collection(attempted_fan)
    axis.add_collection(successful_fan)
    axis.add_collection(rejected_fan)
    axis.add_collection(current_fan)
    candidate_points = axis.scatter([], [], marker="D", s=40, color="#2ca02c", zorder=7)
    rejected_points = axis.scatter([], [], marker="x", s=52, color="#d62728", zorder=8)
    label = axis.text(0.02, 0.98, "", transform=axis.transAxes, va="top", family="monospace")
    container = simulator.config["container"]
    axis.set_xlim(container["x_min"], container["x_max"])
    axis.set_ylim(container["y_min"], container["y_max"])
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("X [m]")
    axis.set_ylabel("Y [m]")
    axis.grid(True, alpha=0.25)

    positions = np.asarray([frame["position"] for frame in frames])

    def update(index):
        frame = frames[index]
        trail.set_data(positions[: index + 1, 0], positions[: index + 1, 1])
        probe.set_offsets(np.asarray(frame["position"]).reshape(1, 2))
        boundary = np.asarray(frame["boundary_points"])
        points.set_offsets(boundary.reshape(-1, 2) if boundary.size else np.empty((0, 2)))
        anchor = frame["corner_anchor"]
        anchor_marker.set_offsets(np.empty((0, 2)) if anchor is None else np.asarray(anchor).reshape(1, 2))
        attempted, successful, rejected = [], [], []
        for ray in frame["corner_rays"]:
            start = ray.anchor_pose[:2]
            segment = [start, start + ray.direction_xy * ray.planned_length]
            if ray.result == "NEW_EDGE_CONFIRMED":
                successful.append(segment)
            elif ray.result.startswith(("REJECT", "CONFIRM_FAILED")):
                rejected.append(segment)
            else:
                attempted.append(segment)
        attempted_fan.set_segments(attempted)
        successful_fan.set_segments(successful)
        rejected_fan.set_segments(rejected)
        current = frame["current_corner_ray"]
        current_fan.set_segments([] if current is None else [[
            current["anchor_pose"][:2],
            current["anchor_pose"][:2] + current["direction_xy"] * current["planned_length"],
        ]])
        waypoints = frame["policy_waypoints"]
        candidates = np.asarray([
            item.pose[:2] for item in waypoints
            if item.event_type in {"CANDIDATE_CONTACT", "CONFIRM_CONTACT", "NEW_EDGE_CONFIRMED"}
        ])
        rejected_contacts = np.asarray([
            item.pose[:2] for item in waypoints if item.event_type == "REJECTED_CONTACT"
        ])
        candidate_points.set_offsets(
            candidates.reshape(-1, 2) if candidates.size else np.empty((0, 2))
        )
        rejected_points.set_offsets(
            rejected_contacts.reshape(-1, 2) if rejected_contacts.size else np.empty((0, 2))
        )
        label.set_text(
            f"t={frame['time']:.2f} s\nstate={frame['state']}\n"
            f"Fxy={frame['fxy']:.3f} N\ntheta={frame['corner_theta_deg']:.1f} deg"
        )
        return (trail, probe, points, anchor_marker, attempted_fan, successful_fan,
                rejected_fan, current_fan, candidate_points, rejected_points, label)

    replay = animation.FuncAnimation(figure, update, frames=len(frames), blit=False)
    fps = int(simulator.config["visualization"]["animation_fps"])
    if animation.writers.is_available("ffmpeg"):
        destination = output_dir / "simulation.mp4"
        try:
            replay.save(destination, writer="ffmpeg", fps=fps, dpi=120)
            plt.close(figure)
            return destination
        except Exception:
            pass
    destination = output_dir / "simulation.gif"
    replay.save(destination, writer=animation.PillowWriter(fps=fps), dpi=90)
    plt.close(figure)
    return destination

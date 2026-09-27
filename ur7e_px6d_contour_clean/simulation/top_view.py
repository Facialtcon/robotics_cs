"""Single XY view of recorded TCP motion, accepted contacts, and the object."""

from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np


class TopViewPlot:
    """Render observations only; no generated contour or motion planning."""

    def __init__(self, simulator):
        self.simulator = simulator
        self.figure, self.axis = plt.subplots(figsize=(8, 7), layout="constrained")
        boundary = simulator.target.boundary_points() * 1000
        self.object_fill = self.axis.fill(boundary[:, 0], boundary[:, 1], color="0.85", label="Target boundary (draggable)")[0]
        self.object_outline, = self.axis.plot(boundary[:, 0], boundary[:, 1], color="0.3", linewidth=1.5)
        self.search_line, = self.axis.plot([], [], color="#666666", linestyle=":", linewidth=1.0, label="P0→P1 search line")
        self.search_points = self.axis.scatter([], [], marker="s", s=22, color="#666666", zorder=4)
        self.trajectory, = self.axis.plot([], [], color="#2271b2", linewidth=1.1, label="Executed TCP trajectory")
        self.points = self.axis.scatter([], [], s=10, color="#008855", label="Accepted contacts", zorder=4)
        self.first = self.axis.scatter([], [], marker="*", s=160, color="#e6b800", edgecolor="black", label="First acquired contact", zorder=6)
        self.tcp = self.axis.scatter([], [], s=65, color="#d64b30", edgecolor="white", label="TCP / final position", zorder=7)
        self.anchor = self.axis.scatter([], [], marker="+", s=70, color="#aa4499", zorder=5)
        self.anchors = self.axis.scatter([], [], marker="+", s=15, color="#aa4499", label="Probe anchors", zorder=3)
        self.candidates = self.axis.scatter([], [], marker="x", s=35, color="#cc6600", label="Recovery contacts", zorder=6)
        self.probes = LineCollection([], colors="#aa4499", linewidths=0.6, alpha=0.45, label="Executed probe segments")
        self.axis.add_collection(self.probes)
        self.tangent_arrow = self.axis.annotate("", xy=(0, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="#008855", lw=2))
        self.probe_arrow = self.axis.annotate("", xy=(0, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="#aa4499", lw=2))
        self.vector_label = self.axis.text(0.02, 0.98, "Green arrow: estimated tangent\nPurple arrow: probe direction", transform=self.axis.transAxes, va="top", fontsize=8)
        self.shape_buttons = {
            "square": self.axis.text(0.57, 0.98, "1  SQUARE", transform=self.axis.transAxes, va="top", fontsize=8, picker=True, bbox=dict(boxstyle="round", pad=0.25)),
            "circle": self.axis.text(0.70, 0.98, "2  CIRCLE", transform=self.axis.transAxes, va="top", fontsize=8, picker=True, bbox=dict(boxstyle="round", pad=0.25)),
            "triangle": self.axis.text(0.82, 0.98, "3  TRIANGLE", transform=self.axis.transAxes, va="top", fontsize=8, picker=True, bbox=dict(boxstyle="round", pad=0.25)),
        }
        self.rays = LineCollection([], colors="#aa4499", linewidths=0.8, linestyles="--", alpha=0.5)
        self.current_ray = LineCollection([], colors="#aa4499", linewidths=1.6)
        self.axis.add_collection(self.rays)
        self.axis.add_collection(self.current_ray)
        workspace = simulator.config["container"]
        self.axis.set_xlim(workspace["x_min"] * 1000, workspace["x_max"] * 1000)
        self.axis.set_ylim(workspace["y_min"] * 1000, workspace["y_max"] * 1000)
        self.axis.set_aspect("equal", adjustable="box")
        self.axis.set_xlabel("X [mm]")
        self.axis.set_ylabel("Y [mm]")
        self.axis.legend(loc="lower left", fontsize=8)
        self.refresh_environment()

    def refresh_environment(self, preview_center=None):
        boundary = self.simulator.target.boundary_points()
        if preview_center is not None:
            boundary = boundary + np.asarray(preview_center) - self.simulator.target.center
        boundary_mm = boundary * 1000
        self.object_fill.set_xy(boundary_mm)
        self.object_outline.set_data(boundary_mm[:, 0], boundary_mm[:, 1])
        calibration = np.asarray([
            self.simulator.config["calibration_point_0"],
            self.simulator.config["calibration_point_1"],
        ]) * 1000
        direction = calibration[1] - calibration[0]
        norm = np.linalg.norm(direction)
        if norm > 0:
            end = calibration[0] + direction / norm * float(self.simulator.config["policy"]["max_search_distance"]) * 1000
            self.search_line.set_data([calibration[0, 0], end[0]], [calibration[0, 1], end[1]])
        self.search_points.set_offsets(calibration)
        for shape, label in self.shape_buttons.items():
            active = shape == self.simulator.target_shape
            label.set_color("white" if active else "black")
            label.get_bbox_patch().set_facecolor("#2271b2" if active else "#eeeeee")

    def draw(self, snapshot, trajectory, *, finished=False):
        xy = np.asarray(trajectory).reshape(-1, 2) * 1000
        self.trajectory.set_data(xy[:, 0], xy[:, 1])
        points = np.asarray(snapshot["boundary_points"]).reshape(-1, 2) * 1000
        self.points.set_offsets(points)
        first_contact = snapshot.get("first_contact")
        self.first.set_offsets(np.empty((0, 2)) if first_contact is None else np.asarray(first_contact).reshape(1, 2) * 1000)
        self.tcp.set_offsets(np.asarray(snapshot["position"]).reshape(1, 2) * 1000)
        active = snapshot["state"] in {"BOUNDARY_RECOVERY", "BOUNDARY_CONFIRMATION"} and not finished
        anchor = snapshot["corner_anchor"] if active else snapshot.get("probe_anchor")
        self.anchor.set_offsets(np.empty((0, 2)) if anchor is None else np.asarray(anchor).reshape(1, 2) * 1000)
        self.anchors.set_offsets(np.asarray(snapshot["probe_anchors"]).reshape(-1, 2) * 1000)
        self.probes.set_segments([np.asarray(segment) * 1000 for segment in snapshot["probe_segments"]])
        # Retain every observed recovery contact after recovery and in exported
        # results. Read the snapshot so animation replay cannot show future rays.
        recovery_contacts = [ray.contact_pose[:2] for ray in snapshot["corner_rays"]
                             if ray.contact_pose is not None]
        self.candidates.set_offsets(np.asarray(recovery_contacts).reshape(-1, 2) * 1000)
        origin = np.asarray(snapshot["position"]) * 1000
        for arrow, vector in ((self.tangent_arrow, snapshot["tangent"]),
                              (self.probe_arrow, snapshot.get("probe_direction") if snapshot.get("probe_direction") is not None else snapshot["normal"])):
            arrow.set_visible(vector is not None)
            if vector is not None:
                arrow.set_position(origin)
                arrow.xy = origin + np.asarray(vector) * 12
        segments = []
        if active and snapshot["corner_records"]:
            recovery_id = snapshot["corner_records"][-1]["corner_id"]
            for ray in snapshot["corner_rays"]:
                if ray.recovery_id == recovery_id:
                    segments.append(np.asarray([ray.anchor_pose[:2], ray.ray_end_xy]) * 1000)
        self.rays.set_segments(segments)
        current = snapshot["current_corner_ray"] if active else None
        self.current_ray.set_segments([] if current is None else [np.asarray([
            current["anchor_pose"][:2],
            current["anchor_pose"][:2] + current["direction_xy"] * current["planned_length"],
        ]) * 1000])
        if finished:
            success = self.simulator.loop_completed
            title = "FULL LOOP COMPLETED — SUCCESS" if success else "LOOP FAILED — FAILED"
            if self.simulator.physical_failure:
                title = "PHYSICAL FAILURE"
            self.axis.set_title(title + f"\nFinal policy state: {self.simulator.final_policy_state}", color="#008855" if success else "#b22222")
        else:
            self.axis.set_title(f"{self.simulator.target_shape.upper()}  |  {snapshot['state']} / {snapshot['policy_sub_state']}  |  {snapshot['time']:.1f} s")
        return (self.trajectory, self.points, self.candidates, self.first, self.tcp, self.anchor, self.rays, self.current_ray)


class SimulationView:
    """Keyboard animation with exactly one axes and no diagnostic panels."""

    def __init__(self, simulator):
        self.simulator = simulator
        self.plot = TopViewPlot(simulator)
        self.running = True
        self._running_before_drag = True
        self._dragging = False
        self._drag_start = None
        self._object_start = None
        self._shift_down = False
        self.plot.figure.canvas.mpl_connect("key_press_event", self._on_key)
        self.plot.figure.canvas.mpl_connect("key_release_event", self._on_key_release)
        self.plot.figure.canvas.mpl_connect("button_press_event", self._on_press)
        self.plot.figure.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.plot.figure.canvas.mpl_connect("button_release_event", self._on_release)
        self.plot.figure.canvas.mpl_connect("close_event", self._on_close)
        self.plot.figure.text(
            0.5, 0.005,
            "1 square  2 circle  3 triangle  |  drag object  |  Shift+drag object & P0/P1  |  SPACE pause  R restart  Q save/stop  ESC close",
            ha="center", fontsize=8,
        )
        self.animation = animation.FuncAnimation(
            self.plot.figure, self._update,
            interval=float(simulator.config["visualization"]["interval_ms"]),
            cache_frame_data=False,
        )

    def _update(self, _frame):
        if self.running and not self.simulator.stopped:
            self.simulator.step()
        if self.simulator.stopped and not self.simulator.finalized:
            self.simulator.finalize()
        return self.plot.draw(self.simulator.snapshot(), self.simulator.trajectory, finished=self.simulator.stopped)

    def _on_key(self, event):
        key = (event.key or "").lower()
        if key == " ":
            self.running = not self.running
        elif key in {"q", "escape"}:
            self.simulator.normal_stop("manual emergency stop" if key == "escape" else "Q operator stop before full loop")
            self._update(None)
            if key == "escape":
                plt.close(self.plot.figure)
        elif key == "r":
            if self.simulator.records and not self.simulator.finalized:
                self.simulator.normal_stop("operator restarted before full loop")
                self.simulator.finalize()
            self.simulator.reset()
            self.plot.refresh_environment()
            self.running = True
        elif key in {"1", "2", "3"}:
            self._restart_with_shape({"1": "square", "2": "circle", "3": "triangle"}[key])
        elif key == "shift":
            self._shift_down = True
        self.plot.figure.canvas.draw_idle()

    def _on_key_release(self, event):
        if (event.key or "").lower() == "shift":
            self._shift_down = False

    def _restart_with_shape(self, shape, center=None, translate_search=False):
        self.simulator.set_interactive_target(shape, center, translate_search=translate_search)
        self.plot.refresh_environment()
        self.running = True

    def _on_press(self, event):
        if event.button != 1:
            return
        for shape, label in self.plot.shape_buttons.items():
            if label.contains(event)[0]:
                self._restart_with_shape(shape)
                self.plot.figure.canvas.draw_idle()
                return
        if event.inaxes is not self.plot.axis or event.xdata is None:
            return
        point = np.asarray((event.xdata, event.ydata), dtype=float) / 1000.0
        distance, _ = self.simulator.target.signed_distance_and_outward_normal(point)
        if distance > 0.005:
            return
        self._dragging = True
        self._drag_start = point
        self._object_start = self.simulator.target.center.copy()
        self._running_before_drag = self.running
        self.running = False
        self._drag_search = self._shift_down or "shift" in str(event.key).lower()

    def _on_motion(self, event):
        if not self._dragging or event.inaxes is not self.plot.axis or event.xdata is None:
            return
        point = np.asarray((event.xdata, event.ydata), dtype=float) / 1000.0
        self.plot.refresh_environment(self._object_start + point - self._drag_start)
        self.plot.figure.canvas.draw_idle()

    def _on_release(self, event):
        if not self._dragging:
            return
        self._dragging = False
        if event.inaxes is self.plot.axis and event.xdata is not None:
            point = np.asarray((event.xdata, event.ydata), dtype=float) / 1000.0
            center = self._object_start + point - self._drag_start
            self._restart_with_shape(self.simulator.target_shape, center, self._drag_search)
        else:
            self.plot.refresh_environment()
            self.running = self._running_before_drag

    def _on_close(self, _event):
        if not self.simulator.finalized:
            self.simulator.normal_stop("animation window closed before full loop")
            self.simulator.finalize()

    def show(self):
        plt.show()


def save_result_figure(simulator, destination: Path) -> None:
    plot = TopViewPlot(simulator)
    plot.draw(simulator.snapshot(), simulator.trajectory, finished=True)
    plot.figure.savefig(destination, dpi=180)
    plt.close(plot.figure)


def save_animation_replay(simulator, output_dir: Path) -> Path:
    """Replay recorded observations on the same single XY plot."""
    stride = max(1, int(simulator.config["visualization"]["animation_stride"]))
    frames = simulator.history[::stride] + [simulator.snapshot()]
    if len(frames) < 2:
        raise ValueError("no recorded animation frames")
    plot = TopViewPlot(simulator)
    positions = np.asarray([frame["position"] for frame in simulator.history])

    def update(index):
        last = index == len(frames) - 1
        path = simulator.trajectory if last else positions[:index * stride + 1]
        return plot.draw(frames[index], path, finished=last)

    movie = animation.FuncAnimation(plot.figure, update, frames=len(frames), blit=False)
    fps = int(simulator.config["visualization"]["animation_fps"])
    try:
        if animation.writers.is_available("ffmpeg"):
            try:
                path = output_dir / "simulation.mp4"
                movie.save(path, writer=animation.FFMpegWriter(fps=fps))
                return path
            except Exception:
                pass
        path = output_dir / "simulation.gif"
        movie.save(path, writer=animation.PillowWriter(fps=fps))
        return path
    finally:
        plt.close(plot.figure)

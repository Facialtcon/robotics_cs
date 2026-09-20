#!/usr/bin/env python3
"""Offline synchronized XY / force replay. Never imported by the control loop."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import yaml


def read_run(run_dir):
    run_dir = Path(run_dir)
    with (run_dir / "samples.csv").open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError("run contains no control samples")
    required = ("monotonic_sec", "tcp_x", "tcp_y", "dfx", "dfy", "fxy", "force_reference",
                "tangent_x", "tangent_y", "contact_direction_x", "contact_direction_y")
    data = {key: np.asarray([float(row[key]) for row in rows]) for key in required}
    if not np.all(np.isfinite(data["monotonic_sec"])) or np.any(np.diff(data["monotonic_sec"]) < 0):
        raise ValueError("sample timestamps must be finite and ordered")
    for key in ('direction_valid', 'command_vx', 'command_vy', 'reacquire_origin_x', 'reacquire_origin_y',
                'reacquire_reference_x', 'reacquire_reference_y'):
        data[key] = np.array([float(row.get(key) or ('0' if key=='direction_valid' else 'nan')) for row in rows])
    data["time"] = data["monotonic_sec"] - data["monotonic_sec"][0]
    data["state"] = [row["current_state"] for row in rows]
    with (run_dir / "config_snapshot.yaml").open(encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    with (run_dir / "policy_waypoints.csv").open(newline="", encoding="utf-8") as handle:
        events = list(csv.DictReader(handle))
    return data, config, events


def frame_indices(times, fps, max_frames=1200):
    if not np.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be finite and positive")
    # Pick the latest sample at each playback tick, not every Nth row: works
    # with real control jitter as well as exact 100 Hz synthetic logs.
    if not isinstance(max_frames, int) or max_frames < 2:
        raise ValueError('max_frames must be an integer >= 2')
    count = int(np.ceil((times[-1]-times[0])*fps)) + 1
    ticks = (np.linspace(times[0], times[-1], max_frames-1, endpoint=False) if count > max_frames
             else np.arange(times[0], times[-1], 1/fps))
    indices = np.searchsorted(times, ticks, side="right") - 1
    return np.r_[indices, len(times)-1].astype(int)


def make_figure(data, config, events, *, local_xy=False):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle, Circle

    fig, (xy, force) = plt.subplots(1, 2, figsize=(10, 4.6), layout="constrained")
    xy.set_aspect("equal", adjustable="box")
    xy.set(xlabel="Base X (m)", ylabel="Base Y (m)", title="TCP trajectory")
    scene = config.get("continuous_simulation")
    if scene:
        if 'target_boundary_xy' in scene:
            outline = np.array(scene['target_boundary_xy'])
        else:
            from simulation.geometry import create_target
            outline = create_target(scene).boundary_points()
        xy.plot(outline[:,0], outline[:,1], color='gray', lw=1, label='simulation target truth')
        b = scene["container"]
        xy.add_patch(Rectangle((b["x_min"], b["y_min"]), b["x_max"]-b["x_min"],
                               b["y_max"]-b["y_min"], fill=False, color="gray", label="container"))
    else:
        points = config.get("continuous_workspace_calibration", {}).get("rectified_points", {})
        if points:
            xy.add_patch(Polygon([points[key][:2] for key in ("R0", "R1", "R2", "R3")],
                                 fill=False, color="gray", label="calibrated container"))
        elif config["workspace"].get("enabled", True):
            b = config["workspace"]["limits"]
            xy.add_patch(Rectangle((b["x_min"], b["y_min"]), b["x_max"]-b["x_min"],
                                   b["y_max"]-b["y_min"], fill=False, color="gray", label="workspace"))
    xy.plot(data["tcp_x"], data["tcp_y"], alpha=0)  # fixed bounds over full replay
    xy.margins(0.08)
    trajectory, = xy.plot([], [], color="tab:blue", lw=1, label="reliable contact TCP path")
    other_path, = xy.plot([], [], color='tab:orange', lw=1, label='search / lost / recovery TCP path')
    reference_path, = xy.plot([], [], '--', color='purple', lw=1, label='recovery position reference')
    recovery_range = Circle((0,0), float(config['continuous_tracking'].get('reacquire_max_distance', .004)),
                            fill=False, ls=':', color='purple', label='recovery displacement limit')
    xy.add_patch(recovery_range)
    recovery_range.set_visible(False)
    probe, = xy.plot([], [], "ko", ms=4, label="TCP (enlarged marker)")
    contact_mask = ((np.array(data['state']) == 'CONTINUOUS_TRACKING') &
                    (data['direction_valid'] > .5) &
                    (data['fxy'] >= float(config['continuous_tracking']['contact_lost_threshold'])))
    if local_xy:
        points = np.column_stack((data['tcp_x'], data['tcp_y']))
        radius = float(config['continuous_tracking']['reacquire_max_distance'])
        xy.set_xlim(np.nanmin(points[:,0])-radius, np.nanmax(points[:,0])+radius)
        xy.set_ylim(np.nanmin(points[:,1])-radius, np.nanmax(points[:,1])+radius)
        xy.set_title('Local TCP view (equal XY scale)')
    arrow_length = 0.002 if local_xy else 0.008  # display length in meters, not force or velocity scale
    tangent = xy.quiver([data["tcp_x"][0]], [data["tcp_y"][0]], [0], [0],
                        angles="xy", scale_units="xy", scale=1, color="tab:green", label="command velocity direction (fixed length)")
    contact = xy.quiver([data["tcp_x"][0]], [data["tcp_y"][0]], [0], [0],
                        angles="xy", scale_units="xy", scale=1, color="tab:red", label="force direction estimate (fixed length)")
    markers = []
    for name, marker, color in (("FIRST_CONTACT", "*", "green"), ("CONTACT_LOST", "x", "red"),
                                 ("LOCAL_REACQUIRE", "+", "orange"), ("REACQUIRED", "s", "purple")):
        selected = [e for e in events if e["event_type"] == name]
        if selected:
            artist = xy.scatter([], [], marker=marker, color=color, s=40, label=name)
            markers.append((artist, selected))
    xy.legend(fontsize=6, loc="best")
    for key, label in (("dfx", "Fx"), ("dfy", "Fy"), ("fxy", "Fxy"), ("force_reference", "F_ref")):
        force.plot(data["time"], data[key], label=label, lw=1)
    cursor = force.axvline(0, color="black", ls="--", lw=1)
    force.set(xlabel="Elapsed time (s)", ylabel="Force (N)", title="Processed Base-frame force")
    force.legend(fontsize=8)
    force.grid(alpha=0.2)
    title = fig.suptitle("")

    def update(index):
        x, y = data["tcp_x"][index], data["tcp_y"][index]
        mask = contact_mask[:index+1]
        for line, selected in ((trajectory, mask), (other_path, ~mask)):
            line.set_data(np.where(selected, data['tcp_x'][:index+1], np.nan),
                          np.where(selected, data['tcp_y'][:index+1], np.nan))
        reference_path.set_data(data['reacquire_reference_x'][:index+1], data['reacquire_reference_y'][:index+1])
        origin = [data['reacquire_origin_x'][index], data['reacquire_origin_y'][index]]
        recovery_range.set_visible(bool(np.all(np.isfinite(origin))))
        if np.all(np.isfinite(origin)):
            recovery_range.center = origin
        probe.set_data([x], [y])
        velocity = np.array([data['command_vx'][index], data['command_vy'][index]])
        speed = np.linalg.norm(velocity)
        direction = velocity/speed if np.isfinite(speed) and speed > 0 else np.zeros(2)
        tangent.set_offsets([[x,y]])
        tangent.set_UVC([direction[0]*arrow_length], [direction[1]*arrow_length])
        contact.set_offsets([[x,y]])
        valid = data['direction_valid'][index] > .5
        contact.set_UVC([data['contact_direction_x'][index]*arrow_length if valid else 0],
                        [data['contact_direction_y'][index]*arrow_length if valid else 0])
        for artist, selected in markers:
            visible = [[float(e["x"]), float(e["y"])] for e in selected
                       if float(e["timestamp"]) <= data["monotonic_sec"][index] + 1e-8]
            artist.set_offsets(np.asarray(visible).reshape(-1, 2))
        cursor.set_xdata([data["time"][index]] * 2)
        title.set_text(f"{data['time'][index]:.2f} s | {data['state'][index]} | direction arrows: {arrow_length*1000:g} mm display length")
        return trajectory, other_path, reference_path, probe, tangent, contact, cursor, title

    return fig, update


def render(run_dir, *, fps=None, output_format="auto", local_xy=False):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter, PillowWriter

    data, config, events = read_run(run_dir)
    fps = float(config["continuous_tracking"]["visualization_fps"] if fps is None else fps)
    if output_format == 'auto':
        output_format = 'mp4' if FFMpegWriter.isAvailable() else 'gif'
    max_frames = 240 if output_format == 'gif' else 1200
    indices = frame_indices(data["time"], fps, max_frames=max_frames)
    if data['time'][-1]*fps+1 > max_frames:
        fps = (len(indices)-1)/max(data['time'][-1], 1/fps)
        print(f'Bounded replay: {len(indices)} frames, {fps:.2f} fps')
    fig, update = make_figure(data, config, events, local_xy=local_xy)
    output = Path(run_dir)
    suffix = "_local" if local_xy else ""
    summary = output / f"continuous_summary{suffix}.png"
    update(len(data["time"])-1)
    fig.savefig(summary, dpi=140)
    artifacts = [summary]
    if output_format == "auto":
        output_format = "mp4" if FFMpegWriter.isAvailable() else "gif"
    try:
        if output_format != "none":
            if output_format == "mp4":
                writer = FFMpegWriter(fps=fps, bitrate=600)
            else:
                # Pillow retains frames in memory. Bound fallback memory to at
                # most 240 low-resolution frames; preserve elapsed replay time.
                if len(indices) > 240:
                    indices = indices[np.linspace(0, len(indices)-1, 240).astype(int)]
                    fps = len(indices) / max(data["time"][-1], 1/fps)
                    print(f"GIF fallback: reduced to {fps:.2f} fps (240-frame memory limit)")
                writer = PillowWriter(fps=fps)
            movie = output / f"continuous_replay{suffix}.{output_format}"
            # Direct frame streaming, no intermediate PNG sequence.
            with writer.saving(fig, str(movie), dpi=80):
                for index in indices:
                    update(index)
                    writer.grab_frame()
            artifacts.append(movie)
    except Exception as exc:
        print(f"Animation unavailable ({type(exc).__name__}: {exc}); static summary saved.")
    finally:
        plt.close(fig)
    return artifacts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--fps", type=float)
    parser.add_argument("--local-xy", action="store_true", help="crop XY around actual path, preserving equal scale")
    parser.add_argument("--format", choices=("auto", "mp4", "gif", "none"), default="auto")
    args = parser.parse_args()
    for path in render(args.run_dir, fps=args.fps, output_format=args.format, local_xy=args.local_xy):
        print(path)


if __name__ == "__main__":
    main()

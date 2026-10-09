"""Standard Base XY figures for one completed run, never a history sweep."""
from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
import shlex
import sys
from zoneinfo import ZoneInfo

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiment_logging.paths import run_sort_key
from tools.visualize_continuous_run import read_run

CUTOFF_RUN = 'run_2026-10-06_11-10-51_804801'
CUTOFF = datetime(2026, 10, 6, 11, 10, 51, 804801, tzinfo=ZoneInfo('Asia/Shanghai'))
PHASES = ('Target search', 'Boundary tracking', 'Lost-edge recovery')
COLORS = ('#3975ae', '#16836b', '#e68621')


def enabled_for_run(run_dir):
    """Creation metadata first; existing old/new directory spellings as fallback."""
    stamp = run_sort_key(Path(run_dir))[0]
    return bool(np.isfinite(stamp) and stamp >= CUTOFF.timestamp())


@dataclass
class RunTrace:
    run_dir: Path
    time: np.ndarray
    monotonic: np.ndarray
    xy_mm: np.ndarray
    force: np.ndarray  # Fx/Fy/Fxy, actual policy-input Base values; NaN if unavailable.
    state: np.ndarray
    phase: np.ndarray
    first_contact_index: int | None
    event_indices: np.ndarray
    force_fields: tuple[str, str]
    actual_velocity: np.ndarray = field(default_factory=lambda: np.empty((0, 6)))
    actual_time: np.ndarray = field(default_factory=lambda: np.empty(0))
    command_velocity: np.ndarray = field(default_factory=lambda: np.empty((0, 6)))
    command_time: np.ndarray = field(default_factory=lambda: np.empty(0))
    events: list = field(default_factory=list)
    config: dict = field(default_factory=dict)
    unfiltered_force: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    contact_source: str = 'unavailable'
    target_mm: np.ndarray | None = None
    stop_position_mm: np.ndarray | None = None

    @property
    def duration(self):
        return float(self.time[-1])

    def sample_index(self, elapsed):
        return int(np.clip(np.searchsorted(self.time, elapsed, side='right')-1, 0, len(self.time)-1))


def load_trace(run_dir):
    """Reuse the existing CSV/snapshot reader; start at recorded P0, skip startup."""
    run_dir = Path(run_dir).expanduser().resolve()
    data, config, events = read_run(run_dir, require_continuous=False)
    states = np.asarray(data['state'])
    scan = np.flatnonzero(states != 'RETURN_TO_START')
    if not len(scan):
        raise ValueError('No scan TCP samples recorded (startup return only).')
    start = int(scan[0])
    times = data['monotonic_sec'][start:]
    xy = np.column_stack((data['tcp_x'], data['tcp_y']))[start:]*1000.
    if not np.isfinite(xy[[0, -1]]).all():
        raise ValueError('Recorded P0/final TCP is unavailable; no position is invented.')
    states = states[start:]
    frames = np.asarray(data['processed_force_frame'])[start:]
    aliases = np.column_stack((data['filtered_force_base_fx'], data['filtered_force_base_fy']))[start:]
    legacy = np.column_stack((data['dfx'], data['dfy']))[start:]
    explicit = np.isfinite(aliases).all(axis=1)
    force_xy = np.where(explicit[:, None], aliases, legacy)
    force_xy[frames != 'Base'] = np.nan  # Never label a Sensor wrench as Base.
    force = np.column_stack((force_xy, np.hypot(force_xy[:, 0], force_xy[:, 1])))
    fields = ('filtered_force_base_fx', 'filtered_force_base_fy') if explicit.any() else ('dfx', 'dfy')
    threshold = next((e for e in events if e['event_type'] == 'FIRST_THRESHOLD_STOP_REQUEST'), None)
    confirmed = next((e for e in events if e['event_type'] == 'FIRST_CONTACT'), None)
    first = threshold or confirmed
    first_index = (int(np.clip(np.searchsorted(times, float(first['timestamp'])), 0, len(times)-1))
                   if first else None)
    contact_source = ('threshold crossing (filtered force; physical collision may be earlier)' if threshold else
                      'legacy confirmed contact; collision onset unavailable' if confirmed else 'unavailable')
    phase = np.zeros(len(times), dtype=int)
    previous = 0
    for i, state in enumerate(states):
        if state in ('CONTACT_LOST', 'LOCAL_REACQUIRE'):
            previous = 2
        elif state in ('CONTINUOUS_TRACKING', 'DIRECTION_RECONFIRM', 'TRACKING', 'TRACE'):
            previous = 1
        elif state in ('READY', 'TARGET_SEARCH', 'FIRST_CONTACT'):
            previous = 0
        phase[i] = previous  # Braking/STOP tail belongs to the preceding phase.
    event_indices = np.array([np.clip(np.searchsorted(times, float(e['timestamp'])), 0, len(times)-1)
                              for e in events if times[0] <= float(e['timestamp']) <= times[-1]], dtype=int)
    actual = np.column_stack([data[k][start:] for k in ('tcp_vx', 'tcp_vy', 'tcp_vz', 'tcp_vrx', 'tcp_vry', 'tcp_vrz')])
    actual_stamps = data['actual_tcp_timestamp'][start:]
    actual_stamps = np.where(np.isfinite(actual_stamps), actual_stamps, times)
    command_stamps, commands = [], []
    command_file = run_dir/'tcp_commands.csv'
    if command_file.exists():
        with command_file.open(newline='', encoding='utf-8') as handle:
            for row in csv.DictReader(handle):
                if row.get('frame') != 'Base':
                    continue
                command_stamps.append(float(row['timestamp']))
                commands.append([float(row.get(k) or 'nan') for k in ('vx', 'vy', 'vz', 'wx', 'wy', 'wz')])
    else:
        # Only saved SDK inputs qualify. Policy command_v* is not an SDK trace.
        explicit = np.isfinite(data['commanded_tcp_timestamp'])
        for i in range(len(data['time'])):
            stamp = data['commanded_tcp_timestamp'][i] if explicit[i] else data['speedl_host_monotonic'][i]
            if np.isfinite(stamp) and (not command_stamps or stamp != command_stamps[-1]):
                command_stamps.append(stamp)
                commands.append([data['commanded_tcp_'+k][i] for k in ('vx', 'vy', 'vz', 'wx', 'wy', 'wz')]
                                if explicit[i] else [data['speedl_vx'][i], data['speedl_vy'][i], *([np.nan]*4)])
    command_stamps = np.asarray(command_stamps, dtype=float)
    commands = np.asarray(commands, dtype=float).reshape(-1, 6)
    if len(command_stamps):
        order = np.argsort(command_stamps, kind='stable')
        command_stamps, commands = command_stamps[order], commands[order]
        begin = max(0, np.searchsorted(command_stamps, times[0], side='right')-1)
        command_stamps, commands = command_stamps[begin:], commands[begin:]
    selected_events = []
    for event in events:
        stamp = float(event['timestamp'])
        if stamp < times[0] or stamp > times[-1]+.1:
            continue
        index = int(np.clip(np.searchsorted(times, stamp), 0, len(times)-1))
        selected_events.append(dict(time=stamp-times[0], index=index, kind=event['event_type'],
            xy_mm=[float(event.get(k) or 'nan')*1000 for k in ('x', 'y')]))
    unfiltered_xy = np.column_stack((data['force_base_fx'], data['force_base_fy']))[start:]
    unfiltered_xy[frames != 'Base'] = np.nan
    unfiltered = np.column_stack((unfiltered_xy, np.hypot(*unfiltered_xy.T)))
    stop_position = None
    for event in reversed(selected_events):
        if event['kind'] == 'EXECUTION_STOP_CONFIRMED' and np.isfinite(event['xy_mm']).all():
            stop_position = np.asarray(event['xy_mm'])
            break
    snapshot_path = run_dir/'scan_stop_snapshot.json'
    if stop_position is None and snapshot_path.exists():
        snapshot = json.loads(snapshot_path.read_text(encoding='utf-8'))
        pose = np.asarray(snapshot.get('tcp_pose', []), dtype=float)
        if snapshot.get('stop_observation', {}).get('standstill_confirmed') is True and pose.shape == (6,) and np.isfinite(pose).all():
            stop_position = pose[:2]*1000.
    return RunTrace(run_dir, times-times[0], times, xy, force, states, phase,
                    first_index, event_indices, fields, actual, actual_stamps-times[0],
                    commands, command_stamps-times[0], selected_events, config, unfiltered,
                    contact_source, target_outline(config.get('experiment', {}).get('target')), stop_position)


def target_outline(target):
    """Only explicit measured geometry; never infer the cube from TCP turns."""
    if not target:
        return None
    try:
        center = np.asarray(target['center_base_mm'], dtype=float)
        side = float(target['side_mm'])
        angle = np.deg2rad(float(target['rotation_deg']))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('Known target requires center_base_mm, side_mm, rotation_deg') from exc
    if center.shape != (2,) or not np.isfinite([*center, side, angle]).all() or side <= 0:
        raise ValueError('Invalid known cube geometry')
    corners = np.array([[-1,-1],[1,-1],[1,1],[-1,1],[-1,-1]])*side/2
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    return corners @ rotation.T+center


def display_indices(trace, rate_hz=40.):
    """Display-only thinning retains both sides of every state change and events."""
    ticks = np.arange(0., trace.duration, 1/float(rate_hz))
    sampled = np.clip(np.searchsorted(trace.time, ticks, side='right')-1, 0, len(trace.time)-1)
    switches = np.flatnonzero(trace.state[1:] != trace.state[:-1])+1
    # Keep short force/velocity extrema even when a spike falls between refresh ticks.
    extrema = []
    signals = np.column_stack((trace.force, trace.actual_velocity[:, :2]))
    buckets = np.floor(trace.time*rate_hz).astype(np.int64)
    for values in signals.T:
        valid = np.flatnonzero(np.isfinite(values))
        if not len(valid):
            continue
        order = valid[np.lexsort((values[valid], buckets[valid]))]
        edges = np.r_[0, np.flatnonzero(np.diff(buckets[order]))+1, len(order)]
        extrema.extend(order[edges[:-1]])
        extrema.extend(order[edges[1:]-1])
    return np.unique(np.r_[sampled, extrema, 0, len(trace.time)-1, switches, switches-1, trace.event_indices]).astype(int)



def xy_limits(points, aspect, *, margin=.18):
    """Expand DATA bounds to the physical axes ratio, never shrink the axes box."""
    points = points[np.isfinite(points).all(axis=1)]
    low, high = points.min(axis=0), points.max(axis=0)
    center = (low+high)/2
    span = high-low
    span += 2*np.maximum(span*margin, max(float(span.max())*.12, 5.))
    span[0] = max(span[0], span[1]*aspect)
    span[1] = max(span[1], span[0]/aspect)
    return center-span/2, center+span/2


def configure_xy(axis, points, title):
    figure = axis.figure
    box = axis.get_position()
    ratio = (figure.get_figwidth()*box.width)/(figure.get_figheight()*box.height)
    low, high = xy_limits(points, ratio)
    axis.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), xlabel='Base X [mm]',
             ylabel='Base Y [mm]', title=title)
    axis.set_aspect('equal', adjustable='box')
    axis.set_autoscale_on(False)
    axis.ticklabel_format(useOffset=False, style='plain')
    axis.grid(color='#d9dfe5', alpha=.7, linewidth=.65)
    axis.tick_params(labelsize=12)
    axis.xaxis.label.set_size(14)
    axis.yaxis.label.set_size(14)
    axis.title.set_size(18)


def path_lines(axis, trace, indices):
    """Adjacent observed samples only; NaNs break each colored phase at gaps."""
    points = trace.xy_mm[indices]
    # A thin complete measured TCP trace also retains the short braking transitions.
    axis.plot(*points.T, color='#cbd1d6', lw=1., zorder=1)
    lines = []
    for phase, (name, color) in enumerate(zip(PHASES, COLORS)):
        values = points.copy()
        values[trace.phase[indices] != phase] = np.nan
        line, = axis.plot(*values.T, color=color, lw=2.4, label=name, zorder=2)
        lines.append(line)
    return lines


def mark_position(axis, point, label, *, marker, color, offset):
    axis.scatter(*point, marker=marker, s=95, color=color, edgecolors='white', linewidths=.8,
                 zorder=5, label=label)
    axis.annotate(label, point, xytext=offset, textcoords='offset points', fontsize=12,
                  color=color, fontweight='bold')


def standard_figures(trace):
    """Two wide PPT-ready figures; no guessed object outline/major-turn labels."""
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    result = []
    for local, filename in ((False, 'trajectory_overview.png'), (True, 'trajectory_local_zoom.png')):
        figure = Figure(figsize=(16, 9), facecolor='white')
        FigureCanvasAgg(figure)
        axis = figure.add_axes([.075, .13, .69, .75])
        if trace is None:
            axis.set_title('No scan TCP samples recorded', fontsize=18)
            axis.text(.5, .5, 'The run ended before scanning.\nNo trajectory or force data is invented.',
                      ha='center', va='center', transform=axis.transAxes, fontsize=16)
            axis.set_axis_off()
            result.append((filename, figure))
            continue
        start = trace.first_contact_index if local and trace.first_contact_index is not None else 0
        indices = np.arange(start, len(trace.time))
        title = 'Experiment trajectory — P0 to final stop' if not local else 'Local trajectory — first contact to final stop'
        if local and trace.first_contact_index is None:
            title = 'Local trajectory — no confirmed first contact recorded'
        bounds = trace.xy_mm[indices] if trace.target_mm is None else np.r_[trace.xy_mm[indices], trace.target_mm]
        configure_xy(axis, bounds, title)
        if trace.target_mm is not None:
            axis.plot(*trace.target_mm.T, '--', color='.25', label='Known cube')
        lines = path_lines(axis, trace, indices)
        if not local:
            mark_position(axis, trace.xy_mm[0], 'P0', marker='o', color='#25364a', offset=(10, 8))
        if trace.first_contact_index is not None:
            mark_position(axis, trace.xy_mm[trace.first_contact_index], ('Contact threshold' if trace.contact_source.startswith('threshold') else 'Contact confirmed'), marker='*',
                          color='#9b287b', offset=(12, 9))
        mark_position(axis, trace.xy_mm[-1] if trace.stop_position_mm is None else trace.stop_position_mm,
                      ('Final sample' if trace.stop_position_mm is None else 'Confirmed stop'), marker='X', color='#b72a33', offset=(12, -18))
        handles = [line for phase, line in enumerate(lines) if (trace.phase[indices] == phase).any()]
        handles += list(axis.collections)
        axis.legend(handles=handles, loc='upper left', bbox_to_anchor=(1.025, 1.),
                    frameon=False, fontsize=12, labelspacing=1.1)
        figure.text(.075, .04, 'Recorded TCP path in Base coordinates; not a known object outline.',
                    fontsize=11, color='#596575')
        result.append((filename, figure))
    return result


def write_launcher(run_dir, source_run_dir=None):
    run_dir = Path(run_dir)
    python = ROOT.parent/'.venv312/bin/python'
    if not python.is_file():
        python = Path(sys.executable)
    script = run_dir/'play_visualization.sh'
    source_arg = '"$RUN_DIR"' if source_run_dir is None else shlex.quote(str(source_run_dir))
    script.write_text('#!/usr/bin/env bash\nset -e\n'
        'RUN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"\n'
        f'PYTHON_BIN={shlex.quote(str(python))}\n'
        'export MPLCONFIGDIR="${TMPDIR:-/tmp}/ur7e-px6d-matplotlib"\n'
        'if [[ $# -eq 0 && -f "$RUN_DIR/实验回放.html" ]]; then\n'
        '  if xdg-open "$RUN_DIR/实验回放.html"; then exit 0; fi\n'
        '  printf "浏览器未能自动打开，请直接双击本目录的 实验回放.html。\\n"\n'
        '  if [[ -t 0 ]]; then read -r -p "按回车关闭…"; fi\n'
        '  exit 1\n'
        'fi\n'
        f'if "$PYTHON_BIN" {shlex.quote(str(ROOT/"visualization/run_animation.py"))} --run-dir {source_arg} "$@"; then exit 0; fi\n'
        'printf "回放未能启动；如有 实验回放.html，请直接双击用浏览器打开。\\n"\n'
        'if [[ -t 0 ]]; then read -r -p "按回车关闭…"; fi\n'
        'exit 1\n',
        encoding='utf-8')
    script.chmod(0o755)
    # File managers may open .sh files in an editor. A desktop entry launches
    # the same offline script directly, without requiring a terminal command.
    desktop = run_dir/'打开实验回放.desktop'
    executable = str(script.resolve()).replace('\\', '\\\\').replace('"', '\\"')
    executable = executable.replace('`', '\\`').replace('$', '\\$').replace('%', '%%')
    desktop.write_text('[Desktop Entry]\nType=Application\n'
        'Name=打开实验回放\nComment=播放本文件夹的实验数据\n'
        f'Exec="{executable}"\n'
        'Icon=media-playback-start\nTerminal=true\n', encoding='utf-8')
    desktop.chmod(0o755)
    return script


def generate_run_visualization(run_dir, *, force=False, output_dir=None):
    """One explicit run only. The cutoff is checked before ANY output write."""
    run_dir = Path(run_dir).expanduser().resolve()
    if not force and not enabled_for_run(run_dir):
        return []
    try:
        trace = load_trace(run_dir)
    except ValueError as exc:
        # Aborted startup still gets its standard entry, with honest empty
        # figures instead of treating the return path as a scanned boundary.
        if str(exc) != 'run contains no control samples' and not str(exc).startswith('No scan TCP samples recorded'):
            raise
        trace = None
    output_dir = run_dir if output_dir is None else Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    artifacts = []
    for name, figure in standard_figures(trace):
        output = output_dir/name
        figure.savefig(output, dpi=180)
        artifacts.append(output)
    if trace is not None:
        from visualization.contact_report import generate_contact_report
        artifacts.extend(generate_contact_report(trace, output_dir))
        from visualization.run_html import write_html_replay
        artifacts.append(write_html_replay(trace, output_dir))
    artifacts.append(write_launcher(output_dir, run_dir if output_dir != run_dir else None))
    artifacts.append(output_dir/'打开实验回放.desktop')
    return artifacts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--historical', action='store_true', help='explicitly replay older runs')
    parser.add_argument('--output', type=Path, help='separate output directory; keeps historic artifacts intact')
    args = parser.parse_args()
    for path in generate_run_visualization(args.run_dir, force=args.historical, output_dir=args.output):
        print(path)


if __name__ == '__main__':
    main()

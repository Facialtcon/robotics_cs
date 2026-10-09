"""Shared offline contact diagnostics and PPT figures; no robot dependencies."""
from pathlib import Path
import json

import numpy as np

EVENT_STYLE = {
    'FIRST_THRESHOLD_STOP_REQUEST': ('Contact threshold', '#9b287b', '*'),
    'FIRST_CONTACT': ('Contact confirmed', '#79558f', '*'),
    'TRACKING_ENTERED': ('Tracking entered', '#16836b', 's'),
    'CONTACT_LOST': ('Contact lost', '#e68621', 'v'),
    'REACQUIRED': ('Contact restored', '#1964b1', '^'),
    'EXECUTION_STOP_CONFIRMED': ('Stop confirmed', '#b72a33', 'X'),
    'USER_STOP': ('Stop requested', '#b72a33', 'X'),
    'BUDGET_STOP': ('Stop requested', '#b72a33', 'X'),
}
COMPONENTS = ('vx', 'vy', 'XY')
SIGNAL_COLORS = ('#3975ae', '#c65050', '#16836b')


def planar_speed(velocity):
    return np.column_stack((velocity[:, :2]*1000., np.hypot(*velocity[:, :2].T)*1000.))


def command_at(trace, elapsed):
    index = np.searchsorted(trace.command_time, elapsed, side='right')-1
    return (trace.command_velocity[index] if index >= 0 else np.full(6, np.nan))


def event_lines(axis, trace):
    seen = set()
    for event in trace.events:
        style = EVENT_STYLE.get(event['kind'])
        if style:
            label, color, _ = style
            axis.axvline(event['time'], color=color, alpha=.55, ls=':', lw=.9,
                         label=label if label not in seen else '_nolegend_')
            seen.add(label)


def force_plot(axis, trace):
    for i, (label, color) in enumerate(zip(('Fx', 'Fy', 'Fxy'), SIGNAL_COLORS)):
        axis.plot(trace.time, trace.force[:, i], color=color, lw=1.1, label=label)
    if np.isfinite(trace.unfiltered_force).any():
        axis.plot(trace.time, trace.unfiltered_force[:, 2], color='.55', lw=.7, alpha=.6, label='Fxy before filter')
    threshold = trace.config.get('policy', {}).get('contact_threshold')
    if threshold is not None:
        axis.axhline(float(threshold), ls='--', color='#9b287b', label=f'Contact {threshold:g} N')
    lost = trace.config.get('continuous_tracking', {}).get('contact_lost_threshold')
    if lost is not None:
        axis.axhline(float(lost), ls='-.', color='#e68621', label=f'Lost {lost:g} N')
    event_lines(axis, trace)
    axis.set(ylabel='Base force [N]', title='Force — filtered Base XY and pre-filter magnitude')
    axis.grid(alpha=.25)


def velocity_plot(axis, trace):
    actual, commanded = planar_speed(trace.actual_velocity), planar_speed(trace.command_velocity)
    for i, (component, color) in enumerate(zip(COMPONENTS, SIGNAL_COLORS)):
        axis.plot(trace.actual_time, actual[:, i], color=color, lw=1.2, label=f'Actual {component}')
        axis.step(trace.command_time, commanded[:, i], where='post', color=color, ls='--', lw=1., label=f'Sent {component}')
    event_lines(axis, trace)
    axis.set(ylabel='Base speed [mm/s]', xlabel='Elapsed host monotonic time [s]', title='TCP velocity — actual (solid), sent command (dashed)')
    axis.grid(alpha=.25)


def finite_list(values):
    return [float(v) if np.isfinite(v) else None for v in values]


def direction_angle(left, right):
    left, right = np.asarray(left), np.asarray(right)
    if not np.isfinite(np.r_[left, right]).all() or min(np.linalg.norm(left), np.linalg.norm(right)) < 1e-9:
        return None
    return float(np.degrees(np.arccos(np.clip(left@right/(np.linalg.norm(left)*np.linalg.norm(right)), -1., 1.))))


def collision_details(trace):
    first = trace.first_contact_index
    if first is None:
        return dict(source=trace.contact_source, note='Contact onset was not recorded.')
    event = next((e for e in trace.events if e['kind'] == 'FIRST_THRESHOLD_STOP_REQUEST'), None)
    if event is None:
        event = next((e for e in trace.events if e['kind'] == 'FIRST_CONTACT'), None)
    t = event['time'] if event else trace.time[first]
    pre = np.flatnonzero((trace.time < t-1e-8) & (trace.actual_time < t))
    cmd = np.flatnonzero(trace.command_time < t)
    before = int(pre[-1]) if len(pre) else None
    sent = int(cmd[-1]) if len(cmd) else None
    after = []
    for offset in (.1, .5, 1.):
        indexes = np.flatnonzero((trace.actual_time >= t+offset) & np.isfinite(trace.actual_velocity[:, :2]).all(axis=1))
        if len(indexes):
            i = int(indexes[0])
            after.append(dict(requested_offset_sec=offset, sample_time_sec=float(trace.actual_time[i]),
                              actual_xy_mm_s=finite_list(planar_speed(trace.actual_velocity[i:i+1])[0])))
    return dict(source=trace.contact_source, time_sec=float(t),
        precontact_history_sec=float(t-trace.time[0]), precontact_one_second_available=bool(
            np.any((trace.actual_time <= t-1.) & np.isfinite(trace.actual_velocity[:,:2]).all(axis=1))),
        actual_before_time_sec=None if before is None else float(trace.actual_time[before]),
        actual_before_xy_mm_s=None if before is None else finite_list(planar_speed(trace.actual_velocity[before:before+1])[0]),
        command_before_time_sec=None if sent is None else float(trace.command_time[sent]),
        command_before_xy_mm_s=None if sent is None else finite_list(planar_speed(trace.command_velocity[sent:sent+1])[0]),
        actual_approach_vs_force_angle_deg=(None if before is None else
            direction_angle(trace.actual_velocity[before,:2], trace.force[first,:2])),
        command_approach_vs_force_angle_deg=(None if sent is None else
            direction_angle(trace.command_velocity[sent,:2], trace.force[first,:2])),
        contact_force_xy_N=finite_list(trace.force[first]), after_contact=after)


def collision_text(trace, *, compact=False):
    detail = collision_details(trace)
    if 'time_sec' not in detail:
        return 'Contact onset unavailable; no collision data inferred.'
    def fmt(value):
        if value is None or any(x is None for x in value):
            return 'unavailable'
        return ', '.join(f'{x:+.2f}' for x in value)
    lines = [f"First recorded contact: {detail['time_sec']:.3f} s",
             'Pre-contact [vx, vy, XY] mm/s:',
             '  Sent: '+fmt(detail['command_before_xy_mm_s']),
             '  Actual: '+fmt(detail['actual_before_xy_mm_s']),
             'Contact [Fx, Fy, Fxy] N: '+fmt(detail['contact_force_xy_N'])]
    if compact:
        post = []
        for item in detail['after_contact']:
            speed = item['actual_xy_mm_s'][2]
            if speed is not None:
                post.append(f"{item['sample_time_sec']:.2f}s: {speed:.2f}")
        lines.append('After actual XY [mm/s]: '+('; '.join(post) or 'unavailable'))
    else:
        for item in detail['after_contact']:
            lines.append(f"Actual at {item['sample_time_sec']:.3f}s: "+fmt(item['actual_xy_mm_s']))
    if not detail['precontact_one_second_available']:
        lines.append('WARNING: actual velocity history missing or < 1 s')
    if not trace.contact_source.startswith('threshold'):
        lines.append('Legacy confirmation; physical collision onset unavailable.')
    return '\n'.join(lines)


def report_data(trace):
    # Values at recorded samples, no counterfactual recovery or contact geometry.
    windows = []
    for event in trace.events:
        if event['kind'] not in ('CONTACT_LOST', 'REACQUIRED'):
            continue
        samples = []
        for offset in (-1., -.1, 0., .1, 1.):
            if not 0 <= event['time']+offset <= trace.duration:
                samples.append(dict(requested_offset_sec=offset, sample=None, reason='outside recorded interval'))
                continue
            i = trace.sample_index(event['time']+offset)
            samples.append(dict(requested_offset_sec=offset, sample_time_sec=float(trace.time[i]),
                xy_mm=finite_list(trace.xy_mm[i]), force_xy_N=finite_list(trace.force[i]),
                actual_xy_mm_s=finite_list(planar_speed(trace.actual_velocity[i:i+1])[0]),
                commanded_xy_mm_s=finite_list(planar_speed(command_at(trace, trace.actual_time[i])[None, :])[0]),
                state=str(trace.state[i])))
        windows.append(dict(event=event, samples=samples))
    differences = []
    for t, v in zip(trace.actual_time, trace.actual_velocity):
        cmd = command_at(trace, t)
        if np.isfinite(np.r_[cmd[:2], v[:2]]).all():
            differences.append((v[:2]-cmd[:2])*1000.)
    saved = {}
    for name in ('summary.json', 'termination.json', 'logging_status.json'):
        path = trace.run_dir/name
        if path.exists():
            saved[name] = json.loads(path.read_text(encoding='utf-8'))
    return dict(first_contact=collision_details(trace), events=trace.events, saved_diagnostics=saved,
        confirmed_stop_xy_mm=None if trace.stop_position_mm is None else finite_list(trace.stop_position_mm),
        contact_windows=windows, sample_count=len(trace.time), command_count=len(trace.command_time),
        actual_command_xy_rmse_mm_s=(finite_list(np.sqrt(np.mean(np.square(differences), axis=0))) if differences else None),
        max_sample_gap_sec=float(np.max(np.diff(trace.time))) if len(trace.time)>1 else None,
        limitations=['Sequential host receive intervals are not hardware synchronization.',
                     'Threshold crossing uses filtered force; physical collision may precede it.',
                     'Lost contact is a policy low-force event, not proof of a geometric edge.',
                     'Unknown measurements remain null; no algorithm or geometry is inferred.'])


def overview(trace, *, zoom=None):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from visualization.run_plots import configure_xy, path_lines
    fig = Figure(figsize=(18, 10), facecolor='white'); FigureCanvasAgg(fig)
    xy = fig.add_axes([.06, .42, .34, .47])
    force = fig.add_axes([.57, .62, .36, .25])
    velocity = fig.add_axes([.57, .25, .36, .25], sharex=force)
    bounds = trace.xy_mm if trace.target_mm is None else np.r_[trace.xy_mm, trace.target_mm]
    configure_xy(xy, bounds, 'TCP trajectory — Base XY')
    path_lines(xy, trace, np.arange(len(trace.time)))
    if trace.target_mm is not None:
        xy.plot(*trace.target_mm.T, color='.25', ls='--', label='Known cube outline')
    xy.scatter(*trace.xy_mm[0], color='#25364a', label='Start', zorder=6)
    seen = set()
    for event in trace.events:
        style = EVENT_STYLE.get(event['kind'])
        if style and np.isfinite(event['xy_mm']).all():
            name, color, marker = style
            xy.scatter(*event['xy_mm'], color=color, marker=marker, s=60, zorder=5,
                       label=name if name not in seen else '_nolegend_')
            seen.add(name)
    final_label = 'Confirmed stop' if trace.stop_position_mm is not None else 'Final recorded sample'
    point = trace.xy_mm[-1] if trace.stop_position_mm is None else trace.stop_position_mm
    xy.scatter(*point, color='#b72a33', marker='X', label=final_label, zorder=6)
    xy.legend(loc='upper left', bbox_to_anchor=(1.01,1), fontsize=8, frameon=False)
    force_plot(force, trace); velocity_plot(velocity, trace)
    for axis in (force, velocity):
        # Signal legends contain components/thresholds. Event legend is beside XY.
        handles, labels = axis.get_legend_handles_labels()
        pairs = [(h,l) for h,l in zip(handles,labels) if l not in {x[0] for x in EVENT_STYLE.values()}]
        axis.legend(*zip(*pairs), loc='lower left', bbox_to_anchor=(0,1.01), ncol=3, fontsize=8, frameon=False)
        axis.set_title(axis.get_title(), y=1.28, pad=0, fontsize=11)
    if zoom:
        force.set_xlim(*zoom)
    else:
        force.set_xlim(0, max(1., trace.duration))
    fig.text(.06, .12, collision_text(trace), fontsize=10, va='bottom', family='monospace')
    fig.text(.57, .08, 'All curves share elapsed host monotonic time.\nExact command send times; actual RTDE read times.\nNo unknown velocity or contact state is filled in.', fontsize=10)
    prefix = 'SYNTHETIC / illustrative — ' if trace.config.get('experiment', {}).get('kind') == 'synthetic_demo' else ''
    fig.suptitle(prefix+trace.run_dir.name, fontsize=16, y=.98)
    return fig


def generate_contact_report(trace, output_dir):
    output = Path(output_dir); output.mkdir(parents=True, exist_ok=True)
    artifacts = []
    figures = [('experiment_overview.png', overview(trace))]
    contact = collision_details(trace)
    if 'time_sec' in contact:
        t = contact['time_sec']
        figures.append(('first_contact_zoom.png', overview(trace, zoom=(max(0.,t-1.), min(trace.duration,t+1.)))))
    for number, event in enumerate((e for e in trace.events if e['kind'] == 'CONTACT_LOST'), 1):
        figures.append((f'contact_lost_{number:02d}_zoom.png', overview(trace,
            zoom=(max(0.,event['time']-1.), min(trace.duration,event['time']+2.)))))
    for name, figure in figures:
        path = output/name; figure.savefig(path, dpi=160); figure.clear(); artifacts.append(path)
    path = output/'contact_analysis.json'
    path.write_text(json.dumps(json_finite(report_data(trace)), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    artifacts.append(path)
    return artifacts


def json_finite(value):
    if isinstance(value, dict):
        return {k: json_finite(v) for k,v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_finite(v) for v in value]
    if isinstance(value, (float, np.floating)) and not np.isfinite(value):
        return None
    return value

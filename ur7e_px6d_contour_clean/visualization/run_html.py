"""Self-contained browser replay built from the existing offline trace reader."""
from pathlib import Path
import csv
import html
import json

import numpy as np

from visualization.contact_report import collision_details, planar_speed


def _clean(value):
    if isinstance(value, np.ndarray):
        return _clean(value.tolist())
    if isinstance(value, dict):
        return {key: _clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(item) for item in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


def deadband_definition(trace):
    """Describe normal_feedback_speed using this run's snapshot and telemetry.

    continuous_tracking.py only calls normal_feedback_speed in its tracking
    branch after stop/direction guards. No deadband is applied during search or
    recovery. Unknown historical guard flags are explicitly left unknown.
    """
    config = trace.config.get('continuous_tracking', {})
    metadata_file = trace.run_dir/'metadata.json'
    metadata = json.loads(metadata_file.read_text()) if metadata_file.exists() else {}
    if metadata.get('strategy') not in (None, 'continuous') or config.get('enabled') is False:
        return dict(enabled=False, reason='本次实验未启用连续跟踪力反馈', windows=[])
    if metadata.get('strategy') is None and config.get('enabled') is not True:
        return dict(enabled=None, reason='缺少本次是否启用连续跟踪的记录', windows=[])
    keys = ('force_reference', 'force_deadband', 'force_gain')
    try:
        reference, deadband, gain = [float(config[key]) for key in keys]
    except (KeyError, TypeError, ValueError):
        return dict(enabled=None, reason='实验快照缺少死区定义，未假设数值', windows=[])
    if not np.isfinite([reference, deadband, gain]).all() or deadband < 0:
        return dict(enabled=None, reason='实验快照中的死区定义无效', windows=[])
    if gain == 0 or deadband == 0:
        return dict(enabled=False, reason='force_gain=0，未启用力反馈' if gain == 0 else
                    'force_deadband=0，未启用死区', windows=[])
    windows = []
    with (trace.run_dir/'samples.csv').open(newline='', encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))
    start = float(trace.monotonic[0])
    previous = None
    for row in rows:
        t = float(row['monotonic_sec'])-start
        if t < 0:
            continue
        if row.get('current_state') != 'CONTINUOUS_TRACKING':
            status = 'off'
        elif row.get('stop_requested') in ('1', '1.0') or row.get('direction_valid') in ('0', '0.0'):
            status = 'off'
        elif row.get('stop_requested') in ('0', '0.0') and row.get('direction_valid') in ('1', '1.0'):
            status = 'active'
        else:
            status = 'unknown'
        try:
            saved_reference = float(row.get('force_reference') or reference)
        except ValueError:
            saved_reference = reference
        if not np.isfinite(saved_reference):
            if status != 'off':
                status = 'unknown'
            saved_reference = reference
        key = (status, saved_reference-deadband, saved_reference+deadband)
        if key != previous:
            if windows:
                windows[-1]['end'] = t
            windows.append(dict(start=t, end=t, status=status, lower=key[1], upper=key[2]))
            previous = key
    if windows:
        windows[-1]['end'] = trace.duration
    return dict(enabled=True, reference=reference, half_width=deadband,
                lower=reference-deadband, upper=reference+deadband,
                definition='|force_reference − Fxy| ≤ force_deadband；法向反馈速度为零',
                source='config_snapshot.yaml；仅标记记录到反馈执行条件的连续跟踪区段', windows=windows)


def write_html_replay(trace, output_dir):
    """Embed every recorded sample; thin only visible points in the browser."""
    states = list(dict.fromkeys(trace.state.tolist()))
    sample_rows = [
        [trace.time[i], *trace.xy_mm[i], *trace.force[i], int(trace.phase[i]), states.index(trace.state[i]),
         trace.unfiltered_force[i, 2]] for i in range(len(trace.time))
    ]
    payload = dict(
        name=trace.run_dir.name, duration=trace.duration, samples=sample_rows, states=states,
        actual=np.column_stack((trace.actual_time, planar_speed(trace.actual_velocity))),
        commands=np.column_stack((trace.command_time, planar_speed(trace.command_velocity))),
        events=trace.events, target=trace.target_mm, stop=trace.stop_position_mm,
        first_contact=collision_details(trace), sample_count=len(trace.time),
        threshold=trace.config.get('policy', {}).get('contact_threshold'),
        lost_threshold=trace.config.get('continuous_tracking', {}).get('contact_lost_threshold'),
        deadband=deadband_definition(trace),
        synthetic=trace.config.get('experiment', {}).get('kind') == 'synthetic_demo',
    )
    encoded = json.dumps(_clean(payload), ensure_ascii=False, separators=(',', ':'), allow_nan=False)
    # The JSON script element must not be closable by a recorded metadata string.
    encoded = encoded.replace('<', '\\u003c').replace('&', '\\u0026')
    template = Path(__file__).with_name('replay_template.html').read_text(encoding='utf-8')
    script = Path(__file__).with_name('replay_interactions.js').read_text(encoding='utf-8')
    output = Path(output_dir)/'实验回放.html'
    output.write_text(template.replace('__REPLAY_SCRIPT__', script)
                      .replace('__RUN_TITLE__', html.escape(trace.run_dir.name))
                      .replace('__REPLAY_DATA__', encoded), encoding='utf-8')
    return output


if __name__ == '__main__':
    import argparse
    from visualization.run_plots import load_trace
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    print(write_html_replay(load_trace(args.run_dir), args.run_dir))

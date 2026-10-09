"""Self-contained browser replay built from the existing offline trace reader."""
from pathlib import Path
import html
import json

import numpy as np

from visualization.contact_report import collision_details, planar_speed
from visualization.run_plots import display_indices


def _clean(value):
    if isinstance(value, np.ndarray):
        return _clean(value.tolist())
    if isinstance(value, dict):
        return {key: _clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(item) for item in value]
    if isinstance(value, (float, np.floating)):
        return round(float(value), 7) if np.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


def write_html_replay(trace, output_dir):
    """Embed display data only; raw CSVs and SDK command timestamps stay intact."""
    indices = display_indices(trace)
    states = list(dict.fromkeys(trace.state.tolist()))
    sample_rows = [
        [trace.time[i], *trace.xy_mm[i], *trace.force[i], int(trace.phase[i]), states.index(trace.state[i]),
         trace.unfiltered_force[i, 2]] for i in indices
    ]
    payload = dict(
        name=trace.run_dir.name, duration=trace.duration, samples=sample_rows, states=states,
        actual=np.column_stack((trace.actual_time[indices], planar_speed(trace.actual_velocity)[indices])),
        commands=np.column_stack((trace.command_time, planar_speed(trace.command_velocity))),
        events=trace.events, target=trace.target_mm, stop=trace.stop_position_mm,
        first_contact=collision_details(trace), sample_count=len(trace.time),
        threshold=trace.config.get('policy', {}).get('contact_threshold'),
        lost_threshold=trace.config.get('continuous_tracking', {}).get('contact_lost_threshold'),
        synthetic=trace.config.get('experiment', {}).get('kind') == 'synthetic_demo',
    )
    encoded = json.dumps(_clean(payload), ensure_ascii=False, separators=(',', ':'), allow_nan=False)
    # The JSON script element must not be closable by a recorded metadata string.
    encoded = encoded.replace('<', '\\u003c').replace('&', '\\u0026')
    template = Path(__file__).with_name('replay_template.html').read_text(encoding='utf-8')
    output = Path(output_dir)/'实验回放.html'
    output.write_text(template.replace('__RUN_TITLE__', html.escape(trace.run_dir.name))
                      .replace('__REPLAY_DATA__', encoded), encoding='utf-8')
    return output

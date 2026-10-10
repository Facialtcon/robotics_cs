"""Offline replay provenance, original-sample access and phase-specific deadband."""
import csv
import json
import re
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from tools.visualize_contact_experiment import make_demo
from visualization.run_html import deadband_definition, write_html_replay
from visualization.run_plots import load_trace


@pytest.fixture
def trace(tmp_path):
    return load_trace(make_demo(tmp_path))


def payload(page):
    return json.loads(re.search(r'<script type="application/json" id="replay-data">(.*?)</script>',
                                page.read_text(), re.S)[1])


def test_html_retains_every_sample_and_exact_time_for_local_zoom(trace):
    before = (trace.run_dir/'samples.csv').read_bytes()
    data = payload(write_html_replay(trace, trace.run_dir))
    assert len(data['samples']) == len(trace.time) == 601
    np.testing.assert_array_equal([row[0] for row in data['samples']], trace.time)
    np.testing.assert_array_equal(np.array(data['actual'])[:, 0], trace.actual_time)
    np.testing.assert_array_equal(np.array(data['commands'])[:, 0], trace.command_time)
    np.testing.assert_array_equal(np.array(data['samples'])[:, 3:6], trace.force)
    assert (trace.run_dir/'samples.csv').read_bytes() == before


def test_deadband_uses_force_error_and_does_not_apply_to_recovery(trace):
    result = deadband_definition(trace)
    assert result['enabled'] is True
    assert result['lower'] == pytest.approx(1.35)
    assert result['upper'] == pytest.approx(1.65)
    for window in result['windows']:
        rows = np.flatnonzero((trace.time >= window['start']) & (trace.time < window['end']))
        if window['status'] != 'off':
            assert all(trace.state[i] == 'CONTINUOUS_TRACKING' for i in rows)
        if any(trace.state[i] == 'LOCAL_REACQUIRE' for i in rows):
            assert window['status'] == 'off'


@pytest.mark.parametrize('field,value', [('force_deadband', 0), ('force_gain', 0), ('enabled', False)])
def test_disabled_deadband_is_explicit_and_has_no_shading(trace, field, value):
    trace.config['continuous_tracking'][field] = value
    result = deadband_definition(trace)
    assert result['enabled'] is False and result['reason'] and result['windows'] == []


def test_missing_deadband_does_not_use_live_config_defaults(trace):
    del trace.config['continuous_tracking']['force_deadband']
    result = deadband_definition(trace)
    assert result['enabled'] is None and result['windows'] == []


def test_unknown_execution_flags_are_not_declared_active(trace):
    path = trace.run_dir/'samples.csv'
    with path.open() as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        row['direction_valid'] = ''
        row['stop_requested'] = ''
    with path.open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0])
        writer.writeheader();writer.writerows(rows)
    result = deadband_definition(trace)
    assert any(w['status'] == 'unknown' for w in result['windows'])
    assert all(w['status'] != 'active' for w in result['windows'])


def test_deadband_respects_recorded_force_reference_changes(trace):
    path = trace.run_dir/'samples.csv'
    with path.open() as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        row['force_reference'] = '2.0' if row['current_state'] == 'CONTINUOUS_TRACKING' else 'nan'
    with path.open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0])
        writer.writeheader();writer.writerows(rows)
    tracking = [w for w in deadband_definition(trace)['windows'] if w['status'] != 'off']
    assert tracking and all(w['lower'] == 1.85 and w['upper'] == 2.15 for w in tracking)
    assert tracking[0]['start'] >= 2.0  # A missing reference never enables shading during search.


@pytest.mark.skipif(shutil.which('gjs') is None, reason='optional system JavaScript engine unavailable')
def test_real_replay_javascript_controls_clock_and_viewports(trace):
    page = write_html_replay(trace, trace.run_dir)
    result = subprocess.run(['gjs', str(Path(__file__).with_name('html_replay_harness.js')), str(page)],
                            capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)['result'] == 'PASS'

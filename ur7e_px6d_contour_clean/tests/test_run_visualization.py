"""Offline geometry, cutoff, force provenance and actual matplotlib widget tests."""
import csv
from datetime import timedelta
import json
from pathlib import Path
import re
import subprocess

import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseEvent
import numpy as np
import pytest
import yaml

from visualization.run_plots import (CUTOFF, COLORS, PHASES, display_indices, enabled_for_run,
                                     generate_run_visualization, load_trace, standard_figures)
from visualization.run_animation import RunAnimation


def recorded_run(tmp_path, config, *, contact=True, aliases=True):
    directory = tmp_path/'run_2026-10-06_11-10-51_804801'
    directory.mkdir()
    (directory/'metadata.json').write_text(json.dumps(dict(mode='real', strategy='continuous',
        timestamp=CUTOFF.isoformat(), git={}, config_source='synthetic test')))
    (directory/'config_snapshot.yaml').write_text(yaml.safe_dump(config))
    states = ['RETURN_TO_START', 'TARGET_SEARCH', 'TARGET_SEARCH', 'FIRST_CONTACT']
    states += ['CONTINUOUS_TRACKING' if contact else 'TARGET_SEARCH']
    for _ in range(8):
        states += ['CONTACT_LOST', 'LOCAL_REACQUIRE', 'LOCAL_REACQUIRE', 'CONTINUOUS_TRACKING']
    states += ['STOP', 'STOP']
    if not contact:
        states[3:-2] = ['TARGET_SEARCH']*(len(states)-5)
    rows = []
    for i, state in enumerate(states):
        x, y = ((999., 999.) if i == 0 else (641.+i*.05, 360.-i*.7))
        fx, fy = i*.02, -i*.015
        rows.append(dict(monotonic_sec=99.+i*.017, tcp_x=x/1000, tcp_y=y/1000,
            current_state=state, dfx=fx, dfy=fy, fxy=np.hypot(fx, fy), filtered_fxy=999.,
            force_reference=1.5, tangent_x=1., tangent_y=0., contact_direction_x=0.,
            contact_direction_y=1., processed_force_frame='Base',
            **(dict(filtered_force_base_fx=fx, filtered_force_base_fy=fy) if aliases else {})))
    with (directory/'samples.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    with (directory/'policy_waypoints.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=('timestamp', 'state', 'event_type', 'x', 'y'))
        writer.writeheader()
        if contact:
            writer.writerow(dict(timestamp=rows[4]['monotonic_sec'], state='CONTINUOUS_TRACKING',
                                 event_type='FIRST_CONTACT', x=rows[4]['tcp_x'], y=rows[4]['tcp_y']))
    return directory, rows


@pytest.mark.parametrize('delta,expected', [(-1, False), (0, True), (1, True)])
def test_cutoff_metadata_wins_over_old_directory_name(tmp_path, delta, expected):
    directory = tmp_path/'run_20261005_093305_896939'; directory.mkdir()
    (directory/'metadata.json').write_text(json.dumps(dict(mode='real', strategy='continuous',
        timestamp=(CUTOFF+timedelta(microseconds=delta)).isoformat(), git={}, config_source=None)))
    assert enabled_for_run(directory) == expected


@pytest.mark.parametrize('name,enabled', [('run_2026-10-06_11-10-51_804801', True),
    ('run_20261005_093305_896939', False), ('unknown', False)])
def test_cutoff_name_fallback_is_safe(tmp_path, name, enabled):
    assert enabled_for_run(tmp_path/name) == enabled


def test_earlier_run_is_untouched_without_even_reading_csv(tmp_path, monkeypatch):
    from visualization import run_plots
    directory = tmp_path/'run_20261005_093305_896939'; directory.mkdir()
    sentinel = directory/'samples.csv'; sentinel.write_text('unchanged historic data')
    before = sentinel.stat()
    def forbidden(*a): raise AssertionError('Old CSV must not be read or plotted')
    monkeypatch.setattr(run_plots, 'load_trace', forbidden)
    assert generate_run_visualization(directory) == []
    assert list(directory.iterdir()) == [sentinel]
    assert sentinel.stat().st_mtime_ns == before.st_mtime_ns


@pytest.mark.parametrize('aliases', [True, False])
def test_trace_starts_at_actual_p0_and_uses_live_base_force(tmp_path, config, aliases):
    directory, rows = recorded_run(tmp_path, config, aliases=aliases)
    trace = load_trace(directory)
    np.testing.assert_allclose(trace.xy_mm[0], [float(rows[1]['tcp_x'])*1000, float(rows[1]['tcp_y'])*1000])
    assert trace.time[0] == 0. and trace.state[0] == 'TARGET_SEARCH'
    assert trace.first_contact_index == 3
    assert trace.force_fields == (('filtered_force_base_fx', 'filtered_force_base_fy') if aliases else ('dfx', 'dfy'))
    np.testing.assert_allclose(trace.force[:, 2], np.hypot(trace.force[:, 0], trace.force[:, 1]))
    assert np.all(trace.force[:, 2] < 999.)  # Frozen direction-estimator telemetry is ignored.


def test_sensor_frame_force_is_never_relabelled_as_base(tmp_path, config):
    directory, rows = recorded_run(tmp_path, config)
    rows[6]['processed_force_frame'] = 'Sensor'
    with (directory/'samples.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    assert np.isnan(load_trace(directory).force[5]).all()


@pytest.mark.parametrize('contact', [True, False])
def test_standard_geometry_legends_and_contact_start(tmp_path, config, contact):
    directory, _ = recorded_run(tmp_path, config, contact=contact)
    trace = load_trace(directory)
    figures = standard_figures(trace)
    assert [name for name, _ in figures] == ['trajectory_overview.png', 'trajectory_local_zoom.png']
    for local, (_, figure) in enumerate(figures):
        figure.canvas.draw()
        axis = figure.axes[0]
        extent = axis.get_window_extent()
        assert extent.width/extent.height > 1.5
        transform = axis.transData
        origin, x, y = transform.transform([[0, 0], [1, 0], [0, 1]])
        assert np.linalg.norm(x-origin) == pytest.approx(np.linalg.norm(y-origin))
        legend = axis.get_legend()
        assert legend.get_window_extent().x0 > extent.x1
        names = [label.get_text() for label in legend.get_texts()]
        assert names.count('Lost-edge recovery') <= 1 and len(names) <= 6
        start = trace.first_contact_index if local and contact else 0
        assert len(axis.lines[0].get_xdata()) == len(trace.time)-start
        np.testing.assert_allclose(axis.lines[0].get_xdata()[0], trace.xy_mm[start, 0])
        assert not axis.get_autoscale_on()


def test_display_thinning_retains_every_state_transition_and_event(tmp_path, config):
    directory, _ = recorded_run(tmp_path, config)
    trace = load_trace(directory)
    indices = display_indices(trace, rate_hz=1.)
    switches = np.flatnonzero(trace.state[1:] != trace.state[:-1])+1
    assert set(np.r_[switches, switches-1, trace.event_indices, 0, len(trace.time)-1]) <= set(indices)


class WallClock:
    now = 0.
    def __call__(self): return self.now


def click(view, key):
    axis = view.buttons[key].ax
    x, y = axis.transAxes.transform((.5, .5))
    for event in ('button_press_event', 'button_release_event'):
        view.figure.canvas.callbacks.process(event, MouseEvent(event, view.figure.canvas, x, y, button=1))


def test_real_widgets_synchronized_prefix_speed_restart_and_final_hold(tmp_path, config):
    directory, _ = recorded_run(tmp_path, config)
    trace = load_trace(directory)
    wall = WallClock()
    view = RunAnimation(trace, clock=wall)
    try:
        view.figure.canvas.draw()
        original_bounds = (view.xy.get_xlim(), view.xy.get_ylim())
        assert view.clock.speed == 1. and not view.first_contact.get_visible()
        wall.now += .02
        view.animation._step()  # Exercise actual FuncAnimation blitting, not a fake renderer.
        current = view.current_index
        np.testing.assert_allclose(view.current_tcp.get_data(), trace.xy_mm[current].reshape(2, 1))
        for component, line in enumerate(view.force_lines):
            t = line.get_xdata()
            assert np.all(t <= view.clock.position)
            np.testing.assert_allclose(line.get_ydata(), trace.force[[trace.sample_index(float(x)) for x in t], component])
        assert view.cursor.get_xdata()[0] == trace.time[current]
        click(view, 'play'); assert not view.clock.playing
        paused = view.clock.position
        wall.now += 2.; view.tick(); assert view.clock.position == paused
        click(view, 'play'); assert view.clock.playing
        for speed in (.5, 1., 2., 4.):
            click(view, f'speed_{speed:g}')
            before = view.clock.position
            wall.now += .001; view.tick()
            assert view.clock.position == pytest.approx(before+.001*speed)
        click(view, 'restart')
        assert view.clock.position == 0. and view.current_index == 0
        assert not view.first_contact.get_visible()
        wall.now += trace.duration+1.; view.animation._step()
        assert view.current_index == len(trace.time)-1 and not view.clock.playing
        assert view.final_stop.get_visible() and view.first_contact.get_visible()
        view.animation._init_draw()  # Resizing must not reset a paused/final frame to P0.
        assert view.current_index == len(trace.time)-1
        assert plt.fignum_exists(view.figure.number)
        assert (view.xy.get_xlim(), view.xy.get_ylim()) == original_bounds
    finally:
        plt.close(view.figure)


def test_outputs_and_launcher_leave_source_logs_unchanged(tmp_path, config):
    directory, _ = recorded_run(tmp_path, config)
    originals = {path.name: path.read_bytes() for path in directory.iterdir()}
    artifacts = generate_run_visualization(directory)
    assert [p.name for p in artifacts] == ['trajectory_overview.png', 'trajectory_local_zoom.png',
        'experiment_overview.png', 'first_contact_zoom.png', 'contact_analysis.json', '实验回放.html', 'play_visualization.sh',
        '打开实验回放.desktop']
    assert all(p.stat().st_size > 100 for p in artifacts)
    for name, content in originals.items():
        assert (directory/name).read_bytes() == content
    result = subprocess.run([str(directory/'play_visualization.sh'), '--help'], text=True, capture_output=True)
    assert result.returncode == 0 and '--run-dir' in result.stdout
    assert 'run_animation.py' in (directory/'play_visualization.sh').read_text()
    assert f'Exec="{directory}/play_visualization.sh"' in artifacts[-1].read_text()
    assert artifacts[-1].stat().st_mode & 0o111
    page = (directory/'实验回放.html').read_text()
    embedded = json.loads(re.search(r'<script type="application/json" id="replay-data">(.*?)</script>', page, re.S)[1])
    np.testing.assert_allclose(embedded['samples'][0][1:3], load_trace(directory).xy_mm[0])
    assert embedded['commands'] == []  # Absent historical commands stay absent.
    assert all(row[1:] == [None, None, None] for row in embedded['actual'])
    assert '<script src=' not in page and 'fetch(' not in page
    assert not list(directory.glob('*.gif')) and not list(directory.glob('*.mp4'))


def test_new_completion_hook_runs_only_after_device_cleanup(config, monkeypatch, tmp_path):
    from app import continuous_runtime as runtime
    from doubles import Devices, Sensor
    from test_continuous_runtime import prepare, args
    from visualization import run_plots
    target = prepare(config, monkeypatch)
    devices, sensor, paths = Devices(config, target), Sensor(), []
    def generate(path):
        assert not sensor.connected and not devices.receive.connected and not devices.control.connected
        assert (path/'summary.json').exists() and (path/'samples.csv').exists()
        paths.append(path)
        return []
    monkeypatch.setattr(run_plots, 'generate_run_visualization', generate)
    assert runtime.run(args(tmp_path), controller_factory=devices.controller, reader_factory=lambda *a: sensor) == 0
    assert len(paths) == 1 and paths[0].parent == tmp_path/'real/continuous'


@pytest.mark.parametrize('empty', [False, True])
def test_aborted_startup_has_launcher_and_no_invented_scan(tmp_path, config, empty):
    directory, rows = recorded_run(tmp_path, config)
    with (directory/'samples.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0]);writer.writeheader()
        if not empty:
            writer.writerow(rows[0])  # Startup return is not a P0 scan.
    artifacts = generate_run_visualization(directory)
    assert len(artifacts) == 4
    with pytest.raises(ValueError, match='No scan TCP|no control samples'):
        load_trace(directory)


def test_logger_records_live_kalman_input_norm_even_when_direction_estimate_is_frozen(tmp_path, config):
    from core.models import RobotState, PolicyCommand, Wrench
    from experiment_logging.data_logger import ExperimentLogger
    logger = ExperimentLogger(tmp_path, config, mode='real', strategy='continuous',
                              extra_sample_fields=('filtered_fxy',), workspace_logging=False)
    vectors = [(.2, .3), (.7, .1)]
    try:
        for i, (fx, fy) in enumerate(vectors):
            wrench = Wrench(fx, fy, 0., 0., 0., 0.)
            logger.log_sample(i*.01, wrench, wrench, RobotState(i*.01, np.zeros(6), np.zeros(6)),
                PolicyCommand(state='LOCAL_REACQUIRE', move=False, direction_xy=np.zeros(2),
                              speed=0., target_pose=np.zeros(6), contact_flag=False,
                              possible_corner=False), None, None,
                extra={'processed_force_frame': 'Base', 'filtered_fxy': 999.})
    finally:
        logger.close()
    with (logger.run_dir/'samples.csv').open() as handle:
        rows = list(csv.DictReader(handle))
    for row, (fx, fy) in zip(rows, vectors):
        assert float(row['fxy']) == pytest.approx(np.hypot(fx, fy))
        assert float(row['filtered_force_base_fx']) == fx
        assert float(row['filtered_force_base_fy']) == fy
        assert float(row['filtered_fxy']) == 999.

"""Offline integration and injectable GUI contracts; never open hardware/show()."""
import argparse
import csv
import json
import subprocess
import sys
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

import run_continuous_tracking as runner
from config.loader import load_config
from simulation.simulator import load_simulation_config
from simulation.continuous_session import SimulationSession, validate_scene
from simulation.continuous_preview import PreviewRun, ContinuousPreview, extrema_indices

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def config():
    cfg = load_config(ROOT / 'config.yaml')
    cfg['continuous_tracking']['max_runtime_sec'] = 12
    cfg['continuous_provenance'] = runner.git_provenance()
    return cfg


@pytest.fixture
def scene():
    return load_simulation_config(ROOT / 'simulation/scene_continuous.yaml')


def rows(path):
    with (path / 'samples.csv').open() as f:
        return list(csv.DictReader(f))


@pytest.mark.parametrize('fps,speed', [(10, 1), (20, 5), (5, 10)])
def test_runner_preview_exact_closed_loop_parity(config, scene, tmp_path, fps, speed):
    args = argparse.Namespace(config=ROOT/'config.yaml', execute=False, duration=12,
                              scene=ROOT/'simulation/scene_continuous.yaml', output=tmp_path/'headless')
    assert runner.run(args) == 0
    expected = rows(next(args.output.glob('run_*')))
    preview = PreviewRun(config, scene, tmp_path/'preview')
    assert preview.status == 'READY' and preview.session.robot.time == 0
    preview.start(0)
    preview.set_speed(speed, 0)
    wall = 0
    while preview.status == 'RUNNING':
        wall += 1/fps
        preview.tick(wall, work_budget_sec=10)
    actual = rows(preview.run_dir)
    assert len(actual) >= len(expected)
    for a, b in zip(actual, expected):
        for field in ('monotonic_sec', 'current_state', 'tcp_x', 'tcp_y', 'tcp_vx', 'tcp_vy',
                      'raw_fx', 'raw_fy', 'dfx', 'dfy', 'fxy', 'commanded_speed_mps',
                      'commanded_direction_x', 'commanded_direction_y', 'direction_valid'):
            assert a[field] == b[field], (field, a, b)
    assert {'FIRST_CONTACT', 'CONTINUOUS_TRACKING'} <= {r['current_state'] for r in actual}
    assert preview.session.policy.reason == 'maximum experiment duration reached'


def test_pause_edit_reset_and_independent_scene(config, scene, tmp_path):
    model = PreviewRun(config, scene, tmp_path)
    model.start(0)
    for n in range(130):
        model.tick((n+1)*.1, work_budget_sec=10)
        if model.status == 'ENDED':
            break
    # Use a fresh run with contact memory, before its time budget.
    model.reset()
    model.start(0)
    for n in range(110):
        model.tick((n+1)*.1, work_budget_sec=10)
    model.pause(11)
    old = model.session
    assert old.policy.last_reliable_contact is not None
    pose, time = old.robot.pose.copy(), old.robot.time
    filtered = old.preprocessor._filtered_sensor.copy()
    model.tick(500, work_budget_sec=10)
    assert old.robot.time == time
    np.testing.assert_array_equal(old.robot.pose, pose)
    np.testing.assert_array_equal(old.preprocessor._filtered_sensor, filtered)
    model.start(500)
    model.tick(500.01, work_budget_sec=10)
    assert model.session is old and old.robot.time == pytest.approx(time+.01)
    old_dir = model.run_dir
    p0, p1 = deepcopy(model.scene['calibration_point_0']), deepcopy(model.scene['calibration_point_1'])
    model.edit(target_center=[.02, .02])
    assert model.status == 'READY' and model.session is not old
    assert not model.history and model.session.robot.time == 0
    assert model.session.policy.last_reliable_contact is None
    assert model.session.preprocessor._filtered_sensor is None
    assert model.scene['calibration_point_0'] == p0 and model.scene['calibration_point_1'] == p1
    with (old_dir/'policy_waypoints.csv').open() as f:
        assert 'SCENE_CHANGED' in {r['event_type'] for r in csv.DictReader(f)}
    assert json.loads((old_dir/'summary.json').read_text())['termination_reason'] == 'STOP_USER_REQUEST'
    saved = tmp_path/'independent_scene.yaml'
    model.save_scene(saved)
    assert validate_scene(load_simulation_config(saved))['target_center'] == [.02, .02]
    with pytest.raises(FileExistsError):
        model.save_scene(saved)
    with pytest.raises(FileExistsError):
        model.save_scene(ROOT/'config.yaml')
    model.edit(target_center=[0, 0])
    model.load_scene(saved)
    assert model.scene['target_center'] == [.02, .02]
    model.start(0)
    assert model.run_dir != old_dir
    snap = yaml.safe_load((model.run_dir/'config_snapshot.yaml').read_text())
    assert snap['continuous_simulation']['force_model']['random_seed'] == 7
    assert snap['continuous_provenance']['branch'] == 'experiment/continuous-tracking'
    assert snap['continuous_simulation']['simulation']['dt'] == .01
    model.stop()


@pytest.mark.parametrize('change', [
    {'target_center': [.17, 0]}, {'target_center': [-.065, 0]},
    {'target_radius': 0}, {'target_radius': float('nan')},
    {'calibration_point_1': [-.065, 0]}, {'calibration_point_0': [float('inf'), 0]},
    {'start_point': [-1, 0]}, {'target_rotation_deg': float('nan')},
])
def test_invalid_layout_rejected(config, scene, tmp_path, change):
    model = PreviewRun(config, scene, tmp_path)
    before = deepcopy(model.scene)
    with pytest.raises(ValueError):
        model.edit(**change)
    assert model.scene == before and model.status == 'READY'
    assert not list(tmp_path.glob('run_*'))


def test_shapes_rotation_no_target_failure(config, scene, tmp_path):
    model = PreviewRun(config, scene, tmp_path)
    for shape in ('square', 'circle', 'triangle'):
        model.choose_shape(shape)
        model.rotate(15)
        assert model.scene['target_rotation_deg'] == 15
        model.rotate(-15)
        if shape == 'square':
            assert model.scene['target_width'] == model.scene['target_height']
    model.edit(target_shape='circle', target_center=[0, .07], target_radius=.02)
    model.base_config['continuous_tracking']['search_max_time_sec'] = .2
    model.reset()
    model.start(0)
    model.tick(1, work_budget_sec=10)
    summary = json.loads((model.run_dir/'summary.json').read_text())
    assert summary['termination_reason'] == 'STOP_SEARCH_LIMIT'
    assert model.session.policy.initial_contact is None
    assert 'SUCCESS' not in model.message


def test_callback_budget_is_bounded(config, scene, tmp_path):
    model = PreviewRun(config, scene, tmp_path)
    model.start(0)
    model.set_speed(10, 0)
    count = model.tick(1000, work_budget_sec=10)
    assert 0 < count <= model.max_steps_per_tick
    assert model.session.robot.time == pytest.approx(count*.01)
    assert model.effective_speed < 10
    model.stop()


def test_gui_callbacks_existing_artists_synchrony_and_no_devices(config, scene, tmp_path, monkeypatch):
    import matplotlib
    matplotlib.use('Agg', force=True)
    import matplotlib.pyplot as plt
    def forbidden(*a, **k):
        raise AssertionError('hardware or blocking GUI')
    monkeypatch.setattr(runner, 'URRTDEController', forbidden)
    monkeypatch.setattr(runner, 'PX6DReader', forbidden)
    monkeypatch.setattr(plt, 'show', forbidden)
    model = PreviewRun(config, scene, tmp_path)
    view = ContinuousPreview(model)
    axes = tuple(view.figure.axes)
    assert view.xy.get_aspect() == 1
    assert set(view.paths) == {'SEARCH', 'RELIABLE', 'UNCERTAIN', 'RECOVERY'}
    view.on_key(SimpleNamespace(key='1'))
    view.on_key(SimpleNamespace(key=']'))
    p0 = deepcopy(model.scene['calibration_point_0'])
    view.on_press(SimpleNamespace(inaxes=view.xy, xdata=0., ydata=0., button=1))
    view.on_release(SimpleNamespace(inaxes=view.xy, xdata=10., ydata=10., button=1))
    assert model.scene['target_center'] == [.01, .01]
    assert model.scene['calibration_point_0'] == p0
    model.start(0)
    model.tick(.2, work_budget_sec=10)
    view.draw()
    sample = model.history[-1]
    assert view.display_time == sample.time
    assert view.cursor.get_xdata()[0] == sample.time
    np.testing.assert_array_equal(view.tcp.get_xdata(), [sample.robot.pose[0]*1000])
    assert max(view.force_lines[0].get_xdata()) <= sample.time
    assert tuple(view.figure.axes) == axes
    model.stop()
    view.save_png()
    assert (model.run_dir/'continuous_preview.png').stat().st_size > 1000
    view.on_close(None)
    assert model.session.policy.stop_reason.value == 'STOP_USER_REQUEST'
    plt.close(view.figure)


def test_force_downsampling_keeps_extrema():
    values = np.zeros((3000, 3))
    values[111, 0] = 19
    values[177, 1] = -27
    values[1555, 2] = 30
    indices = extrema_indices(values, buckets=80)
    assert {111, 177, 1555} <= set(indices)
    assert len(indices) <= 2+6*80


def test_cli_conflict_and_lazy_import(tmp_path):
    conflict = subprocess.run([sys.executable, 'run_continuous_tracking.py', '--preview', '--execute'],
                              cwd=ROOT, capture_output=True, text=True)
    assert conflict.returncode == 2 and 'not allowed' in conflict.stderr
    result = subprocess.run([sys.executable, '-c',
        "import sys; import run_continuous_tracking; assert 'matplotlib.pyplot' not in sys.modules; "
        "assert 'simulation.continuous_preview' not in sys.modules"], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_actual_buttons_start_pause_speed_stop_reset_and_scene_io(config, scene, tmp_path):
    import matplotlib
    matplotlib.use('Agg', force=True)
    import matplotlib.pyplot as plt
    model = PreviewRun(config, scene, tmp_path)
    view = ContinuousPreview(model)
    def click(label):
        button = next(b for b in view.buttons if b.label.get_text() == label)
        button._observers.process('clicked', None)
    click('Start')
    assert model.status == 'RUNNING'
    old = model.session
    click('Pause/Continue')
    assert model.status == 'PAUSED'
    click('Pause/Continue')
    assert model.status == 'RUNNING' and model.session is old
    click('5x')
    assert model.speed == 5
    click('Stop')
    assert model.status == 'ENDED'
    path = model.run_dir
    click('Save PNG')
    assert (path/'continuous_preview.png').is_file()
    click('Reset')
    assert model.status == 'READY' and not model.history
    view.path_box.set_val(str(tmp_path/'buttons_scene.yaml'))
    click('Save scene')
    assert (tmp_path/'buttons_scene.yaml').is_file()
    model.edit(target_center=[.01, .01])
    click('Load scene')
    assert model.scene['target_center'] == [0., 0.]
    view.on_close(None)
    plt.close(view.figure)


def test_drag_running_freezes_then_logs_scene_change(config, scene, tmp_path):
    import matplotlib
    matplotlib.use('Agg', force=True)
    import matplotlib.pyplot as plt
    model = PreviewRun(config, scene, tmp_path)
    view = ContinuousPreview(model)
    model.start(0)
    model.tick(.2, work_budget_sec=10)
    old, path, t = model.session, model.run_dir, model.session.robot.time
    mouse = lambda x, y: SimpleNamespace(inaxes=view.xy, xdata=x, ydata=y, button=1)
    view.on_press(mouse(0, 0))
    view.on_motion(mouse(15, 10))
    model.tick(999, work_budget_sec=10)
    assert old.robot.time == t and model.scene['target_center'] == [0, 0]
    assert view.drag_outline.get_visible()
    view.on_release(mouse(15, 10))
    assert not view.drag_outline.get_visible()
    assert model.scene['target_center'] == [.015, .01] and model.status == 'READY'
    assert model.session is not old
    assert 'SCENE_CHANGED' in (path/'policy_waypoints.csv').read_text()
    plt.close(view.figure)


def test_scene_never_delivered_to_policy(config, scene, monkeypatch):
    import simulation.continuous_session as module
    actual = module.ContinuousTrackingPolicy
    def check(cfg):
        assert set(cfg) == {'policy', 'continuous_tracking', 'robot'}
        assert not any('target_center' in part for part in cfg.values())
        return actual(cfg)
    monkeypatch.setattr(module, 'ContinuousTrackingPolicy', check)
    session = SimulationSession(config, scene)
    session.capture_bias()
    session.step()


def test_no_gui_fallback_headless_and_replay(tmp_path):
    import os
    env = dict(os.environ, MPLBACKEND='Agg', MPLCONFIGDIR='/tmp/continuous-mpl', DISPLAY='', WAYLAND_DISPLAY='')
    result = subprocess.run([sys.executable, 'run_continuous_tracking.py', '--dry-run', '--duration', '.1',
                             '--output', str(tmp_path)], cwd=ROOT, env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    run_dir = next(tmp_path.glob('run_*'))
    result = subprocess.run([sys.executable, 'tools/visualize_continuous_run.py', str(run_dir), '--format', 'none'],
                             cwd=ROOT, env=env, capture_output=True, text=True)
    assert result.returncode == 0 and (run_dir/'continuous_summary.png').is_file(), result.stderr
    result = subprocess.run([sys.executable, 'run_continuous_tracking.py', '--preview'],
                             cwd=ROOT, env=env, capture_output=True, text=True)
    assert result.returncode != 0
    assert 'No interactive Matplotlib backend' in result.stdout+result.stderr


def test_pause_during_first_contact_does_not_satisfy_confirmation(config, scene, tmp_path):
    model = PreviewRun(config, scene, tmp_path)
    model.start(0)
    wall = 0
    while model.session.policy.state.value != 'FIRST_CONTACT':
        wall += .01
        model.tick(wall, work_budget_sec=10)
        assert wall < 11
    model.pause(wall)
    before = (model.session.robot.time, model.session.policy.contact_hold_elapsed,
              model.session.policy.settle_hold_elapsed, model.session.policy._confirm_started)
    model.tick(wall+10000, work_budget_sec=10)
    assert before == (model.session.robot.time, model.session.policy.contact_hold_elapsed,
                      model.session.policy.settle_hold_elapsed, model.session.policy._confirm_started)
    model.start(wall+10000)
    assert model.session.policy.state.value == 'FIRST_CONTACT'
    model.stop()


def test_programmatic_conflict_precedes_construction(monkeypatch, tmp_path):
    def forbidden(*a, **k):
        raise AssertionError('hardware construction')
    monkeypatch.setattr(runner, 'URRTDEController', forbidden)
    monkeypatch.setattr(runner, 'PX6DReader', forbidden)
    with pytest.raises(ValueError, match='mutually exclusive'):
        runner.run(argparse.Namespace(preview=True, execute=True))


def test_full_path_bounded_and_cross_state_decimation_conservative(config, scene, tmp_path):
    model = PreviewRun(config, scene, tmp_path)
    model.max_path_points = 8
    model.start(0)
    for i in range(30):
        model.tick((i+1)*.01, work_budget_sec=10)
    assert 2 <= len(model.path_history) <= 8
    assert model.path_history[0][0] == 0
    assert model.path_history[-1][0] == model.history[-1].time
    # Force an incoming phase transition in a pair about to be merged. The
    # decimated bridge must not advertise an uninterrupted reliable segment.
    point = model.path_history[-1]
    model.path_history = [(i, point[1], point[2], 'RELIABLE' if i % 2 else 'RECOVERY') for i in range(8)]
    model.tick(.31, work_budget_sec=10)
    assert all(p[3] == 'UNCERTAIN' for p in model.path_history[1:])
    model.stop()

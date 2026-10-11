"""Optional straight-edge measurement; all devices/files are offline doubles."""
from copy import deepcopy
import json
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace

import numpy as np
import pytest

from app.contact_force_direction import analyze_contact, force_angle, snapshot_edge
from app import single_point_ui
from calibration import target_edge
from calibration.target_edge import edge_geometry, make_edge, load_edge, save_edge
from robot.control_mode import ControlMode
from simulation.single_point import demo_calibration
from tests.doubles import Devices, Keyboard
from tests.test_single_point import rig, settings
from tests.test_html_replay import payload
from tools.visualize_contact_experiment import make_demo
from visualization.run_html import write_html_replay
from visualization.run_plots import load_trace
import run_single_point_contact as entry


def edge(config, a=None, b=None):
    return make_edge(a or [.54, .1, .3, .1, .2, .3], b or [.64, .1, -.01, 2., 1., 0.],
                     config['robot']['robot_ip'], config['tcp']['offset'])


def trigger(force=(0., 1.), before=(.2, 1.2, 0.)):
    return dict(threshold_trigger_sec=2.5, force_trigger_sample_sec=2.499,
        actual_at_threshold=dict(timestamp=2.5, pose=[.58, .1, -.01, 1., 2., 0.]),
        force_at_threshold=dict(frame='Base', processed_wrench_base=[*force, 0., 0., 0., 0.],
                                unfiltered_force_base=list(before), raw_wrench_sensor=[7., 0., 0., 0., 0., 0.]))


def snapshot(config):
    return dict(status='available', calibration=edge(config), source_path='offline.yaml')


def test_edge_geometry_uses_xy_only_preserves_original_pose_precision(config):
    a = [.1234567890123456, .2, .8, .1, .2, .3]
    b = [.1534567890123456, .24, -.7, 2., -1., .9]
    result = edge(config, a, b)
    np.testing.assert_allclose(result['tangent_base_xy'], [.6, .8])
    np.testing.assert_allclose(result['normal_base_xy'], [-.8, .6])
    assert result['length_xy_m'] == pytest.approx(.05)
    assert result['direction_deg'] == pytest.approx(53.13010235415598)
    assert result['A']['tcp_pose'] == a and result['B']['tcp_pose'] == b


@pytest.mark.parametrize('tangent,force,angle', [([1, 0], [0, 1], 90), ([1, 1], [-1, 1], 90),
                                               ([1, 1], [1, 0], 45), ([1, 0], [1, 0], 0)])
def test_angles_and_force_reversal(tangent, force, angle):
    normal = force_angle(force, tangent)
    reverse = force_angle([-v for v in force], tangent)
    assert normal['angle_to_edge_deg'] == pytest.approx(angle)
    assert normal['perpendicular_error_deg'] == pytest.approx(90-angle)
    assert reverse['perpendicular_error_deg'] == pytest.approx(normal['perpendicular_error_deg'])


@pytest.mark.parametrize('distance', [0, .000015, .004999])
def test_near_edge_points_rejected_with_mm_reason(distance):
    with pytest.raises(ValueError, match='mm.*最小距离'):
        edge_geometry([0]*6, [distance, 0, 99, 1, 2, 3])
    assert edge_geometry([0]*6, [.005, 0, 99, 1, 2, 3])['length_xy_m'] == .005
    assert edge_geometry([0]*6, [.003, 0, 99, 1, 2, 3], .002)['length_xy_m'] == .003


@pytest.mark.parametrize('force', [[0, 0], [1e-9, 0], [np.nan, 1], [np.inf, 1], [1], None])
def test_invalid_force_never_produces_an_angle(force):
    with pytest.raises((ValueError, TypeError)):
        force_angle(force, [1, 0])


def test_analysis_uses_saved_base_trigger_frame_and_records_unfiltered_comparison(config):
    result = trigger()
    result['actual_stop_pose'] = [99]*6
    original = deepcopy(result)
    analysis = analyze_contact(snapshot(config), result)
    assert analysis['status'] == 'available' and analysis['angle_to_edge_deg'] == 90
    assert analysis['unfiltered_base_xy_N'] == [.2, 1.2]
    assert analysis['unfiltered_analysis']['angle_to_edge_deg'] < 90
    assert analysis['force_at_threshold'] == result['force_at_threshold']
    assert analysis['actual_at_threshold']['pose'][:2] == [.58, .1]
    assert analysis['threshold_trigger_sec'] == 2.5 and analysis['force_sample_sec'] == 2.499
    assert result == original


@pytest.mark.parametrize('fault', ['no_trigger', 'sensor_frame', 'missing_frame', 'nan_force', 'tiny_force'])
def test_missing_or_invalid_trigger_has_no_fabricated_result(config, fault):
    result = trigger()
    if fault == 'no_trigger': result['force_at_threshold'] = None
    elif fault == 'sensor_frame': result['force_at_threshold']['frame'] = 'Sensor'
    elif fault == 'missing_frame': del result['force_at_threshold']['frame']
    elif fault == 'nan_force': result['force_at_threshold']['processed_wrench_base'][0] = np.nan
    else: result['force_at_threshold']['processed_wrench_base'] = [0]*6
    analysis = analyze_contact(snapshot(config), result)
    assert analysis['status'] == 'unavailable' and analysis['reason']
    assert 'angle_to_edge_deg' not in analysis


def test_old_trigger_missing_unfiltered_does_not_use_sensor_force(config):
    result = trigger()
    del result['force_at_threshold']['unfiltered_force_base']
    analysis = analyze_contact(snapshot(config), result)
    assert analysis['status'] == 'available' and analysis['unfiltered_base_xy_N'] is None
    assert analysis['unfiltered_analysis'] is None and 'Sensor' in analysis['unfiltered_reason']


def test_atomic_edge_save_backup_readback_and_full_precision(tmp_path, config, monkeypatch):
    path = tmp_path/'edge.yaml'
    first, _ = save_edge(path, edge(config))
    original = path.read_bytes()
    candidate = edge(config, b=[.6412345678901234, .11, .9, .3, -.4, .1])
    reads = []
    load = target_edge.load_edge
    monkeypatch.setattr(target_edge, 'load_edge', lambda p: (reads.append(p), load(p))[1])
    saved, backup = save_edge(path, candidate)
    assert saved == candidate and reads == [path.resolve()]
    assert backup.read_bytes() == original and load(path) != first


@pytest.mark.parametrize('failure', ['replace', 'readback'])
def test_save_failure_retains_old_edge_and_backup(tmp_path, config, monkeypatch, failure):
    path = tmp_path/'edge.yaml'
    save_edge(path, edge(config))
    original = path.read_bytes()
    if failure == 'replace':
        def fail(*args): raise OSError('mock disk failure')
        monkeypatch.setattr(target_edge, '_replace_bytes', fail)
    else:
        def fail(*args): raise ValueError('mock readback failure')
        monkeypatch.setattr(target_edge, 'load_edge', fail)
    with pytest.raises((OSError, ValueError), match='mock'):
        save_edge(path, edge(config, b=[.7, .12, 0, 1, 2, 3]))
    assert path.read_bytes() == original
    assert all(p.read_bytes() == original for p in tmp_path.glob('edge.backup-*.yaml'))


@pytest.mark.parametrize('failure', ['cancel_b', 'close_points', 'tcp_mismatch'])
def test_capture_failure_never_replaces_old_edge_or_p0(tmp_path, config, failure):
    args = SimpleNamespace(target_edge_calibration=tmp_path/'edge.yaml')
    save_edge(args.target_edge_calibration, edge(config))
    original = args.target_edge_calibration.read_bytes()
    points = iter([[.54, .1, .3, .1, .2, .3], [.64, .1, .1, 1, 2, 3]])
    calls = []
    def capture(project, point_name):
        calls.append(point_name)
        if len(calls) == 2 and failure == 'cancel_b': raise KeyboardInterrupt('cancel')
        point = next(points)
        if len(calls) == 2 and failure == 'close_points': point[0] = .540015
        tcp = list(project['tcp']['offset'])
        if len(calls) == 2 and failure == 'tcp_mismatch': tcp[0] += .01
        return np.array(point), tcp
    with pytest.raises((KeyboardInterrupt, ValueError)):
        entry.calibrate_target_edge(args, config, capture=capture)
    assert args.target_edge_calibration.read_bytes() == original
    assert len(list(tmp_path.glob('edge.backup*'))) == 0


def test_cannot_save_edge_over_p0_calibration(tmp_path, config):
    path = tmp_path/'single_point_calibration.yaml'
    path.write_text('protected P_ref/P0 laboratory data', encoding='utf-8')
    original = path.read_bytes()
    args = SimpleNamespace(calibration=path, target_edge_calibration=path)
    with pytest.raises(ValueError, match='禁止覆盖'):
        entry.calibrate_target_edge(args, config, capture=lambda *a, **k: pytest.fail('must not connect'))
    assert path.read_bytes() == original


def test_first_save_readback_failure_does_not_leave_unverified_edge(tmp_path, config, monkeypatch):
    path = tmp_path/'edge.yaml'
    def fail(*args): raise ValueError('mock readback failure')
    monkeypatch.setattr(target_edge, 'load_edge', fail)
    with pytest.raises(ValueError, match='mock'):
        save_edge(path, edge(config))
    assert not path.exists() and not list(tmp_path.glob('*.tmp'))


def test_menu8_independent_of_p0_and_uses_readonly_receive_local(tmp_path, config, settings, monkeypatch, capsys):
    args = SimpleNamespace(execute=True, calibration=tmp_path/'broken.yaml', target_edge_calibration=tmp_path/'edge.yaml')
    args.calibration.write_text('not a calibration file', encoding='utf-8')
    before = args.calibration.read_bytes()
    points = iter([[.54, .1, .3, .1, .2, .3], [.64, .1, -.01, 2., 1., 0.]])
    devices = []
    monkeypatch.setattr(single_point_ui, 'read_control_mode', lambda ip: ControlMode('local', 'mock false'))
    monkeypatch.setattr(entry, 'read_tcp_offset_readonly', lambda ip: config['tcp']['offset'])
    original_capture = entry.capture_pose
    def capture(project, **kw):
        device = Devices(config, next(points)); devices.append(device)
        return original_capture(project, confirm=lambda prompt: True, controller_factory=device.controller, **kw)
    monkeypatch.setattr(entry, 'capture_pose', capture)
    answers = iter(['8', '0'])
    monkeypatch.setattr('builtins.input', lambda prompt: next(answers))
    entry.menu(args, config, settings)
    output = capsys.readouterr().out
    assert '目标直边标定成功' in output and '回读校验：通过' in output and 'Local' in output
    assert all(d.receive_count == 1 and d.control_count == 0 and not d.control.calls for d in devices)
    assert len(devices) == 2 and args.calibration.read_bytes() == before
    assert load_edge(args.target_edge_calibration)['length_xy_m'] == pytest.approx(.1)


@pytest.mark.parametrize('mode', ['missing', 'malformed', 'tcp_mismatch', 'ip_mismatch'])
def test_optional_edge_failure_never_blocks_experiment(tmp_path, config, mode):
    path = tmp_path/'edge.yaml'
    if mode == 'malformed': path.write_text('x: [broken', encoding='utf-8')
    elif mode != 'missing':
        candidate = edge(config)
        if mode == 'tcp_mismatch': candidate['tcp_offset'][0] += .01
        else: candidate['robot_ip'] = 'different'
        save_edge(path, candidate)
    result = snapshot_edge(path, config)
    assert result['status'] == 'unavailable' and result['reason']
    assert analyze_contact(result, trigger())['status'] == 'unavailable'


def test_runtime_retains_exact_trigger_before_and_after_filter_after_brake(config, settings):
    trial, controller, logger = rig(config, settings)
    captured = {}
    stop = controller.request_stop
    def request(**kw):
        captured.update(before=trial.processor.force_base.copy(), after=trial.processed.array().copy(),
                        sensor=trial.raw.array().copy(), tcp=trial.robot.pose.copy(), end=trial.reader.end)
        return stop(**kw)
    controller.request_stop = request
    trial.record_precontact()
    result = trial.run()
    assert result['status'] == 'contact'
    saved = result['force_at_threshold']
    np.testing.assert_array_equal(saved['unfiltered_force_base'], captured['before'])
    np.testing.assert_array_equal(saved['processed_wrench_base'], captured['after'])
    np.testing.assert_array_equal(saved['raw_wrench_sensor'], captured['sensor'])
    np.testing.assert_array_equal(result['actual_at_threshold']['pose'], captured['tcp'])
    assert result['force_trigger_sample_sec'] == captured['end']
    assert not np.array_equal(captured['after'][:3], trial.processor.filtered_force_base)


@pytest.mark.parametrize('with_edge', [True, False])
def test_run_writes_optional_analysis_and_immutable_edge_snapshot(tmp_path, config, settings, with_edge):
    args = SimpleNamespace(execute=False, config=entry.ROOT/'config.yaml', output=tmp_path/'runs',
                           target_edge_calibration=tmp_path/'edge.yaml')
    if with_edge: save_edge(args.target_edge_calibration, edge(config))
    result = entry.run_trial(args, config, settings, demo_calibration(config), 'A', keyboard_factory=Keyboard)
    assert result['status'] == 'contact'
    run = next((args.output/'simulation'/'single_point').glob('run_*'))
    analysis = json.loads((run/'contact_force_direction_analysis.json').read_text())
    frozen = json.loads((run/'target_edge_calibration_snapshot.json').read_text())
    assert analysis['force_at_threshold'] == result['force_at_threshold']
    assert analysis['target_edge'] == frozen
    assert analysis['status'] == ('available' if with_edge else 'unavailable')
    assert payload(run/'实验回放.html')['contact_force_direction_analysis'] == analysis
    if with_edge:
        save_edge(args.target_edge_calibration, edge(config, b=[.54, .2, 1, 2, 3, 4]))
        assert analysis['target_edge']['calibration']['tangent_base_xy'] == [1, 0]


@pytest.mark.skipif(shutil.which('gjs') is None, reason='system JavaScript engine unavailable')
@pytest.mark.parametrize('force', [(0., 1.), (-1., -2.)])
def test_replay_edge_force_signs_equal_scale_toggles_and_legacy(tmp_path, config, force):
    trace = load_trace(make_demo(tmp_path))
    trace.config['single_point_experiment'] = {}
    analysis = analyze_contact(snapshot(config), trigger(force))
    (trace.run_dir/'contact_force_direction_analysis.json').write_text(json.dumps(analysis), encoding='utf-8')
    page = write_html_replay(trace, trace.run_dir)
    harness = Path(__file__).with_name('html_replay_harness.js').read_text().split("check(api.rate===.25")[0]
    harness += """
const overlay=api.geometry.xy.edgeOverlay,g=api.geometry.xy;
check(JSON.stringify(overlay.boundary)===JSON.stringify([[540,100],[640,100]]),'edge SI to mm');
const f=data.contact_force_direction_analysis.filtered_base_xy_N,a=overlay.forceArrow;
check(JSON.stringify(a.start)===JSON.stringify([580,100]),'actual threshold TCP, not stop');
const dx=a.end[0]-a.start[0],dy=a.end[1]-a.start[1];
check(Math.abs(dx*f[1]-dy*f[0])<1e-8&&dx*f[0]+dy*f[1]>0,'measured force signs');
check(Math.abs(g.x(1)-g.x(0)+g.y(1)-g.y(0))<1e-8,'equal Base XY aspect');
const inputs=elements['edge-options'].children.map(v=>v.children[0]);
check(inputs.length===3&&inputs.every(v=>v.checked),'three default checked options');
inputs[0].checked=false;inputs[0].onchange();
check(api.geometry.xy.edgeOverlay.boundary===null&&api.geometry.xy.edgeOverlay.forceArrow,'boundary independent');
inputs[1].checked=false;inputs[1].onchange();
check(api.geometry.xy.edgeOverlay.forceArrow===null,'force arrow hidden');
inputs[2].checked=false;inputs[2].onchange();
check(elements['edge-analysis-panel'].hidden,'analysis hidden');
check(elements['edge-analysis'].textContent.includes('理想垂直夹角：90°'),'analysis values');
print(JSON.stringify({result:'PASS',tests}));
"""
    js = tmp_path/'edge-harness.js'; js.write_text(harness, encoding='utf-8')
    result = subprocess.run(['gjs', str(js), str(page)], capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)['result'] == 'PASS'
    # Old records have no analysis file: the original replay remains functional.
    (trace.run_dir/'contact_force_direction_analysis.json').unlink()
    page = write_html_replay(trace, trace.run_dir)
    assert payload(page)['contact_force_direction_analysis'] is None
    legacy_js = tmp_path/'legacy-harness.js'
    legacy_js.write_text(harness.split('const overlay=')[0]+"check(api.geometry.xy.edgeOverlay.forceArrow===null,'no fabricated legacy force');print('PASS');")
    result = subprocess.run(['gjs', str(legacy_js), str(page)], capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stderr

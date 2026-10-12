"""Fixed scene acceptance tests: production runtime over hardware-free interfaces."""
from argparse import Namespace
from copy import deepcopy
import csv
import json
from pathlib import Path
import re
import shutil
import subprocess
import time

import numpy as np
import pytest
import yaml

from app import continuous_runtime as runtime
from app.fixed_target import ROOT, configure_fixed, prepare_fixed, run_fixed
from calibration.probe_alignment import downward_probe_orientation, probe_tilt_deg
from calibration.target_geometry import make_geometry, save_geometry, load_geometry, teach_target
from core.models import Wrench, RobotState
from doubles import Devices, Sensor, Keyboard
from policy.continuous_tracking import ContinuousTrackingPolicy
from robot.rtde_controller import RobotError, _rotvec_to_matrix, check_continuous_xy
from sensor.force_preprocess import WrenchPreprocessor
from visualization.run_html import write_html_replay
from visualization.run_plots import load_trace


@pytest.fixture
def settings(tmp_path):
    path = tmp_path/'points.yaml'
    shutil.copyfile(ROOT/'single_point_calibration.yaml', path)
    return path


def prepared(config, settings, name='A'):
    configure_fixed(config)
    return prepare_fixed(config, ROOT/'config.yaml', settings, name)


@pytest.mark.parametrize('name', ['A', 'B', 'C'])
def test_inherits_controller_and_policy_with_reference_z(config, settings, name):
    before = deepcopy(config)
    robot, start = prepared(config, settings, name)
    saved = yaml.safe_load((ROOT/'single_point_calibration.yaml').read_text())
    np.testing.assert_array_equal(start[:2], saved['groups'][name]['tcp_pose'][:2])
    assert start[2] == saved['P_ref'][2]
    direction = np.array(saved['P_ref'][:2])-start[:2]
    np.testing.assert_allclose(config['policy']['search_direction_xy'], direction/np.linalg.norm(direction))
    assert probe_tilt_deg(start[3:], config['calibration']['probe_axis_tcp']) < 1e-5
    for section in ('robot', 'sensor', 'tcp', 'safe_return', 'logging', 'workspace'):
        assert config[section] == before[section]
    assert config['continuous_tracking'] == {**before['continuous_tracking'],
        'search_boundary_margin': config['continuous_tracking']['search_boundary_margin']}
    assert config['policy'] == {**before['policy'], 'search_direction_xy': config['policy']['search_direction_xy']}
    assert config['preprocessing']['kalman'] == before['preprocessing']['kalman']
    assert config['preprocessing']['coordinate_transform'] == before['preprocessing']['coordinate_transform']
    assert config['preprocessing']['baseline'] == before['preprocessing']['baseline']
    assert config['preprocessing']['granular_baseline']['capture_on_start'] is False
    assert robot['max_tcp_speed'] == before['robot']['max_tcp_speed']
    assert robot['speed_acceleration'] == before['robot']['speed_acceleration']
    assert robot['stop_deceleration'] == before['robot']['stop_deceleration']
    # Workspace-only guard supplies the existing ray distance; never 2*P_ref.
    geometric = config['continuous_search_geometry']['geometric_distance_m']
    assert geometric != pytest.approx(2*np.linalg.norm(direction))
    assert config['continuous_search_geometry']['usable_distance_m'] == pytest.approx(
        geometric-config['continuous_tracking']['search_boundary_margin'])
    with pytest.raises(RobotError):
        check_continuous_xy(robot, [.95, .4])


@pytest.mark.parametrize('name', ['A', 'B', 'C'])
def test_same_workspace_and_search_envelope_as_main(config, settings, monkeypatch, name):
    from app import configuration, fixed_target
    original = deepcopy(config)
    workspace_path = configuration.workspace_calibration_path(config, ROOT/'config.yaml')
    before = workspace_path.read_bytes()
    loader = configuration.load_workspace_calibration
    loaded = []
    def record(path):
        loaded.append(Path(path).resolve())
        return loader(path)
    monkeypatch.setattr(configuration, 'load_workspace_calibration', record)
    monkeypatch.setattr(fixed_target, 'load_workspace_calibration', record)
    robot, start = prepared(config, settings, name)
    # Give the original preflight the identical taught start/direction. It still
    # loads and calculates all workspace geometry through its normal path.
    monkeypatch.setattr(configuration, 'load_scan_calibration',
        lambda *a, **kw: deepcopy(config['continuous_saved_calibration']))
    main_robot, main_start = configuration.prepare_real(original, ROOT/'config.yaml')
    assert loaded == [workspace_path.resolve()] * 2
    assert config['continuous_calibration_sources']['workspace'] == original['continuous_calibration_sources']['workspace']
    assert config['continuous_workspace_calibration'] == original['continuous_workspace_calibration']
    raw = yaml.safe_load(before)['raw_points']
    np.testing.assert_array_equal(robot['continuous_xy_polygon'],
        [[raw[f'P{i}'][axis] for axis in 'xy'] for i in range(4)])
    assert config['continuous_execution_envelope'] == original['continuous_execution_envelope']
    assert config['continuous_search_geometry'] == original['continuous_search_geometry']
    assert robot == main_robot
    np.testing.assert_array_equal(start, main_start)
    assert config['workspace'] == original['workspace']
    assert config['workspace']['enabled'] is False
    assert config['continuous_tracking']['real_test_xy_limits'] is None
    assert workspace_path.read_bytes() == before


def test_original_polygon_rejects_point_inside_its_bounding_rectangle(config, settings):
    robot, start = prepared(config, settings)
    check_continuous_xy(robot, start[:2])
    point = [.885, .56]  # Within the saved AABB, beyond its sloping right edge.
    rectangle_only = {key: value for key, value in robot.items() if key != 'continuous_xy_polygon'}
    check_continuous_xy(rectangle_only, point)
    with pytest.raises(RobotError, match='calibrated sandbox boundary'):
        check_continuous_xy(robot, point)


@pytest.mark.parametrize('point', [[.885, .56], [.95, .4]])
def test_start_outside_saved_polygon_is_rejected(config, settings, point):
    calibration = yaml.safe_load(settings.read_text())
    calibration['groups']['A']['tcp_pose'][:2] = point
    settings.write_text(yaml.safe_dump(calibration))
    with pytest.raises(RobotError, match='strictly inside'):
        prepared(config, settings)


def test_main_scan_calibration_and_single_point_budgets_are_never_read(config, settings, monkeypatch):
    from app import configuration
    def forbidden(*a, **kw):
        raise AssertionError('main scan calibration read')
    monkeypatch.setattr(configuration, 'load_scan_calibration', forbidden)
    monkeypatch.setattr(configuration, 'site_configuration_digest', forbidden)
    config['calibration']['file'] = 'missing.yaml'
    calibration = yaml.safe_load(settings.read_text())
    calibration['groups']['A']['path_review'] = dict(note='', max_distance_m=1e-10)
    settings.write_text(yaml.safe_dump(calibration))
    _, start = prepared(config, settings)
    assert start[2] == calibration['P_ref'][2]
    assert config['continuous_search_geometry']['usable_distance_m'] > .1


def test_shared_policy_identical_for_tracking_loss_and_recovery(config, settings):
    original = deepcopy(config)
    prepared(config, settings)
    # Compare the same real-execution envelope/observations to the unadapted
    # policy. No geometry file or special force rule participates in either.
    for key in ('continuous_real_execution', 'continuous_search_geometry'):
        original[key] = deepcopy(config[key])
    original['policy']['search_direction_xy'] = config['policy']['search_direction_xy']
    a, b = ContinuousTrackingPolicy(original), ContinuousTrackingPolicy(config)
    states, kinds = set(), set()
    approach = np.asarray(config['policy']['search_direction_xy'])
    pose = np.asarray(config['continuous_calibration']['start_tcp_pose'])
    for i in range(300):
        magnitude = min(1.5, max(0., (i-5)*.05)) if i < 130 else 0. if i < 190 else 1.5
        raw = Wrench.from_sequence([*(-magnitude*approach), 0., 0., 0., 0.])
        observation = RobotState(i*.01, pose.copy(), np.zeros(6))
        x, y = a.update(i*.01, raw, raw, observation), b.update(i*.01, raw, raw, observation)
        assert (x.state, x.move, x.speed, x.reason) == (y.state, y.move, y.speed, y.reason)
        np.testing.assert_array_equal(x.direction_xy, y.direction_xy)
        assert a.v_n == b.v_n and a.v_t == b.v_t
        states.add(x.state)
        kinds.update(e.event_type for e in b.events)
    assert {'TARGET_SEARCH', 'FIRST_CONTACT', 'CONTINUOUS_TRACKING', 'CONTACT_LOST', 'LOCAL_REACQUIRE'} <= states
    assert {'FIRST_CONTACT', 'TRACKING_ENTERED', 'CONTACT_LOST', 'REACQUIRED'} <= kinds


def fake_run(config, settings, tmp_path, monkeypatch, *, failure=None):
    recovery_failure = failure is not None and failure.endswith('_recovery')
    effective = deepcopy(config)
    effective['preprocessing']['baseline']['sample_count'] = 2  # explicit test acceleration
    effective['safe_return']['startup_bias_sample_count'] = 2
    monkeypatch.setattr(runtime, 'load_config', lambda _: deepcopy(effective))
    prepared_config = deepcopy(effective)
    _, start = prepared(prepared_config, settings)
    initial = start.copy()
    initial[0] += .003
    initial[3:] = [0., 0., 0.]  # exercise genuine orientation alignment
    devices = Devices(config, initial)
    approach = np.array(prepared_config['policy']['search_direction_xy'])
    contexts, prompts = [], []
    class FixedKeyboard(Keyboard):
        def __enter__(self):
            contexts.append(self)
            self.main = len(contexts) > 1
            self.started = time.monotonic()
            return self
        def poll(self):
            return 'Q' if self.main and time.monotonic()-self.started > (2. if recovery_failure else .7) else None
        def read_line(self, prompt):
            prompts.append(prompt)
            return ''
        def read_text(self, prompt):
            # Same point in startup as the main runner: raised air-zero position.
            np.testing.assert_allclose(devices.receive.pose[:2], start[:2])
            assert devices.receive.pose[2] == pytest.approx(start[2]+config['safe_return']['return_lift_distance'])
            return ''
    monkeypatch.setattr(runtime, 'OperatorKeyboard', FixedKeyboard)
    transform = np.asarray(config['preprocessing']['coordinate_transform']['rotation_sensor_to_tool'])
    class FixedSensor(Sensor):
        scan_reads = 0
        failed = False
        def read_wrench(self):
            assert not self.failed, 'failed sensor must not be read during braking'
            magnitude = 0.
            if not devices.owner._return_mode:
                self.scan_reads += 1
                if recovery_failure and devices.owner.continuous_phase == 'LOCAL_REACQUIRE':
                    if failure == 'sensor_recovery':
                        from sensor.px6d_reader import PX6DError
                        self.failed = True
                        raise PX6DError('synthetic recovery sensor fault')
                    if failure == 'rtde_recovery':
                        devices.receive.connected = False
                    if failure == 'workspace_recovery':
                        devices.receive.pose[0] = .95
                    if failure == 'force_recovery':
                        return Wrench(100., 0., 0., 0., 0., 0.)
                    if failure == 'torque_recovery':
                        return Wrench(0., 0., 0., 6., 0., 0.)
                if failure == 'sensor' and self.scan_reads == 12:
                    from sensor.px6d_reader import PX6DError
                    self.failed = True
                    raise PX6DError('synthetic sensor fault')
                if failure == 'rtde' and self.scan_reads == 12:
                    devices.receive.connected = False
                magnitude = min(1.5, max(0., (self.scan_reads-8)*.1))
                if recovery_failure and self.scan_reads >= 35:
                    magnitude = 0.
            base = np.r_[-magnitude*approach, 0.]
            rotation = _rotvec_to_matrix(devices.receive.pose[3:]) @ transform
            return Wrench.from_sequence([*(rotation.T @ base), 0., 0., 0.])
    sensor = FixedSensor()
    geometry = tmp_path/'target.yaml'
    save_geometry(geometry, make_geometry(dict(P1=[.6,.2],P2=[.7,.2],P3=[.7,.3],P4=[.6,.3])))
    from calibration import target_geometry
    loader = target_geometry.load_geometry
    def offline_read(path):
        assert not sensor.connected and not devices.receive.connected
        return loader(path)
    monkeypatch.setattr(target_geometry, 'load_geometry', offline_read)
    from visualization import fixed_target
    monkeypatch.setattr(fixed_target, 'load_geometry', offline_read)
    # HTML replay exercises the same trace loader; avoid repeating costly PNGs.
    from visualization import run_plots
    monkeypatch.setattr(run_plots, 'generate_run_visualization', lambda p: [write_html_replay(load_trace(p), p)])
    args = Namespace(config=ROOT/'config.yaml', calibration=settings, p0='A', execute=True,
        scene=ROOT/'simulation/scene_fixed_target.yaml', duration=None, output=tmp_path/'logs',
        target_geometry=geometry)
    result = run_fixed(args, controller_factory=devices.controller, reader_factory=lambda *a: sensor)
    log, = (args.output/'real/continuous').glob('run_*')
    return result, log, devices, prompts


def test_fixed_real_path_mock_alignment_air_bias_contact_then_tracking(config, settings, tmp_path, monkeypatch):
    result, log, devices, prompts = fake_run(config, settings, tmp_path, monkeypatch)
    assert result == 0
    with (log/'samples.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert {'TARGET_SEARCH', 'FIRST_CONTACT', 'CONTINUOUS_TRACKING'} <= {r['current_state'] for r in rows}
    assert len(prompts) == 3  # same startup, air zero, descent/search confirmation as main
    assert all('颗粒' not in p for p in prompts)
    alignment = json.loads((log/'startup_alignment.json').read_text())
    assert alignment['actual_before_tilt_deg'] == pytest.approx(180.)
    assert alignment['actual_after_tilt_deg'] < 1e-5
    bias = json.loads((log/'scan_bias_at_start.json').read_text())
    snapshot = yaml.safe_load((log/'config_snapshot.yaml').read_text())
    assert bias['tcp_pose'][2] == pytest.approx(snapshot['continuous_calibration']['fixed_z']+config['safe_return']['return_lift_distance'])
    assert json.loads((log/'granular_baseline_at_start.json').read_text())['sample_count'] == 0
    assert json.loads((log/'summary.json').read_text())['stop_observation']['standstill_confirmed'] is True
    assert devices.control_count == devices.receive_count == 1
    assert any(c[0] == 'speedL' for c in devices.control.calls)
    assert not devices.control.connected and not devices.receive.connected
    assert (log/'实验回放.html').is_file() and (log/'target_geometry_calibration.yaml').is_file()
    assert 'corners' not in str(devices.owner.config)
    assert 'target_geometry' not in str(snapshot)


@pytest.mark.parametrize('failure', ['sensor', 'rtde'])
def test_fixed_faults_stop_through_original_adapter(config, settings, tmp_path, monkeypatch, failure):
    result, log, devices, _ = fake_run(config, settings, tmp_path, monkeypatch, failure=failure)
    assert result == 1
    assert any(c[0] == 'speedStop' for c in devices.control.calls)
    termination = json.loads((log/'termination.json').read_text())
    assert termination['reason'] == ('STOP_SENSOR_ERROR' if failure == 'sensor' else 'STOP_MOTION_ERROR')


@pytest.mark.parametrize('failure,reason', [('sensor_recovery', 'STOP_SENSOR_ERROR'),
    ('rtde_recovery', 'STOP_MOTION_ERROR'), ('workspace_recovery', 'STOP_MOTION_ERROR'),
    ('force_recovery', 'STOP_FORCE_LIMIT'), ('torque_recovery', 'STOP_FORCE_LIMIT')])
def test_shared_runtime_protections_remain_active_during_recovery(config, settings, tmp_path, monkeypatch, failure, reason):
    result, log, devices, _ = fake_run(config, settings, tmp_path, monkeypatch, failure=failure)
    assert result == 1
    with (log/'samples.csv').open() as stream:
        assert any(row['current_state'] == 'LOCAL_REACQUIRE' for row in csv.DictReader(stream))
    assert any(call[0] == 'speedStop' for call in devices.control.calls)
    termination = json.loads((log/'termination.json').read_text())
    assert termination['reason'] == reason
    if failure == 'workspace_recovery':
        assert 'boundary' in termination['detail']


def test_target_teaching_reads_only_xy_and_preserves_old_file(config, tmp_path):
    devices = Devices(config, [.5,.2,0.,0.,0.,0.])
    corners = iter([[.5,.2,0,0,0,0],[.6,.2,1,2,3,4],[.6,.3,2,3,4,5],[.5,.3,3,4,5,6]])
    def confirm(_):
        devices.receive.pose = np.array(next(corners), dtype=float)
        return True
    path = tmp_path/'target_geometry_calibration.yaml'
    teach_target(config, path, controller_factory=devices.controller, confirm=confirm)
    assert devices.control_count == 0 and not devices.control.calls
    data = load_geometry(path)
    assert data['corners']['P3'] == [.6,.3]
    assert all(len(p) == 2 for p in data['corners'].values())
    before = path.read_bytes()
    save_geometry(path, data)
    assert list(tmp_path.glob('*.backup-*.yaml'))[0].read_bytes() == before


def test_html_overlay_uses_snapshot_and_exact_rtde_samples(tmp_path):
    from tools.visualize_contact_experiment import make_demo
    run = make_demo(tmp_path)
    trace = load_trace(run)
    trace.config['fixed_target_experiment'] = True
    snapshot = run/'config_snapshot.yaml'
    snapshot.write_text(yaml.safe_dump(trace.config))
    save_geometry(run/'target_geometry_calibration.yaml', make_geometry(
        dict(P1=[-.01,-.01,999], P2=[.01,-.01,-999], P3=[.01,.01], P4=[-.01,.01])))
    before = (run/'samples.csv').read_bytes()
    trace = load_trace(run)
    page = write_html_replay(trace, run)
    data = json.loads(re.search(r'id="replay-data">(.*?)</script>', page.read_text(), re.S)[1])
    assert data['fixed_target'] and data['target'] == [[-10,-10],[10,-10],[10,10],[-10,10],[-10,-10]]
    np.testing.assert_array_equal(np.array(data['samples'])[:,1:3], trace.xy_mm)
    assert (run/'samples.csv').read_bytes() == before
    if shutil.which('gjs'):
        result = subprocess.run(['gjs', str(Path(__file__).with_name('html_replay_harness.js')), str(page)],
            capture_output=True, text=True, timeout=20)
        assert result.returncode == 0, result.stderr
        assert json.loads(result.stdout)['result'] == 'PASS'


def test_reference_posture_and_taught_p0_z_do_not_affect_execution(config, settings):
    before = deepcopy(config)
    _, a = prepared(before, settings)
    record = yaml.safe_load(settings.read_text())
    record['reference_tcp_pose'][3:] = [100., -300., 900.]
    record['groups']['A']['tcp_pose'][2] = 500.
    settings.write_text(yaml.safe_dump(record))
    _, b = prepared(config, settings)
    np.testing.assert_array_equal(a, b)
    np.testing.assert_array_equal(before['policy']['search_direction_xy'], config['policy']['search_direction_xy'])


def test_offline_entry_reuses_simulator_and_standard_html_without_hardware(tmp_path, monkeypatch):
    from run_fixed_target_tracking import main
    monkeypatch.setattr(runtime, 'OperatorKeyboard', Keyboard)
    output = tmp_path/'output'
    assert main(['--dry-run', '--duration', '.08', '--output', str(output)]) == 0
    run, = (output/'simulation/continuous').glob('run_*')
    with (run/'samples.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert any(row['current_state'] == 'TARGET_SEARCH' for row in rows)
    snapshot = yaml.safe_load((run/'config_snapshot.yaml').read_text())
    from simulation.continuous_session import SimulationSession
    from simulation.simulator import load_simulation_config
    shared = SimulationSession(config_from_disk(), load_simulation_config(ROOT/'simulation/scene_fixed_target.yaml'))
    assert snapshot['continuous_simulation_execution'] == shared.config['continuous_simulation_execution']
    assert snapshot['continuous_simulation']['target_shape'] == 'rectangle'
    assert snapshot['continuous_simulation']['force_model']['granular_drag_force'] == 0.
    assert snapshot['experiment']['kind'] == 'synthetic_demo'
    assert (run/'实验回放.html').is_file()
    assert not (output/'real').exists()


def test_missing_workspace_rejects_before_device_construction(config, tmp_path, monkeypatch):
    config['continuous_tracking']['workspace_calibration_file'] = str(tmp_path/'missing_workspace.yaml')
    monkeypatch.setattr(runtime, 'load_config', lambda _: deepcopy(config))
    args = Namespace(config=ROOT/'config.yaml', calibration=ROOT/'single_point_calibration.yaml',
        p0='A', execute=True, scene=ROOT/'simulation/scene_fixed_target.yaml', duration=None,
        output=tmp_path, target_geometry=tmp_path/'not_present.yaml')
    def forbidden(*a, **kw):
        raise AssertionError('device constructed before valid workspace')
    assert run_fixed(args, controller_factory=forbidden, reader_factory=forbidden) == 1


def test_entry_accepts_saved_workspace_with_original_default_config():
    from run_fixed_target_tracking import main
    assert main(['--check-calibration', '--p0', 'A']) == 0


def config_from_disk():
    from config.loader import load_config
    return load_config(ROOT/'config.yaml')


def test_no_experiment_config_file_or_cli_gate():
    from run_fixed_target_tracking import main
    assert not (ROOT/'fixed_target_experiment.yaml').exists()
    with pytest.raises(SystemExit) as exc:
        main(['--experiment-config', 'obsolete.yaml'])
    assert exc.value.code == 2


def test_long_synthetic_contact_keeps_original_force_guard(tmp_path, monkeypatch):
    from run_fixed_target_tracking import main
    monkeypatch.setattr(runtime, 'OperatorKeyboard', Keyboard)
    from visualization import run_plots
    monkeypatch.setattr(run_plots, 'generate_run_visualization', lambda _: [])
    output = tmp_path/'synthetic'
    # Same model dynamics and protection as the main runner; no scene-specific actuator tuning.
    assert main(['--dry-run', '--duration', '5', '--output', str(output)]) == 1
    run, = (output/'simulation/continuous').glob('run_*')
    reason = json.loads((run/'termination.json').read_text())['reason']
    assert reason == 'STOP_FORCE_LIMIT'
    with (run/'samples.csv').open() as stream:
        assert any(row['current_state'] == 'FIRST_CONTACT' for row in csv.DictReader(stream))

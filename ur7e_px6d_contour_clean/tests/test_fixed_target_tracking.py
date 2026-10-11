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
from app.fixed_target import ROOT, configure_fixed, prepare_fixed, run_fixed, require_remote
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
    data = dict(calibration_file=str(ROOT/'single_point_calibration.yaml'),
                workspace=dict(verified=True, limits=dict(x_min=.4, x_max=.9, y_min=-.1,
                    y_max=.55, z_min=-.1, z_max=.15)))
    path = tmp_path/'fixed.yaml'
    path.write_text(yaml.safe_dump(data))
    return data, path


def prepared(config, settings, name='A'):
    configure_fixed(config)
    data, path = settings
    return prepare_fixed(config, ROOT/'config.yaml', data, path, name)


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
    for section in ('robot', 'sensor', 'tcp', 'safe_return', 'logging'):
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


@pytest.mark.parametrize('workspace', [None, {'verified': False, 'limits': None},
    {'verified': True, 'limits': None}, {'verified': True, 'limits': dict(x_min=0, x_max=1)},
    {'verified': True, 'limits': dict(x_min=1, x_max=0,y_min=0,y_max=1,z_min=0,z_max=1)}])
def test_missing_invalid_or_unverified_region_rejected_offline(config, settings, workspace):
    data, path = settings
    data['workspace'] = workspace
    with pytest.raises(RobotError):
        prepare_fixed(config, ROOT/'config.yaml', data, path, 'A')


def test_particle_files_and_single_point_budgets_are_never_read(config, settings, monkeypatch):
    from app import configuration
    def forbidden(*a, **kw):
        raise AssertionError('particle calibration read')
    monkeypatch.setattr(configuration, 'load_scan_calibration', forbidden)
    monkeypatch.setattr(configuration, 'load_workspace_calibration', forbidden)
    monkeypatch.setattr(configuration, 'site_configuration_digest', forbidden)
    config['calibration']['file'] = 'missing.yaml'
    config['continuous_tracking']['workspace_calibration_file'] = 'missing_box.yaml'
    data, path = settings
    calibration = yaml.safe_load((ROOT/'single_point_calibration.yaml').read_text())
    calibration['groups']['A']['path_review'] = dict(note='', max_distance_m=1e-10)
    other = path.parent/'points.yaml'
    other.write_text(yaml.safe_dump(calibration))
    data['calibration_file'] = str(other)
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
    data, path = settings
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
            return 'Q' if self.main and time.monotonic()-self.started > .7 else None
        def read_line(self, prompt):
            prompts.append(prompt)
            return ''
        def read_text(self, prompt):
            # Speed selection must occur after arrival, using the same chooser.
            np.testing.assert_allclose(devices.receive.pose, start)
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
                if failure == 'sensor' and self.scan_reads == 12:
                    from sensor.px6d_reader import PX6DError
                    self.failed = True
                    raise PX6DError('synthetic sensor fault')
                if failure == 'rtde' and self.scan_reads == 12:
                    devices.receive.connected = False
                magnitude = min(1.5, max(0., (self.scan_reads-8)*.1))
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
    args = Namespace(config=ROOT/'config.yaml', experiment_config=path, p0='A', execute=True,
        scene=ROOT/'simulation/scene_fixed_target.yaml', duration=None, output=tmp_path/'logs',
        target_geometry=geometry)
    modes = []
    result = run_fixed(args, controller_factory=devices.controller, reader_factory=lambda *a: sensor,
                       mode_check=lambda ip: modes.append(ip))
    log, = (args.output/'real/continuous').glob('run_*')
    return result, log, devices, prompts, modes


def test_fixed_real_path_mock_alignment_air_bias_contact_then_tracking(config, settings, tmp_path, monkeypatch):
    result, log, devices, prompts, modes = fake_run(config, settings, tmp_path, monkeypatch)
    assert result == 0
    assert modes == [config['robot']['robot_ip']]
    with (log/'samples.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert {'TARGET_SEARCH', 'FIRST_CONTACT', 'CONTINUOUS_TRACKING'} <= {r['current_state'] for r in rows}
    assert len(prompts) == 3  # shared startup, air zero, final search; no extra verification
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
    result, log, devices, _, _ = fake_run(config, settings, tmp_path, monkeypatch, failure=failure)
    assert result == 1
    assert any(c[0] == 'speedStop' for c in devices.control.calls)
    termination = json.loads((log/'termination.json').read_text())
    assert termination['reason'] == ('STOP_SENSOR_ERROR' if failure == 'sensor' else 'STOP_MOTION_ERROR')


def test_remote_mode_unknown_or_local_refuses_motion(monkeypatch):
    from app import fixed_target
    from robot.control_mode import ControlMode
    for mode in ('unknown', 'local'):
        monkeypatch.setattr(fixed_target, 'read_control_mode', lambda _: ControlMode(mode, 'mock'))
        with pytest.raises(RobotError):
            require_remote('mock')


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
    data, path = settings
    record = yaml.safe_load(Path(data['calibration_file']).read_text())
    record['reference_tcp_pose'][3:] = [100., -300., 900.]
    record['groups']['A']['tcp_pose'][2] = 500.
    changed = path.parent/'modified_points.yaml'
    changed.write_text(yaml.safe_dump(record))
    data['calibration_file'] = str(changed)
    _, b = prepared(config, settings)
    np.testing.assert_array_equal(a, b)
    np.testing.assert_array_equal(before['policy']['search_direction_xy'], config['policy']['search_direction_xy'])


def test_offline_entry_reuses_simulator_and_standard_html_without_hardware(tmp_path, monkeypatch):
    from run_fixed_target_tracking import main
    monkeypatch.setattr(runtime, 'OperatorKeyboard', Keyboard)
    output = tmp_path/'output'
    assert main(['--dry-run', '--duration', '3', '--output', str(output)]) == 0
    run, = (output/'simulation/continuous').glob('run_*')
    with (run/'samples.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert any(row['current_state'] == 'CONTINUOUS_TRACKING' for row in rows)
    snapshot = yaml.safe_load((run/'config_snapshot.yaml').read_text())
    assert snapshot['continuous_simulation_execution']['acceleration_limit'] == snapshot['robot']['speed_acceleration']
    assert snapshot['continuous_simulation']['target_shape'] == 'rectangle'
    assert snapshot['continuous_simulation']['force_model']['granular_drag_force'] == 0.
    assert snapshot['experiment']['kind'] == 'synthetic_demo'
    assert (run/'实验回放.html').is_file()
    assert not (output/'real').exists()


def test_disabled_boundaries_reject_before_device_construction(config, tmp_path):
    args = Namespace(config=ROOT/'config.yaml', experiment_config=ROOT/'fixed_target_experiment.yaml',
        p0='A', execute=True, scene=ROOT/'simulation/scene_fixed_target.yaml', duration=None,
        output=tmp_path, target_geometry=tmp_path/'not_present.yaml')
    def forbidden(*a, **kw):
        raise AssertionError('device constructed before valid workspace')
    assert run_fixed(args, controller_factory=forbidden, reader_factory=forbidden, mode_check=forbidden) == 1


def test_new_settings_cannot_shadow_control_defaults(tmp_path):
    from app.fixed_target import load_settings
    path = tmp_path/'settings.yaml'
    path.write_text(yaml.safe_dump(dict(force_reference=2., site_validation_note='anything')))
    with pytest.raises(ValueError, match='unknown fixed-target settings'):
        load_settings(path)


def test_air_zero_lift_must_fit_existing_workspace_guard(config, settings):
    data, _ = settings
    data['workspace']['limits']['z_max'] = .0
    with pytest.raises(RobotError, match='return trajectory outside workspace'):
        prepared(config, settings)


def test_long_synthetic_contact_keeps_original_unload_budget(tmp_path, monkeypatch):
    from run_fixed_target_tracking import main
    monkeypatch.setattr(runtime, 'OperatorKeyboard', Keyboard)
    from visualization import run_plots
    monkeypatch.setattr(run_plots, 'generate_run_visualization', lambda _: [])
    output = tmp_path/'synthetic'
    # Do not weaken the policy to keep an idealized stiff target moving longer.
    assert main(['--dry-run', '--duration', '5', '--output', str(output)]) == 1
    run, = (output/'simulation/continuous').glob('run_*')
    reason = json.loads((run/'termination.json').read_text())['reason']
    assert reason == 'STOP_UNLOAD_BUDGET'
    with (run/'samples.csv').open() as stream:
        assert any(row['current_state'] == 'CONTINUOUS_TRACKING' for row in csv.DictReader(stream))

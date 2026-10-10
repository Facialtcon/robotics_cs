"""No hardware: independent calibration, straight approach, braking and replay."""
from copy import deepcopy
import csv
import json
from pathlib import Path
import shutil
import subprocess

import numpy as np
import pytest

from app.single_point_config import load_settings, prepare, check_segment, validate_speed, validate_settings
from app.single_point_runtime import FreshForceReader, SinglePointTrial
from calibration.single_point import (approach_direction, load_calibration, save_group, set_reference,
                                      validate_group, write_calibration)
from core.models import Wrench
from robot.rtde_controller import RobotError
from run_single_point_contact import ROOT, main, plan_return
from sensor.force_preprocess import WrenchPreprocessor
from sensor.px6d_reader import PX6DTimeout
from simulation.single_point import OfflineController, OfflineForceReader, demo_calibration


@pytest.fixture
def settings():
    return load_settings(ROOT/'single_point_experiment.yaml')


def test_multiple_p0_roundtrip_and_fixed_reference(tmp_path, config):
    data = demo_calibration(config)
    path = tmp_path/'single.yaml'
    write_calibration(path, data)
    loaded = load_calibration(path)
    assert loaded == data and set(loaded['groups']) == {'A', 'B', 'C'}
    np.testing.assert_allclose(loaded['groups']['A']['direction_xy'], [1, 0])
    np.testing.assert_allclose(loaded['groups']['B']['direction_xy'], [0, 1])
    np.testing.assert_allclose(loaded['groups']['C']['direction_xy'], [-1, 0])
    before = deepcopy(data['P_ref'])
    data = save_group(data, 'D', [.4, .01, .2, np.pi, 0, 0])
    assert data['P_ref'] == before
    with pytest.raises(ValueError, match='fixed'):
        set_reference(data, [.5, 0, .2, np.pi, 0, 0], 'ip', [0]*6)


@pytest.mark.parametrize('pose', [[.4, 0, .2, np.pi, 0, 0], [np.nan, 0, .2, 0, 0, 0], [0]*5])
def test_invalid_p0_rejected(config, pose):
    with pytest.raises(ValueError): save_group(demo_calibration(config), 'bad', pose)


def test_corrupt_direction_and_reference_rejected(tmp_path, config):
    data = demo_calibration(config)
    data['groups']['B']['direction_xy'] = [1, 0]
    path = tmp_path/'cal.yaml'; write_calibration(path, data)
    with pytest.raises(ValueError, match='direction'): load_calibration(path)
    data = demo_calibration(config); data['P_ref'][0] += .1
    write_calibration(path, data)
    with pytest.raises(ValueError, match='reference'): load_calibration(path)


def bounded(settings):
    settings['workspace_limits'] = dict(x_min=.3, x_max=.5, y_min=-.1, y_max=.1, z_min=.1, z_max=.3)
    return settings


def test_calibration_path_review_identity_z_and_vertical(config, settings):
    data = demo_calibration(config)
    with pytest.raises(ValueError, match='reviewed'): validate_group(data, 'A', settings, config)
    group = data['groups']['A']
    data = save_group(data, 'A', group['tcp_pose'], path_note='whole probe and fixture checked', reviewed_distance_m=.02)
    validate_group(data, 'A', settings, config)
    for key, value, message in [('tcp_pose', [.39, 0, .205, np.pi, 0, 0], 'fixed Z'),
                                ('active_tcp_offset', [0]*6, 'TCP')]:
        bad = deepcopy(data); bad['groups']['A'][key] = value
        with pytest.raises(ValueError, match=message): validate_group(bad, 'A', settings, config)
    bad = deepcopy(data); bad['reference_tcp_pose'][3:] = [0]*3; bad['groups']['A']['tcp_pose'][3:] = [0]*3
    with pytest.raises(ValueError, match='vertically'): validate_group(bad, 'A', settings, config)


def test_whole_ray_checks_fixture_and_braking_clearance(settings):
    bounded(settings)
    check_segment([.39, 0, .2], [.411, 0, .2], settings)
    settings['forbidden_boxes'] = [dict(x_min=.409, x_max=.413, y_min=-.01, y_max=.01, z_min=.19, z_max=.21)]
    with pytest.raises(ValueError, match='intersects'): check_segment([.39, 0, .2], [.411, 0, .2], settings)
    with pytest.raises(ValueError, match='workspace'): check_segment([.39, 0, .2], [.499, 0, .2], settings)


def test_no_implicit_workspace_or_site_validation(config, settings):
    data = demo_calibration(config)
    data = save_group(data, 'A', data['groups']['A']['tcp_pose'], path_note='checked', reviewed_distance_m=.02)
    with pytest.raises(ValueError, match='on-site'): prepare(config, settings, data, 'A', execute=True)
    settings['site_validation_note'] = 'checked'
    with pytest.raises(ValueError, match='workspace'): prepare(config, settings, data, 'A', execute=True)
    bounded(settings)
    before = deepcopy(config)
    effective, robot_config, _, _ = prepare(config, settings, data, 'A', execute=True)
    assert config == before
    assert effective['preprocessing']['granular_baseline_output'] == [0]*6
    assert robot_config['continuous_require_watchdog'] is True
    assert not robot_config.get('continuous_real_execution')
    assert 'continuous_speed_limits' not in robot_config


@pytest.mark.parametrize('speed', [0, -1, .003, float('inf'), float('nan')])
def test_speed_range_validation(settings, speed):
    with pytest.raises(ValueError): validate_speed(speed, settings)


def test_one_second_history_enforced(settings):
    settings['precontact_record_sec'] = .9
    with pytest.raises(ValueError, match='one second'): validate_settings(settings)


class MemoryLog:
    def __init__(self): self.samples = []; self.commands = []; self.events = []
    def log_commands(self, records): self.commands.extend(deepcopy(records))
    def log_sample(self, stamp, raw, processed, robot, command, *args, **kwargs):
        self.samples.append(dict(stamp=stamp, raw=raw, processed=processed, robot=robot,
                                 state=command.state, extra=kwargs['extra']))
    def log_waypoint(self, event): self.events.append(event)
    def check_health(self): pass


def rig(config, settings, *, contact=.002, sensor=None, poll=lambda: None):
    data = demo_calibration(config)
    pose = data['groups']['A']['tcp_pose']; direction = data['groups']['A']['direction_xy']
    processor = WrenchPreprocessor.from_config(config['preprocessing'], tool_orientation=pose[3:])
    controller = OfflineController(pose, settings)
    raw_reader = sensor(controller) if sensor else OfflineForceReader(controller, pose, direction, processor, contact_distance=contact)
    fresh = FreshForceReader(raw_reader, settings, config['policy'], clock=controller.clock)
    logger = MemoryLog()
    trial = SinglePointTrial(config, settings, data, 'A', settings['speed_default_mps'],
        controller, fresh, processor, logger, poll=poll, clock=controller.clock, sleep=controller.sleep)
    return trial, controller, logger


def test_threshold_requests_braking_before_logging_and_records_real_settling(config, settings):
    trial, controller, logger = rig(config, settings)
    trial.record_precontact()
    order = []
    request, event = controller.request_stop, logger.log_waypoint
    def stop(**kw): order.append('stop'); return request(**kw)
    def log(e): order.append(e.event_type); event(e)
    controller.request_stop = stop; logger.log_waypoint = log
    result = trial.run()
    assert result['status'] == 'contact'
    assert order[:2] == ['stop', 'FIRST_THRESHOLD_STOP_REQUEST']
    assert result['threshold_trigger_sec'] == result['braking_request_sec']
    assert result['actual_standstill_sec'] > result['braking_request_sec']
    assert result['actual_at_threshold']['pose'][0] < result['P_ref'][0]
    assert result['actual_before_threshold']['timestamp'] < result['threshold_trigger_sec']
    assert result['actual_at_threshold']['velocity'][0] == pytest.approx(.0005)
    assert result['actual_stop_pose'][0] > result['actual_at_threshold']['pose'][0]
    assert result['peak_filtered_fxy_N'] >= settings['contact_threshold_N']
    braking = next(c for c in logger.commands if c['kind'] == 'braking')
    assert braking['velocity'] == [0]*6
    assert all(c['kind'] != 'motion' for c in logger.commands[logger.commands.index(braking):])
    tail = [r for r in logger.samples if r['state'] == 'SETTLING']
    assert any(r['robot'].tcp_speed[0] > 0 for r in tail)
    assert any(e.event_type == 'EXECUTION_STOP_CONFIRMED' for e in logger.events)
    assert logger.samples[100]['stamp'] >= 1.
    assert logger.samples[-1]['extra']['standstill_confirmed_sec'] == result['actual_standstill_sec']


def test_passing_reference_does_not_mean_contact(config, settings):
    trial, controller, logger = rig(config, settings, contact=1.)
    # A near reference along the same direction; no reference-based termination.
    trial.calibration['P_ref'] = [.3905, 0, .2]
    settings['max_search_distance_m'] = .001
    result = trial.run()
    assert result['status'] == 'no_contact' and result['reason'] == 'NO_CONTACT_DISTANCE'
    assert controller.read_state().pose[0] > .3905
    assert result['threshold_trigger_sec'] is None
    assert not any(e.event_type == 'FIRST_THRESHOLD_STOP_REQUEST' for e in logger.events)


def test_contact_after_passing_reference_still_uses_force(config, settings):
    trial, _, _ = rig(config, settings, contact=.001)
    trial.calibration['P_ref'] = [.3902, 0, .2]
    result = trial.run()
    assert result['status'] == 'contact'
    assert result['actual_at_threshold']['pose'][0] > .3902


@pytest.mark.parametrize('speed', [.0001, .001, .002])
def test_supported_nominal_speeds_complete_and_command_exact_values(config, settings, speed):
    trial, _, logger = rig(config, settings, contact=.0001)
    trial.speed = speed
    result = trial.run()
    assert result['status'] == 'contact'
    assert all(c['velocity'][0] == speed for c in logger.commands if c['kind'] == 'motion')


def test_stop_confirmation_revocation_never_reports_success(config, settings):
    trial, c, _ = rig(config, settings)
    normal_poll = c.poll_stop
    def revoke():
        result = normal_poll()
        return False if trial.stopped_time is not None else result
    c.poll_stop = revoke
    result = trial.run()
    assert result['actual_standstill_sec'] is not None and result['status'] == 'aborted'
    assert 'revoked' in result['fault']


def test_independent_time_budget(config, settings):
    settings['max_approach_time_sec'] = .1
    trial, _, _ = rig(config, settings, contact=1.)
    assert trial.run()['reason'] == 'NO_CONTACT_TIME'


@pytest.mark.parametrize('kind', ['timeout', 'nan', 'stale', 'hard_force'])
def test_invalid_sensor_stops_without_motion_and_keeps_tcp_tail(config, settings, kind):
    class BadSensor:
        def __init__(self, controller): self.controller = controller
        @property
        def sample_timestamp(self): return self.controller.clock()-1 if kind == 'stale' else self.controller.clock()
        def read_wrench(self):
            if kind == 'timeout': raise PX6DTimeout('timeout')
            return Wrench.from_sequence([np.nan if kind == 'nan' else 100 if kind == 'hard_force' else 0, 0, 0, 0, 0, 0])
    trial, _, logger = rig(config, settings, sensor=BadSensor)
    result = trial.run()
    assert result['status'] == 'aborted' and result['actual_standstill_sec'] is not None
    assert not any(c['kind'] == 'motion' for c in logger.commands)
    assert logger.samples and all(r['extra']['force_valid'] == 0 for r in logger.samples)


def test_operator_abort(config, settings):
    trial, _, logger = rig(config, settings, poll=lambda: 'Q')
    result = trial.run()
    assert result['reason'] == 'USER_ABORT' and result['status'] == 'aborted'
    assert result['actual_standstill_sec'] is not None
    assert not any(c['kind'] == 'motion' for c in logger.commands)


def test_sensor_failure_after_motion(config, settings):
    class Interrupted:
        def __init__(self, c): self.c = c
        def read_wrench(self):
            if self.c.clock() >= .05: raise PX6DTimeout('lost sensor')
            return Wrench.from_sequence([0]*6)
    trial, _, logger = rig(config, settings, sensor=Interrupted)
    result = trial.run()
    assert result['status'] == 'aborted' and result['actual_standstill_sec'] is not None
    assert any(c['kind'] == 'motion' for c in logger.commands)
    assert logger.samples[-1]['extra']['force_valid'] == 0


def test_stop_not_confirmed_is_never_success(config, settings):
    settings['stop_timeout_sec'] = .1
    trial, c, _ = rig(config, settings, contact=0.)
    c.poll_stop = lambda: False
    result = trial.run()
    assert result['threshold_trigger_sec'] is not None and result['status'] == 'aborted'
    assert result['actual_standstill_sec'] is None


def test_return_reuses_plan_and_requires_verified_clearance(config, settings):
    pose = [.39, 0, .2, np.pi, 0, 0]
    with pytest.raises(RobotError, match='30 mm'): plan_return(config, settings, pose, pose)
    bounded(settings); settings['return_lift_distance_m'] = .04
    effective, segments = plan_return(config, settings, pose, [.4, 0, .2, np.pi, 0, 0])
    assert segments[0][1][2] == pytest.approx(.24)
    assert segments[-1][1][0] == .4
    assert effective['safe_return']['return_speed'] == settings['return_speed_mps']
    settings['return_lift_distance_m'] = .2
    with pytest.raises(RobotError, match='workspace'): plan_return(config, settings, pose, pose)


def test_before_send_rechecks_force_after_shared_rtde_read(config, settings):
    from tests.doubles import Devices
    from app.single_point_config import prepare
    data = demo_calibration(config)
    effective, rc, pose, _ = prepare(config, settings, data, 'A', execute=False)
    devices = Devices(effective, pose); owner = devices.controller(rc)
    owner.connect(); owner.activate_control(confirmed=True)
    def reject(state):
        raise RobotError('expired force')
    owner.before_velocity_send = reject
    with pytest.raises(RobotError, match='expired force'): owner.command_planar_velocity([1, 0], .0005, .01)
    assert not any(c[0] == 'speedL' for c in devices.control.calls)
    owner.close()


def test_trial_before_send_rejects_expired_force_and_fresh_distance(config, settings):
    from app.single_point_runtime import SinglePointSearchLimit
    trial, c, _ = rig(config, settings)
    trial.hold_once()
    c.sleep(settings['max_force_age_sec']+.01)
    with pytest.raises(RobotError, match='expired'): trial._before_send(c.read_state())
    trial.hold_once()
    c.model.pose[0] = trial.p0[0]+settings['max_search_distance_m']
    with pytest.raises(SinglePointSearchLimit): trial._before_send(c.read_state())


def test_scheduling_gap_stops_persistent_command(config, settings):
    trial, c, logger = rig(config, settings, contact=1.)
    normal_sleep = c.sleep
    def delayed(duration):
        normal_sleep(.09 if c.stop_report is None else duration)
    trial.sleep = delayed
    result = trial.run()
    assert result['status'] == 'aborted' and 'gap exceeded' in result['fault']
    assert len([r for r in logger.commands if r['kind'] == 'motion']) == 1


def test_logging_failure_still_confirms_actual_stop(config, settings):
    trial, _, logger = rig(config, settings)
    def failed(*a, **k): raise OSError('disk failure')
    logger.log_sample = failed
    result = trial.run()
    assert result['status'] == 'aborted' and result['actual_standstill_sec'] is not None
    assert 'disk failure' in result['fault']


def test_full_hardware_mode_with_injected_devices_only(tmp_path, monkeypatch, config, settings):
    from argparse import Namespace
    import run_single_point_contact as entry
    from tests.doubles import Devices, Keyboard
    bounded(settings); settings['site_validation_note'] = 'fake site validation for tests'
    data = demo_calibration(config)
    data = save_group(data, 'A', data['groups']['A']['tcp_pose'], path_note='fake path check', reviewed_distance_m=.02)
    devices = Devices(config, data['groups']['A']['tcp_pose'])
    monkeypatch.setattr(entry, 'confirm_enter', lambda *a, **kw: True)
    monkeypatch.setattr(entry, 'read_tcp_offset_readonly', lambda *a: config['tcp']['offset'])
    class Sensor:
        def connect(self): pass
        def close(self): pass
        def read_wrench(self):
            force = 4. if np.linalg.norm(devices.receive.speed[:2]) else 0.
            return Wrench(force, 0., 0., 0., 0., 0.)
    args = Namespace(execute=True, output=tmp_path, config=ROOT/'config.yaml')
    result = entry.run_trial(args, config, settings, data, 'A', controller_factory=devices.controller,
                             reader_factory=lambda project: Sensor(), keyboard_factory=Keyboard)
    assert result['status'] == 'contact'
    assert devices.receive_count == devices.control_count == 1
    calls = devices.control.calls
    assert sum(c[0] == 'setWatchdog' for c in calls) == 1
    assert any(c[0] == 'speedL' and c[1] == [0]*6 for c in calls)
    assert any(c[0] == 'stopScript' for c in calls)
    run = next((tmp_path/'real'/'single_point').glob('run_*'))
    assert (run/'air_zero.json').exists()
    with (run/'samples.csv').open() as f: rows = list(csv.DictReader(f))
    assert float(rows[100]['actual_tcp_timestamp'])-float(rows[0]['actual_tcp_timestamp']) >= .99


def test_offline_cli_logs_independent_timestamps_full_snapshots_and_six_wrench(config, tmp_path):
    before = (ROOT/'scan_calibration.yaml').read_bytes()
    assert main(['--simulate', '--output', str(tmp_path)]) == 0
    run = next((tmp_path/'simulation'/'single_point').glob('run_*'))
    with (run/'samples.csv').open() as f: rows = list(csv.DictReader(f))
    with (run/'tcp_commands.csv').open() as f: commands = list(csv.DictReader(f))
    result = json.loads((run/'single_point_result.json').read_text())
    assert result['status'] == 'contact'
    assert len(rows) > 100 and commands
    assert all(all(k in r for k in ('tcp_vx', 'tcp_vrz', 'actual_tcp_timestamp', 'force_read_end_sec',
                                   'raw_tz', 'force_base_tz', 'filtered_force_base_fz')) for r in rows)
    assert any(float(r['actual_tcp_timestamp']) != float(r['commanded_tcp_timestamp']) for r in rows if r['commanded_tcp_timestamp'])
    for name in ('config_snapshot.yaml', 'single_point_experiment_snapshot.json', 'single_point_calibration_snapshot.json'):
        assert (run/name).is_file()
    page = run/'实验回放.html'
    from tests.test_html_replay import payload
    data = payload(page)
    assert data['single_point'] is True and len(data['wrench']) == len(rows)
    assert data['target'] is None and data['deadband']['enabled'] is False
    assert data['threshold'] == 1 and data['lost_threshold'] is None
    actual_before = result['actual_before_threshold']
    assert data['first_contact']['actual_before_xy_mm_s'][0] == actual_before['velocity'][0]*1000
    assert data['first_contact']['actual_before_time_sec'] < data['first_contact']['time_sec']
    assert (ROOT/'scan_calibration.yaml').read_bytes() == before
    if shutil.which('gjs'):
        # Execute the complete embedded replay; exercise all new wrench curves.
        js = tmp_path/'smoke.js'
        harness = (ROOT/'tests/html_replay_harness.js').read_text()
        prefix = harness[:harness.index("check(api.rate===.25")]
        js.write_text(prefix + "api.seek(data.duration);api.render();elements['events-all'].onclick();"
            "check(elements['force-legend'].children.length===9,'six wrench and magnitude legends');"
            "check(api.cache.force.labels.length>0,'threshold and stop labels');print('PASS');")
        completed = subprocess.run(['gjs', str(js), str(page)], capture_output=True, text=True, timeout=20)
        assert completed.returncode == 0, completed.stderr

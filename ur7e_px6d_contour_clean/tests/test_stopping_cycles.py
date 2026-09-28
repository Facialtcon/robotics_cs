import csv
import json
import time

import numpy as np
import pytest

from app import continuous_runtime as runtime
from config.loader import runtime_robot_config
from core.models import Wrench
from doubles import Devices, Sensor
from experiment_logging.termination import TerminationReason
from robot.rtde_controller import RobotError
from safety.safe_return import SafeReturnExecutor
from test_continuous_runtime import args, prepare


def braking_device(devices, *, stop_value=True, stuck=False):
    """speedL zero starts physical braking; speedStop is synchronous like 1.6.5."""
    control, receive = devices.control, devices.receive
    original_speed = control.speedL
    record = dict(start=None, speeds=[], kicks=[], script_stop=None, mode_exits=[])
    def speed_l(velocity, acceleration, duration):
        if np.any(velocity):
            record['start'] = None
            return original_speed(velocity, acceleration, duration)
        control.calls.append(('speedL', list(velocity)))
        assert acceleration == devices.owner.config['stop_deceleration']
        record['entry_speed'] = float(np.linalg.norm(receive.speed[:3]))
        record['start'] = time.monotonic()
        return False if stuck else True
    def speed():
        if record['start'] is not None:
            i = min(int((time.monotonic()-record['start'])/.01), 6)
            value = .018 if stuck else min(record['entry_speed'], [18, 12, 6, 2, .08, .05, .04][i]/1000)
            receive.speed[:] = [value, 0, 0, 0, 0, 0]
            record['speeds'].append((time.monotonic(), value))
        return receive.speed.copy()
    def speed_stop(deceleration):
        # This is the regression: invoking the primitive while moving costs
        # 105 ms. A zero target must brake before entering this synchronous API.
        if np.linalg.norm(speed()[:3]) > 1e-4:
            time.sleep(.105)
        record['mode_exits'].append(time.monotonic())
        control.calls.append(('speedStop', deceleration))
        return stop_value
    def kick():
        record['kicks'].append(time.monotonic())
        return True
    def finish():
        assert devices.owner.standstill_confirmed
        assert time.monotonic()-record['kicks'][-1] < .05
        record['script_stop'] = time.monotonic()
        return True
    control.speedL, receive.getActualTCPSpeed = speed_l, speed
    control.speedStop, control.kickWatchdog, control.stopScript = speed_stop, kick, finish
    return record


@pytest.mark.parametrize('stop_value', [True, False])
def test_search_limit_brakes_across_logged_cycles(config, monkeypatch, tmp_path, stop_value):
    target = prepare(config, monkeypatch)
    devices = Devices(config, target)
    record = braking_device(devices, stop_value=stop_value)
    original_prepare = runtime.prepare_real
    def short_search(c, path):
        result = original_prepare(c, path)
        c['continuous_search_geometry']['usable_distance_m'] = .002
        return result
    monkeypatch.setattr(runtime, 'prepare_real', short_search)
    settings = args(tmp_path); settings.duration = 2.
    def factory(c):
        owner = devices.controller(c)
        original_wait = owner.wait_for_standstill
        def wait(**kwargs):
            assert not owner.watchdog_active, 'blocking wait entered a control cycle'
            return original_wait(**kwargs)
        owner.wait_for_standstill = wait
        return owner
    assert runtime.run(settings, controller_factory=factory, reader_factory=Sensor) == 0
    run_dir, = (tmp_path/'real/continuous').glob('run_*')
    rows = list(csv.DictReader((run_dir/'samples.csv').open()))
    stopping = [r for r in rows if r['runtime_stop_state'] == 'STOPPING']
    assert len(stopping) >= 10
    assert record['script_stop']-record['start'] > .1
    assert len(record['mode_exits']) == 1
    assert record['mode_exits'][0]-record['start'] >= .12
    kicks = [t for t in record['kicks'] if t >= record['start']]
    assert len(kicks) >= 10 and max(np.diff(kicks)) < .03
    assert all(float(r['command_return_time'])-float(r['cycle_start_time']) < .03 for r in stopping)
    assert all(float(r['loop_start_interval_sec']) < .03 for r in stopping)
    assert any(float(r['actual_xyz_speed_mps']) > .001 for r in stopping)
    assert any(0 < float(r['actual_xyz_speed_mps']) <= .0001 for r in stopping)
    summary = json.loads((run_dir/'summary.json').read_text())
    report, = summary['stop_observation']['stop_requests']
    assert report['physical_stop'] == 'confirmed'
    assert report['api_anomaly'] is (not stop_value)
    termination = json.loads((run_dir/'termination.json').read_text())
    assert termination['reason'] == TerminationReason.STOP_SEARCH_LIMIT


def test_first_contact_uses_same_braking_monitor(config, monkeypatch, tmp_path):
    target = prepare(config, monkeypatch)
    devices = Devices(config, target)
    record = braking_device(devices)
    class ContactSensor(Sensor):
        def read_wrench(self):
            self.count += 1
            return Wrench(0 if self.count < 10 else 1.5, 0, 0, 0, 0, 0)
    settings = args(tmp_path); settings.duration = .65
    assert runtime.run(settings, controller_factory=devices.controller, reader_factory=ContactSensor) == 0
    run_dir, = (tmp_path/'real/continuous').glob('run_*')
    rows = list(csv.DictReader((run_dir/'samples.csv').open()))
    contact = [r for r in rows if r['current_state'] == 'FIRST_CONTACT']
    assert len([r for r in contact if r['runtime_stop_state'] == 'STOPPING']) >= 10
    assert any(r['current_state'] == 'CONTINUOUS_TRACKING' for r in rows)
    assert len(record['mode_exits']) == 2  # contact and final time limit


def test_unaccepted_brake_and_persistent_motion_have_finite_timeout(config):
    devices = Devices(config, config['dry_run']['start_pose'])
    owner = devices.controller(runtime_robot_config(config))
    owner.connect(); owner.activate_control(confirmed=True)
    record = braking_device(devices, stop_value=False, stuck=True)
    started = time.monotonic()
    report = owner.request_stop(nonblocking=True)
    assert time.monotonic()-started < .03
    assert report['api_anomaly']
    with pytest.raises(RobotError, match='STOP_MOTION_ERROR'):
        while True:
            owner.read_state()
            owner.poll_stop()
            time.sleep(.01)
    assert .9 < time.monotonic()-started < 1.2
    assert owner.stop_state == 'FAILED' and report['physical_stop'] == 'failed'
    assert not record['mode_exits']  # Never enter synchronous speedStop while moving.
    with pytest.raises(RobotError):
        owner.close()
    assert not devices.receive.connected and not devices.control.connected


def test_terminal_physical_failure_upgrades_reason(config, monkeypatch, tmp_path):
    target = prepare(config, monkeypatch)
    devices = Devices(config, target)
    record = braking_device(devices, stop_value=False, stuck=True)
    assert runtime.run(args(tmp_path), controller_factory=devices.controller, reader_factory=Sensor) == 1
    run_dir, = (tmp_path/'real/continuous').glob('run_*')
    termination = json.loads((run_dir/'termination.json').read_text())
    assert termination['reason'] == TerminationReason.STOP_MOTION_ERROR
    assert 'did not settle' in termination['detail']
    assert devices.owner.stop_state == 'FAILED'
    summary = json.loads((run_dir/'summary.json').read_text())
    report, = summary['stop_observation']['stop_requests']
    assert report['api_anomaly'] and report['physical_stop'] == 'failed'
    assert record['script_stop'] is None
    assert not devices.receive.connected and not devices.control.connected


@pytest.mark.parametrize('fault', ['force', 'nonfinite', 'stale_rtde'])
def test_stopping_keeps_hard_force_and_freshness_guards(config, monkeypatch, tmp_path, fault):
    target = prepare(config, monkeypatch)
    devices = Devices(config, target)
    record = braking_device(devices)
    devices.control.stopScript = lambda: True  # Faults deliberately stop healthy watchdog kicks.
    class FaultSensor(Sensor):
        def read_wrench(self):
            if record['start'] is not None:
                if fault == 'stale_rtde':
                    devices.receive.frozen_stamp = devices.owner._packet_stamp
                else:
                    return Wrench(100 if fault == 'force' else np.nan, 0, 0, 0, 0, 0)
            return super().read_wrench()
    assert runtime.run(args(tmp_path), controller_factory=devices.controller, reader_factory=FaultSensor) == 1
    run_dir, = (tmp_path/'real/continuous').glob('run_*')
    termination = json.loads((run_dir/'termination.json').read_text())
    assert termination['reason'] != TerminationReason.STOP_TIME_LIMIT
    if fault == 'force':
        assert termination['reason'] == TerminationReason.STOP_FORCE_LIMIT
    if fault == 'stale_rtde':
        summary = json.loads((run_dir/'summary.json').read_text())
        assert not summary['stop_observation']['standstill_confirmed']
    assert not devices.receive.connected and not devices.control.connected


def test_return_segments_poll_without_blocking_standstill_wait(config, monkeypatch):
    target = np.array(config['dry_run']['start_pose'])
    initial = target.copy(); initial[0] += .01
    devices = Devices(config, initial)
    owner = devices.controller(runtime_robot_config(config))
    owner.connect(); owner.activate_control(confirmed=True)
    def forbidden(**kwargs):
        raise AssertionError('return segment used blocking wait_for_standstill')
    monkeypatch.setattr(owner, 'wait_for_standstill', forbidden)
    executor = SafeReturnExecutor(config, target, owner)
    seen = []
    original = executor.observe
    def observe():
        state = original()
        seen.append((executor.phase, time.monotonic()))
        return state
    monkeypatch.setattr(executor, 'observe', observe)
    result = executor.execute()
    assert result.status == 'complete'
    for phase in ('VERTICAL_RETREAT', 'MOVE_ABOVE_START', 'DESCEND_TO_START'):
        ticks = [t for p, t in seen if p == phase+'_STOPPING']
        assert len(ticks) >= 8 and ticks[-1]-ticks[0] >= .08
    assert len([c for c in devices.control.calls if c[0] == 'stopL']) == 3
    owner.close()

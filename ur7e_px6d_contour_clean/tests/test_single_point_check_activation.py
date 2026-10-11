"""Operator waiting is read-only; authorized motion keeps its protection."""
from types import SimpleNamespace
from argparse import Namespace

import pytest

from app.single_point_config import prepare
from app.single_point_runtime import FreshForceReader
from app.single_point_startup import prepare_startup
from core.models import Wrench
from robot.rtde_controller import RobotError
from safety.safe_return import ReturnAborted
from sensor.force_preprocess import WrenchPreprocessor
from tests.doubles import Devices, Sensor, Keyboard
from tests.test_single_point import rig, settings, MemoryLog, bounded, fake_remote
from simulation.single_point import demo_calibration
import run_single_point_contact as entry


@pytest.mark.parametrize('wrench', [Wrench(2., 0., 0., 0., 0., 0.),
                                  Wrench(0., 0., 0., 1.2, 0., 0.),
                                  Wrench(61., 0., 0., 0., 0., 0.),
                                  Wrench(0., 0., 0., 5.1, 0., 0.)])
def test_waiting_records_force_without_experiment_limits_or_motion(config, settings, wrench):
    sensor = SimpleNamespace(read_wrench=lambda: wrench)
    trial, owner, log = rig(config, settings, sensor=lambda c: sensor)
    trial.record_precontact()
    trial.hold_once()  # Same callback used while the final Enter is pending.
    assert not trial.experiment_checks_active and trial.experiment_checks_activated_sec is None
    assert not trial.safety_events and not trial.fault and not trial.stop_requested
    assert trial.threshold_time is None and trial.state == 'READY'
    assert not owner.command_records and not log.commands and not log.events
    assert len(log.samples) >= 100 and all(row['extra']['force_valid'] == 1 for row in log.samples)
    assert all(row['extra']['experiment_checks_active'] == 0 for row in log.samples)
    with pytest.raises(RobotError, match='not authorized'):
        owner.command_planar_velocity(trial.direction, trial.speed, .01)
    assert not owner.command_records
    # The existing final Enter must re-read and validate before any motion.
    with pytest.raises(RobotError, match='(not unloaded|safety threshold exceeded)'):
        trial.authorize_approach()
    assert trial.experiment_checks_active
    assert trial.experiment_checks_activated_sec == owner.clock()
    assert not owner.command_records and not log.commands


def test_formal_approach_still_stops_on_first_force_threshold(config, settings):
    trial, owner, log = rig(config, settings, contact=.0002)
    trial.record_precontact()
    trial.authorize_approach()
    authorized = trial.experiment_checks_activated_sec
    result = trial.run()
    assert result['status'] == 'contact' and result['contact_triggered']
    assert result['experiment_checks_activated_sec'] == authorized
    assert result['threshold_trigger_sec'] > authorized
    assert result['post_stop_record_completed'] and not result['safety_events']
    assert any(row['extra']['experiment_checks_active'] == 1 for row in log.samples)


@pytest.mark.parametrize('confirm_motion', [False, True])
def test_preparation_wait_never_reads_force_but_motion_preflight_does(config, settings, confirm_motion):
    settings['return_lift_distance_m'] = .03
    data = demo_calibration(config)
    effective, robot_config, target, _ = prepare(config, settings, data, 'A', execute=False)
    devices = Devices(effective, target)
    owner = devices.controller(robot_config)
    owner.connect()
    processor = WrenchPreprocessor.from_config(effective['preprocessing'], tool_orientation=target[3:])
    class LoadedSensor(Sensor):
        def read_wrench(self):
            self.count += 1
            return Wrench(0., 0., 0., 5.01, 0., 0.)
    raw = LoadedSensor()
    reader = FreshForceReader(raw, settings, effective['policy'])
    keyboard = SimpleNamespace(read_line=lambda prompt: '')
    def confirm(prompt, **kw):
        keyboard.on_wait()
        keyboard.on_wait()
        assert raw.count == 0 and devices.control_count == 0
        return confirm_motion
    try:
        expected = ReturnAborted if confirm_motion else KeyboardInterrupt
        with pytest.raises(expected, match='safety threshold exceeded' if confirm_motion else 'path not confirmed'):
            prepare_startup(effective, settings, target, owner, reader, processor, MemoryLog(), keyboard,
                            confirm=confirm)
        assert raw.count == (1 if confirm_motion else 0)
        assert devices.control_count == 0 and not devices.control.calls
    finally:
        owner.close()


def test_speed_selection_wait_does_not_run_preparation_force_monitor(
        tmp_path, monkeypatch, config, settings, fake_remote):
    bounded(settings)
    settings['return_lift_distance_m'] = .03
    config['preprocessing']['baseline']['sample_count'] = 3
    data = demo_calibration(config)
    devices = Devices(config, data['groups']['A']['tcp_pose'])
    class Feedback(Sensor):
        torque = 0.
        def read_wrench(self):
            self.count += 1
            return Wrench(0., 0., 0., self.torque, 0., 0.)
    sensor = Feedback()
    speed_wait_observed = []
    class Input(Keyboard):
        def read_text(self, prompt):
            sensor.torque = 2.
            count = sensor.count
            try:
                self.on_wait()
                assert sensor.count == count  # Robot-only read while waiting.
                speed_wait_observed.append(True)
            finally:
                sensor.torque = 0.
            return ''
    monkeypatch.setattr(entry, 'read_tcp_offset_readonly', lambda ip: config['tcp']['offset'])
    monkeypatch.setattr(entry, 'confirm_enter', lambda prompt, **kw: '确认沿所示方向接近' not in prompt)
    args = Namespace(execute=True, output=tmp_path, config=entry.ROOT/'config.yaml')
    result = entry.run_trial(args, config, settings, data, 'A', controller_factory=devices.controller,
                             reader_factory=lambda project: sensor, keyboard_factory=Input)
    assert speed_wait_observed == [True]
    assert not result['experiment_checks_active'] and result['experiment_checks_activated_sec'] is None
    assert not result['contact_triggered'] and not result['safety_events']
    assert not any(call[0] == 'speedL' and any(call[1]) for call in devices.control.calls)

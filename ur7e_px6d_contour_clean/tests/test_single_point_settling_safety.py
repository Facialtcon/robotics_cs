"""Separate real contact, valid over-limit feedback and missing stop telemetry."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from app.single_point_ui import show_trial_result
from core.models import Wrench
from experiment_logging.termination import TerminationRecorder
from robot.rtde_controller import RobotError
from sensor.px6d_reader import PX6DTimeout
from tests.test_single_point import rig, settings
import run_single_point_contact as entry


def stop_sensor(trial, *, torque=0., force=0., failure=None, after_confirmation=False):
    original = trial.reader.reader
    class Feedback:
        calls_after_stop = 0
        @property
        def sample_timestamp(self):
            return trial.clock() - (1. if failure == 'stale' and self.active() else 0.)
        def active(self):
            return trial.stopped_time is not None if after_confirmation else trial.stop_requested
        def read_wrench(self):
            raw = original.read_wrench()
            if not self.active():
                return raw
            self.calls_after_stop += 1
            if failure == 'timeout':
                raise PX6DTimeout('fake PX6D disconnected after contact')
            if failure == 'nan':
                return Wrench.from_sequence([np.nan]*6)
            values = raw.array().copy()
            values[0] += force
            values[3] += torque
            return Wrench.from_sequence(values)
    trial.reader.reader = Feedback()
    return trial.reader.reader


def test_safe_contact_remains_normal_completion(config, settings, capsys):
    trial, controller, log = rig(config, settings, contact=.0002)
    result = trial.run()
    assert result['status'] == 'contact' and result['contact_triggered']
    assert not result['safety_events'] and not result['force_read_error']
    assert result['post_stop_record_completed'] and not result['stop_failures']
    show_trial_result(result)
    output = capsys.readouterr().out
    assert '接触触发：Fxy' in output and '停稳后记录：完成' in output
    assert '异常' not in output and '未完成' not in output


@pytest.mark.parametrize('torque,force,expected', [
    (1.2, 0., 'processed torque safety threshold exceeded'),
    (5.1, 0., 'absolute raw torque safety threshold exceeded'),
    (0., 61., 'absolute raw force safety threshold exceeded'),
])
@pytest.mark.parametrize('after_confirmation', [False, True])
def test_safety_overshoot_keeps_contact_and_fresh_stop_records(config, settings, capsys,
                                                            torque, force, expected, after_confirmation):
    original_policy = deepcopy(config['policy'])
    trial, controller, log = rig(config, settings, contact=.0002)
    sensor = stop_sensor(trial, torque=torque, force=force, after_confirmation=after_confirmation)
    result = trial.run()
    assert result['contact_triggered'] and result['reason'] == 'CONTACT_THRESHOLD'
    assert result['status'] == 'aborted'  # Never conceal a real hard-safety crossing.
    assert result['actual_standstill_sec'] is not None and result['post_stop_record_completed']
    assert result['stop_phase'] == 'COMPLETE' and result['stop_failure_kind'] is None
    assert not result['force_read_error'] and not trial.force_failed and not result['stop_failures']
    event = next(item for item in result['safety_events'] if item['reason'] == expected)
    assert event['value'] >= event['limit'] and event['peak_value'] >= event['value']
    assert event['phase'] == ('POST_STOP_RECORDING' if after_confirmation else 'BRAKING_CONFIRMATION')
    assert event['first_sec'] >= result['braking_request_sec']
    assert event['force_read_start_sec'] <= event['force_read_end_sec'] <= event['first_sec']
    assert np.isfinite(event['raw_wrench_sensor'] + event['processed_wrench_base']).all()
    assert event['sample_count'] > 1 and sensor.calls_after_stop > 20
    assert all(row['extra']['force_valid'] == 1 for row in log.samples)
    assert log.samples[-1]['extra']['force_safety_reason']
    assert np.isfinite(log.samples[-1]['raw'].array()).all()
    assert config['policy'] == original_policy
    brake_index = next(i for i, item in enumerate(log.commands) if item['kind'] == 'braking')
    assert all(item['kind'] != 'motion' for item in log.commands[brake_index:])
    show_trial_result(result)
    output = capsys.readouterr().out
    assert '接触阈值已触发' in output and '安全超限' in output
    assert '实际停稳已确认；停稳后记录：完成' in output
    assert 'sensor failed' not in output and '停车数据记录未完成' not in output


@pytest.mark.parametrize('failure', ['timeout', 'nan', 'stale'])
def test_real_force_failure_is_reported_with_evidence(config, settings, failure):
    trial, controller, log = rig(config, settings, contact=.0002)
    stop_sensor(trial, failure=failure)
    result = trial.run()
    assert result['contact_triggered'] and result['status'] == 'aborted'
    assert result['actual_standstill_sec'] is not None and not result['post_stop_record_completed']
    assert result['force_read_error'] and not result['safety_events']
    assert result['stop_failure_kind'] == 'POST_STOP_RECORDING_INCOMPLETE'
    assert log.samples[-1]['extra']['force_valid'] == 0
    assert log.samples[-1]['extra']['force_read_error'] == result['force_read_error']
    if failure == 'timeout':
        assert 'fake PX6D disconnected' in result['fault']


def test_tcp_observation_error_is_not_reported_as_sensor_failure(config, settings):
    trial, controller, log = rig(config, settings, contact=.0002)
    original = controller.read_diagnostic_state
    raised = False
    def fail_once():
        nonlocal raised
        if trial.stop_requested and not raised:
            raised = True
            raise RobotError('fake RTDE read failed')
        return original()
    controller.read_diagnostic_state = fail_once
    result = trial.run()
    assert result['contact_triggered'] and result['status'] == 'aborted'
    assert result['actual_standstill_sec'] is not None and result['post_stop_record_completed']
    assert result['stop_failure_kind'] == 'STOP_TELEMETRY_INCOMPLETE'
    assert not result['force_read_error'] and not trial.force_failed
    assert 'fake RTDE read failed' in result['fault'] and 'sensor failed' not in result['fault']
    assert log.samples[-1]['extra']['force_valid'] == 1


@pytest.mark.parametrize('safety', [False, True])
def test_termination_records_force_limit_or_success_without_fake_nan(config, settings, tmp_path, safety):
    trial, _, _ = rig(config, settings, contact=.0002)
    if safety:
        stop_sensor(trial, torque=1.2)
    result = trial.run()
    recorder = TerminationRecorder(mode='simulation', strategy='single_point')
    recorder.run_dir = tmp_path
    # Even an unrelated last NaN must not overwrite explicit trial evidence.
    recorder.observe(raw=Wrench.from_sequence([np.nan]*6))
    entry.record_trial_termination(SimpleNamespace(termination=recorder), result)
    assert recorder.record['reason'] == ('STOP_FORCE_LIMIT' if safety else 'SUCCESS')
    if safety:
        assert np.isfinite(recorder.record['raw_wrench']).all()
        assert recorder.record['phase'] == 'BRAKING_CONFIRMATION'
    recorder.flush()
    assert (tmp_path/'termination.json').is_file()

"""Sensor startup failure must not invent an RTDE motion-stop failure."""
import json

import pytest

from app import continuous_runtime as runtime
from doubles import Devices, Sensor
from sensor.px6d_reader import PX6DError, PX6DTimeout
from test_continuous_runtime import args, prepare


@pytest.mark.parametrize('failure_stage', ['connect', 'first_wrench'])
def test_startup_sensor_failure_distinguishes_unopened_robot_from_receive_only(
        config, monkeypatch, tmp_path, failure_stage):
    target = prepare(config, monkeypatch)
    devices = Devices(config, target)
    diagnostic_reads = []
    evidence = {'requires_resynchronization': True,
                'requests': [{'request_id': 1, 'command': 7, 'tx_bytes': 6, 'rx_bytes': 0}]}

    class StartupFailureSensor(Sensor):
        closed = False
        failed = False

        def fail(self):
            self.failed = True
            timeout = PX6DTimeout('offline startup response timeout')
            timeout.sensor_diagnostics = evidence
            if failure_stage == 'connect':
                error = PX6DError('cannot initialize PX6D: offline startup response timeout')
                error.sensor_diagnostics = evidence
                raise error from timeout
            raise timeout

        def connect(self):
            if failure_stage == 'connect':
                self.fail()
            super().connect()

        def read_wrench(self):
            assert not self.failed, 'cleanup must not retry the failed sensor'
            self.fail()

        def close(self):
            self.closed = True
            super().close()

    def controller_factory(settings):
        owner = devices.controller(settings)
        read_state = owner.read_diagnostic_state
        def observed():
            diagnostic_reads.append(owner.receive is not None)
            return read_state()
        owner.read_diagnostic_state = observed
        return owner

    sensor = StartupFailureSensor()
    assert runtime.run(args(tmp_path), controller_factory=controller_factory,
                       reader_factory=lambda *a: sensor) == 1
    run_dir, = (tmp_path/'real/continuous').glob('run_*')
    termination = json.loads((run_dir/'termination.json').read_text())
    stop = json.loads((run_dir/'summary.json').read_text())['stop_observation']
    assert termination['reason'] == 'STOP_SENSOR_ERROR'
    assert termination['sensor_diagnostics'] == evidence
    assert not termination['secondary_errors']
    assert sensor.closed and not sensor.connected
    assert devices.control_count == 0 and not devices.control.calls
    if failure_stage == 'connect':
        assert devices.receive_count == 0 and not diagnostic_reads
        assert stop['source'] == 'not_connected'
        assert stop['motion_status'] == 'no_motion_commanded_by_this_task'
        assert stop['standstill_confirmed'] is False
        assert stop['runtime_stop_state'] == 'NOT_CONNECTED'
        assert stop['stop_requests'] == []
    else:
        assert devices.receive_count == 1 and diagnostic_reads and all(diagnostic_reads)
        assert stop['source'] == 'fresh_rtde_actual_speed'
        assert stop['standstill_confirmed'] is True
        assert not devices.receive.connected

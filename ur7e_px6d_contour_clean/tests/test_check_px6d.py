"""Standalone diagnostic observations never retry or return a late force."""
import json
import sys
from types import SimpleNamespace

import pytest

from sensor import px6d_reader as px
from tools import check_px6d as tool
from test_px6d_reader import Clock, Port, packet, version_packet


@pytest.fixture
def diagnostic_reader(monkeypatch):
    clock = Clock()
    timer = SimpleNamespace(monotonic=clock.monotonic, sleep=clock.sleep)
    monkeypatch.setattr(px, 'time', timer)
    monkeypatch.setattr(tool, 'time', timer)
    reader = tool.DiagnosticPX6DReader('offline-fake-port', startup_delay_sec=0.)
    port = Port(clock)
    port.reset_output_buffer = lambda: None
    reader._port = port
    return reader, port, clock


@pytest.mark.parametrize('response', [b'', packet(), packet()[:10], packet()[:-1]+b'\0'])
def test_passive_failed_frame_capture_is_bounded_without_retry(diagnostic_reader, response):
    reader, port, clock = diagnostic_reader
    port.on_write = lambda command: port.schedule(.075, response)
    with pytest.raises(px.PX6DTimeout):
        reader.read_wrench()
    failure_record = dict(reader.diagnostics_snapshot()['requests'][-1])
    started = clock.now
    reader.close()
    observed = reader.passive_observation
    assert 1. <= clock.now-started < 1.002
    assert len(port.writes) == 1 and observed['tx_bytes'] == 0
    assert observed['rx_bytes'] == len(response)
    assert len(observed['valid_packets']) == int(response == packet())
    assert reader.diagnostics_snapshot()['requests'][-1] == failure_record
    assert reader._requires_resynchronization and port.closed
    reader.close()  # Cleanup is idempotent, never another observation window.
    assert reader.passive_observation is observed


def test_passive_observation_joins_partial_failure_buffer_without_relabeling_it(diagnostic_reader):
    reader, port, _ = diagnostic_reader
    def respond(command):
        port.schedule(.001, packet()[:10])
        port.schedule(.075, packet()[10:])
    port.on_write = respond
    with pytest.raises(px.PX6DTimeout):
        reader.read_wrench()
    reader.close()
    assert reader.passive_observation['rx_bytes'] == 19
    assert reader.passive_observation['buffered_bytes_before'] == 10
    assert len(reader.passive_observation['valid_packets']) == 1
    assert reader.diagnostics_snapshot()['requests'][-1]['rx_bytes'] == 10
    assert len(port.writes) == 1


def test_startup_version_failure_is_observed_before_close(diagnostic_reader, monkeypatch, tmp_path):
    import serial
    reader, port, _ = diagnostic_reader
    reader._port = None
    port.on_write = lambda command: port.schedule(2.05, version_packet())
    monkeypatch.setattr(serial, 'Serial', lambda *a, **k: port)
    monkeypatch.setattr(tool, 'DiagnosticPX6DReader', lambda *args: reader)
    output = tmp_path/'nested/diagnostics.json'
    monkeypatch.setattr(sys, 'argv', ['check_px6d', '--samples', '3', '--diagnostics-output', str(output)])
    assert tool.main() == 1
    result = json.loads(output.read_text())
    assert result['samples_completed'] == 0
    assert result['passive_after_failure']['valid_packets'][0]['command'] == px.CMD_GET_VERSION
    assert result['sensor_diagnostics']['requests'][-1]['rx_bytes'] == 0
    assert result['passive_after_failure']['rx_bytes'] == 13
    assert port.closed and len(port.writes) == 1


def test_passive_disconnect_still_closes_and_retains_diagnostic(diagnostic_reader, monkeypatch):
    reader, port, _ = diagnostic_reader
    reader._requires_resynchronization = True
    def disconnected(self):
        raise OSError('USB disconnected during passive observation')
    monkeypatch.setattr(Port, 'in_waiting', property(disconnected))
    reader.close()
    assert port.closed
    assert 'USB disconnected' in reader.passive_observation['error']
    assert reader._requires_resynchronization and not port.writes


def test_success_records_whole_collection_without_passive_wait(diagnostic_reader, monkeypatch, tmp_path):
    reader, port, clock = diagnostic_reader
    monkeypatch.setattr(reader, 'connect', lambda: None)
    monkeypatch.setattr(tool, 'DiagnosticPX6DReader', lambda *args: reader)
    port.on_write = lambda command: port.schedule(.001, packet())
    output = tmp_path/'sensor.json'
    monkeypatch.setattr(sys, 'argv', ['check_px6d', '--samples', '4', '--diagnostics-output', str(output)])
    assert tool.main() == 0
    result = json.loads(output.read_text())
    assert result['read_call_timing']['count'] == 4
    assert result['samples_completed'] == result['requested_samples'] == 4
    assert result['collection_elapsed_sec'] < .05
    assert result['passive_after_failure'] is None and port.closed
    assert result['wrench_statistics']['Fx']['mean'] == 1.
    assert result['wrench_statistics']['Tz']['max'] == 6.


def test_connect_interrupt_closes_and_writes_failure(diagnostic_reader, monkeypatch, tmp_path):
    reader, port, _ = diagnostic_reader
    def interrupted():
        raise KeyboardInterrupt
    monkeypatch.setattr(reader, 'connect', interrupted)
    monkeypatch.setattr(tool, 'DiagnosticPX6DReader', lambda *args: reader)
    output = tmp_path/'sensor.json'
    monkeypatch.setattr(sys, 'argv', ['check_px6d', '--samples', '4', '--diagnostics-output', str(output)])
    assert tool.main() == 130
    assert port.closed
    assert 'KeyboardInterrupt' in json.loads(output.read_text())['error']


def test_close_error_is_not_reported_as_success(diagnostic_reader, monkeypatch, tmp_path):
    reader, port, _ = diagnostic_reader
    monkeypatch.setattr(reader, 'connect', lambda: None)
    monkeypatch.setattr(tool, 'DiagnosticPX6DReader', lambda *args: reader)
    port.on_write = lambda command: port.schedule(.001, packet())
    def failed_close():
        port.closed = True
        raise OSError('close disconnected')
    port.close = failed_close
    output = tmp_path/'sensor.json'
    monkeypatch.setattr(sys, 'argv', ['check_px6d', '--samples', '1', '--diagnostics-output', str(output)])
    assert tool.main() == 1
    result = json.loads(output.read_text())
    assert result['samples_completed'] == 1 and 'close disconnected' in result['error']
    assert port.closed and reader._port is None


@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_invalid_values_do_not_break_diagnostic_output(diagnostic_reader, monkeypatch, tmp_path, value):
    reader, port, _ = diagnostic_reader
    monkeypatch.setattr(reader, 'connect', lambda: None)
    monkeypatch.setattr(tool, 'DiagnosticPX6DReader', lambda *args: reader)
    port.on_write = lambda command: port.schedule(.001, packet((value, 0, 0, 0, 0, 0)))
    output = tmp_path/'sensor.json'
    monkeypatch.setattr(sys, 'argv', ['check_px6d', '--samples', '2', '--diagnostics-output', str(output)])
    assert tool.main() == 1
    result = json.loads(output.read_text())
    assert result['samples_completed'] == 0 and result['wrench_statistics'] == {}
    assert 'nonfinite' in result['error'] and result['invalid_sample'][0] == str(value)
    assert len(port.writes) == 1 and port.closed


def test_cli_real_posix_serial_on_pty_stops_on_missing_response(config, monkeypatch, tmp_path):
    """Exercise real pyserial/ioctl/os.write on a local PTY, never a device node."""
    import os
    import pty
    import select
    import serial
    import serial.serialposix
    import threading
    master, slave = pty.openpty()
    device = os.ttyname(slave)
    requests, errors, ports = [], [], []
    stopped = threading.Event()
    def peer():
        pending = bytearray()
        try:
            while not stopped.is_set():
                if not select.select([master], [], [], .02)[0]:
                    continue
                pending.extend(os.read(master, 1024))
                while len(pending) >= 6:
                    command = bytes(pending[:6]); del pending[:6]
                    assert command in (px.GET_VERSION_COMMAND, px.GET_FRAME_COMMAND)
                    requests.append(command)
                    if command == px.GET_VERSION_COMMAND:
                        reply = version_packet()
                    elif len(requests) <= 6:
                        reply = packet((len(requests)-1, 2, 3, 4, 5, 6))
                    else:
                        continue  # The sixth force request receives no response.
                    os.write(master, reply[:1])
                    os.write(master, reply[1:])
        except BaseException as exc:
            errors.append(exc)
    def serial_factory(path, **kwargs):
        assert path == device and path.startswith('/dev/pts/')
        port = serial.serialposix.Serial(path, **kwargs)
        ports.append(port)
        return port
    config['sensor']['serial_port'] = device
    config['sensor']['startup_delay_sec'] = 0.
    monkeypatch.setattr(tool, 'load_config', lambda _: config)
    monkeypatch.setattr(serial, 'Serial', serial_factory)
    output = tmp_path/'pty.json'
    monkeypatch.setattr(sys, 'argv', ['check_px6d', '--samples', '10', '--diagnostics-output', str(output)])
    thread = threading.Thread(target=peer, daemon=True)
    thread.start()
    try:
        assert tool.main() == 1
        result = json.loads(output.read_text())
        assert result['samples_completed'] == 5
        assert result['wrench_statistics']['Fx']['min'] == 1.
        assert result['wrench_statistics']['Fx']['max'] == 5.
        assert result['sensor_diagnostics']['requests'][-1]['write_method'] == 'single nonblocking os.write'
        assert result['sensor_diagnostics']['requests'][-1]['rx_bytes'] == 0
        assert result['passive_after_failure']['rx_bytes'] == 0
        assert requests == [px.GET_VERSION_COMMAND]+[px.GET_FRAME_COMMAND]*6
        assert not errors and all(not port.is_open for port in ports)
    finally:
        stopped.set()
        thread.join(2)
        for port in ports:
            port.close()
        os.close(master)
        os.close(slave)

"""Independent reference probe tests: only local PTYs and fixed wire fixtures."""
import json
import os
import select
import threading
import time
from types import SimpleNamespace

import pytest

from tools.px6d_reference_probe import run_reference, _crc8


VERSION_COMMAND = bytes.fromhex('aa557f0701d0')
FRAME_COMMAND = bytes.fromhex('aa557f0501fa')
VERSION_REPLY = bytes.fromhex('aa557f0776312e302e31000019')
# Fixed independently encoded 1,2,3 N / 4,5,6 Nm wire fixture; no runtime parser.
FRAME_REPLY = bytes.fromhex('aa557f030000803f0000004000004040000080400000a0400000c040f2')


class Capture:
    def __init__(self):
        self.records = []

    def record(self, kind, data=b'', **fields):
        assert isinstance(data, bytes)
        self.records.append(dict(kind=kind, data=data, **fields))


@pytest.fixture
def local_sensor(monkeypatch):
    if os.name != 'posix':
        pytest.skip('POSIX reference probe')
    import pty
    import serial
    import serial.serialposix
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    requests, ports, errors = [], [], []
    done = threading.Event()
    peer_state = SimpleNamespace(version_response=VERSION_REPLY,
                                 force_response=lambda number: [(0., FRAME_REPLY)])

    def peer():
        incoming = bytearray()
        force_count = 0
        try:
            while not done.is_set():
                if not select.select([master], [], [], .01)[0]:
                    continue
                incoming.extend(os.read(master, 4096))
                while len(incoming) >= 6:
                    command = bytes(incoming[:6])
                    del incoming[:6]
                    requests.append(command)
                    if command == VERSION_COMMAND:
                        pieces = [(0., peer_state.version_response)]
                    else:
                        assert command == FRAME_COMMAND
                        force_count += 1
                        pieces = peer_state.force_response(force_count)
                    for delay, data in pieces:
                        time.sleep(delay)
                        if data:
                            os.write(master, data)
        except BaseException as exc:
            errors.append(exc)

    def factory(device, **kwargs):
        assert device == path and device.startswith('/dev/pts/')
        assert kwargs['exclusive'] is True
        assert kwargs['timeout'] == kwargs['write_timeout'] == 0.
        assert kwargs['baudrate'] == 921600
        assert kwargs['bytesize'] == 8 and kwargs['parity'] == 'N' and kwargs['stopbits'] == 1
        port = serial.serialposix.Serial(device, **kwargs)
        ports.append(port)
        return port

    def forbidden_in_waiting(_):
        raise AssertionError('reference receiver must not depend on in_waiting')

    monkeypatch.setattr(serial, 'Serial', factory)
    monkeypatch.setattr(serial.serialposix.Serial, 'in_waiting', property(forbidden_in_waiting))
    thread = threading.Thread(target=peer, daemon=True)
    thread.start()
    sensor = dict(serial_port=path, baudrate=921600, timeout_sec=.05,
                  poll_rate_hz=100., startup_delay_sec=0.)
    try:
        yield SimpleNamespace(config=sensor, peer=peer_state, requests=requests, ports=ports,
                              capture=Capture(), errors=errors)
    finally:
        done.set()
        thread.join(2.)
        for port in ports:
            port.close()
        os.close(master)
        os.close(slave)
        assert not errors


def run(peer, samples=3, duration=1.):
    result = run_reference(peer.config, samples, duration, peer.capture)
    assert all(not port.is_open for port in peer.ports)
    json.dumps(result)  # The API summary contains no raw bytes or numpy values.
    assert all('host_monotonic' in row and 'request_id' in row for row in peer.capture.records)
    return result


def test_independent_crc_matches_fixed_commands_and_frames():
    for wire in (VERSION_COMMAND, FRAME_COMMAND, VERSION_REPLY, FRAME_REPLY):
        assert _crc8(wire[:-1]) == wire[-1]


def test_fragmented_response_uses_select_and_records_raw_bytes(local_sensor):
    peer = local_sensor
    peer.peer.force_response = lambda _: [(0., FRAME_REPLY[:1]), (.001, FRAME_REPLY[1:12]),
                                         (.001, FRAME_REPLY[12:])]
    result = run(peer)
    assert result['samples_completed'] == result['valid_samples'] == 3
    assert result['firmware'] == 'v1.0.1' and result['error'] is None
    assert peer.requests == [VERSION_COMMAND]+[FRAME_COMMAND]*3
    replies = [r['data'] for r in peer.capture.records if r['kind'] == 'response']
    assert replies == [VERSION_REPLY]+[FRAME_REPLY]*3
    assert {'request_start', 'tx', 'rx', 'response', 'pace', 'close'} <= {
        r['kind'] for r in peer.capture.records}
    tx_times = [r['host_monotonic'] for r in peer.capture.records if r['kind'] == 'tx']
    assert all(b-a >= .0099 for a, b in zip(tx_times, tx_times[1:]))


def test_coalesced_extra_reply_is_not_accepted_for_next_request(local_sensor):
    peer = local_sensor
    peer.peer.force_response = lambda _: [(0., FRAME_REPLY+FRAME_REPLY)]
    result = run(peer)
    assert result['samples_completed'] == 1
    assert result['first_failure']['type'] == 'ReferenceProtocolError'
    assert result['first_failure']['request']['stage'] == 'preflight'
    assert peer.requests == [VERSION_COMMAND, FRAME_COMMAND]
    assert result['passive_after_failure']['tx_bytes'] == 0


def test_crc_failure_is_counted_before_accepting_following_valid_frame(local_sensor):
    peer = local_sensor
    bad = FRAME_REPLY[:-1]+bytes([FRAME_REPLY[-1] ^ 1])
    peer.peer.force_response = lambda _: [(0., bad+FRAME_REPLY)]
    result = run(peer, samples=1)
    assert result['samples_completed'] == 1 and result['error'] is None
    assert result['counters']['crc_failures'] == 1
    assert result['counters']['rx_bytes'] == len(VERSION_REPLY)+2*len(FRAME_REPLY)


@pytest.mark.parametrize('reply', [b'', FRAME_REPLY[:12], FRAME_REPLY[:-1]+b'\xf3'])
def test_no_response_partial_or_bad_crc_fails_without_retry(local_sensor, reply):
    peer = local_sensor
    peer.peer.force_response = lambda _: [(0., reply)]
    result = run(peer)
    assert result['samples_completed'] == 0
    failed = result['first_failure']['request']
    assert failed['rx_bytes'] == len(reply)
    assert failed['stage'] == 'read'
    assert .05 <= failed['elapsed_sec'] < .5
    assert peer.requests == [VERSION_COMMAND, FRAME_COMMAND]
    assert result['passive_after_failure']['tx_bytes'] == 0
    assert result['counters']['crc_failures'] == int(len(reply) == 29)


def test_late_force_is_only_passively_recorded_and_never_becomes_a_sample(local_sensor):
    peer = local_sensor
    peer.peer.force_response = lambda _: [(.075, FRAME_REPLY)]
    result = run(peer)
    assert result['samples_completed'] == 0 and result['first_failure'] is not None
    assert result['first_failure']['request']['rx_bytes'] == 0
    assert result['passive_after_failure']['rx_bytes'] == len(FRAME_REPLY)
    assert peer.requests == [VERSION_COMMAND, FRAME_COMMAND]
    assert [r['data'] for r in peer.capture.records if r['kind'] == 'response'] == [VERSION_REPLY]
    assert b''.join(r['data'] for r in peer.capture.records if r['kind'] == 'rx' and r['passive']) == FRAME_REPLY


def test_initial_version_timeout_also_closes_and_captures_diagnostics(local_sensor):
    peer = local_sensor
    peer.peer.version_response = b''
    result = run(peer)
    assert result['samples_completed'] == 0 and result['firmware'] is None
    assert result['first_failure']['phase'] == 'version'
    assert result['first_failure']['request']['timeout_sec'] == 2.
    assert peer.requests == [VERSION_COMMAND]
    assert result['collection_elapsed_sec'] == 0.
    assert result['passive_after_failure']['rx_bytes'] == 0


def test_duration_limit_does_not_start_a_request_without_its_full_budget(local_sensor):
    peer = local_sensor
    result = run(peer, duration=.025)
    assert result['stop_reason'] == 'duration' and result['error'] is None
    assert result['samples_completed'] == 0
    assert peer.requests == [VERSION_COMMAND]
    assert result['collection_elapsed_sec'] < .025

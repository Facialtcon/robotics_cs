"""Serial protocol and deadline tests use only a fake clock and byte queues."""
import struct
from types import SimpleNamespace

import pytest

from sensor import px6d_reader as px


def packet(values=(1., 2., 3., 4., 5., 6.), *, command=px.CMD_STREAM, device=0x7f):
    body = px.HEADER + bytes((device, command)) + struct.pack('<6f', *values)
    return body + bytes((px.crc8(body),))


def version_packet():
    body = px.HEADER + bytes((0x7f, px.CMD_GET_VERSION)) + b'v1.0.1\0\0'
    return body + bytes((px.crc8(body),))


class Clock:
    def __init__(self):
        self.now = 100.
        self.sleep_extra = 0.

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds + self.sleep_extra


class Port:
    def __init__(self, clock):
        self.clock = clock
        self.rx = bytearray()
        self.pending = []
        self.writes = []
        self.on_write = lambda command: None
        self.write_delay = self.flush_delay = self.read_delay = 0.
        self.short_write = False
        self.closed = False
        self.drain_until = clock.now

    def schedule(self, delay, data):
        self.pending.append((self.clock.now + delay, data))

    @property
    def in_waiting(self):
        due = [item for item in self.pending if item[0] <= self.clock.now]
        self.pending = [item for item in self.pending if item[0] > self.clock.now]
        for _, data in due:
            self.rx.extend(data)
        return len(self.rx)

    def read(self, size):
        self.clock.now += self.read_delay
        data = bytes(self.rx[:size])
        del self.rx[:size]
        return data

    def write(self, command):
        self.writes.append((self.clock.now, command))
        self.clock.now += self.write_delay
        self.on_write(command)
        self.drain_until = self.clock.now+self.flush_delay
        return len(command) - int(self.short_write)

    def flush(self):
        raise AssertionError('unbounded serial flush must never be called')

    @property
    def out_waiting(self):
        return 6 if self.clock.now < self.drain_until else 0

    def reset_input_buffer(self):
        self.in_waiting
        self.rx.clear()

    def close(self):
        self.closed = True


@pytest.fixture
def serial_reader(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(px, 'time', SimpleNamespace(monotonic=clock.monotonic, sleep=clock.sleep))
    reader = px.PX6DReader('offline-fake-port')
    port = Port(clock)
    reader._port = port
    return reader, port, clock


def test_complete_internal_packet_is_parsed_with_empty_serial_queue(serial_reader):
    reader, port, clock = serial_reader
    reader._buffer.extend(packet())
    assert port.in_waiting == 0
    started = clock.now
    assert reader._read_packet(29, .05) == packet()
    assert clock.now == started


def test_fragmented_header_and_payload(serial_reader):
    reader, port, _ = serial_reader
    frame = packet()
    def respond(command):
        for delay, part in [(0., b'noise'+frame[:1]), (.001, frame[1:9]), (.002, frame[9:28]), (.003, frame[28:])]:
            port.schedule(delay, part)
    port.on_write = respond
    assert reader.read_wrench().array().tolist() == [1, 2, 3, 4, 5, 6]
    record = reader.diagnostics_snapshot()['requests'][-1]
    assert record['rx_bytes'] == 34 and record['tx_bytes'] == 6
    assert record['crc_failures'] == 0
    assert record['last_rx_host_monotonic'] >= record['first_rx_host_monotonic'] + .003 - 1e-9


def test_coalesced_packets_can_be_extracted_without_another_serial_read(serial_reader):
    reader, port, _ = serial_reader
    second = packet((6, 5, 4, 3, 2, 1))
    port.rx.extend(packet()+second)
    assert reader._read_packet(29, .05) == packet()
    assert port.in_waiting == 0
    assert reader._read_packet(29, .05) == second


@pytest.mark.parametrize('where', ['internal', 'serial'])
def test_preexisting_reply_is_not_returned_for_a_new_request(serial_reader, where):
    reader, port, _ = serial_reader
    (reader._buffer if where == 'internal' else port.rx).extend(packet())
    with pytest.raises(px.ProtocolError, match='unconsumed') as failure:
        reader.read_wrench()
    assert not port.writes
    assert failure.value.sensor_diagnostics['requires_resynchronization']


def test_extra_coalesced_reply_does_not_cross_request_boundary(serial_reader):
    reader, port, _ = serial_reader
    port.on_write = lambda command: port.rx.extend(packet()+packet((9, 9, 9, 9, 9, 9)))
    assert reader.read_wrench().fx == 1
    with pytest.raises(px.ProtocolError, match='unconsumed'):
        reader.read_wrench()
    assert len(port.writes) == 1


def test_bad_crc_followed_by_valid_frame_is_counted_and_resynchronized(serial_reader):
    reader, port, _ = serial_reader
    bad = packet()[:-1]+bytes((packet()[-1] ^ 1,))
    port.on_write = lambda command: port.rx.extend(bad+packet())
    assert reader.read_wrench().fx == 1
    diagnostic = reader.diagnostics_snapshot()
    assert diagnostic['crc_failures_total'] == diagnostic['requests'][-1]['crc_failures'] == 1
    assert diagnostic['requests'][-1]['rx_bytes'] == 58


@pytest.mark.parametrize('reply,crc_failures', [(b'', 0), (packet()[:12], 0), (packet()[:-1]+bytes((packet()[-1]^1,)), 1)])
def test_timeout_distinguishes_no_bytes_partial_packet_and_bad_crc(serial_reader, reply, crc_failures):
    reader, port, _ = serial_reader
    port.on_write = lambda command: port.rx.extend(reply)
    with pytest.raises(px.PX6DTimeout) as failure:
        reader.read_wrench()
    record = failure.value.sensor_diagnostics['requests'][-1]
    assert record['rx_bytes'] == len(reply)
    assert record['crc_failures'] == crc_failures
    assert .05 <= record['read_elapsed_sec'] < .0503
    assert record['buffer_bytes_end'] == (12 if len(reply) == 12 else int(bool(reply)))
    assert not record['valid_packet_buffered_at_failure']
    assert record['parse_attempts'] > 0


@pytest.mark.parametrize('delivery', ['before_next_call', 'after_next_call_would_write'])
def test_timeout_latches_and_late_reply_cannot_be_used_as_new_force(serial_reader, delivery):
    reader, port, clock = serial_reader
    port.on_write = lambda command: port.schedule(.051, packet())
    with pytest.raises(px.PX6DTimeout):
        reader.read_wrench()
    if delivery == 'before_next_call':
        clock.sleep(.01)
    with pytest.raises(px.PX6DError, match='stationary resynchronization'):
        reader.read_wrench()
    assert len(port.writes) == 1


def test_read_that_returns_past_deadline_keeps_unparsed_packet_as_evidence(serial_reader):
    reader, port, _ = serial_reader
    port.on_write = lambda command: port.rx.extend(packet())
    port.read_delay = .06
    with pytest.raises(px.PX6DTimeout) as failure:
        reader.read_wrench()
    record = failure.value.sensor_diagnostics['requests'][-1]
    assert record['valid_packet_buffered_at_failure']
    assert record['buffer_bytes_end'] == 29 and record['rx_bytes'] == 29
    assert record['max_read_loop_gap_sec'] == pytest.approx(.06)


def test_long_poll_gap_is_visible_even_without_received_bytes(serial_reader):
    reader, _, clock = serial_reader
    clock.sleep_extra = .06
    with pytest.raises(px.PX6DTimeout) as failure:
        reader.read_wrench()
    record = failure.value.sensor_diagnostics['requests'][-1]
    assert record['rx_bytes'] == 0
    assert record['max_read_loop_gap_sec'] >= .06


def test_poll_gap_does_not_mislabel_unread_os_bytes_as_no_response(serial_reader):
    reader, port, clock = serial_reader
    port.on_write = lambda command: port.schedule(.001, packet())
    clock.sleep_extra = .06
    with pytest.raises(px.PX6DTimeout) as failure:
        reader.read_wrench()
    record = failure.value.sensor_diagnostics['requests'][-1]
    assert record['rx_bytes'] == 0 and record['serial_bytes_at_failure'] == 29
    assert record['max_read_loop_gap_sec'] >= .06


def test_deadline_bounds_pacing_write_and_output_drain_together(serial_reader):
    reader, port, clock = serial_reader
    port.on_write = lambda command: port.rx.extend(packet())
    reader.read_wrench()
    port.on_write = lambda command: None
    port.write_delay, port.flush_delay = .02, .03
    with pytest.raises(px.PX6DTimeout) as failure:
        reader.read_wrench()
    record = failure.value.sensor_diagnostics['requests'][-1]
    assert record['pacing_elapsed_sec'] == pytest.approx(.01)
    assert record['previous_request_interval_sec'] >= .01 - 1e-9
    assert record['write_elapsed_sec'] == pytest.approx(.02)
    assert record['flush_elapsed_sec'] == pytest.approx(.02, abs=.0003)
    assert record['read_iterations'] == 0
    assert .05-1e-9 <= record['request_elapsed_sec'] < .0503
    assert record['failure_stage'] == 'output_drain'


def test_stuck_output_queue_is_bounded_without_calling_flush(serial_reader):
    reader, port, _ = serial_reader
    port.flush_delay = 1000.
    with pytest.raises(px.PX6DTimeout) as failure:
        reader.read_wrench()
    record = failure.value.sensor_diagnostics['requests'][-1]
    assert .05-1e-9 <= record['request_elapsed_sec'] < .0503
    assert record['failure_stage'] == 'output_drain' and record['read_iterations'] == 0
    assert 'actual request' in str(failure.value)


def test_parser_overrun_is_measured_and_cannot_return_force(serial_reader, monkeypatch):
    reader, port, clock = serial_reader
    port.on_write = lambda command: port.rx.extend(packet())
    validate = px.validate_packet
    def delayed_validation(*args):
        clock.now += .06
        return validate(*args)
    monkeypatch.setattr(px, 'validate_packet', delayed_validation)
    with pytest.raises(px.PX6DTimeout) as failure:
        reader.read_wrench()
    record = failure.value.sensor_diagnostics['requests'][-1]
    assert record['parse_elapsed_sec'] == pytest.approx(.06)
    assert record['packet_parsed_after_deadline']


def test_write_scheduling_overrun_is_measured_and_cannot_accept_queued_force(serial_reader):
    reader, port, _ = serial_reader
    port.write_delay = .06
    port.on_write = lambda command: port.rx.extend(packet())
    with pytest.raises(px.PX6DTimeout) as failure:
        reader.read_wrench()
    record = failure.value.sensor_diagnostics['requests'][-1]
    assert record['failure_stage'] == 'write' and record['read_iterations'] == 0
    assert record['request_elapsed_sec'] == pytest.approx(.06)


def test_operator_interrupted_request_also_blocks_late_response_reuse(serial_reader):
    reader, port, _ = serial_reader
    def interrupt(command):
        raise KeyboardInterrupt
    port.on_write = interrupt
    with pytest.raises(KeyboardInterrupt):
        reader.read_wrench()
    port.rx.extend(packet())
    with pytest.raises(px.PX6DError, match='stationary resynchronization'):
        reader.read_wrench()
    assert len(port.writes) == 1


@pytest.mark.parametrize('bad', [packet(command=0x07), packet(device=0x01)])
def test_crc_valid_wrong_command_or_device_is_not_force(serial_reader, bad):
    reader, port, _ = serial_reader
    port.on_write = lambda command: port.rx.extend(bad+packet())
    assert reader.read_wrench().fx == 1
    assert reader.diagnostics_snapshot()['requests'][-1]['candidate_rejections'] == 1


def test_explicit_stationary_resync_requires_version_barrier_before_new_force(serial_reader):
    reader, port, _ = serial_reader
    with pytest.raises(px.PX6DTimeout):
        reader.read_wrench()
    # The old force reply arrives while waiting for the different response type.
    port.on_write = lambda command: port.rx.extend(packet()+version_packet())
    reader.resynchronize_after_timeout()
    assert reader.firmware == 'v1.0.1'
    port.on_write = lambda command: port.rx.extend(packet((9, 8, 7, 6, 5, 4)))
    assert reader.read_wrench().fx == 9
    assert [cmd for _, cmd in port.writes] == [px.GET_FRAME_COMMAND, px.GET_VERSION_COMMAND, px.GET_FRAME_COMMAND]


def test_failed_stationary_resync_does_not_release_timeout_latch(serial_reader):
    reader, port, _ = serial_reader
    with pytest.raises(px.PX6DTimeout):
        reader.read_wrench()
    port.on_write = lambda command: port.rx.extend(packet())
    with pytest.raises(px.PX6DError, match='resynchronization failed'):
        reader.resynchronize_after_timeout()
    with pytest.raises(px.PX6DError, match='requires explicit'):
        reader.read_wrench()
    assert len(port.writes) == 2


def test_short_write_fails_without_waiting_for_a_response(serial_reader):
    reader, port, _ = serial_reader
    port.short_write = True
    with pytest.raises(px.PX6DError, match='write was incomplete') as failure:
        reader.read_wrench()
    assert failure.value.sensor_diagnostics['requests'][-1]['tx_bytes'] == 5
    assert failure.value.sensor_diagnostics['requests'][-1]['read_iterations'] == 0


def test_request_history_is_bounded_and_snapshot_detached(serial_reader):
    reader, port, _ = serial_reader
    port.on_write = lambda command: port.rx.extend(packet())
    for _ in range(40):
        reader.read_wrench()
    snapshot = reader.diagnostics_snapshot()
    assert len(snapshot['requests']) == 32
    assert snapshot['requests'][0]['request_id'] == 9
    snapshot['requests'][-1]['rx_bytes'] = 999
    assert reader.diagnostics_snapshot()['requests'][-1]['rx_bytes'] == 29


def test_duplicate_connect_does_not_open_or_close_another_port(serial_reader):
    reader, port, _ = serial_reader
    with pytest.raises(px.PX6DError, match='already connected'):
        reader.connect()
    assert reader._port is port and not port.closed


@pytest.mark.parametrize('respond', [True, False])
def test_read_only_cli_writes_diagnostics_after_close_without_retries(serial_reader, monkeypatch, tmp_path, capsys, respond):
    import json
    import sys
    from tools import check_px6d
    reader, port, _ = serial_reader
    monkeypatch.setattr(reader, 'connect', lambda: None)
    monkeypatch.setattr(check_px6d, 'PX6DReader', lambda *args: reader)
    output = tmp_path/'sensor.json'
    monkeypatch.setattr(sys, 'argv', ['check_px6d', '--samples', '2', '--diagnostics-output', str(output)])
    if respond:
        corrupt = packet()[:-1]+bytes((packet()[-1]^1,))
        port.on_write = lambda command: port.rx.extend(corrupt+packet())
    assert check_px6d.main() == (0 if respond else 1)
    assert port.closed
    result = json.loads(output.read_text())
    assert result['samples_completed'] == (2 if respond else 0)
    assert len(port.writes) == (2 if respond else 1)
    if respond:
        assert 'crc_errors=2' in capsys.readouterr().out
    else:
        assert 'PX6DTimeout' in result['error']
        assert result['sensor_diagnostics']['requests'][-1]['rx_bytes'] == 0

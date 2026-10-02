"""Comparison orchestration uses local PTYs/fakes; never physical devices."""
import json
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import pytest

from tools import px6d_comparison as tool

VERSION_COMMAND = bytes.fromhex('aa557f0701d0')
FRAME_COMMAND = bytes.fromhex('aa557f0501fa')
VERSION_REPLY = bytes.fromhex('aa557f0776312e302e31000019')
FRAME_REPLY = bytes.fromhex('aa557f030000803f0000004000004040000080400000a0400000c040f2')


@pytest.fixture
def peer():
    master, slave = pty.openpty()
    stop = threading.Event()
    state = SimpleNamespace(requests=[], errors=[], delay=0., fail_at=None,
                            sensor=dict(serial_port=os.ttyname(slave), baudrate=921600,
                                        timeout_sec=.05, poll_rate_hz=100., startup_delay_sec=0.))

    def serve():
        buffer = bytearray()
        count = 0
        try:
            while not stop.is_set():
                if not select.select([master], [], [], .01)[0]:
                    continue
                buffer.extend(os.read(master, 4096))
                while len(buffer) >= 6:
                    command = bytes(buffer[:6]); del buffer[:6]
                    state.requests.append(command)
                    if command == VERSION_COMMAND:
                        count = 0
                        response = VERSION_REPLY
                    else:
                        assert command == FRAME_COMMAND
                        count += 1
                        if count == state.fail_at:
                            continue
                        response = FRAME_REPLY
                        time.sleep(state.delay)
                    os.write(master, response[:5])
                    os.write(master, response[5:])
        except BaseException as exc:
            state.errors.append(exc)

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    try:
        yield state
    finally:
        stop.set()
        thread.join(2.)
        os.close(master); os.close(slave)
        assert not state.errors


@pytest.mark.parametrize('backend', ['reader', 'reference'])
@pytest.mark.parametrize('fail_at', [None, 3])
def test_fresh_worker_raw_bytes_and_independent_imports(peer, tmp_path, backend, fail_at):
    peer.fail_at = fail_at
    tool.save_json(tmp_path/'request.json', dict(backend=backend, sensor=peer.sensor, samples=5, duration_sec=1.))
    result = subprocess.run([sys.executable, '-m', 'tools.px6d_comparison', '--worker',
                             str(tmp_path/'request.json')], cwd=tool.ROOT, capture_output=True, timeout=6.)
    assert result.returncode == int(fail_at is not None), result.stderr
    summary = json.loads((tmp_path/'result.json').read_text())
    assert summary['samples_completed'] == (5 if fail_at is None else 2)
    assert summary['imported_production_reader'] == (backend == 'reader')
    assert summary['imported_robot_modules'] == []
    events = [json.loads(line) for line in (tmp_path/'wire.jsonl').read_text().splitlines()]
    assert events[0]['complete']
    assert [bytes.fromhex(e['data_hex']) for e in events if e['kind'] == 'tx'] == peer.requests
    rx = b''.join(bytes.fromhex(e['data_hex']) for e in events if e['kind'] == 'rx' and e.get('data_hex'))
    assert rx == VERSION_REPLY+FRAME_REPLY*summary['samples_completed']
    assert all('host_monotonic' in e for e in events[1:])
    assert events[-1]['kind'] == 'close'
    if fail_at is not None:
        assert len(peer.requests) == 4  # Version, two good force samples, one failure; no retry.
        assert summary['passive_after_failure']['tx_bytes'] == 0


def test_reader_trace_late_bytes_are_raw_evidence_only(peer, tmp_path, monkeypatch):
    import serial
    import serial.serialposix
    def factory(device, **kwargs):
        assert device == peer.sensor['serial_port'] and device.startswith('/dev/pts/')
        return serial.serialposix.Serial(device, **kwargs)
    monkeypatch.setattr(serial, 'Serial', factory)
    peer.delay = .075
    capture = tool.WireCapture()
    result = tool.run_reader(peer.sensor, 3, 1., capture)
    capture.finish(tmp_path/'wire.jsonl')
    assert result['samples_completed'] == 0
    assert result['passive_after_failure']['rx_bytes'] == 29
    assert peer.requests == [VERSION_COMMAND, FRAME_COMMAND]
    assert b''.join(bytes.fromhex(e['data_hex']) for e in capture.events
                    if e['kind'] == 'rx' and e.get('passive')) == FRAME_REPLY


def test_wire_capture_bounds_prefix_and_tail_and_preserves_explicit_time(tmp_path):
    capture = tool.WireCapture(max_events=2, max_bytes=4)
    capture.record('tx', b'12', host_monotonic=123.)
    for _ in range(100):
        capture.record('rx', b'34')
    metadata = capture.finish(tmp_path/'wire.jsonl')
    assert capture.events[0]['host_monotonic'] == 123.
    assert len(capture.events) == 2 and len(capture.tail) == 64
    assert metadata['dropped_events'] == 99 and not metadata['complete']


def test_initialization_failure_time_excludes_passive_observation(monkeypatch):
    import serial
    from sensor import px6d_reader as px
    from tools import check_px6d
    from test_px6d_reader import Clock, Port
    clock = Clock()
    timer = SimpleNamespace(monotonic=clock.monotonic, sleep=clock.sleep)
    for module in (tool, px, check_px6d):
        monkeypatch.setattr(module, 'time', timer)
    port = Port(clock)
    port.reset_output_buffer = lambda: None
    monkeypatch.setattr(serial, 'Serial', lambda *a, **k: port)
    sensor = dict(serial_port='offline-fake', baudrate=921600, poll_rate_hz=100., timeout_sec=.05, startup_delay_sec=0.)
    result = tool.run_reader(sensor, 3, 1., tool.WireCapture())
    failure = result['first_failure']
    assert failure['host_monotonic'] == result['requests'][-1]['request_end_host_monotonic']
    assert failure['caught_host_monotonic']-failure['host_monotonic'] == pytest.approx(1., abs=.002)
    assert result['samples_completed'] == 0 and port.closed


def test_repeated_interrupt_still_kills_and_reaps_worker():
    class Process:
        killed = False
        waited = 0
        returncode = None
        def poll(self):
            return self.returncode
        def send_signal(self, _):
            pass
        def wait(self, timeout):
            self.waited += 1
            if not self.killed:
                raise KeyboardInterrupt
            self.returncode = -9
        def kill(self):
            self.killed = True
    process = Process()
    errors = tool.stop_worker(process)
    assert process.killed and process.returncode == -9
    assert process.waited == 2 and 'KeyboardInterrupt' in errors[0]


def test_invalid_low_frequency_never_confirms_or_opens(monkeypatch, tmp_path):
    from app import operator_input
    monkeypatch.setattr(operator_input, 'confirm_enter', lambda _: pytest.fail('should reject before prompting'))
    args = SimpleNamespace(comparison_dir=tmp_path, poll_rate_hz=10., samples=3,
                           duration=1., repeats=2, campaign=True, system_only=False)
    sensor = dict(serial_port='never-open', baudrate=921600, poll_rate_hz=100., timeout_sec=.05, startup_delay_sec=2.)
    with pytest.raises(ValueError, match='预算包含节拍'):
        tool.comparison_main(args, sensor)
    assert sensor['poll_rate_hz'] == 100.


def test_campaign_one_fresh_physical_confirmation_per_round_and_save_before_next(monkeypatch, tmp_path):
    from app import operator_input
    prompts, actions = [], []
    args = SimpleNamespace(comparison_dir=tmp_path, poll_rate_hz=None, samples=3,
                           duration=1., repeats=2, campaign=True, system_only=False)
    sensor = dict(serial_port='never-open', baudrate=921600, poll_rate_hz=100.,
                  timeout_sec=.05, startup_delay_sec=2.)

    def confirm(prompt):
        if actions:
            assert (tmp_path/'comparison.json').exists()
            assert json.loads((tmp_path/'comparison.json').read_text())['rounds'][-1]['wire_complete']
        prompts.append(prompt)
        return True

    def collect(path, config, backend, samples, duration, context):
        assert config == sensor
        if context['group'] == 'untouched':
            assert len(prompts) == 1  # Software baseline first, no physical changes.
        else:
            assert len(prompts) == sum(a[0] != 'untouched' for a in actions)+2
        path.mkdir()
        actions.append((context['group'], context['repeat'], backend))
        value = dict(backend=backend, context=context, conditions=tool.conditions(config, samples, duration),
                     error=None, samples_completed=3, stop_reason='samples', wire_capture={'complete': True})
        tool.save_json(path/'result.json', value)
        return value

    monkeypatch.setattr(operator_input, 'confirm_enter', confirm)
    monkeypatch.setattr(tool, 'run_one', collect)
    assert tool.comparison_main(args, sensor) == 0
    assert len(actions) == 16 and len(prompts) == 13
    assert [a[0] for a in actions] == ['untouched']*4+['original_reconnected']*4+['known_good_same_port']*4+['original_returned']*4
    assert [a[2] for a in actions[:4]] == ['reader', 'reference', 'reference', 'reader']
    assert all('5 秒' in prompt for prompt in prompts[1:])


def test_canceled_confirmation_never_opens_sensor(monkeypatch, tmp_path):
    from app import operator_input
    monkeypatch.setattr(operator_input, 'confirm_enter', lambda _: False)
    monkeypatch.setattr(tool, 'run_one', lambda *a: pytest.fail('unexpected serial attempt'))
    args = SimpleNamespace(comparison_dir=tmp_path, poll_rate_hz=None, samples=3,
                           duration=1., repeats=2, campaign=True, system_only=False)
    sensor = dict(serial_port='never-open', baudrate=921600, poll_rate_hz=100., timeout_sec=.05, startup_delay_sec=2.)
    assert tool.comparison_main(args, sensor) == 130


def test_busy_or_uninspectable_serial_does_not_launch_worker(monkeypatch, tmp_path):
    from tools import px6d_usb_evidence
    class Evidence:
        def __init__(self, device, directory, **kwargs):
            self.path = Path(directory)
        def start(self):
            tool.save_json(self.path/'system_before.json', dict(device_path_exists=True,
                ownership={'available': True, 'process_ids': [123]}))
            return self
        def stop(self):
            return {'usbmon': {'available': False, 'reason': 'test unavailable'}}
    monkeypatch.setattr(px6d_usb_evidence, 'SystemEvidence', Evidence)
    monkeypatch.setattr(subprocess, 'Popen', lambda *a, **k: pytest.fail('busy serial must not launch'))
    sensor = dict(serial_port='never-open', baudrate=921600, poll_rate_hz=100., timeout_sec=.05, startup_delay_sec=2.)
    result = tool.run_one(tmp_path/'round', sensor, 'reference', 3, 1., {})
    assert result['stop_reason'] == 'not_started'
    assert 'occupied' in result['error']
    with pytest.raises(FileExistsError):
        tool.run_one(tmp_path/'round', sensor, 'reference', 3, 1., {})


def test_interrupt_while_starting_evidence_still_stops_collector(monkeypatch, tmp_path):
    from tools import px6d_usb_evidence
    calls = []
    class Evidence:
        def __init__(self, *args, **kwargs):
            pass
        def start(self):
            calls.append('start')
            raise KeyboardInterrupt
        def stop(self):
            calls.append('stop')
            return {'usbmon': {'available': False, 'reason': 'interrupted'}}
    monkeypatch.setattr(px6d_usb_evidence, 'SystemEvidence', Evidence)
    monkeypatch.setattr(subprocess, 'Popen', lambda *a, **k: pytest.fail('must not start serial worker'))
    sensor = dict(serial_port='never-open', baudrate=921600, poll_rate_hz=100., timeout_sec=.05, startup_delay_sec=2.)
    result = tool.run_one(tmp_path/'round', sensor, 'reference', 3, 1., {})
    assert calls == ['start', 'stop'] and result['stop_reason'] == 'not_started'
    assert 'KeyboardInterrupt' in result['error']


def test_usb_correlation_pairs_reused_urb_ids_and_never_assumes_clock_alignment(tmp_path):
    capture = tool.WireCapture()
    capture.record('tx', VERSION_COMMAND, request_id=1)
    capture.record('tx', FRAME_COMMAND, request_id=2)
    capture.record('close')
    capture.finish(tmp_path/'wire.jsonl')
    (tmp_path/'usbmon.log').write_text(
        'a 100 S Bo:1:12:2 -115 6 = aa557f0701d0\n'
        'a 101 C Bo:1:12:2 0 6 >\n'
        'b 103 C Bi:1:12:1 0 13 = aa557f0776312e302e31000019\n'
        'a 110 S Bo:1:12:2 -115 6 = aa557f0501fa\n'
        'a 111 C Bo:1:12:2 0 6 >\n')
    system = dict(usbmon={'available': True}, status='finished', stop_reason='stop_requested',
                  sources_ready_host_monotonic=capture.started['host_monotonic'],
                  capture_finished_host_monotonic=time.monotonic())
    result = tool.correlate_usb(tmp_path, {'system_capture': system})
    assert result['available'] and len(result['requests']) == 2
    assert result['requests'][0]['usb_in_completed_bytes'] == 13
    assert result['requests'][1]['usb_out_completed']
    assert result['requests'][1]['usb_in_completed_bytes'] == 0
    (tmp_path/'usbmon.log').write_text('a 110 S Bo:1:12:2 -115 6 = aa55\n')
    assert not tool.correlate_usb(tmp_path, {'system_capture': system})['available']


@pytest.mark.parametrize('bad_field', [dict(stop_reason='output_limit'), dict(stop_reason='duration_limit'),
    dict(parse_errors=1), dict(incomplete_line_bytes=23), dict(worker_alive=True),
    dict(capture_finished_host_monotonic=0.), dict(sources_ready_host_monotonic=float('inf')),
    dict(usbmon={'available': True, 'stream_eof': True})])
def test_incomplete_usb_capture_never_becomes_negative_device_evidence(tmp_path, bad_field):
    capture = tool.WireCapture()
    capture.record('tx', FRAME_COMMAND, request_id=1)
    capture.record('close')
    capture.finish(tmp_path/'wire.jsonl')
    (tmp_path/'usbmon.log').write_text('a 110 S Bo:1:12:2 -115 6 = aa557f0501fa\n'
                                     'a 111 C Bo:1:12:2 0 6 >\n')
    system = dict(usbmon={'available': True}, status='finished', stop_reason='stop_requested',
                  sources_ready_host_monotonic=capture.started['host_monotonic'],
                  capture_finished_host_monotonic=time.monotonic())
    system.update(bad_field)
    result = tool.correlate_usb(tmp_path, {'system_capture': system})
    assert not result['available'] and result['requests'] == []


def test_comparison_refuses_cable_attribution_without_same_starting_state(tmp_path):
    for group in ('original_reconnected', 'known_good_same_port', 'original_returned'):
        for repeat in (1, 2):
            for backend in ('reader', 'reference'):
                directory = tmp_path/f'round_{group}_{repeat}_{backend}'
                directory.mkdir()
                tool.save_json(directory/'result.json', dict(backend=backend,
                    context={'group': group, 'power_cycle': 'not_performed'},
                    conditions={'poll_rate_hz': 100}, samples_completed=10,
                    error=None if group == 'known_good_same_port' else 'timeout',
                    stop_reason='error' if group != 'known_good_same_port' else 'samples',
                    wire_capture={'complete': True}))
                tool.save_json(directory/'system_before.json', {'usb_device': {
                    'topology': '1-11', 'attributes': {'serial': {'value': 'same'}}}})
    report = tool.compare(tmp_path)
    assert all(not row['comparable'] for row in report['comparability'])
    assert not any('证据支持线缆' in text for text in report['conclusions'])


def test_cable_evidence_requires_ordered_same_code_aba_excluding_untouched(tmp_path):
    groups = ('untouched', 'original_reconnected', 'known_good_same_port', 'original_returned')
    index = 0
    for group in groups:
        for repeat in (1, 2):
            for backend in ('reader', 'reference'):
                index += 1
                directory = tmp_path/f'round_{group}_{repeat}_{backend}'
                directory.mkdir()
                tool.save_json(directory/'request.json', dict(source_sha256={'reader': 'same-source'},
                    started_utc=f'2026-10-03T10:{index:02d}:00+00:00'))
                tool.save_json(directory/'result.json', dict(backend=backend,
                    context={'group': group, 'power_cycle': 'not_performed' if group == 'untouched' else 'operator_confirmed'},
                    conditions={'poll_rate_hz': 100}, samples_completed=10,
                    error=None if group == 'known_good_same_port' else 'timeout',
                    stop_reason='error' if group != 'known_good_same_port' else 'samples',
                    finished_utc=f'2026-10-03T10:{index:02d}:30+00:00', wire_capture={'complete': True}))
                tool.save_json(directory/'system_before.json', {'usb_device': {
                    'topology': '1-11', 'attributes': {'serial': {'value': 'same'}}}})
    report = tool.compare(tmp_path)
    assert any('证据支持线缆' in text for text in report['conclusions'])
    changed = tmp_path/'round_known_good_same_port_1_reader/request.json'
    spec = json.loads(changed.read_text())
    spec['source_sha256']['reader'] = 'changed-source'
    tool.save_json(changed, spec)
    report = tool.compare(tmp_path)
    assert not any('证据支持线缆' in text for text in report['conclusions'])
    assert any('source hashes' in ' '.join(row['reasons']) for row in report['comparability'])

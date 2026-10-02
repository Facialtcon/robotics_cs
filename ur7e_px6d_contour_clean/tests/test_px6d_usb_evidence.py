"""USB evidence tests use temporary files and local pipes, never hardware."""
import json
import os
from pathlib import Path
import subprocess
import threading
import time
from types import SimpleNamespace

import pytest

from tools.px6d_usb_evidence import (SystemEvidence, _URBSummary, _journal_access_probe,
                                     parse_usbmon_line, snapshot_usb)


OUT_SUBMIT = 'aaaa 123456 S Bo:1:003:2 -115 6 = 09050000 abcd\n'
OUT_COMPLETE = 'aaaa 123460 C Bo:1:003:2 0 6 >\n'
IN_SUBMIT = 'bbbb 123450 S Bi:1:003:1 -115 29 <\n'
IN_COMPLETE = 'bbbb 123470 C Bi:1:003:1 0 3 = 010203\n'
OTHER_DEVICE = 'cccc 123471 C Bi:1:004:1 0 3 = ffffff\n'


@pytest.fixture(autouse=True)
def no_real_journal_probe(monkeypatch):
    # Every collector test uses a fake bounded command; tests never read the
    # actual machine's journal, sysfs, process table, or a device.
    monkeypatch.setattr(subprocess, 'run', lambda *a, **k:
        SimpleNamespace(returncode=0, stdout='', stderr=''))


def fake_sysfs(tmp_path):
    root = tmp_path/'sys'
    usb = root/'devices/pci0000:00/usb1/1-2'
    tty = usb/'1-2:1.0/tty/ttyACM0'
    tty.mkdir(parents=True)
    klass = root/'class/tty/ttyACM0'
    klass.mkdir(parents=True)
    (klass/'device').symlink_to(tty)
    driver = root/'bus/usb/drivers/cdc_acm'
    driver.mkdir(parents=True)
    (usb/'1-2:1.0/driver').symlink_to(driver)
    for name, value in dict(idVendor='1234', idProduct='abcd', busnum='1', devnum='3',
                            devpath='2', speed='12', product='test PX6D').items():
        (usb/name).write_text(value+'\n')
    (usb/'power').mkdir()
    (usb/'power/control').write_text('auto\n')
    (usb/'power/runtime_status').write_text('active\n')
    device = tmp_path/'dev/ttyACM0'
    device.parent.mkdir()
    device.touch()
    alias = device.parent/'px6d'
    alias.symlink_to(device)
    return root, usb, device, alias


def test_snapshot_resolves_tty_usb_parent_and_owner_without_opening_tty(tmp_path):
    root, usb, device, alias = fake_sysfs(tmp_path)
    calls = []
    def run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout='123 456', stderr=str(device)+':\n')
    result = snapshot_usb(alias, sysfs_root=root, command_runner=run)
    assert result['resolved_device'] == str(device)
    assert result['device_path_exists'] is True
    assert result['usb_device']['sysfs_path'] == str(usb)
    assert result['usb_device']['busnum'] == 1
    assert result['usb_device']['devnum'] == 3
    assert result['usb_device']['power']['control']['value'] == 'auto'
    assert result['usb_device']['attributes']['serial']['available'] is False
    assert result['ancestor_drivers'][0]['target'].endswith('/cdc_acm')
    assert result['ownership']['available'] is True
    assert result['ownership']['process_ids'] == [123, 456]
    assert calls == [(['fuser', str(device)],
                      dict(capture_output=True, text=True, timeout=2., check=False))]
    assert result['snapshot_elapsed_sec'] >= 0


@pytest.mark.parametrize('error', [FileNotFoundError('fuser absent'),
                                  PermissionError('ownership denied'),
                                  subprocess.TimeoutExpired('fuser', 2.)])
def test_snapshot_missing_sysfs_and_fuser_errors_are_unknown(tmp_path, error):
    def run(*args, **kwargs):
        raise error
    result = snapshot_usb(tmp_path/'missing', sysfs_root=tmp_path, command_runner=run)
    assert result['tty_sysfs_device']['available'] is False
    assert result['usb_device']['available'] is False
    assert result['ownership']['available'] is False
    assert type(error).__name__ in result['ownership']['error']


@pytest.mark.parametrize('code, stderr, expected', [(1, '', True),
    (0, 'Cannot stat /proc/123/fd/4: Permission denied', False), (2, '', False)])
def test_snapshot_ownership_is_conservative(tmp_path, code, stderr, expected):
    device = tmp_path/'ttyACM0'
    device.touch()
    result = snapshot_usb(device, sysfs_root=tmp_path,
        command_runner=lambda *a, **k: SimpleNamespace(returncode=code, stdout='', stderr=stderr))
    assert result['ownership']['available'] is expected
    if expected:
        assert result['ownership']['process_ids'] == []


@pytest.mark.parametrize('code, stdout, stderr, status, available', [
    (0, '{"MESSAGE":"historical kernel message"}\n', '', 'readable', True),
    (0, '', '', 'no_visible_records', False),
    (1, '', 'No journal files were opened due to insufficient permissions.',
     'permission_denied_or_partial_visibility', False),
    (0, '', 'You are currently not seeing messages from other users and the system.',
     'permission_denied_or_partial_visibility', False),
    (1, '', 'journal is corrupt', 'command_failed', False)])
def test_journal_history_probe_has_bounded_command_and_explicit_access_status(
        code, stdout, stderr, status, available):
    calls = []
    def run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=code, stdout=stdout, stderr=stderr)
    result = _journal_access_probe(run)
    assert result['available'] is available
    assert result['status'] == status
    assert result['historical_probe'] is True
    assert calls == [(['journalctl', '-k', '-n', '1', '-o', 'json', '--no-pager'],
                     dict(capture_output=True, text=True, timeout=2., check=False))]


def test_journal_history_probe_timeout_is_unavailable():
    def timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired('journalctl', 2.)
    result = _journal_access_probe(timeout)
    assert result['available'] is False
    assert result['status'] == 'unavailable'
    assert 'TimeoutExpired' in result['reason']


def test_parse_and_pair_bulk_urbs_preserves_status_data_and_timestamp():
    records = [parse_usbmon_line(line) for line in
               (OUT_SUBMIT, IN_SUBMIT, OUT_COMPLETE, IN_COMPLETE)]
    assert records[0]['direction'] == 'out'
    assert records[0]['data_hex'] == '09050000abcd'
    assert records[0]['status'] == -115
    assert records[0]['length'] == 6
    assert records[0]['host_usbmon_timestamp_us'] == 123456
    assert records[3]['direction'] == 'in'
    assert records[3]['data_hex'] == '010203'
    summary = _URBSummary()
    for record in records:
        summary.observe(record)
    result = summary.snapshot()
    assert result['matched_urb_count'] == 2
    assert result['pending_count'] == 0
    assert result['completion_status_counts'] == {'0': 2}
    assert result['last_matched_urbs'][0]['submission']['urb_id'] == 'aaaa'
    assert 'does not prove device application receipt' in result['interpretation']
    assert 'alignment with host monotonic time is not assumed' in result['interpretation']


def test_parse_control_setup_errors_and_unmatched_completion():
    record = parse_usbmon_line('ffff 123 S Co:1:003:0 s 21 22 0000 0000 0000 0')
    assert record['setup_hex_fields'] == ['21', '22', '0000', '0000', '0000']
    assert record['length'] == 0 and record['status'] is None
    summary = _URBSummary()
    summary.observe(parse_usbmon_line('eeee 124 E Bo:1:003:2 -32 0'))
    result = summary.snapshot()
    assert result['completion_status_counts'] == {'-32': 1}
    assert result['last_unmatched_completions'][0]['event'] == 'E'
    assert parse_usbmon_line('broken') is None
    assert parse_usbmon_line('tag bad C Bi:1:003:1 0 0') is None
    assert parse_usbmon_line('tag 12 C unsupported 0 0') is None


def test_pending_and_completed_urb_memory_is_bounded():
    summary = _URBSummary()
    for number in range(4100):
        summary.observe(parse_usbmon_line(f'{number} 123 S Bi:1:003:1 -115 29 <'))
    assert summary.snapshot()['pending_count'] == 4096
    assert summary.snapshot()['evicted_submissions'] == 4
    for number in range(4100):
        summary.observe(parse_usbmon_line(f'{number} 124 C Bi:1:003:1 0 0'))
    result = summary.snapshot()
    assert len(result['last_matched_urbs']) == 64
    assert len(result['last_unmatched_completions']) == 4
    assert result['pending_count'] == 0


class PipeProcess:
    """A subprocess stand-in containing only local, closed-writer pipes."""
    def __init__(self, stdout=b'', stderr=b'', *, hang=False):
        self.stdout = self.pipe(stdout)
        self.stderr = self.pipe(stderr)
        self.hang = hang
        self.killed = self.terminated = False
        self.wait_timeouts = []

    @staticmethod
    def pipe(data):
        read_fd, write_fd = os.pipe()
        os.write(write_fd, data)
        os.close(write_fd)
        return os.fdopen(read_fd, 'rb', buffering=0)

    def poll(self):
        return None if self.hang and not self.killed else 0

    def terminate(self):
        self.terminated = True

    def kill(self):
        self.killed = True

    def wait(self, timeout):
        self.wait_timeouts.append(timeout)
        if self.hang and not self.killed:
            raise subprocess.TimeoutExpired('fake journal', timeout)
        return 0


def snapshot(_device):
    return dict(usb_device=dict(available=True, busnum=1, devnum=3),
                ownership=dict(available=False, error='fake ownership unknown'))


def missing_journal(*args, **kwargs):
    raise FileNotFoundError('fake journal unavailable')


def finish(collector):
    collector.start()
    assert collector._done.wait(2.), 'fake diagnostic worker failed to finish'
    result = collector.stop()
    assert result['worker_alive'] is False
    return result


def test_collector_filters_records_preserves_raw_and_writes_only_in_worker(tmp_path):
    monitor = tmp_path/'mon'
    monitor.mkdir()
    raw = OUT_SUBMIT+IN_SUBMIT+OTHER_DEVICE+OUT_COMPLETE+IN_COMPLETE
    (monitor/'1u').write_text(raw)
    kernel = b'{"MESSAGE":"fake USB event","__MONOTONIC_TIMESTAMP":"123"}\n'
    process = PipeProcess(stdout=kernel)
    calls, snapshots = [], []
    def popen(command, **kwargs):
        calls.append((command, kwargs))
        return process
    def take_snapshot(device):
        snapshots.append(threading.current_thread().name)
        return snapshot(device)
    output = tmp_path/'out'
    collector = SystemEvidence('fake-tty', output, max_duration_sec=.05,
        snapshot_func=take_snapshot, popen_factory=popen, usbmon_root=monitor)
    writes = []
    original_write = collector._write_json
    def write(name, value):
        writes.append(threading.current_thread().name)
        original_write(name, value)
    collector._write_json = write
    result = finish(collector)
    assert result['status'] == 'finished'
    assert result['stop_reason'] == 'duration_limit'
    assert result['kernel']['available'] is True
    assert result['usbmon']['available'] is True
    assert result['usbmon_other_device_lines'] == 1
    assert result['usbmon_lines'] == 4
    assert result['kernel_lines'] == 1
    assert (output/'usbmon.log').read_text() == raw.replace(OTHER_DEVICE, '')
    assert (output/'kernel_events.jsonl').read_bytes() == kernel
    pairs = result['usbmon_summary']['last_matched_urbs']
    assert len(pairs) == 2
    for pair in pairs:
        for record in pair.values():
            assert result['started_host_monotonic'] <= record['capture_host_monotonic_sec']
            assert record['capture_host_monotonic_sec'] <= result['capture_finished_host_monotonic']
    assert set(snapshots+writes) == {'px6d-system-evidence'}
    assert len(snapshots) == 2
    assert set(p.name for p in output.iterdir()) == {'system_before.json', 'system_after.json',
        'system_summary.json', 'kernel_events.jsonl', 'usbmon.log'}
    saved = json.loads((output/'system_summary.json').read_text())
    assert saved['usbmon_summary']['matched_urb_count'] == 2
    assert calls[0][0] == ['journalctl', '-k', '-f', '-n', '0', '-o', 'json', '--no-pager']
    assert process.stdout.closed and process.stderr.closed


def test_unavailable_sources_are_reported_without_claiming_healthy_link(tmp_path):
    result = finish(SystemEvidence('fake-tty', tmp_path/'out', max_duration_sec=.02,
        snapshot_func=snapshot, popen_factory=missing_journal, usbmon_root=tmp_path/'missing'))
    assert result['kernel']['available'] is False
    assert 'FileNotFoundError' in result['kernel']['reason']
    assert result['usbmon']['available'] is False
    assert 'FileNotFoundError' in result['usbmon']['reason']
    assert result['capture_bytes'] == 0


def test_quiet_follow_with_readable_history_is_available_without_fabricated_new_event(tmp_path):
    process = PipeProcess()
    commands = []
    def history(command, **kwargs):
        commands.append(command)
        return SimpleNamespace(returncode=0,
            stdout='{"MESSAGE":"old kernel record","__MONOTONIC_TIMESTAMP":"1"}\n', stderr='')
    output = tmp_path/'out'
    result = finish(SystemEvidence('fake-tty', output, max_duration_sec=.02,
        snapshot_func=snapshot, popen_factory=lambda *a, **k: process,
        command_runner=history, usbmon_root=tmp_path/'missing'))
    assert result['kernel']['available'] is True
    assert result['kernel']['access_probe']['status'] == 'readable'
    assert result['kernel']['follow_status'] == 'no_events_during_capture'
    assert result['kernel']['follow_events_received'] is False
    assert result['kernel_lines'] == 0
    assert (output/'kernel_events.jsonl').read_bytes() == b''
    assert len(commands) == 1


def test_journal_permission_denied_and_usbmon_permission_denied(tmp_path, monkeypatch):
    from tools import px6d_usb_evidence as module
    process = PipeProcess(stderr=b'No journal files were opened due to insufficient permissions.\n')
    original_open = os.open
    monitor = tmp_path/'mon'
    def denied(path, *args, **kwargs):
        if Path(path) == monitor/'1u':
            raise PermissionError('fake usbmon access denied')
        return original_open(path, *args, **kwargs)
    monkeypatch.setattr(module.os, 'open', denied)
    result = finish(SystemEvidence('fake-tty', tmp_path/'out', max_duration_sec=.02,
        snapshot_func=snapshot, popen_factory=lambda *a, **k: process, usbmon_root=monitor))
    assert result['kernel']['available'] is False
    assert 'insufficient permissions' in result['kernel']['reason']
    assert result['usbmon']['available'] is False
    assert 'PermissionError' in result['usbmon']['reason']


def test_capture_byte_budget_stops_with_complete_raw_records(tmp_path):
    monitor = tmp_path/'mon'
    monitor.mkdir()
    (monitor/'1u').write_text(OUT_SUBMIT+OUT_COMPLETE)
    output = tmp_path/'out'
    result = finish(SystemEvidence('fake-tty', output, max_duration_sec=10.,
        max_output_bytes=len(OUT_SUBMIT), snapshot_func=snapshot,
        popen_factory=missing_journal, usbmon_root=monitor))
    assert result['stop_reason'] == 'output_limit'
    assert result['capture_bytes'] == len(OUT_SUBMIT)
    assert (output/'usbmon.log').read_text() == OUT_SUBMIT
    assert result['usbmon_summary']['pending_count'] == 1


def test_stop_is_bounded_and_kills_unresponsive_journal(tmp_path):
    process = PipeProcess(hang=True)
    collector = SystemEvidence('fake-tty', tmp_path/'out', max_duration_sec=10.,
        snapshot_func=snapshot, popen_factory=lambda *a, **k: process,
        usbmon_root=tmp_path/'missing')
    collector.start()
    started = time.monotonic()
    result = collector.stop()
    assert time.monotonic()-started < 1.
    assert result['worker_alive'] is False
    assert result['stop_reason'] == 'stop_requested'
    assert process.terminated and process.killed
    assert process.wait_timeouts == [.5, .5]
    assert collector.stop() == result


def test_after_snapshot_failure_still_writes_summary(tmp_path):
    calls = []
    def failing_after(device):
        calls.append(device)
        if len(calls) == 2:
            raise PermissionError('fake sysfs revoked')
        return snapshot(device)
    output = tmp_path/'out'
    result = finish(SystemEvidence('fake-tty', output, max_duration_sec=.02,
        snapshot_func=failing_after, popen_factory=missing_journal, usbmon_root=tmp_path/'missing'))
    assert result['status'] == 'failed'
    assert 'fake sysfs revoked' in result['errors'][0]
    assert json.loads((output/'system_summary.json').read_text())['status'] == 'failed'


def test_usbmon_read_failure_is_unavailable_even_after_open(tmp_path, monkeypatch):
    from tools import px6d_usb_evidence as module
    monitor = tmp_path/'mon'
    monitor.mkdir()
    (monitor/'1u').write_text(OUT_SUBMIT)
    def denied(fd, count):
        raise PermissionError('fake permission revoked during capture')
    monkeypatch.setattr(module.os, 'read', denied)
    result = finish(SystemEvidence('fake-tty', tmp_path/'out', max_duration_sec=.02,
        snapshot_func=snapshot, popen_factory=missing_journal, usbmon_root=monitor))
    assert result['usbmon']['available'] is False
    assert 'permission revoked' in result['usbmon']['reason']
    assert result['capture_bytes'] == 0


def test_pipe_cleanup_failure_does_not_skip_remaining_cleanup_or_summary(tmp_path):
    process = PipeProcess()
    original = process.stdout
    class FailedClose:
        def fileno(self):
            return original.fileno()
        def close(self):
            original.close()
            raise OSError('fake close failure')
    process.stdout = FailedClose()
    output = tmp_path/'out'
    result = finish(SystemEvidence('fake-tty', output, max_duration_sec=.02,
        snapshot_func=snapshot, popen_factory=lambda *a, **k: process, usbmon_root=tmp_path/'missing'))
    assert original.closed and process.stderr.closed
    assert any('fake close failure' in error for error in result['errors'])
    assert (output/'system_after.json').exists()
    assert (output/'system_summary.json').exists()


def test_stalled_worker_stop_reports_timeout_without_blocking(tmp_path):
    entered, release = threading.Event(), threading.Event()
    def blocked_snapshot(device):
        entered.set()
        assert release.wait(2.)
        return snapshot(device)
    collector = SystemEvidence('fake-tty', tmp_path/'out', stop_timeout_sec=.02,
        snapshot_func=blocked_snapshot, popen_factory=missing_journal, usbmon_root=tmp_path/'missing')
    try:
        collector.start()
        assert entered.wait(.2)
        started = time.monotonic()
        result = collector.stop()
        assert time.monotonic()-started < .2
        assert result['status'] == 'stop_timeout'
        assert result['worker_alive'] is True
    finally:
        release.set()
        assert collector._done.wait(2.)
    assert collector.stop()['worker_alive'] is False


@pytest.mark.parametrize('kwargs', [dict(max_duration_sec=0), dict(max_duration_sec=float('inf')),
    dict(max_output_bytes=0), dict(max_output_bytes=.5), dict(stop_timeout_sec=0)])
def test_invalid_budgets_are_rejected(tmp_path, kwargs):
    with pytest.raises(ValueError):
        SystemEvidence('fake-tty', tmp_path, **kwargs)

"""Bounded host-side USB evidence for the standalone PX6D diagnostic tool.

Never opens the sensor tty, loads modules, changes USB settings, or uses sudo.
USB URB completion proves a host USB transaction, not firmware application
receipt or a fresh force measurement. Missing capture permission is unavailable
evidence, never a healthy-link result.
"""
from __future__ import annotations

from collections import Counter, deque
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import select
import subprocess
import threading
import time


def _text(path, limit=4096):
    try:
        with Path(path).open('rb') as stream:
            raw = stream.read(limit + 1)
        return dict(available=True, value=raw[:limit].decode('utf-8', 'replace').strip(),
                    truncated=len(raw) > limit)
    except OSError as exc:
        return dict(available=False, error=f'{type(exc).__name__}: {exc}')


def _link(path):
    try:
        return dict(available=True, target=str(Path(path).resolve(strict=True)))
    except OSError as exc:
        return dict(available=False, error=f'{type(exc).__name__}: {exc}')


def snapshot_usb(device, *, sysfs_root='/sys', command_runner=subprocess.run):
    """Read current tty topology/sysfs and bounded fuser evidence; no tty open."""
    started = time.monotonic()
    resolved = Path(os.path.realpath(os.fspath(device)))
    result = dict(host_monotonic_sec=started, timestamp_utc=datetime.now(timezone.utc).isoformat(),
                  device=str(device), resolved_device=str(resolved), device_path_exists=resolved.exists(),
                  usb_device=dict(available=False, reason='USB parent not found'), ancestor_drivers=[])
    tty_path = Path(sysfs_root)/'class/tty'/resolved.name/'device'
    result['tty_sysfs_device'] = _link(tty_path)
    if result['tty_sysfs_device']['available']:
        tty = Path(result['tty_sysfs_device']['target'])
        for node in (tty, *tty.parents):
            if (node/'driver').is_symlink():
                result['ancestor_drivers'].append(dict(node=str(node), **_link(node/'driver')))
            if not (node/'idVendor').exists():
                continue
            names = ('idVendor', 'idProduct', 'busnum', 'devnum', 'devpath', 'speed',
                     'bcdDevice', 'bConfigurationValue', 'authorized', 'manufacturer',
                     'product', 'serial', 'uevent')
            attributes = {name: _text(node/name) for name in names}
            usb = dict(available=True, sysfs_path=str(node), topology=node.name,
                       attributes=attributes, driver=_link(node/'driver'),
                       power={name: _text(node/'power'/name) for name in
                              ('control', 'runtime_status', 'autosuspend_delay_ms',
                               'runtime_active_time', 'runtime_suspended_time', 'wakeup')})
            for name in ('busnum', 'devnum'):
                try:
                    usb[name] = int(attributes[name]['value'])
                except (KeyError, ValueError):
                    usb[name] = None
            result['usb_device'] = usb
            break
    # fuser inspects ownership; it does not open the tty or claim exclusivity.
    # Some installed fuser variants reject "--". realpath is absolute, so it
    # cannot be interpreted as an option without an option terminator.
    command = ['fuser', str(resolved)]
    ownership = dict(command=command, timeout_sec=2., available=False)
    try:
        completed = command_runner(command, capture_output=True, text=True, timeout=2., check=False)
        stdout, stderr = completed.stdout or '', completed.stderr or ''
        ownership.update(returncode=completed.returncode, stdout=stdout[:4096], stderr=stderr[:4096],
                         truncated=len(stdout) > 4096 or len(stderr) > 4096)
        # fuser prints its ordinary path label on stderr. Other diagnostics may
        # indicate partial visibility (for example /proc permission errors).
        unexpected_stderr = stderr.replace(str(resolved)+':', '').strip()
        ownership['visibility'] = 'processes visible to the current user; not an exclusive device lock'
        if not unexpected_stderr and (completed.returncode == 0 or (
                completed.returncode == 1 and result['device_path_exists'])):
            ownership.update(available=True, process_ids=[int(x) for x in stdout.split() if x.isdigit()])
        else:
            ownership['error'] = 'fuser failed or device is absent; ownership is unknown'
    except (OSError, subprocess.SubprocessError) as exc:
        ownership['error'] = f'{type(exc).__name__}: {exc}'
    result['ownership'] = ownership
    result['snapshot_elapsed_sec'] = time.monotonic()-started
    return result


def _journal_access_probe(command_runner):
    """One bounded historical read distinguishes quiet follow from no access."""
    command = ['journalctl', '-k', '-n', '1', '-o', 'json', '--no-pager']
    started = time.monotonic()
    result = dict(command=command, timeout_sec=2., available=False,
                  host_monotonic_sec=started, historical_probe=True)
    try:
        completed = command_runner(command, capture_output=True, text=True, timeout=2., check=False)
        stdout, stderr = completed.stdout or '', completed.stderr or ''
        result.update(returncode=completed.returncode, stdout=stdout[:16384], stderr=stderr[:4096],
                      truncated=len(stdout) > 16384 or len(stderr) > 4096)
        records = []
        for line in stdout.splitlines():
            try:
                record = json.loads(line)
            except ValueError:
                continue
            if isinstance(record, dict):
                records.append(record)
        permission_warning = any(word in stderr.lower() for word in
                                 ('permission', 'not seeing messages', 'access denied'))
        if completed.returncode == 0 and records and not permission_warning:
            result.update(available=True, status='readable', reason=None)
        elif permission_warning:
            result.update(status='permission_denied_or_partial_visibility', reason=stderr[:4096])
        elif completed.returncode == 0:
            result.update(status='no_visible_records', reason='no historical kernel record visible; access is unconfirmed')
        else:
            result.update(status='command_failed', reason=stderr[:4096] or f'journalctl exit {completed.returncode}')
    except (OSError, subprocess.SubprocessError) as exc:
        result.update(status='unavailable', reason=f'{type(exc).__name__}: {exc}')
    result['elapsed_sec'] = time.monotonic()-started
    return result


_PIPE = re.compile(r'^([BCIZ])([io]):(\d+):(\d+):(\d+)$')


def parse_usbmon_line(line):
    """Parse text usbmon bulk/control records; retain URB identity and direction.

    Isochronous descriptor layouts are deliberately not interpreted. PX6D CDC
    data uses bulk endpoints; unsupported records remain identifiable by pipe.
    """
    fields = line.split()
    if len(fields) < 5 or fields[2] not in ('S', 'C', 'E'):
        return None
    address = _PIPE.match(fields[3])
    if address is None:
        return None
    try:
        stamp = int(fields[1])
    except ValueError:
        return None
    kind, direction, bus, device, endpoint = address.groups()
    result = dict(urb_id=fields[0], host_usbmon_timestamp_us=stamp, event=fields[2],
                  transfer_type={'B': 'bulk', 'C': 'control', 'I': 'interrupt', 'Z': 'isochronous'}[kind],
                  direction='in' if direction == 'i' else 'out', busnum=int(bus),
                  devnum=int(device), endpoint=int(endpoint), status_raw=fields[4],
                  status=None, length=None, data_hex=None)
    try:
        result['status'] = int(fields[4].split(':')[0])
    except ValueError:
        pass
    length_index = 5
    if fields[4] == 's':
        result['setup_hex_fields'] = fields[5:10]
        length_index = 10
    if kind != 'Z' and len(fields) > length_index:
        try:
            result['length'] = int(fields[length_index])
        except ValueError:
            pass
    if '=' in fields:
        data = ''.join(fields[fields.index('=')+1:])
        if re.fullmatch(r'(?:[0-9a-fA-F]{2})*', data):
            result['data_hex'] = data.lower()
    return result


class _URBSummary:
    def __init__(self):
        self.pending = {}
        self.events = Counter()
        self.statuses = Counter()
        self.pairs = deque(maxlen=64)
        self.unmatched = deque(maxlen=64)
        self.matched_count = self.evicted_submissions = 0

    def observe(self, record):
        self.events[record['event']] += 1
        key = record['urb_id']
        if record['event'] == 'S':
            if len(self.pending) >= 4096 and key not in self.pending:
                self.pending.pop(next(iter(self.pending)))
                self.evicted_submissions += 1
            self.pending[key] = record
            return
        self.statuses[str(record['status_raw'])] += 1
        submission = self.pending.pop(key, None)
        if submission is None:
            self.unmatched.append(record)
        else:
            self.matched_count += 1
            self.pairs.append(dict(submission=submission, completion=record))

    def snapshot(self):
        return dict(event_counts=dict(self.events), completion_status_counts=dict(self.statuses),
                    matched_urb_count=self.matched_count, pending_count=len(self.pending),
                    pending_tail=list(self.pending.values())[-64:], last_matched_urbs=list(self.pairs),
                    last_unmatched_completions=list(self.unmatched), evicted_submissions=self.evicted_submissions,
                    interpretation='S=host submission; C=host completion; E=submission error. Status 0 does not prove device application receipt or a force reply. USBmon timestamps are preserved as reported; alignment with host monotonic time is not assumed. capture_host_monotonic_sec is collector receipt time, not sensor sample time.')


class SystemEvidence:
    """Bounded background collector; file writes occur only in its worker.

    start waits at most three seconds for capture setup. stop joins at most five
    seconds by default and reports incomplete cleanup rather than waiting forever.
    """
    def __init__(self, device, output_dir, *, max_duration_sec=180., max_output_bytes=8*1024*1024,
                 stop_timeout_sec=5., snapshot_func=snapshot_usb, popen_factory=subprocess.Popen,
                 usbmon_root='/sys/kernel/debug/usb/usbmon', command_runner=None):
        if not math.isfinite(max_duration_sec) or max_duration_sec <= 0 or int(max_output_bytes) <= 0:
            raise ValueError('evidence duration and output byte budgets must be positive')
        if not math.isfinite(stop_timeout_sec) or stop_timeout_sec <= 0:
            raise ValueError('stop timeout must be positive')
        self.device, self.output_dir = str(device), Path(output_dir)
        self.max_duration_sec, self.max_output_bytes = float(max_duration_sec), int(max_output_bytes)
        self.stop_timeout_sec = float(stop_timeout_sec)
        self._snapshot, self._popen, self._usbmon_root = snapshot_func, popen_factory, Path(usbmon_root)
        self._run_command = command_runner or subprocess.run
        self._stop, self._ready, self._done = threading.Event(), threading.Event(), threading.Event()
        self._lock, self._thread = threading.Lock(), None
        self._result = dict(status='not_started', output_directory=str(self.output_dir),
            limits=dict(max_duration_sec=self.max_duration_sec, max_capture_bytes=self.max_output_bytes),
            kernel=dict(available=False, reason='not started'), usbmon=dict(available=False, reason='not started'))

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name='px6d-system-evidence', daemon=True)
            self._thread.start()
            self._ready.wait(min(3., self.stop_timeout_sec))
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(self.stop_timeout_sec)
        with self._lock:
            result = deepcopy(self._result)
        result['worker_alive'] = self._thread is not None and self._thread.is_alive()
        if result['worker_alive']:
            result.update(status='stop_timeout', error='diagnostic worker did not finish within join budget')
        return result

    def _run(self):
        started = time.monotonic()
        result = deepcopy(self._result)
        result.update(status='collecting', started_host_monotonic=started, capture_bytes=0,
                      kernel_lines=0, usbmon_lines=0, usbmon_other_device_lines=0,
                      parse_errors=0, oversized_lines=0, errors=[])
        process = None
        usb_fd = None
        streams, buffers, files = {}, {}, {}
        urbs = _URBSummary()
        stderr = bytearray()
        stop_reason = 'stop_requested'
        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            before = self._snapshot(self.device)
            self._write_json('system_before.json', before)
            for name in ('kernel_events.jsonl', 'usbmon.log'):
                files[name] = (self.output_dir/name).open('wb')
            command = ['journalctl', '-k', '-f', '-n', '0', '-o', 'json', '--no-pager']
            result['kernel'] = dict(available=False, command=command,
                                    timeout_sec=self.max_duration_sec, reason='no journal event received yet',
                                    follow_events_received=False)
            try:
                process = self._popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
                for stream, name in ((process.stdout, 'kernel'), (process.stderr, 'stderr')):
                    fd = stream.fileno()
                    os.set_blocking(fd, False)
                    streams[fd], buffers[fd] = name, bytearray()
                result['kernel'].update(process_started=True)
            except (OSError, ValueError) as exc:
                result['kernel']['reason'] = f'{type(exc).__name__}: {exc}'
            usb = before.get('usb_device', {})
            bus, device = usb.get('busnum'), usb.get('devnum')
            result['usbmon'] = dict(available=False, busnum=bus, devnum=device,
                reason='USB bus/device address unavailable',
                filter_binding='bus/device address at start; enumeration changes require comparing system_before/after and kernel events')
            if bus is not None and device is not None:
                monitor_path = self._usbmon_root/f'{int(bus)}u'
                result['usbmon']['source_path'] = str(monitor_path)
                try:
                    usb_fd = os.open(monitor_path, os.O_RDONLY | os.O_NONBLOCK)
                    streams[usb_fd], buffers[usb_fd] = 'usbmon', bytearray()
                    result['usbmon'].update(available=True, reason=None)
                except OSError as exc:
                    result['usbmon']['reason'] = f'{type(exc).__name__}: {exc}'
            result['sources_ready_host_monotonic'] = time.monotonic()
            self._ready.set()
            while not self._stop.is_set():
                remaining = self.max_duration_sec-(time.monotonic()-started)
                if remaining <= 0:
                    stop_reason = 'duration_limit'
                    break
                if not streams:
                    self._stop.wait(min(.05, remaining))
                    continue
                ready, _, _ = select.select(list(streams), [], [], min(.05, remaining))
                for fd in ready:
                    try:
                        chunk = os.read(fd, 4096)
                    except BlockingIOError:
                        continue
                    except OSError as exc:
                        result['errors'].append(f'{streams[fd]} read: {type(exc).__name__}: {exc}')
                        if streams[fd] in ('usbmon', 'kernel'):
                            result[streams[fd]].update(available=False,
                                reason=f'read failed: {type(exc).__name__}: {exc}')
                            if streams[fd] == 'kernel':
                                result['kernel']['follow_error'] = result['kernel']['reason']
                        streams.pop(fd)
                        continue
                    if not chunk:
                        if streams[fd] in ('usbmon', 'kernel'):
                            result[streams[fd]]['stream_eof'] = True
                        streams.pop(fd)
                        continue
                    received = time.monotonic()
                    if streams[fd] == 'stderr':
                        stderr.extend(chunk[:max(0, 8192-len(stderr))])
                        continue
                    buffers[fd].extend(chunk)
                    while b'\n' in buffers[fd]:
                        raw, _, tail = buffers[fd].partition(b'\n')
                        buffers[fd] = bytearray(tail)
                        raw = bytes(raw)+b'\n'
                        name = streams[fd]
                        if name == 'usbmon':
                            record = parse_usbmon_line(raw.decode('ascii', 'replace'))
                            if record is None:
                                result['parse_errors'] += 1
                                continue
                            if record['busnum'] != bus or record['devnum'] != device:
                                result['usbmon_other_device_lines'] += 1
                                continue
                            record['capture_host_monotonic_sec'] = received
                            destination = 'usbmon.log'
                        else:
                            try:
                                json.loads(raw)
                            except (ValueError, UnicodeDecodeError):
                                result['parse_errors'] += 1
                                continue
                            destination = 'kernel_events.jsonl'
                        if result['capture_bytes']+len(raw) > self.max_output_bytes:
                            stop_reason = 'output_limit'
                            self._stop.set()
                            break
                        files[destination].write(raw)
                        result['capture_bytes'] += len(raw)
                        result[name+'_lines'] += 1
                        if name == 'usbmon':
                            urbs.observe(record)
                        else:
                            result['kernel'].update(available=True, reason=None, follow_events_received=True)
                    if len(buffers[fd]) > 65536:
                        buffers[fd].clear()
                        result['oversized_lines'] += 1
            result['status'] = 'finished'
        except Exception as exc:
            result.update(status='failed', error=f'{type(exc).__name__}: {exc}')
        finally:
            self._ready.set()
            if process is not None:
                try:
                    if process.poll() is None:
                        result['kernel']['terminated_by_collector'] = True
                        process.terminate()
                        try:
                            process.wait(timeout=.5)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait(timeout=.5)
                    result['kernel']['returncode'] = process.poll()
                except Exception as exc:
                    result['errors'].append(f'journal cleanup: {type(exc).__name__}: {exc}')
                for stream in (process.stdout, process.stderr):
                    if stream is not None:
                        try:
                            stream.close()
                        except Exception as exc:
                            result['errors'].append(f'journal pipe close: {type(exc).__name__}: {exc}')
            if usb_fd is not None:
                try:
                    os.close(usb_fd)
                except OSError as exc:
                    result['errors'].append(f'usbmon close: {type(exc).__name__}: {exc}')
            for stream in files.values():
                try:
                    stream.close()
                except Exception as exc:
                    result['errors'].append(f'evidence file close: {type(exc).__name__}: {exc}')
                    result['status'] = 'failed'
            result['kernel']['stderr'] = stderr.decode('utf-8', 'replace')
            if stderr and not result['kernel'].get('available'):
                result['kernel']['reason'] = result['kernel']['stderr']
            result.update(stop_reason=stop_reason, elapsed_sec=time.monotonic()-started,
                          usbmon_summary=urbs.snapshot(), capture_finished_host_monotonic=time.monotonic(),
                          incomplete_line_bytes=sum(len(value) for value in buffers.values()))
            # Probe after capture so its bounded subprocess cannot delay opening
            # usbmon or consuming USB records while the sensor is being read.
            probe = _journal_access_probe(self._run_command)
            result['kernel']['access_probe'] = probe
            follow_permission_warning = any(word in result['kernel']['stderr'].lower() for word in
                ('permission', 'not seeing messages', 'access denied'))
            follow_failed = (not result['kernel'].get('process_started') or
                bool(result['kernel'].get('follow_error')) or follow_permission_warning or
                (not result['kernel'].get('terminated_by_collector') and
                 result['kernel'].get('returncode', 0) != 0))
            if not result['kernel'].get('follow_events_received') and not follow_failed:
                result['kernel'].update(available=probe['available'], reason=probe['reason'],
                    follow_status='no_events_during_capture')
            if follow_failed:
                result['kernel']['available'] = False
                if result['kernel']['stderr']:
                    result['kernel']['reason'] = result['kernel']['stderr']
                result['kernel']['follow_status'] = 'unavailable'
            elif result['kernel'].get('follow_events_received'):
                result['kernel']['follow_status'] = 'events_received'
            try:
                self._write_json('system_after.json', self._snapshot(self.device))
            except Exception as exc:
                result['errors'].append(f'after snapshot write: {type(exc).__name__}: {exc}')
                result['status'] = 'failed'
            result['worker_elapsed_sec'] = time.monotonic()-started
            try:
                self._write_json('system_summary.json', result)
            except Exception as exc:
                result['errors'].append(f'summary write: {type(exc).__name__}: {exc}')
                result['status'] = 'failed'
            with self._lock:
                self._result = result
            self._done.set()

    def _write_json(self, name, value):
        (self.output_dir/name).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')

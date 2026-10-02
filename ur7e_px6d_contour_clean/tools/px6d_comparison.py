"""Finite, sequential sensor-only comparisons used by tools/check_px6d.py.

The independent worker imports neither the production reader nor robot/control
modules. Wire events stay in bounded memory until the serial port is closed.
USB evidence is collected separately. No automatic reconnect/recovery occurs.
"""
from __future__ import annotations

from collections import Counter, deque
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
SERIAL_KEYS = ('serial_port', 'baudrate', 'timeout_sec', 'poll_rate_hz', 'startup_delay_sec')


def save_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


class WireCapture:
    """Host observations, never claimed to be device receipt or sample time."""
    def __init__(self, max_events=50000, max_bytes=4*1024*1024):
        self.events = []
        self.tail = deque(maxlen=64)
        self.max_events, self.max_bytes = max_events, max_bytes
        self.raw_bytes = self.dropped_events = 0
        self.counts = Counter()
        self.started = dict(host_monotonic=time.monotonic(), utc=datetime.now(timezone.utc).isoformat())

    def record(self, kind, data=b'', **fields):
        event = dict(fields, kind=kind)
        event.setdefault('host_monotonic', time.monotonic())
        if data:
            event.update(data_hex=bytes(data).hex(), byte_count=len(data))
        self.counts[kind] += 1
        if len(self.events) >= self.max_events or self.raw_bytes+len(data) > self.max_bytes:
            self.dropped_events += 1
            self.tail.append(event)
        else:
            self.events.append(event)
            self.raw_bytes += len(data)

    def finish(self, path):
        metadata = dict(kind='capture_metadata', started=self.started,
            time_source='host monotonic; UTC anchor only; NOT sensor sampling time',
            tx_semantics='bytes accepted by host write; NOT proof of device receipt',
            complete=self.dropped_events == 0, dropped_events=self.dropped_events,
            retained_tail_events=len(self.tail), counts=dict(self.counts))
        with Path(path).open('w', encoding='utf-8') as stream:
            for event in [metadata, *self.events, *self.tail]:
                stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False)+'\n')
        return metadata


def run_reader(sensor, samples, duration_sec, capture):
    # Lazy import: the independent worker never imports this branch.
    from sensor import px6d_reader as px
    from tools.check_px6d import DiagnosticPX6DReader

    class PortTrace:
        def __init__(self, port, owner):
            self.port, self.owner = port, owner

        def __getattr__(self, name):
            return getattr(self.port, name)

        def read(self, size):
            began = time.monotonic()
            data = self.port.read(size)
            capture.record('rx', data, request_id=self.owner._request_id,
                           phase=self.owner.phase, passive=self.owner.passive,
                           read_started_host_monotonic=began)
            return data

        def write(self, data):
            written = self.port.write(data)
            capture.record('tx', data[:written], request_id=self.owner._request_id,
                           phase=self.owner.phase, requested_bytes=len(data))
            return written

        def reset_input_buffer(self):
            self.port.reset_input_buffer()
            capture.record('startup_input_reset', discarded_bytes='unknown; existing startup behavior')

        def reset_output_buffer(self):
            self.port.reset_output_buffer()
            capture.record('startup_output_reset')

    class ReaderTrace(DiagnosticPX6DReader):
        phase = 'open'
        passive = False

        def connect(self):
            import serial
            original_serial = serial.Serial
            def open_port(*args, **kwargs):
                started = time.monotonic()
                port = original_serial(*args, **kwargs)
                capture.record('open', open_started_host_monotonic=started,
                               elapsed_sec=time.monotonic()-started)
                return PortTrace(port, self)
            serial.Serial = open_port
            try:
                return super().connect()
            finally:
                serial.Serial = original_serial

        def _request(self, command, size, timeout_sec):
            self.phase = 'version' if command[3] == 7 else 'force'
            if not isinstance(self._port, PortTrace):
                self._port = PortTrace(self._port, self)
            capture.record('request_start', command, request_id=self._request_id+1,
                           phase=self.phase, timeout_sec=timeout_sec)
            # This is confined to a dedicated reader-only subprocess. Replace
            # this module's os reference, never the global os.write function.
            original_os = px.os

            def write(fd, data):
                began = time.monotonic()
                written = original_os.write(fd, data)
                capture.record('tx', data[:written], request_id=self._request_id,
                               phase=self.phase, requested_bytes=len(data),
                               write_started_host_monotonic=began)
                return written

            px.os = SimpleNamespace(name=original_os.name, get_blocking=original_os.get_blocking, write=write)
            try:
                response = super()._request(command, size, timeout_sec)
                capture.record('response', response, request_id=self._request_id, phase=self.phase)
                return response
            except (Exception, KeyboardInterrupt) as exc:
                capture.record('error', request_id=self._request_id, phase=self.phase,
                               error_type=type(exc).__name__, message=str(exc))
                raise
            finally:
                px.os = original_os
                if self._request_history:
                    capture.record('request_end', request_id=self._request_id,
                                   phase=self.phase, diagnostic=dict(self._request_history[-1]))

        def close(self):
            was_open = self._port is not None
            self.passive = True
            try:
                super().close()
            finally:
                if was_open:
                    capture.record('close', request_id=self._request_id)

    reader = ReaderTrace(sensor['serial_port'], sensor['baudrate'], sensor['timeout_sec'],
                         sensor['poll_rate_hz'], sensor['startup_delay_sec'])
    started = time.monotonic()
    collection_start = collection_end = None
    valid = 0
    failure = None
    try:
        reader.connect()
        collection_start = time.monotonic()
        deadline = collection_start+duration_sec
        while valid < samples and time.monotonic()+sensor['timeout_sec'] <= deadline:
            value = reader.read_wrench().array()
            if not all(math.isfinite(float(component)) for component in value):
                raise px.PX6DError('nonfinite PX6D wrench; collection stopped')
            valid += 1
        collection_end = time.monotonic()
    except (Exception, KeyboardInterrupt) as exc:
        collection_end = time.monotonic()
        failed_requests = getattr(exc, 'sensor_diagnostics', {}).get('requests', [])
        request = failed_requests[-1] if failed_requests and failed_requests[-1].get('outcome') != 'ok' else None
        failure = dict(type=type(exc).__name__, message=str(exc),
                       host_monotonic=request['request_end_host_monotonic'] if request else collection_end,
                       caught_host_monotonic=collection_end, request=request,
                       request_id=reader._request_id, phase=reader.phase)
    finally:
        try:
            reader.close()
        except Exception as exc:
            failure = failure or dict(type=type(exc).__name__, message=str(exc), phase='close',
                                      host_monotonic=time.monotonic(), request_id=reader._request_id)
    diagnostics = reader.diagnostics_snapshot()
    return dict(implementation='production_reader', firmware=reader.firmware,
                error=None if failure is None else f"{failure['type']}: {failure['message']}",
                first_failure=failure, samples_completed=valid, requested_samples=samples,
                duration_budget_sec=duration_sec, total_elapsed_sec=time.monotonic()-started,
                collection_elapsed_sec=0. if collection_start is None else collection_end-collection_start,
                requests=diagnostics['requests'], sensor_diagnostics=diagnostics,
                passive_after_failure=reader.passive_observation,
                stop_reason='error' if failure else 'samples' if valid == samples else 'duration')


def worker(spec_path):
    spec = json.loads(Path(spec_path).read_text())
    capture = WireCapture()
    output = Path(spec_path).parent
    result = None
    try:
        if spec['backend'] == 'reference':
            from tools.px6d_reference_probe import run_reference
            run = run_reference
        elif spec['backend'] == 'reader':
            run = run_reader
        else:
            raise ValueError('unknown backend')
        result = run(spec['sensor'], spec['samples'], spec['duration_sec'], capture)
    except (Exception, KeyboardInterrupt) as exc:
        result = dict(error=f'{type(exc).__name__}: {exc}', samples_completed=0, stop_reason='worker_error')
    finally:
        result['wire_capture'] = capture.finish(output/'wire.jsonl')
        result['imported_production_reader'] = 'sensor.px6d_reader' in sys.modules
        result['imported_robot_modules'] = sorted(name for name in sys.modules if
            name.startswith(('robot.', 'rtde_control', 'rtde_receive', 'app.continuous_runtime')))
        save_json(output/'result.json', result)
    return 1 if result['error'] else 0


def conditions(sensor, samples, duration_sec):
    values = {key: sensor[key] for key in SERIAL_KEYS}
    values.update(bytesize=8, parity='N', stopbits=1, flow_control=False,
                  version_timeout_sec=2., passive_after_failure_sec=1., samples=samples,
                  duration_sec=duration_sec, power_off_min_sec=5.)
    return values


def stop_worker(process):
    """All parent exit paths reap the sensor owner; repeated Ctrl+C cannot skip it."""
    if process is None or process.poll() is not None:
        return []
    errors = []
    original_sigint = None
    try:
        # Only the parent temporarily ignores subsequent interrupt keystrokes;
        # the child receives SIGINT and saves evidence before closing its tty.
        try:
            original_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        except ValueError:
            pass  # Unit callers outside the main thread still use bounded cleanup.
        try:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=4.)
        except (OSError, subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
            errors.append(f'interrupt cleanup: {type(exc).__name__}: {exc}')
            try:
                if process.poll() is None:
                    process.kill()
                process.wait(timeout=2.)
            except (OSError, subprocess.TimeoutExpired, KeyboardInterrupt) as final_exc:
                errors.append(f'kill cleanup: {type(final_exc).__name__}: {final_exc}')
    finally:
        if original_sigint is not None:
            signal.signal(signal.SIGINT, original_sigint)
    return errors


def run_one(directory, sensor, backend, samples, duration_sec, context):
    """A new directory per attempt; artifacts cannot overwrite an earlier run."""
    from tools.px6d_usb_evidence import SystemEvidence
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    spec = dict(backend=backend, sensor={key: sensor[key] for key in SERIAL_KEYS},
                samples=samples, duration_sec=duration_sec, context=context,
                conditions=conditions(sensor, samples, duration_sec),
                started_utc=datetime.now(timezone.utc).isoformat(), host_monotonic=time.monotonic(),
                source_sha256={str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in (Path(__file__), ROOT/'tools/px6d_reference_probe.py',
                                 ROOT/'tools/check_px6d.py', ROOT/'tools/px6d_usb_evidence.py',
                                 ROOT/'sensor/px6d_reader.py')})
    save_json(directory/'request.json', spec)
    budget = duration_sec+float(sensor['startup_delay_sec'])+10.
    evidence = SystemEvidence(sensor['serial_port'], directory, max_duration_sec=budget+6.)
    process = None
    result = None
    cleanup_errors = []
    try:
        evidence.start()
        before = json.loads((directory/'system_before.json').read_text())
        ownership = before.get('ownership', {})
        if not before.get('device_path_exists'):
            raise RuntimeError('serial device is absent; no open attempted')
        if not ownership.get('available') or ownership.get('process_ids'):
            raise RuntimeError('serial ownership unknown or occupied; no open attempted')
        # Separate process gives the independent implementation clean imports.
        # stdout/stderr are final worker errors only, never frame-by-frame output.
        with (directory/'worker_output.txt').open('w') as output:
            process = subprocess.Popen([sys.executable, '-m', 'tools.px6d_comparison',
                                        '--worker', str((directory/'request.json').resolve())],
                                       cwd=ROOT, stdout=output, stderr=subprocess.STDOUT)
            try:
                process.wait(timeout=budget)
            except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
                cleanup_errors.extend(stop_worker(process))
                result = dict(error=f'{type(exc).__name__}: collection interrupted by parent',
                              samples_completed=0, stop_reason='parent_interrupted')
        if (directory/'result.json').exists():
            worker_result = json.loads((directory/'result.json').read_text())
            if result:
                worker_result['parent_error'] = result['error']
                worker_result['error'] = worker_result.get('error') or result['error']
                worker_result['stop_reason'] = 'parent_interrupted'
            result = worker_result
        elif result is None:
            result = dict(error=f'worker exited {process.returncode} without result',
                          samples_completed=0, stop_reason='worker_error')
    except (Exception, KeyboardInterrupt) as exc:
        result = dict(error=f'{type(exc).__name__}: {exc}', samples_completed=0,
                      stop_reason='not_started' if process is None else 'worker_error')
    finally:
        previous_sigint = None
        try:
            try:
                previous_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
            except ValueError:
                pass
            cleanup_errors.extend(stop_worker(process))
            if process is not None:
                time.sleep(.06)  # Let the independent collector consume queued close/late URBs.
            system = evidence.stop()
        finally:
            if previous_sigint is not None:
                signal.signal(signal.SIGINT, previous_sigint)
    result.update(backend=backend, context=context, conditions=spec['conditions'],
                  system_capture=system, worker_cleanup_errors=cleanup_errors,
                  worker_still_running=process is not None and process.poll() is None,
                  finished_utc=datetime.now(timezone.utc).isoformat())
    try:
        result['usb_request_evidence'] = correlate_usb(directory, result)
    except Exception as exc:
        result['usb_request_evidence'] = dict(available=False, requests=[],
            reason=f'USB evidence interpretation failed: {type(exc).__name__}: {exc}')
    save_json(directory/'result.json', result)
    return result


def correlate_usb(directory, result):
    """Conservative order/payload correlation, without assuming clock alignment.

    Match every accepted request to a distinct full bulk OUT submission. If
    bytes are truncated/coalesced/missing, refuse per-request attribution. IN
    callbacks are reported including the passive interval; their arrival must
    not be mistaken for an on-time, decoded, fresh force sample.
    """
    from tools.px6d_usb_evidence import parse_usbmon_line
    directory = Path(directory)
    system = result.get('system_capture', {})
    monitor = system.get('usbmon', {})
    evidence = dict(available=False, reason=monitor.get('reason'), requests=[],
                    clock_alignment='not assumed; match full OUT payloads in sequence',
                    limitation='USB completion is HCD evidence, not device application acknowledgement; IN includes passive interval')
    if not monitor.get('available') or not (directory/'wire.jsonl').exists():
        return evidence
    if (system.get('status') != 'finished' or system.get('stop_reason') != 'stop_requested'
            or system.get('worker_alive') or system.get('errors') or monitor.get('stream_eof')
            or any(system.get(key, 0) for key in ('parse_errors', 'oversized_lines', 'incomplete_line_bytes'))):
        evidence['reason'] = 'system capture incomplete/failed/truncated; cannot infer absence of IN data'
        return evidence
    tx = []
    anchor = last_close = None
    for line in (directory/'wire.jsonl').read_text().splitlines():
        event = json.loads(line)
        if event['kind'] == 'capture_metadata' and not event['complete']:
            evidence['reason'] = 'wire capture truncated; request correlation unavailable'
            return evidence
        if event['kind'] == 'capture_metadata':
            anchor = event['started']['host_monotonic']
        if event['kind'] == 'close':
            last_close = event['host_monotonic']
        if event['kind'] == 'tx':
            tx.append(event)
    if (anchor is None or last_close is None or system.get('sources_ready_host_monotonic', float('inf')) > anchor
            or system.get('capture_finished_host_monotonic', float('-inf')) < last_close):
        evidence['reason'] = 'system capture does not cover complete serial lifetime/passive observation'
        return evidence
    if not (directory/'usbmon.log').exists():
        evidence['reason'] = 'usbmon raw file unavailable'
        return evidence
    records = [record for line in (directory/'usbmon.log').read_text().splitlines()
               if (record := parse_usbmon_line(line)) is not None]
    submissions, pending = [], {}
    for index, record in enumerate(records):
        if record['transfer_type'] != 'bulk' or record['direction'] != 'out':
            continue
        if record['event'] == 'S':
            item = dict(index=index, submit=record, completion=None)
            submissions.append(item)
            pending[record['urb_id']] = item
        elif record['event'] in ('C', 'E') and record['urb_id'] in pending:
            pending.pop(record['urb_id'])['completion'] = record
    if not tx or len(submissions) != len(tx) or any(
            sub['submit']['data_hex'] != event.get('data_hex') or sub['submit']['length'] != event.get('byte_count')
            for sub, event in zip(submissions, tx)):
        evidence.update(reason='OUT counts/full payloads do not map one-to-one; no per-request attribution',
                        host_tx_events=len(tx), usb_out_submissions=len(submissions))
        return evidence
    # Retain the last 32 requests in the summary. The complete raw streams are
    # already saved, and can be reinterpreted with a hardware trace if needed.
    for offset in range(max(0, len(tx)-32), len(tx)):
        event, sub = tx[offset], submissions[offset]
        end = submissions[offset+1]['index'] if offset+1 < len(tx) else len(records)
        incoming = [r for r in records[sub['index']+1:end] if
                    r['transfer_type'] == 'bulk' and r['direction'] == 'in' and r['event'] == 'C']
        completion = sub['completion']
        evidence['requests'].append(dict(request_id=event.get('request_id'),
            phase=event.get('phase'), host_tx=event, usb_submit=sub['submit'], usb_completion=completion,
            usb_out_completed=bool(completion and completion['event'] == 'C' and completion['status'] == 0
                                   and completion['length'] == event['byte_count']),
            usb_in_completed_bytes=sum(r['length'] or 0 for r in incoming if r['status'] == 0),
            usb_in_callbacks=incoming[-16:],
            interval='after this OUT submission through next OUT or end of capture (includes late bytes)'))
    evidence.update(available=True, reason=None)
    return evidence


def compare(directory):
    """Report evidence and controlled A/B/A; never infer hardware from rx=0."""
    directory = Path(directory)
    rows = []
    for path in sorted(directory.glob('round_*/result.json')):
        value = json.loads(path.read_text())
        before_path = path.parent/'system_before.json'
        before = json.loads(before_path.read_text()) if before_path.exists() else {}
        spec_path = path.parent/'request.json'
        spec = json.loads(spec_path.read_text()) if spec_path.exists() else {}
        usb = before.get('usb_device', {})
        requests = value.get('requests', [])
        last = requests[-1] if requests else {}
        rows.append(dict(directory=path.parent.name, backend=value.get('backend'),
            context=value.get('context', {}), conditions=value.get('conditions', {}),
            source_sha256=spec.get('source_sha256'),
            started_utc=spec.get('started_utc'), finished_utc=value.get('finished_utc'),
            parent_error=value.get('parent_error'),
            usb_topology=usb.get('topology'), usb_attributes=usb.get('attributes', {}),
            valid_frames=value.get('samples_completed', 0), elapsed_sec=value.get('collection_elapsed_sec', 0.),
            error=value.get('error'), stop_reason=value.get('stop_reason'),
            last_request=last, passive_after_failure=value.get('passive_after_failure'),
            usbmon=value.get('system_capture', {}).get('usbmon', {}),
            kernel=value.get('system_capture', {}).get('kernel', {}),
            usbmon_summary=value.get('system_capture', {}).get('usbmon_summary', {}),
            usb_request_evidence=value.get('usb_request_evidence', {}),
            wire_complete=value.get('wire_capture', {}).get('complete', False)))
    rows.sort(key=lambda row: row['started_utc'] or row['directory'])
    groups = {}
    for row in rows:
        key = row['context'].get('group', 'unknown')
        groups.setdefault(key, []).append(row)
    baseline = next((row for row in rows if row['context'].get('group') == 'original_reconnected'),
                    rows[0] if rows else None)
    comparisons = []
    for row in rows:
        reasons = []
        if row['conditions'] != baseline['conditions']:
            reasons.append('test settings differ')
        if not row['source_sha256'] or row['source_sha256'] != baseline['source_sha256']:
            reasons.append('source hashes missing or changed across comparison rounds')
        if not row['usb_topology'] or row['usb_topology'] != baseline['usb_topology']:
            reasons.append('same physical USB port not established')
        serial = row['usb_attributes'].get('serial', {}).get('value')
        if not serial or serial != baseline['usb_attributes'].get('serial', {}).get('value'):
            reasons.append('same device identity not established')
        if row['context'].get('power_cycle') != 'operator_confirmed':
            reasons.append('equal power-on state not confirmed')
        if row['stop_reason'] in ('not_started', 'worker_error', 'parent_interrupted'):
            reasons.append('collection did not run normally')
        if row['parent_error']:
            reasons.append('parent interrupted collection')
        if not row['wire_complete']:
            reasons.append('raw capture unavailable or truncated')
        if not row['error'] and not row['valid_frames']:
            reasons.append('no valid force frame collected')
        comparisons.append(dict(directory=row['directory'], comparable=not reasons, reasons=reasons))
    conclusion = ['主机 write 成功不证明设备应用收到请求；rx_bytes=0 不能单独定位硬件。',
                  '独立实现仍共享主机 USB 驱动、pyserial 配置及同一设备；两者失败不能排除共享层问题。',
                  '无内核报错或 usbmon 不可用均不能排除链路故障。USB 完成回调也不证明固件已处理命令。']
    if not rows:
        conclusion.append('尚未运行串口对照；系统检查不等于真实采集。')
    for row in rows:
        linked = row['usb_request_evidence'].get('requests', [])
        if not row['error'] or not linked:
            continue
        last = linked[-1]
        if last['request_id'] != row['last_request'].get('request_id'):
            continue  # A residual/preflight failure did not send a new USB request.
        if last['usb_in_completed_bytes']:
            conclusion.append(f"{row['directory']}：末请求后驱动层记录 IN 返回 {last['usb_in_completed_bytes']} 字节（含被动监听时段）；需比对原文及期限，不能声称设备完全未返回。")
        elif last['usb_out_completed']:
            conclusion.append(f"{row['directory']}：末请求有成功 OUT 完成记录，但捕获窗口内没有成功 IN 数据回调；问题更靠近共享 USB/端点/设备路径，尚不能指定线缆或固件。")
        else:
            conclusion.append(f"{row['directory']}：末请求缺少成功 OUT 完成记录，先审查主机驱动/USB 传输及捕获完整性；不能据此判定设备已收到。")
    backends = {row['backend'] for row in rows if row['stop_reason'] != 'not_started'}
    if backends == {'reader', 'reference'}:
        outcomes = {backend: [row['error'] is not None for row in rows if row['backend'] == backend]
                    for backend in backends}
        if all(all(failures) for failures in outcomes.values()):
            conclusion.append('两套读取实现均失败：单独归因于现有解析器不足，需结合起始状态与 USB 证据。')
        elif any(all(failures) for failures in outcomes.values()) and any(not any(failures) for failures in outcomes.values()):
            conclusion.append('实现之间结果不同：优先审查读取路径；同条件重复及原始/USB 数据核对后才能归因。')
    cable_groups = {key: items for key, items in groups.items() if key in
                    ('original_reconnected', 'known_good_same_port', 'original_returned')}
    cable_directories = {row['directory'] for items in cable_groups.values() for row in items}
    controlled = bool(cable_directories) and all(item['comparable'] for item in comparisons
                                                if item['directory'] in cable_directories)
    cable_pattern = controlled and len(cable_groups) == 3
    if cable_pattern:
        boundaries = {}
        for key, items in cable_groups.items():
            starts, ends = [row['started_utc'] for row in items], [row['finished_utc'] for row in items]
            if all(starts) and all(ends):
                boundaries[key] = (min(starts), max(ends))
        ordered = len(boundaries) == 3 and (boundaries['original_reconnected'][1] <= boundaries['known_good_same_port'][0]
            and boundaries['known_good_same_port'][1] <= boundaries['original_returned'][0])
        enough = all(all(sum(row['backend'] == backend for row in cable_groups[key]) >= 2
                            for backend in ('reader', 'reference')) for key in cable_groups)
        if enough and ordered and all(row['error'] for key in ('original_reconnected', 'original_returned') for row in groups[key]) and all(
                not row['error'] for row in groups['known_good_same_port']):
            conclusion.append('同口、同设备、各轮均报告重新上电的 A/B/A 重复呈原线失败、对照线成功、原线再失败；证据支持线缆/连接相关，不能排除插接差异。')
    report = dict(schema_version=1, rounds=rows, comparability=comparisons, conclusions=conclusion,
                  groups={key: dict(rounds=len(items), valid_frames=sum(row['valid_frames'] for row in items),
                                   failed_rounds=sum(row['error'] is not None for row in items))
                          for key, items in groups.items()})
    audits = sorted(directory.glob('audit_*/system_summary.json'))
    if audits:
        report['latest_system_only_audit'] = dict(path=str(audits[-1].relative_to(directory)),
            sensor_opened=False, summary=json.loads(audits[-1].read_text()))
    save_json(directory/'comparison.json', report)
    lines = ['PX6D 串口/USB 对照（主机时间，不是采样时间）', *conclusion, '']
    lines += [f"{row['directory']}: {row['backend']}, {row['valid_frames']} 帧 / {row['elapsed_sec']:.3f}s, {row['error'] or row['stop_reason']}" for row in rows]
    (directory/'conclusion.txt').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    device_side_evidence = {row['backend'] for row in rows if row['error'] and any(
        request.get('usb_out_completed') and not request.get('usb_in_completed_bytes')
        and request.get('request_id') == row['last_request'].get('request_id')
        for request in row['usb_request_evidence'].get('requests', []))}
    if device_side_evidence == {'reader', 'reference'}:
        (directory/'vendor_reproduction.txt').write_text('\n'.join([
            'PX6D 请求响应断流复现材料（请求厂家核查，尚未证明设备固件故障）',
            '两种独立读取实现均失败，捕获末请求成功 OUT 完成但窗口内无成功 IN 数据回调。',
            '主机 HCD 记录不能代替设备端应用确认；仍需核对捕获完整性、线缆和端点状态。',
            '详见 comparison.json 的条件、USB 身份、每轮结果及可比性限制；不包含机器人运动。',
            '各 round_*/request.json 保存源码哈希/参数；wire.jsonl 原始收发；usbmon.log/系统 JSON 为系统证据。',
            '若已执行 A/B/A，请结合功率重新上电报告和时序；电压并未测量。',
            '请核对该型号固件协议、请求频率、CDC 控制线/缓冲要求及设备端收包和响应状态。',
            *lines[1:]])+'\n', encoding='utf-8')
    return report


def comparison_main(args, sensor):
    from tools.px6d_usb_evidence import SystemEvidence
    from app.operator_input import confirm_enter
    directory = args.comparison_dir.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    sensor = {key: sensor[key] for key in SERIAL_KEYS}
    if args.poll_rate_hz is not None:
        sensor['poll_rate_hz'] = args.poll_rate_hz  # Diagnostic copy; never config.yaml.
    if 1./sensor['poll_rate_hz'] >= sensor['timeout_sec']:
        raise ValueError('诊断频率过低：请求预算包含节拍，周期必须小于 timeout_sec；未改扫描配置、未打开串口。')
    samples = 3000 if args.samples is None else args.samples
    plan = dict(conditions=conditions(sensor, samples, args.duration), campaign=args.campaign,
                repeats=args.repeats, stationary_confirmation='pending', robot_connection=False)
    plan_path = directory/'plan.json'
    if plan_path.exists():
        previous = json.loads(plan_path.read_text())
        if any(previous.get(key) != plan[key] for key in ('conditions', 'campaign', 'repeats')):
            raise ValueError('existing comparison has different conditions; use a new output directory')
    if args.system_only:
        audit = directory/f"audit_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f')}"
        monitor = SystemEvidence(sensor['serial_port'], audit, max_duration_sec=4.)
        try:
            monitor.start()
            time.sleep(2.)
        finally:
            result = monitor.stop()
        if not plan_path.exists():
            save_json(plan_path, plan)
        compare(directory)
        print(f"系统检查已保存：{audit}；未打开串口、未连接机器人。usbmon={result.get('usbmon')}")
        return 0
    if not confirm_enter('现场确认机器人已停稳且不会自动运动；下面只读 PX6D，不连接机器人。'):
        return 130
    plan.update(stationary_confirmation='operator_confirmed', confirmed_utc=datetime.now(timezone.utc).isoformat())
    save_json(plan_path, plan)
    groups = [('untouched', None), ('original_reconnected', '原线接回原 USB 口'),
              ('known_good_same_port', '用已知正常的数据线连接同一个 USB 口'),
              ('original_returned', '换回原线连接原 USB 口')] if args.campaign else [('untouched', None)]
    try:
        for group, action in groups:
            for repeat in range(1, args.repeats+1):
                order = ('reader', 'reference') if repeat % 2 else ('reference', 'reader')
                for backend in order:
                    name = f'round_{group}_{repeat:02d}_{backend}'
                    target = directory/name
                    if (target/'result.json').exists():
                        continue  # Resume without overwriting or silently repeating a failed run.
                    context = dict(group=group, repeat=repeat, power_cycle='not_performed',
                                   starting_state='unknown; serial open is not a power cycle')
                    if action:
                        prompt = f'本轮 {group} / {repeat} / {backend}：断开 PX6D 供电至少 5 秒，再{action}（独立电源也需断开）。完成后'
                        if not confirm_enter(prompt):
                            return 130
                        context.update(power_cycle='operator_confirmed',
                                       starting_state='operator reports power off >=5 s; voltage not measured',
                                       action_confirmed_utc=datetime.now(timezone.utc).isoformat())
                    print(f'{name}：有限采集 {args.duration:g}s / 最多 {samples} 帧。', flush=True)
                    result = run_one(target, sensor, backend, samples, args.duration, context)
                    compare(directory)  # Saved BEFORE next physical instruction or connection.
                    print(f"已保存 {target}：{result['samples_completed']} 帧；{result['error'] or result['stop_reason']}", flush=True)
                    if result.get('parent_error') or result.get('worker_still_running') or result.get('stop_reason') in (
                            'not_started', 'worker_error', 'parent_interrupted') or (
                            (result.get('first_failure') or {}).get('type') == 'KeyboardInterrupt'):
                        return 1
        return 0
    except (KeyboardInterrupt, EOFError):
        return 130
    finally:
        compare(directory)


if __name__ == '__main__':
    if len(sys.argv) != 3 or sys.argv[1] != '--worker':
        raise SystemExit('Use tools/check_px6d.py --comparison-dir DIR --campaign')
    raise SystemExit(worker(sys.argv[2]))

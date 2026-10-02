"""Independent, read-only POSIX PX6D reference exchange for wire comparison.

No runtime reader, force preprocessing, robot connection, retry, or zeroing is
used. A failed exchange ends acquisition; the final one-second observation only
captures incoming bytes. Times below describe the host, never sensor sampling.
"""
from __future__ import annotations

from collections import deque
import math
import os
import select
import struct
import time


_VERSION = bytes.fromhex('aa557f0701d0')
_FRAME = bytes.fromhex('aa557f0501fa')
_HEADER = bytes.fromhex('aa55')


class ReferenceProtocolError(RuntimeError):
    pass


def _crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (0x07 if value & 0x80 else 0)) & 0xff
    return value


def run_reference(sensor: dict, samples: int, duration_sec: float, wire_capture) -> dict:
    """Return a bounded summary; ``record(kind, data=b'', **fields)`` saves wire data.

    ``duration_sec`` bounds main collection, excluding initial USB settling and
    version query. A new request needs its full configured deadline remaining.
    The default serial settings match the project, with a two-second version
    deadline. Only serial settings are consumed from ``sensor``.
    """
    if os.name != 'posix':
        raise ValueError('the independent select/fd probe requires POSIX')
    if type(samples) is not int or samples <= 0:
        raise ValueError('samples must be a positive integer')
    timeout = float(sensor.get('timeout_sec', .05))
    rate = float(sensor.get('poll_rate_hz', 100.))
    startup_delay = float(sensor.get('startup_delay_sec', 2.))
    duration_sec = float(duration_sec)
    if not all(math.isfinite(v) and v > 0 for v in (timeout, rate, duration_sec)):
        raise ValueError('request timeout, rate and collection duration must be positive and finite')
    if not math.isfinite(startup_delay) or startup_delay < 0:
        raise ValueError('startup delay must be nonnegative and finite')

    began = time.monotonic()
    port = None
    pending = bytearray()
    request_id = 0
    last_tx = None
    request_history = deque(maxlen=32)
    current = None
    active_phase = 'open'
    main_started = main_ended = None
    valid = 0
    first_failure = None
    passive = None
    firmware = None
    counts = dict(tx_bytes=0, rx_bytes=0, crc_failures=0, candidate_rejections=0,
                  discarded_bytes=0, successful_requests=0)
    totals = dict(count=0, request_elapsed_sec=0., write_elapsed_sec=0.,
                  drain_elapsed_sec=0., read_elapsed_sec=0., max_request_elapsed_sec=0.)

    def record(kind, data=b'', **fields):
        wire_capture.record(kind, data=data, host_monotonic=time.monotonic(),
                            request_id=request_id, phase=active_phase, **fields)

    def extract(size, response_command, deadline):
        while time.monotonic() < deadline:
            offset = pending.find(_HEADER)
            if offset < 0:
                keep = 1 if pending.endswith(b'\xaa') else 0
                counts['discarded_bytes'] += len(pending)-keep
                del pending[:len(pending)-keep]
                return None
            if offset:
                counts['discarded_bytes'] += offset
                del pending[:offset]
            if len(pending) < size:
                return None
            candidate = bytes(pending[:size])
            if _crc8(candidate[:-1]) != candidate[-1]:
                counts['crc_failures'] += 1
                del pending[0]
                continue
            if candidate[2] != 0x7f or candidate[3] != response_command:
                counts['candidate_rejections'] += 1
                del pending[0]
                continue
            del pending[:size]
            return candidate
        return None

    def request(command, size, budget):
        nonlocal request_id, last_tx, current, active_phase
        request_id += 1
        active_phase = 'version' if command == _VERSION else 'force'
        started = time.monotonic()
        deadline = started+budget
        current = dict(request_id=request_id, command=command.hex(), phase=active_phase,
                       started_host_monotonic=started, deadline_host_monotonic=deadline,
                       timeout_sec=budget, tx_bytes=0, rx_bytes=0, stage='pace',
                       pace_elapsed_sec=0., write_elapsed_sec=0., drain_elapsed_sec=0.,
                       read_elapsed_sec=0.)
        record('request_start', command, timeout_sec=budget, deadline_host_monotonic=deadline)
        try:
            wait = 0. if last_tx is None else max(0., 1/rate-(started-last_tx))
            if wait:
                time.sleep(min(wait, budget))
            current['pace_elapsed_sec'] = time.monotonic()-started
            record('pace', planned_sec=wait, elapsed_sec=current['pace_elapsed_sec'])
            if time.monotonic() >= deadline:
                raise TimeoutError('reference request deadline exhausted during pacing')
            current['stage'] = 'preflight'
            if pending:
                raise ReferenceProtocolError('residual buffered bytes before a new request')
            fd = port.fileno()
            if select.select([fd], [], [], 0.)[0]:
                data = os.read(fd, 4096)
                pending.extend(data)
                current['rx_bytes'] += len(data)
                counts['rx_bytes'] += len(data)
                record('rx', data, preflight=True, passive=False)
                raise ReferenceProtocolError('serial data or EOF before a new request')
            if os.get_blocking(fd):
                raise ReferenceProtocolError('serial descriptor is blocking')
            current['stage'] = 'write'
            write_started = time.monotonic()
            last_tx = write_started
            current['tx_bytes'] = None
            try:
                written = os.write(fd, command)  # One attempt; never send a remainder.
                current['tx_bytes'] = written
                counts['tx_bytes'] += written
                record('tx', command[:written], requested_bytes=len(command))
            finally:
                current['write_elapsed_sec'] = time.monotonic()-write_started
            if written != len(command):
                raise ReferenceProtocolError('short reference serial write')
            if time.monotonic() >= deadline:
                raise TimeoutError('reference request deadline exhausted during write')
            current['stage'] = 'output_drain'
            drain_started = time.monotonic()
            try:
                while port.out_waiting:
                    if time.monotonic() >= deadline:
                        raise TimeoutError('reference output queue did not drain within deadline')
                    time.sleep(min(.0002, max(0., deadline-time.monotonic())))
            finally:
                current['drain_elapsed_sec'] = time.monotonic()-drain_started
            record('output_drain', elapsed_sec=current['drain_elapsed_sec'])
            if time.monotonic() >= deadline:
                raise TimeoutError('reference request deadline exhausted during output drain')
            current['stage'] = 'read'
            read_started = time.monotonic()
            try:
                while time.monotonic() < deadline:
                    candidate = extract(size, 0x07 if command == _VERSION else 0x03, deadline)
                    if candidate is not None:
                        if time.monotonic() >= deadline:
                            break
                        record('response', candidate)
                        current['outcome'] = 'ok'
                        counts['successful_requests'] += 1
                        return candidate
                    # Readability and os.read are independent of TIOCINQ/in_waiting.
                    ready = select.select([fd], [], [], max(0., deadline-time.monotonic()))[0]
                    if ready:
                        data = os.read(fd, 4096)
                        pending.extend(data)
                        counts['rx_bytes'] += len(data)
                        current['rx_bytes'] += len(data)
                        record('rx', data, passive=False)
                        if not data:
                            raise ReferenceProtocolError('serial EOF during reference response')
            finally:
                current['read_elapsed_sec'] = time.monotonic()-read_started
            raise TimeoutError('reference response deadline exhausted')
        except (Exception, KeyboardInterrupt) as exc:
            current['outcome'] = type(exc).__name__
            raise
        finally:
            current['elapsed_sec'] = time.monotonic()-started
            current['buffer_bytes_end'] = len(pending)
            totals['count'] += 1
            totals['request_elapsed_sec'] += current['elapsed_sec']
            for key in ('write_elapsed_sec', 'drain_elapsed_sec', 'read_elapsed_sec'):
                totals[key] += current[key]
            totals['max_request_elapsed_sec'] = max(totals['max_request_elapsed_sec'], current['elapsed_sec'])
            request_history.append(dict(current))

    try:
        import serial
        port = serial.Serial(sensor['serial_port'], baudrate=int(sensor.get('baudrate', 921600)),
                             bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                             stopbits=serial.STOPBITS_ONE, timeout=0., write_timeout=0.,
                             xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True)
        record('open')
        time.sleep(startup_delay)
        port.reset_input_buffer()
        record('startup_input_reset', discarded_bytes='unknown; matched production startup behavior')
        port.reset_output_buffer()
        record('startup_output_reset')
        firmware = request(_VERSION, 13, 2.)[4:-1].strip(b'\x00').decode('ascii', errors='replace')
        main_started = time.monotonic()
        main_deadline = main_started+duration_sec
        while valid < samples and time.monotonic()+timeout <= main_deadline:
            response = request(_FRAME, 29, timeout)
            values = struct.unpack('<6f', response[4:28])
            if not all(math.isfinite(v) for v in values):
                raise ReferenceProtocolError('nonfinite decoded reference wrench')
            valid += 1
        main_ended = time.monotonic()
    except (Exception, KeyboardInterrupt) as exc:
        main_ended = time.monotonic()
        first_failure = dict(type=type(exc).__name__, message=str(exc), phase=active_phase,
                             host_monotonic=main_ended, request_id=request_id,
                             request=None if current is None else dict(current))
        record('error', error_type=type(exc).__name__, message=str(exc),
               buffer_bytes=len(pending))
        if port is not None:
            passive_started = time.monotonic()
            passive = dict(tx_bytes=0, rx_bytes=0, elapsed_sec=0., error=None)
            try:
                while time.monotonic()-passive_started < 1.:
                    ready = select.select([port.fileno()], [], [],
                        max(0., 1.-(time.monotonic()-passive_started)))[0]
                    if ready:
                        data = os.read(port.fileno(), 4096)
                        passive['rx_bytes'] += len(data)
                        record('rx', data, passive=True)
                        if not data:
                            break
            except (Exception, KeyboardInterrupt) as passive_error:
                passive['error'] = f'{type(passive_error).__name__}: {passive_error}'
            passive['elapsed_sec'] = time.monotonic()-passive_started
    finally:
        if port is not None:
            try:
                port.close()
                record('close')
            except Exception as exc:
                if first_failure is None:
                    first_failure = dict(type=type(exc).__name__, message=str(exc), phase='close',
                                         host_monotonic=time.monotonic(), request_id=request_id)
    elapsed = 0. if main_started is None else main_ended-main_started
    return dict(schema_version=1, implementation='independent_pyserial_select',
                time_source='host monotonic; sensor sampling time unavailable', firmware=firmware,
                requested_samples=samples, duration_budget_sec=duration_sec,
                samples_completed=valid, valid_samples=valid, request_count=request_id,
                collection_elapsed_sec=elapsed, total_elapsed_sec=time.monotonic()-began,
                error=None if first_failure is None else f"{first_failure['type']}: {first_failure['message']}",
                first_failure=first_failure, request_timing=totals, counters=counts,
                requests=list(request_history), passive_after_failure=passive,
                stop_reason=('error' if first_failure else 'samples' if valid == samples else 'duration'))

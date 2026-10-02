"""Direct serial reader for a PaXini PX6D sensor (no ROS dependency).

Protocol source: PaXini PX6D user manual, USB section 5.2.  The byte layout
and measured test frames are also cross-checked against the existing lab
driver in this workspace.
"""

from __future__ import annotations

import math
import struct
import time
from collections import deque
from dataclasses import dataclass

from core.models import Wrench

HEADER = b"\xAA\x55"
DEFAULT_DEVICE_ID = 0x7F
CMD_STREAM = 0x03
CMD_GET_FRAME = 0x05
CMD_GET_VERSION = 0x07
WRENCH_PACKET_SIZE = 29
VERSION_PACKET_SIZE = 13


class PX6DError(RuntimeError):
    """Base error for serial, timeout, and protocol failures."""


class PX6DTimeout(PX6DError):
    """No complete valid response within the existing request deadline."""


class ProtocolError(PX6DError):
    """Raised when a PX6D frame is malformed."""


def crc8(data: bytes) -> int:
    """CRC-8: polynomial 0x07, initial value 0x00, no reflection."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_command(command: int, data: int = 0x01, device_id: int = DEFAULT_DEVICE_ID) -> bytes:
    body = HEADER + bytes((device_id, command, data))
    return body + bytes((crc8(body),))


GET_FRAME_COMMAND = build_command(CMD_GET_FRAME)
GET_VERSION_COMMAND = build_command(CMD_GET_VERSION)


def validate_packet(packet: bytes, expected_size: int) -> None:
    if len(packet) != expected_size:
        raise ProtocolError(f"expected {expected_size} bytes, got {len(packet)}")
    if packet[:2] != HEADER:
        raise ProtocolError("invalid frame header")
    if crc8(packet[:-1]) != packet[-1]:
        raise ProtocolError("CRC-8 mismatch")


def parse_wrench_packet(packet: bytes) -> Wrench:
    validate_packet(packet, WRENCH_PACKET_SIZE)
    if packet[3] != CMD_STREAM:
        raise ProtocolError(f"unexpected wrench command 0x{packet[3]:02X}")
    return Wrench.from_sequence(struct.unpack("<6f", packet[4:28]))


def parse_version_packet(packet: bytes) -> str:
    validate_packet(packet, VERSION_PACKET_SIZE)
    if packet[3] != CMD_GET_VERSION:
        raise ProtocolError(f"unexpected version command 0x{packet[3]:02X}")
    return packet[4:-1].strip(b"\x00").decode("ascii", errors="replace")


class PX6DReader:
    """Synchronous request/response PX6D reader with a fail-fast timeout."""

    def __init__(
        self,
        device: str,
        baudrate: int = 921600,
        timeout_sec: float = 0.05,
        poll_rate_hz: float = 100.0,
        startup_delay_sec: float = 2.0,
    ):
        self.device = device
        self.baudrate = int(baudrate)
        self.timeout_sec = float(timeout_sec)
        self.poll_rate_hz = float(poll_rate_hz)
        self.startup_delay_sec = float(startup_delay_sec)
        if self.poll_rate_hz <= 0.0:
            raise ValueError("poll_rate_hz must be positive")
        if self.startup_delay_sec < 0.0:
            raise ValueError("startup_delay_sec cannot be negative")
        self._request_interval_sec = 1.0 / self.poll_rate_hz
        self._last_request_started: float | None = None
        self._port = None
        self._buffer = bytearray()
        self._requires_resynchronization = False
        self._request_id = 0
        self._request_history = deque(maxlen=32)
        self._active_diagnostic = None
        self._crc_failures = 0
        self._candidate_rejections = 0
        self.firmware = "unknown"

    def connect(self) -> None:
        if self._port is not None:
            raise PX6DError("PX6D is already connected")
        try:
            import serial

            self._port = serial.Serial(
                self.device,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0,
                write_timeout=0.0,  # One nonblocking 6-byte write; short writes fail closed.
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            # Opening this USB CDC port can reset or temporarily stall the
            # PX6D firmware.  v1.0.1 on the lab sensor needs a settling delay.
            time.sleep(self.startup_delay_sec)
            self._port.reset_input_buffer()
            self._port.reset_output_buffer()
            self._buffer.clear()
            self._requires_resynchronization = False
            self.firmware = parse_version_packet(
                self._request(GET_VERSION_COMMAND, VERSION_PACKET_SIZE, 2.0)
            )
        except Exception as exc:
            self.close()
            error = PX6DError(f"cannot initialize PX6D at {self.device}: {exc}")
            error.sensor_diagnostics = getattr(exc, 'sensor_diagnostics', self.diagnostics_snapshot())
            raise error from exc

    def _extract_packet(self, size: int, expected_command=None, device_id=None) -> bytes | None:
        if self._active_diagnostic is not None:
            self._active_diagnostic['parse_attempts'] += 1
        while True:
            if (self._active_diagnostic is not None and
                    time.monotonic() >= self._active_diagnostic['request_deadline_host_monotonic']):
                return None
            index = self._buffer.find(HEADER)
            if index < 0:
                if len(self._buffer) > 1:
                    del self._buffer[:-1]
                return None
            if index:
                del self._buffer[:index]
            if len(self._buffer) < size:
                return None
            candidate = bytes(self._buffer[:size])
            try:
                validate_packet(candidate, size)
            except ProtocolError:
                self._crc_failures += 1
                del self._buffer[0]
                continue
            if ((expected_command is not None and candidate[3] != expected_command) or
                    (device_id is not None and candidate[2] != device_id)):
                self._candidate_rejections += 1
                del self._buffer[0]
                continue
            del self._buffer[:size]
            return candidate

    def _read_packet(self, size: int, timeout_sec: float, expected_command=None, device_id=None, *, deadline=None) -> bytes:
        if self._port is None:
            raise PX6DError("PX6D is not connected")
        started = previous_poll = time.monotonic()
        deadline = started + timeout_sec if deadline is None else deadline
        diagnostic = self._active_diagnostic
        if diagnostic is not None:
            diagnostic.update(read_start_host_monotonic=started,
                              buffer_bytes_at_read_start=len(self._buffer))
        try:
            while True:
                now = time.monotonic()
                if diagnostic is not None:
                    diagnostic['max_read_loop_gap_sec'] = max(
                        diagnostic['max_read_loop_gap_sec'], now-previous_poll)
                previous_poll = now
                if now >= deadline:
                    break
                # A complete packet may already be buffered even when the OS
                # serial queue is empty. Parsing must not depend on in_waiting.
                parse_started = time.monotonic()
                packet = self._extract_packet(size, expected_command, device_id)
                if diagnostic is not None:
                    diagnostic['parse_elapsed_sec'] += time.monotonic()-parse_started
                if packet is not None:
                    if time.monotonic() >= deadline:
                        if diagnostic is not None:
                            diagnostic['packet_parsed_after_deadline'] = True
                        break
                    if diagnostic is not None:
                        diagnostic['buffered_packet_returns'] += 1
                    return packet
                waiting = self._port.in_waiting
                if diagnostic is not None:
                    diagnostic['read_iterations'] += 1
                if waiting:
                    data = self._port.read(min(waiting, 4096))
                    received = time.monotonic()
                    self._buffer.extend(data)
                    if diagnostic is not None:
                        diagnostic['rx_bytes'] += len(data)
                        diagnostic['buffer_bytes_peak'] = max(diagnostic['buffer_bytes_peak'], len(self._buffer))
                        if data:
                            if diagnostic['first_rx_host_monotonic'] is None:
                                diagnostic['first_rx_host_monotonic'] = received
                            diagnostic['last_rx_host_monotonic'] = received
                    if received >= deadline:
                        break  # Never accept a response observed after this deadline.
                    parse_started = time.monotonic()
                    packet = self._extract_packet(size, expected_command, device_id)
                    if diagnostic is not None:
                        diagnostic['parse_elapsed_sec'] += time.monotonic()-parse_started
                    if packet is not None:
                        if time.monotonic() >= deadline:
                            if diagnostic is not None:
                                diagnostic['packet_parsed_after_deadline'] = True
                            break
                        return packet
                time.sleep(0.0002)
        except Exception as exc:
            raise PX6DError(f"PX6D serial read failed: {exc}") from exc
        finally:
            ended = time.monotonic()
            if diagnostic is not None:
                diagnostic.update(read_end_host_monotonic=ended, read_elapsed_sec=ended-started,
                    max_read_loop_gap_sec=max(diagnostic['max_read_loop_gap_sec'], ended-previous_poll))
        raise PX6DTimeout(f"PX6D response timeout after {timeout_sec:.3f} s")

    def diagnostics_snapshot(self) -> dict:
        """Bounded in-memory evidence; no printing or disk I/O on the device path."""
        return dict(time_source='host monotonic; PX6D sample timestamp unavailable',
                    crc_failures_total=self._crc_failures,
                    candidate_rejections_total=self._candidate_rejections,
                    requires_resynchronization=self._requires_resynchronization,
                    requests=[dict(record) for record in self._request_history])

    def resynchronize_after_timeout(self) -> None:
        """Explicit stationary-startup recovery; normal reads never retry.

        Caller must establish stationary, unloaded startup before calling this.
        Discard the partial response and verify a different response type.
        This does not establish arbitrary wire-level freshness (no sequence ID),
        capture bias, issue hardware zero, or reconnect. Never called by runtime.
        """
        if self._port is None:
            raise PX6DError('PX6D is not connected')
        self._buffer.clear()
        try:
            self._port.reset_input_buffer()
            self._requires_resynchronization = False
            self.firmware = parse_version_packet(
                self._request(GET_VERSION_COMMAND, VERSION_PACKET_SIZE, self.timeout_sec))
            if self._buffer or self._port.in_waiting:
                raise ProtocolError('residual PX6D bytes after stationary resynchronization')
        except Exception as exc:
            self._requires_resynchronization = True
            error = PX6DError(f'PX6D stationary resynchronization failed: {exc}')
            error.sensor_diagnostics = self.diagnostics_snapshot()
            raise error from exc

    def _request(self, command: bytes, size: int, timeout_sec: float) -> bytes:
        if self._port is None:
            raise PX6DError("PX6D is not connected")
        self._request_id += 1
        started = time.monotonic()
        deadline = started+timeout_sec
        expected_command = CMD_STREAM if command[3] == CMD_GET_FRAME else command[3]
        diagnostic = dict(request_id=self._request_id, command=command[3],
            request_start_host_monotonic=started, timeout_sec=timeout_sec,
            timeout_scope='whole request: pacing/write/output drain/read/parse',
            request_deadline_host_monotonic=deadline, parse_elapsed_sec=0.,
            flush_method='bounded out_waiting polling; no blocking tcdrain',
            previous_request_interval_sec=None, pacing_elapsed_sec=0., write_elapsed_sec=0., flush_elapsed_sec=0.,
            tx_bytes=0, rx_bytes=0, buffer_bytes_start=len(self._buffer),
            buffer_bytes_peak=len(self._buffer), serial_bytes_before_write=None,
            read_iterations=0, parse_attempts=0, buffered_packet_returns=0,
            max_read_loop_gap_sec=0., first_rx_host_monotonic=None, last_rx_host_monotonic=None)
        self._active_diagnostic = diagnostic
        crc_before, rejected_before = self._crc_failures, self._candidate_rejections
        error = None
        stage = 'preflight'
        try:
            # PX6D replies have no request sequence number. After an uncertain
            # exchange, another force request could mistake a late reply for new
            # data. Recovery is explicit and stationary, never a read retry.
            if self._requires_resynchronization:
                raise PX6DError('PX6D requires explicit stationary resynchronization before another request')
            if self._last_request_started is not None:
                remaining = self._request_interval_sec - (
                    time.monotonic() - self._last_request_started
                )
                if remaining > 0.0:
                    time.sleep(remaining)
            diagnostic['pacing_elapsed_sec'] = time.monotonic()-started
            if time.monotonic() >= deadline:
                raise PX6DTimeout('PX6D request pacing exceeded deadline')
            waiting = self._port.in_waiting
            diagnostic['serial_bytes_before_write'] = waiting
            if self._buffer or waiting:
                raise ProtocolError('unconsumed PX6D response bytes before request; stationary resynchronization required')
            previous = self._last_request_started
            self._last_request_started = time.monotonic()
            diagnostic['tx_start_host_monotonic'] = self._last_request_started
            diagnostic['previous_request_interval_sec'] = None if previous is None else self._last_request_started-previous
            stage = 'write'
            diagnostic['tx_bytes'] = None  # Unknown if write raises after a partial send.
            try:
                diagnostic['tx_bytes'] = self._port.write(command)
            finally:
                diagnostic['write_elapsed_sec'] = time.monotonic()-self._last_request_started
            if diagnostic['tx_bytes'] != len(command):
                raise PX6DError('PX6D serial write was incomplete')
            if time.monotonic() >= deadline:
                raise PX6DTimeout('PX6D write exceeded deadline')
            stage = 'output_drain'
            flush_started = time.monotonic()
            try:
                # pyserial.flush() uses an unbounded tcdrain on Linux. A stuck
                # USB output queue must yield to the existing sensor stop path.
                while self._port.out_waiting:
                    if time.monotonic() >= deadline:
                        raise PX6DTimeout('PX6D output drain exceeded deadline')
                    time.sleep(min(.0002, max(0., deadline-time.monotonic())))
            finally:
                diagnostic['flush_elapsed_sec'] = time.monotonic()-flush_started
            if time.monotonic() >= deadline:
                raise PX6DTimeout('PX6D output drain exceeded deadline')
            stage = 'read'
            packet = self._read_packet(size, timeout_sec, expected_command, command[2], deadline=deadline)
            diagnostic['outcome'] = 'ok'
            return packet
        except (PX6DError, KeyboardInterrupt) as exc:
            error = exc
            raise
        except Exception as exc:
            error = PX6DError(f"PX6D serial {stage} failed: {exc}")
            raise error from exc
        finally:
            ended = time.monotonic()
            diagnostic.update(request_end_host_monotonic=ended, request_elapsed_sec=ended-started,
                buffer_bytes_end=len(self._buffer), crc_failures=self._crc_failures-crc_before,
                candidate_rejections=self._candidate_rejections-rejected_before)
            if error is not None:
                self._requires_resynchronization = True
                diagnostic.update(outcome=type(error).__name__, failure_stage=stage)
                if isinstance(error, PX6DTimeout):
                    error.args = (f'PX6D response timeout: budget {timeout_sec:.3f} s, '
                                  f'actual request {ended-started:.6f} s, phase={stage}',)
                try:
                    diagnostic['serial_bytes_at_failure'] = self._port.in_waiting
                except Exception:
                    diagnostic['serial_bytes_at_failure'] = None
                # Inspect only on failure; do not consume evidence or accept a
                # packet after the deadline. This detects unparsed valid bytes.
                # Bound failure reporting too: do not scan a noisy backlog
                # before the runtime gets a chance to request robot braking.
                candidate = self._buffer[:size]
                diagnostic['buffer_candidate_scope'] = 'head only'
                diagnostic['buffer_prefix_hex'] = bytes(self._buffer[:32]).hex()
                diagnostic['valid_packet_buffered_at_failure'] = bool(
                    len(candidate) == size and candidate[:2] == HEADER and candidate[2] == command[2]
                    and candidate[3] == expected_command and crc8(candidate[:-1]) == candidate[-1])
            self._request_history.append(diagnostic)
            self._active_diagnostic = None
            if error is not None:
                error.sensor_diagnostics = self.diagnostics_snapshot()

    def read_wrench(self) -> Wrench:
        try:
            return parse_wrench_packet(
                self._request(GET_FRAME_COMMAND, WRENCH_PACKET_SIZE, self.timeout_sec)
            )
        except PX6DError:
            raise
        except Exception as exc:
            raise PX6DError(f"PX6D frame parsing failed: {exc}") from exc

    def close(self) -> None:
        if self._port is not None:
            try:
                self._port.close()
            finally:
                self._port = None
                self._last_request_started = None

    def __enter__(self) -> "PX6DReader":
        self.connect()
        return self

    def __exit__(self, *_args) -> None:
        self.close()


@dataclass
class MockPX6DReader:
    """Deterministic state-aware source used only for offline dry-run."""

    sample_rate_hz: float = 100.0
    contact_force: float = 4.0
    baseline_samples: int = 50

    def __post_init__(self) -> None:
        self._state = "BASELINE"
        self._samples_in_state = 0
        self._angle = 0.0
        self.firmware = "mock"

    def connect(self) -> None:
        return None

    def set_context(self, state: str) -> None:
        if state != self._state:
            self._state = state
            self._samples_in_state = 0

    def read_wrench(self) -> Wrench:
        self._samples_in_state += 1
        noise = 0.015 * math.sin(self._samples_in_state * 0.37)
        active = (
            self._state == "SEARCH" and self._samples_in_state >= int(0.5 * self.sample_rate_hz)
        ) or (
            self._state == "PROBE" and self._samples_in_state >= int(0.12 * self.sample_rate_hz)
        )
        if active:
            if self._state == "PROBE" and self._samples_in_state == int(0.12 * self.sample_rate_hz):
                self._angle += math.radians(8.0)
            onset_sample = (
                int(0.5 * self.sample_rate_hz)
                if self._state == "SEARCH"
                else int(0.12 * self.sample_rate_hz)
            )
            # A finite 0.2 s ramp keeps the dry-run source physically plausible
            # enough to exercise the same force-rate guard as real probes.
            ramp_samples = max(1, int(0.2 * self.sample_rate_hz))
            ramp = min(1.0, (self._samples_in_state - onset_sample + 1) / ramp_samples)
            force = self.contact_force * ramp
            fx = force * math.cos(self._angle) + noise
            fy = force * math.sin(self._angle) - noise
            return Wrench(fx, fy, 0.1, 0.01, -0.01, 0.005)
        return Wrench(noise, -noise, 0.02, 0.001, -0.001, 0.0)

    def close(self) -> None:
        return None

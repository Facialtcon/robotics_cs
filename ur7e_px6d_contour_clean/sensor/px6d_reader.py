"""Direct serial reader for a PaXini PX6D sensor (no ROS dependency).

Protocol source: PaXini PX6D user manual, USB section 5.2.  The byte layout
and measured test frames are also cross-checked against the existing lab
driver in this workspace.
"""

from __future__ import annotations

import math
import struct
import time
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
        self.firmware = "unknown"

    def connect(self) -> None:
        try:
            import serial

            self._port = serial.Serial(
                self.device,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0,
                write_timeout=0.5,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            # Opening this USB CDC port can reset or temporarily stall the
            # PX6D firmware.  v1.0.1 on the lab sensor needs a settling delay.
            time.sleep(self.startup_delay_sec)
            self._port.reset_input_buffer()
            self._port.reset_output_buffer()
            self.firmware = parse_version_packet(
                self._request(GET_VERSION_COMMAND, VERSION_PACKET_SIZE, 2.0)
            )
        except Exception as exc:
            self.close()
            raise PX6DError(f"cannot initialize PX6D at {self.device}: {exc}") from exc

    def _extract_packet(self, size: int) -> bytes | None:
        while True:
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
                del self._buffer[0]
                continue
            del self._buffer[:size]
            return candidate

    def _read_packet(self, size: int, timeout_sec: float) -> bytes:
        if self._port is None:
            raise PX6DError("PX6D is not connected")
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                waiting = self._port.in_waiting
                if waiting:
                    self._buffer.extend(self._port.read(waiting))
                    packet = self._extract_packet(size)
                    if packet is not None:
                        return packet
            except Exception as exc:
                raise PX6DError(f"PX6D serial read failed: {exc}") from exc
            time.sleep(0.0002)
        raise PX6DTimeout(f"PX6D response timeout after {timeout_sec:.3f} s")

    def resynchronize_after_timeout(self) -> None:
        """Explicit stationary-startup recovery; normal reads never retry.

        Discard the partial response and verify a different response type
        before collecting a new bias window. No hardware zero or reconnect.
        """
        if self._port is None:
            raise PX6DError('PX6D is not connected')
        self._buffer.clear()
        try:
            self._port.reset_input_buffer()
        except Exception as exc:
            raise PX6DError(f'PX6D input reset failed: {exc}') from exc
        self.firmware = parse_version_packet(
            self._request(GET_VERSION_COMMAND, VERSION_PACKET_SIZE, self.timeout_sec))
        self._buffer.clear()

    def _request(self, command: bytes, size: int, timeout_sec: float) -> bytes:
        if self._port is None:
            raise PX6DError("PX6D is not connected")
        try:
            if self._last_request_started is not None:
                remaining = self._request_interval_sec - (
                    time.monotonic() - self._last_request_started
                )
                if remaining > 0.0:
                    time.sleep(remaining)
            self._last_request_started = time.monotonic()
            self._port.write(command)
            self._port.flush()
        except Exception as exc:
            raise PX6DError(f"PX6D serial write failed: {exc}") from exc
        return self._read_packet(size, timeout_sec)

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

"""Active-TCP normalization, comparison, and read-only RTDE lookup."""

from __future__ import annotations

from typing import Sequence

import numpy as np


class TCPIdentityError(ValueError):
    """Raised when an active TCP cannot be read or validated."""


def normalize_tcp_offset(values: Sequence[float], name: str = "TCP offset") -> list[float]:
    try:
        offset = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise TCPIdentityError(f"{name} must contain six finite values") from exc
    if offset.shape != (6,) or not np.all(np.isfinite(offset)):
        raise TCPIdentityError(f"{name} must contain six finite values")
    return offset.tolist()


def tcp_offsets_match(left, right, tolerance: float) -> bool:
    if float(tolerance) < 0.0:
        raise TCPIdentityError("TCP offset tolerance cannot be negative")
    return bool(
        np.allclose(
            normalize_tcp_offset(left, "left TCP offset"),
            normalize_tcp_offset(right, "right TCP offset"),
            atol=float(tolerance),
            rtol=0.0,
        )
    )


def read_active_tcp_offset(control) -> list[float]:
    """Read getTCPOffset from an already-connected RTDE Control interface."""
    if not hasattr(control, "getTCPOffset"):
        raise TCPIdentityError(
            "this ur-rtde build cannot read the active TCP; refusing unverified motion"
        )
    return normalize_tcp_offset(control.getTCPOffset(), "active TCP offset")


def read_tcp_offset_readonly(robot_ip, *, timeout=2.0, connect=None):
    """Read UR Cartesian Info on read-only port 30012; never upload a script.

    Protocol: https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/primary-secondary-guide.html
    """
    import socket
    import struct
    import time
    deadline = time.monotonic() + timeout
    connect = connect or socket.create_connection
    with connect((robot_ip, 30012), timeout=timeout) as stream:
        def receive_exact(count):
            data = bytearray()
            while len(data) < count:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise TCPIdentityError('read-only TCP offset timed out')
                stream.settimeout(remaining)
                chunk = stream.recv(count-len(data))
                if not chunk:
                    raise TCPIdentityError('read-only TCP stream closed')
                data.extend(chunk)
            return bytes(data)
        while time.monotonic() < deadline:
            size = struct.unpack('!I', receive_exact(4))[0]
            if not 5 <= size <= 1024*1024:
                raise TCPIdentityError('invalid UR state packet length')
            packet = receive_exact(size-4)
            if packet[0] != 16:
                continue
            offset = 1
            while offset < len(packet):
                if len(packet)-offset < 5:
                    raise TCPIdentityError('truncated UR subpacket')
                length, kind = struct.unpack_from('!IB', packet, offset)
                if length < 5 or offset+length > len(packet):
                    raise TCPIdentityError('invalid UR subpacket length')
                if kind == 4:
                    if length < 101:
                        raise TCPIdentityError('truncated Cartesian Info')
                    values = struct.unpack_from('!12d', packet, offset+5)
                    return normalize_tcp_offset(values[6:])
                offset += length
    raise TCPIdentityError('Cartesian Info not received before deadline')

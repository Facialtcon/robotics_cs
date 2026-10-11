"""Read-only e-Series Dashboard control-mode query; never changes mode.

Protocol: https://www.universal-robots.com/manuals/EN/HTML/SW5_23/Content/prod-dashboard/Dashboard_table.htm
"""
from dataclasses import dataclass
import socket
import time


@dataclass(frozen=True)
class ControlMode:
    mode: str
    detail: str


def read_control_mode(robot_ip, *, timeout=2.0, connect=None):
    """Only send 'is in remote control'; failures remain explicitly unknown."""
    connect = connect or socket.create_connection
    deadline = time.monotonic() + timeout
    try:
        with connect((robot_ip, 29999), timeout=timeout) as stream:
            pending = bytearray()
            def line():
                while b'\n' not in pending:
                    remaining = deadline-time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError('Dashboard 查询超时')
                    stream.settimeout(remaining)
                    chunk = stream.recv(1024)
                    if not chunk:
                        raise ConnectionError('Dashboard 连接关闭，未收到完整回复')
                    pending.extend(chunk)
                    if len(pending) > 4096:
                        raise ValueError('Dashboard 回复过长')
                value, _, rest = pending.partition(b'\n')
                pending[:] = rest
                return value.decode('utf-8').strip()
            greeting = line()
            if not greeting.startswith('Connected: Universal Robots Dashboard Server'):
                return ControlMode('unknown', f'Dashboard 握手异常：{greeting}')
            stream.sendall(b'is in remote control\n')
            reply = line()
            if reply.lower() == 'true':
                return ControlMode('remote', 'Dashboard 回复 true，已确认 Remote')
            if reply.lower() == 'false':
                return ControlMode('local', 'Dashboard 回复 false：Local 或未启用 Remote')
            return ControlMode('unknown', f'Dashboard 无法识别的回复：{reply}')
    except (OSError, ValueError) as exc:
        return ControlMode('unknown', f'Dashboard 查询失败：{type(exc).__name__}: {exc}')

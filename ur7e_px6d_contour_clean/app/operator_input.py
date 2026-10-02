"""Non-blocking operator input; kept outside the contour algorithm."""

from __future__ import annotations

import os
import select
import sys
import termios
import tty


def confirm_enter(message: str, *, read_line=None) -> bool:
    """Each action needs a fresh Enter; words and Q/Esc do not authorize it."""
    prompt = message + ' 按 Enter 确认，Q/Esc 取消：'
    if read_line is not None:
        return read_line(prompt) == ''
    with OperatorKeyboard() as keyboard:
        return keyboard.read_line(prompt) == ''


class OperatorKeyboard:
    def __init__(self):
        self.enabled = bool(sys.stdin.isatty() and os.name == "posix")
        self._original = None
        self.on_wait = None

    def __enter__(self) -> "OperatorKeyboard":
        if self.enabled:
            self._original = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
        return self

    def poll(self) -> str | None:
        if not self.enabled:
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None
        value = os.read(sys.stdin.fileno(), 1)
        if value == b"\x1b":
            return "ESC"
        try:
            key = value.decode("utf-8").lower()
        except UnicodeDecodeError:
            return None
        return "Q" if key == "q" else None

    def read_line(self, prompt: str) -> str:
        """Read a fresh confirmation key, never a buffered newline from input()."""
        if not self.enabled:
            raise EOFError('确认需要前台交互终端，不能通过管道预先输入回车。')
        fd = sys.stdin.fileno()
        previous = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            termios.tcflush(fd, termios.TCIFLUSH)
            print(prompt, end='', flush=True)
            while True:
                if self.on_wait is not None:
                    self.on_wait()
                readable, _, _ = select.select([fd], [], [], .01)
                if not readable:
                    continue
                key = os.read(fd, 1)
                if not key or key == b'\x04':
                    raise EOFError('confirmation input closed')
                print(flush=True)
                if key == b'\x03':
                    raise KeyboardInterrupt
                return '' if key in (b'\r', b'\n') else key.decode('utf-8', errors='replace')
        finally:
            # Discard repeated Enter/CRLF before restoring the caller's mode.
            termios.tcflush(fd, termios.TCIFLUSH)
            termios.tcsetattr(fd, termios.TCSANOW, previous)

    def __exit__(self, *_args) -> None:
        if self.enabled and self._original is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._original)

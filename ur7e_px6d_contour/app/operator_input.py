"""Non-blocking operator input; kept outside the contour algorithm."""

from __future__ import annotations

import os
import select
import sys
import termios
import tty


class OperatorKeyboard:
    def __init__(self):
        self.enabled = bool(sys.stdin.isatty() and os.name == "posix")
        self._original = None

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
        """Temporarily restore visible, editable line input inside cbreak mode."""
        if not self.enabled:
            return input(prompt)
        fd = sys.stdin.fileno()
        previous = termios.tcgetattr(fd)
        line_mode = termios.tcgetattr(fd)
        line_mode[3] |= termios.ICANON | termios.ECHO | termios.ISIG
        try:
            # Do not flush input that the operator has already typed.
            termios.tcsetattr(fd, termios.TCSANOW, line_mode)
            return input(prompt)
        finally:
            # Also restore Q/Esc polling after EOF or Ctrl+C. The enclosing
            # context restores the original terminal when the run exits.
            termios.tcsetattr(fd, termios.TCSANOW, previous)

    def __exit__(self, *_args) -> None:
        if self.enabled and self._original is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._original)

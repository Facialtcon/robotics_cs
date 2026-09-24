"""Real pseudo-terminal input checks; no robot or sensor access."""
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import time

import pytest


CHILD = '''
import termios
import sys
import time
from app.operator_input import OperatorKeyboard
original = termios.tcgetattr(sys.stdin.fileno())
try:
    with OperatorKeyboard() as keyboard:
        try:
            value = keyboard.read_line('START> ')
            print('ACCEPTED=' + value, flush=True)
        except (KeyboardInterrupt, EOFError) as exc:
            print('CANCELLED=' + type(exc).__name__, flush=True)
        attrs = termios.tcgetattr(sys.stdin.fileno())
        print('CBREAK=' + str(not bool(attrs[3] & termios.ICANON)), flush=True)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            key = keyboard.poll()
            if key:
                print('KEY=' + key, flush=True)
                break
            time.sleep(.005)
finally:
    print('RESTORED=' + str(termios.tcgetattr(sys.stdin.fileno()) == original), flush=True)
'''


@pytest.mark.parametrize('ending', ['enter', 'interrupt', 'eof'])
def test_confirmation_echo_editing_and_keyboard_restoration(ending):
    master, slave = pty.openpty()
    proc = subprocess.Popen([sys.executable, '-c', CHILD], stdin=slave, stdout=slave,
                            stderr=slave, cwd=Path(__file__).resolve().parents[1])
    output = bytearray()
    def until(marker):
        deadline = time.monotonic()+3
        while marker not in output and time.monotonic() < deadline:
            ready, _, _ = select.select([master], [], [], .05)
            if ready:
                output.extend(os.read(master, 4096))
            if proc.poll() is not None: break
        assert marker in output, output.decode(errors='replace')
    try:
        until(b'START> ')
        os.write(master, b'STARX')
        until(b'STARX')  # visible before Enter, not just a printed result
        if ending == 'enter':
            os.write(master, b'\x7fT\n')
            until(b'ACCEPTED=START')
        elif ending == 'interrupt':
            proc.send_signal(signal.SIGINT)
            until(b'CANCELLED=KeyboardInterrupt')
        else:
            os.write(master, b'\x15\x04')  # erase draft, then EOF
            until(b'CANCELLED=EOFError')
        until(b'CBREAK=True')
        os.write(master, b'q')  # no Enter needed after confirmation
        until(b'KEY=Q')
        until(b'RESTORED=True')
        assert proc.wait(timeout=3) == 0
    finally:
        if proc.poll() is None:
            proc.kill(); proc.wait(timeout=3)
        os.close(master); os.close(slave)

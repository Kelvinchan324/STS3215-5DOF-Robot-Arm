"""Minimal raw-terminal key reader for interactive motor control scripts."""

import os
import select
import sys
import termios
import tty


class RawTerminal:
    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError("This control script must run in a terminal window")
        self.descriptor = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.descriptor)
        tty.setraw(self.descriptor)
        return self

    def __exit__(self, _exception_type, _exception, _traceback):
        termios.tcsetattr(self.descriptor, termios.TCSADRAIN, self.old_settings)

    def read_key(self, timeout=0.0):
        ready, _, _ = select.select([self.descriptor], [], [], timeout)
        if not ready:
            return None
        return os.read(self.descriptor, 1).decode("utf-8", errors="ignore").lower()

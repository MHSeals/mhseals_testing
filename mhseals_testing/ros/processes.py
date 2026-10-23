"""Supervised child processes (launch files, drivers, the bag recorder).

Children get their own process group and /dev/null stdin, so they can't
steal TUI keystrokes, and their output goes to per-process log files next
to the session bag. Stopping sends SIGINT first, so ros2 bag and ros2
launch shut down cleanly, then escalates.
"""

import ctypes
import os
from pathlib import Path
import signal
import subprocess
import sys

PR_SET_PDEATHSIG = 1


def die_with_parent():
    """Linux: SIGINT the child when the suite dies, however it dies (even kill -9).

    Without this, a killed suite leaves the thruster driver running with its outputs
    armed; with it, the driver gets the same clean shutdown as a normal disarm.
    """
    if sys.platform.startswith('linux'):
        ctypes.CDLL(None, use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGINT)


class ProcessManager:
    def __init__(self, log_directory, popen=subprocess.Popen):
        self.log_directory = Path(log_directory)
        self.popen = popen
        self.processes = {}
        self.logs = {}

    def start(self, name, command):
        if self.running(name):
            raise RuntimeError(f'{name} is already running')
        self.stop(name)
        self.log_directory.mkdir(parents=True, exist_ok=True)
        log = (self.log_directory / f'{name}.log').open('a', encoding='utf-8')
        self.logs[name] = log
        # Unbuffered, so a child's log is current while it runs, not only after it exits.
        environment = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_LOGGING_BUFFERED_STREAM='0')
        self.processes[name] = self.popen(
            [str(part) for part in command], start_new_session=True, env=environment,
            preexec_fn=die_with_parent, stdin=subprocess.DEVNULL, stdout=log,
            stderr=subprocess.STDOUT)
        return self.processes[name]

    def running(self, name):
        process = self.processes.get(name)
        return process is not None and process.poll() is None

    def last_line(self, name):
        """The last non-empty line of a child's log, or '' (why it died)."""
        try:
            lines = (self.log_directory / f'{name}.log').read_text(errors='replace').splitlines()
        except OSError:
            return ''
        return next((line.strip() for line in reversed(lines) if line.strip()), '')

    def exited(self):
        """Names of children that stopped on their own."""
        return [name for name, process in self.processes.items() if process.poll() is not None]

    def stop(self, name, timeout=10):
        process = self.processes.pop(name, None)
        if process is not None and process.poll() is None:
            for signum, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 3), (signal.SIGKILL, None)):
                try:
                    os.killpg(process.pid, signum)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=wait)
                    break
                except subprocess.TimeoutExpired:
                    continue
        log = self.logs.pop(name, None)
        if log is not None:
            log.close()

    def stop_all(self, last=('bag',)):
        """Stop everything, the recorder last so it captures the shutdown."""
        for name in sorted(self.processes, key=lambda name: name in last):
            self.stop(name, timeout=20 if name in last else 10)

    def summary(self):
        return '\n'.join(
            f"{name}: {'RUNNING' if process.poll() is None else f'EXITED {process.returncode}'}"
            f' | {self.log_directory / (name + ".log")}'
            for name, process in self.processes.items()) or 'None'


def unique_path(path):
    """``path``, or ``path_01``, ``path_02``... if it already exists."""
    path, candidate, index = Path(path), Path(path), 1
    while candidate.exists():
        candidate = path.with_name(f'{path.name}_{index:02d}')
        index += 1
    return candidate


def bag_command(path):
    # -a records every topic, including /mhseals_testing/events, so results
    # can be re-scored from the bag alone. Jazzy's default storage is MCAP.
    return ['ros2', 'bag', 'record', '-a', '-o', str(path)]

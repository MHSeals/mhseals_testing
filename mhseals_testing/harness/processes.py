"""Supervised child processes (launch files, drivers, the bag recorder), ROS-free.

Every child runs under a keeper (harness/keeper.py): its own process group, /dev/null
stdin, output in a per-process log next to the session bag. The keeper stops the whole
group when the owner dies (kill -9 included) and writes the return code to ``<name>.exit``.
Stopping signals the group, SIGINT first so ros2 bag and ros2 launch shut down cleanly.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

from mhseals_testing.harness.registry import alive


class Child:
    """A keeper-managed child: pid/pgid of the real process, the keeper's exit file.

    ``keeper`` is the Popen of a keeper we started (reaped by poll); an adopted child
    has none and is judged by its exit file and liveness.
    """

    def __init__(self, name, pid, exit_file, keeper=None, log=None):
        self.name, self.pid, self.pgid = name, pid, pid
        self.exit_file, self.keeper, self.log = Path(exit_file), keeper, log
        self.returncode = None

    def poll(self):
        if self.returncode is None:
            if self.keeper is not None:
                self.keeper.poll()
            try:
                self.returncode = int(self.exit_file.read_text())
            except (OSError, ValueError):
                # Adopted, or the keeper itself died (kill -9): no exit file will come.
                if (self.keeper is None or self.keeper.returncode is not None) and not alive(self.pid):
                    self.returncode = -1
        return self.returncode

    def wait(self, timeout=None):
        deadline = None if timeout is None else time.time() + timeout
        while self.poll() is None:
            if deadline is not None and time.time() > deadline:
                raise subprocess.TimeoutExpired(self.name, timeout)
            time.sleep(0.05)
        return self.returncode

    def signal(self, signum):
        os.killpg(self.pgid, signum)

    def terminate(self):
        self.signal(signal.SIGTERM)

    def record(self):
        return {'pid': self.pid, 'pgid': self.pgid, 'keeper': self.keeper.pid if self.keeper else None,
                'log': str(self.log), 'exit': str(self.exit_file)}


class ProcessManager:
    def __init__(self, log_directory, registry=None, popen=subprocess.Popen, grace=10,
                 last=('bag',), last_grace=20):
        self.log_directory = Path(log_directory)
        self.registry, self.popen, self.grace = registry, popen, grace
        self.last, self.last_grace = tuple(last), last_grace  # the recorder gets longer (tui.md §6)
        self.processes = {}

    def start(self, name, command):
        if self.running(name):
            raise RuntimeError(f'{name} is already running')
        self.stop(name)
        self.log_directory.mkdir(parents=True, exist_ok=True)
        log, exit_file = self.log_directory / f'{name}.log', self.log_directory / f'{name}.exit'
        for stale in (exit_file, exit_file.with_suffix('.pid')):
            stale.unlink(missing_ok=True)
        # Unbuffered, so a child's log is current while it runs, not only after it exits.
        environment = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_LOGGING_BUFFERED_STREAM='0')
        argv = [sys.executable, '-m', 'mhseals_testing.harness.keeper', '--owner', str(os.getpid()),
                '--log', str(log), '--exit', str(exit_file), '--grace', str(self.last_grace if name in self.last else self.grace)]
        if self.registry is not None:
            argv += ['--registry', str(self.registry.path)]
        keeper = self.popen(argv + ['--', *[str(part) for part in command]], start_new_session=True,
                            env=environment, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
        pid_file = exit_file.with_suffix('.pid')
        deadline = time.time() + 5
        while not pid_file.exists() and keeper.poll() is None and time.time() < deadline:
            time.sleep(0.01)
        try:
            pid = int(pid_file.read_text())
        except (OSError, ValueError):
            raise RuntimeError(f'keeper for {name} did not start the child') from None
        self.processes[name] = Child(name, pid, exit_file, keeper, log)
        return self.processes[name]

    def adopt(self, name, record):
        """Take over a child another instance started (a registry record)."""
        self.processes[name] = Child(name, record['pid'], record['exit'], None, record.get('log'))
        return self.processes[name]

    def records(self):
        return {name: child.record() for name, child in self.processes.items()}

    def release(self, names):
        """Forget ``names`` without stopping them (a handover: their keepers wait for the adopter)."""
        for name in names:
            self.processes.pop(name, None)

    def running(self, name):
        process = self.processes.get(name)
        return process is not None and process.poll() is None

    def returncode(self, name):
        """Exit code of a child that stopped on its own, else None (running or unknown)."""
        process = self.processes.get(name)
        return None if process is None else process.poll()

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
        if process is None:
            return
        for signum, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 3), (signal.SIGKILL, 5)):
            if process.poll() is not None:
                break
            try:
                process.signal(signum)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=wait)
            except subprocess.TimeoutExpired:
                continue

    def stop_all(self, last=None):
        """Stop everything, the recorder last so it captures the shutdown."""
        last = self.last if last is None else last
        for name in sorted(self.processes, key=lambda name: name in last):
            self.stop(name, timeout=self.last_grace if name in last else self.grace)

    def summary(self):
        return '\n'.join(
            f"{name}: {'RUNNING' if process.poll() is None else f'EXITED {process.returncode}'}"
            f' | {self.log_directory / (name + ".log")}'
            for name, process in self.processes.items()) or 'None'


def unique_path(path):
    """``path``, or ``path_01``, ``path_02``... if it is taken, claimed atomically.

    ros2 bag only creates its directory later, so existence is no claim: two suites
    started together would share one name. The session's ``logs/<name>`` directory is
    created with mkdir, which fails for the second claimant, and the name is retried.
    """
    path, index = Path(path), 0
    (path.parent / 'logs').mkdir(parents=True, exist_ok=True)
    while True:
        candidate = path if index == 0 else path.with_name(f'{path.name}_{index:02d}')
        index += 1
        if candidate.exists():
            continue
        try:
            (path.parent / 'logs' / candidate.name).mkdir()
            return candidate
        except FileExistsError:
            continue


def bag_filters():
    """The (exclude_regex, exclude_types) from config/bags.yaml (source tree, else the installed share)."""
    path = Path(__file__).resolve().parents[2] / 'config' / 'bags.yaml'
    if not path.exists():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('mhseals_testing')) / 'config' / 'bags.yaml'
    config = yaml.safe_load(path.read_text())
    return config['exclude_regex'], list(config.get('exclude_types', []))


def bag_command(path):
    # Every topic except the heavy ones, so /mhseals_testing/events is kept and results
    # can be re-scored from the bag alone. Not -a: it records images and clouds at
    # ~50 MB/s. The name regex and the message-type list both apply. Jazzy's default storage is MCAP.
    regex, types = bag_filters()
    type_args = ['--exclude-topic-types', *types] if types else []
    return ['ros2', 'bag', 'record', '--all-topics', '--exclude-regex', regex, *type_args,
            '-o', str(path)]

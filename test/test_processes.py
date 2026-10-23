import os
from pathlib import Path
import re
import subprocess
import sys
import time

from mhseals_testing.ros.processes import ProcessManager, bag_command, unique_path


def test_children_log_stop_cleanly_and_report_exit(tmp_path):
    manager = ProcessManager(tmp_path / 'logs')
    manager.start('sleeper', [sys.executable, '-c', 'import time; print("up", flush=True); time.sleep(60)'])
    manager.start('quick', [sys.executable, '-c', 'raise SystemExit(3)'])
    manager.processes['quick'].wait(timeout=5)
    assert manager.running('sleeper') and manager.exited() == ['quick']
    assert 'EXITED 3' in manager.summary()
    manager.stop_all()
    assert manager.processes == {} and manager.summary() == 'None'
    assert 'up' in (tmp_path / 'logs' / 'sleeper.log').read_text()


def test_unique_paths_and_bag_records_everything(tmp_path):
    (tmp_path / 'session').mkdir()
    (tmp_path / 'session_01').mkdir()
    assert unique_path(tmp_path / 'session').name == 'session_02'
    assert unique_path(tmp_path / 'other').name == 'other'


def test_bag_command_excludes_heavy_topics_instead_of_recording_all(tmp_path):
    command = bag_command(tmp_path / 'b')
    assert '-a' not in command and '--all-topics' in command
    regex = re.compile(command[command.index('--exclude-regex') + 1])
    for heavy in ('/front_camera/depth/image', '/points', '/costmap/costmap'):
        assert regex.fullmatch(heavy)
    for kept in ('/mhseals_testing/events', '/mavros/local_position/odom'):
        assert not regex.fullmatch(kept)
    assert command[-2:] == ['-o', str(tmp_path / 'b')]


def test_child_dies_with_a_killed_parent_that_ignored_sigint(tmp_path):
    # A suite started with & or from a script inherits SIGINT=SIG_IGN; the child must not.
    parent = (
        'import signal, subprocess, sys, time\n'
        'signal.signal(signal.SIGINT, signal.SIG_IGN)\n'
        'from mhseals_testing.ros.processes import ProcessManager\n'
        f'manager = ProcessManager({str(tmp_path / "logs")!r})\n'
        'child = manager.start("sleeper", [sys.executable, "-c", "import time; time.sleep(60)"])\n'
        'print(child.pid, flush=True)\n'
        'time.sleep(60)\n')
    holder = subprocess.Popen([sys.executable, '-c', parent], stdout=subprocess.PIPE, text=True)
    child = int(holder.stdout.readline())
    holder.kill()
    holder.wait()
    deadline = time.time() + 5
    while time.time() < deadline and os.path.exists(f'/proc/{child}') and 'Z' not in _state(child):
        time.sleep(0.1)
    assert not os.path.exists(f'/proc/{child}') or 'Z' in _state(child)


def test_stop_interrupts_a_child_when_the_suite_ignores_signals(tmp_path):
    ignored = [signal.signal(signum, signal.SIG_IGN) for signum in (signal.SIGINT, signal.SIGTERM)]
    manager = ProcessManager(tmp_path / 'logs')
    try:
        child = manager.start('sleeper', [sys.executable, '-c', 'import time; time.sleep(60)'])
        started = time.time()
        manager.stop('sleeper', timeout=3)
        assert child.returncode == -signal.SIGINT
        assert time.time() - started < 3
    finally:
        signal.signal(signal.SIGINT, ignored[0])
        signal.signal(signal.SIGTERM, ignored[1])


def _state(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0]
    except OSError:
        return 'Z'

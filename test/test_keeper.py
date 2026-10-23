"""The keeper stops its child (and the child's whole group) when the owner is gone."""

import os
import signal
import subprocess
import sys
import time

from mhseals_testing.harness.processes import ProcessManager
from mhseals_testing.harness.registry import Registry, alive

SLEEP = 'import time; time.sleep(60)'
# Like `ros2 run`: a wrapper that starts the real node and ignores SIGINT.
WRAPPER = ('import signal, subprocess, sys, time\n'
           'signal.signal(signal.SIGINT, signal.SIG_IGN)\n'
           f'g = subprocess.Popen([sys.executable, "-c", {SLEEP!r}])\n'
           'print(g.pid, flush=True); time.sleep(60)')


def start_keeper(tmp_path, command, owner, registry=None, grace=1.0):
    argv = [sys.executable, '-m', 'mhseals_testing.harness.keeper', '--owner', str(owner),
            '--log', str(tmp_path / 'c.log'), '--exit', str(tmp_path / 'c.exit'), '--grace', str(grace)]
    if registry:
        argv += ['--registry', str(registry.path)]
    keeper = subprocess.Popen(argv + ['--', *command], start_new_session=True)
    pid_file = tmp_path / 'c.pid'
    wait_for(lambda: pid_file.exists() and pid_file.read_text())
    return keeper, int(pid_file.read_text())


def wait_for(condition, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if condition():
            return True
        time.sleep(0.05)
    return False


def owner_process():
    """A stand-in owner we can kill: a sleeping process we reap ourselves."""
    return subprocess.Popen([sys.executable, '-c', SLEEP])


def test_child_stops_within_3_s_of_the_owner_dying(tmp_path):
    owner = owner_process()
    keeper, child = start_keeper(tmp_path, [sys.executable, '-c', SLEEP], owner.pid)
    assert alive(child)
    owner.kill()
    owner.wait()
    started = time.time()
    assert wait_for(lambda: not alive(child), 3) and time.time() - started < 3
    keeper.wait(timeout=5)
    assert int((tmp_path / 'c.exit').read_text()) in (-signal.SIGINT, 1)  # 1: stopped during interpreter start


def test_grandchild_under_a_sigint_ignoring_wrapper_is_gone_after_the_owner_is_killed(tmp_path):
    owner = owner_process()
    keeper, wrapper = start_keeper(tmp_path, [sys.executable, '-c', WRAPPER], owner.pid)
    assert wait_for(lambda: (tmp_path / 'c.log').read_text().strip())
    grandchild = int((tmp_path / 'c.log').read_text())
    assert alive(grandchild)
    owner.kill()
    owner.wait()
    try:
        assert wait_for(lambda: not alive(grandchild) and not alive(wrapper), 3)
    finally:
        for pid in (grandchild, wrapper):
            if alive(pid):
                os.kill(pid, signal.SIGKILL)
    keeper.wait(timeout=5)
    assert (tmp_path / 'c.exit').exists()


def test_keeper_sigterm_stops_the_child_and_writes_the_exit_file(tmp_path):
    owner = owner_process()
    keeper, child = start_keeper(tmp_path, [sys.executable, '-c', SLEEP], owner.pid)
    keeper.send_signal(signal.SIGTERM)
    assert keeper.wait(timeout=5) == 0
    assert not alive(child)
    assert int((tmp_path / 'c.exit').read_text()) in (-signal.SIGINT, 1)  # 1: stopped during interpreter start
    owner.kill()
    owner.wait()


def test_handover_keeps_the_child_alive_until_adopted_then_follows_the_new_owner(tmp_path):
    registry = Registry(tmp_path / 'session.json')
    old, new = owner_process(), owner_process()
    registry.claim({'pid': old.pid}, 'bags/x')
    keeper, child = start_keeper(tmp_path, [sys.executable, '-c', SLEEP], old.pid, registry)
    registry.update(handover={'to': new.pid, 'deadline': time.time() + 30})
    old.kill()
    old.wait()
    registry.update(owner=None)
    time.sleep(1.5)
    assert alive(child) and keeper.poll() is None  # survives the handover
    registry.claim({'pid': new.pid}, 'bags/x')
    time.sleep(1.2)
    assert alive(child)
    new.kill()
    new.wait()
    assert wait_for(lambda: not alive(child), 3)
    keeper.wait(timeout=5)


def test_an_expired_handover_stops_the_child(tmp_path):
    registry = Registry(tmp_path / 'session.json')
    registry.claim({'pid': os.getpid()}, 'bags/x')
    registry.update(owner=None, handover={'to': 1, 'deadline': time.time() + 1})
    keeper, child = start_keeper(tmp_path, [sys.executable, '-c', SLEEP], os.getpid(), registry)
    assert wait_for(lambda: not alive(child), 4)
    keeper.wait(timeout=5)


def test_a_killed_keeper_leaves_no_phantom_running_child(tmp_path):
    manager = ProcessManager(tmp_path / 'logs')
    child = manager.start('s', [sys.executable, '-c', SLEEP])
    os.kill(child.keeper.pid, signal.SIGKILL)
    assert wait_for(lambda: not manager.running('s'), 5)
    assert child.returncode is not None
    manager.start('s', [sys.executable, '-c', SLEEP])  # no 'already running'
    manager.stop('s', timeout=3)


def test_the_bag_keeper_gets_the_longer_grace(tmp_path):
    argvs = []

    class Fake:
        pid = 1

        def poll(self):
            return 0

    def popen(argv, **kwargs):
        argvs.append(argv)
        (tmp_path / 'logs' / f'{argv[argv.index("--exit") + 1].rsplit("/", 1)[1][:-5]}.pid').write_text('1')
        return Fake()

    manager = ProcessManager(tmp_path / 'logs', popen=popen)
    (tmp_path / 'logs').mkdir()
    manager.start('bag', ['x'])
    manager.start('launch', ['x'])
    graces = [argv[argv.index('--grace') + 1] for argv in argvs]
    assert graces == ['20', '10']

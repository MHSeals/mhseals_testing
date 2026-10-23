"""The session registry: ownership claims, guests and the runtime directory."""

import json
import os
import signal
import subprocess
import sys

import pytest

from mhseals_testing.harness.processes import ProcessManager
from mhseals_testing.harness.registry import Busy, Registry, alive


def test_alive_is_false_for_a_dead_or_bogus_pid(tmp_path):
    assert alive(os.getpid()) and not alive('x') and not alive(2 ** 30)


def test_claim_refuses_a_live_owner_and_replaces_a_dead_one(tmp_path):
    registry = Registry(tmp_path / 'session.json')
    other = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    registry.claim({'pid': other.pid}, 'bags/x')
    with pytest.raises(Busy) as busy:
        registry.claim({'pid': os.getpid()}, 'bags/y')
    assert busy.value.entry['session'] == 'bags/x'
    other.kill()
    other.wait()
    registry.claim({'pid': os.getpid()}, 'bags/y')
    assert registry.read()['owner']['pid'] == os.getpid()
    registry.add_guest(7, 'boat_trial_7', 'characterize')
    registry.remove_guest(7)
    assert registry.read()['guests'] == {}


def test_release_drops_the_children_unless_they_are_handed_on(tmp_path):
    registry = Registry(tmp_path / 'session.json')
    registry.claim({'pid': os.getpid()}, 's')
    registry.update(children={'bag': {'pid': 1}})
    registry.release(keep_children=True)
    assert registry.read()['owner'] is None and registry.read()['children']
    registry.release(keep_children=False)
    assert registry.read()['children'] == {}


def test_registry_lives_in_the_runtime_dir(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    Registry().claim({'pid': os.getpid()}, 's')
    assert json.loads((tmp_path / 'mhseals' / 'session.json').read_text())['session'] == 's'
    assert oct((tmp_path / 'mhseals').stat().st_mode & 0o777) == '0o700'


class World:
    """Fake clock, pid table and signal delivery shared by the registries of two instances."""

    def __init__(self, tmp_path, alive=()):
        self.now, self.alive, self.signals = 0.0, set(alive), []
        self.path = tmp_path / 'run' / 'session.json'
        self.responder = None

    def registry(self):
        return Registry(self.path, alive=lambda pid: pid in self.alive,
                        clock=lambda: self.now, sleep=self.sleep, kill=self.kill)

    def sleep(self, seconds):
        self.now += seconds

    def kill(self, pid, signum):
        self.signals.append((pid, signum))
        if self.responder:
            self.responder()


def info(pid):
    return {'pid': pid, 'node': f'mhseals_testing_h_{pid}'}


def test_a_dead_owner_is_stale_and_can_be_claimed(tmp_path):
    world = World(tmp_path, alive={1, 2})
    registry = world.registry()
    registry.claim(info(1))
    with pytest.raises(Busy):
        registry.claim(info(2))
    world.alive.discard(1)
    assert registry.stale()
    registry.claim(info(2))
    assert registry.owner()['pid'] == 2


def test_take_over_signals_the_owner_and_claims_once_it_lets_go(tmp_path):
    world = World(tmp_path, alive={1, 2})
    a, b = world.registry(), world.registry()
    a.claim(info(1), 's')
    a.set_children({'bag': {'pid': 9, 'pgid': 9}})
    world.responder = lambda: a.release(1)       # A's SIGUSR1 handler
    assert b.request_handover(2)
    assert world.signals == [(1, signal.SIGUSR1)]
    b.claim(info(2))
    assert b.children() == {'bag': {'pid': 9, 'pgid': 9}}   # detaching kept the children
    assert b.read()['session'] == 's'                        # the adopter continues the session


def test_an_owner_that_never_lets_go_times_out(tmp_path):
    world = World(tmp_path, alive={1, 2})
    registry = world.registry()
    registry.claim(info(1))
    assert not registry.request_handover(2, timeout=1.0)
    assert registry.owner()['pid'] == 1


def test_release_by_pid_leaves_another_owner_alone(tmp_path):
    registry = World(tmp_path, alive={1}).registry()
    registry.claim(info(1))
    registry.release(2)
    assert registry.owner()['pid'] == 1


def test_released_children_keep_running_and_are_adopted(tmp_path):
    registry = Registry(tmp_path / 'session.json')
    registry.claim({'pid': os.getpid()}, 's')
    first = ProcessManager(tmp_path / 'a', registry=registry)
    second = ProcessManager(tmp_path / 'b', registry=registry)
    first.start('bag', [sys.executable, '-c', 'import time; time.sleep(60)'])
    record = first.records()
    first.release(record)
    for name, child in record.items():
        second.adopt(name, child)
    try:
        assert second.running('bag')
        second.stop('bag', timeout=2)
        assert not second.running('bag')
    finally:
        for child in record.values():
            try:
                os.killpg(child['pgid'], signal.SIGKILL)
            except ProcessLookupError:
                pass

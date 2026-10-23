"""The session registry: ownership claims, guests and the runtime directory."""

import json
import os
import subprocess
import sys

import pytest

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

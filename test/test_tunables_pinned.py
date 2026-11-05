"""A tunable moved out of code keeps its old code value: the shipped config equals the constant it replaced.

Each row is (config file, key, the code constant before the move, where the code reads it now). A move adds a
row; a deliberate retune edits the row's value in the same commit as the config.
"""
import importlib
import os

import pytest

from mhseals_testing import tunables
from mhseals_testing.health.config import config_path
from mhseals_testing.strict_yaml import load_strict

MOVED = [
    ('health', 'tune.toasts.lifetime', {'warn': 6.0, 'ok': 3.0, 'fail': None}, 'mhseals_testing.ui.toasts:LIFETIME'),
    ('health', 'tune.toasts.max_shown', 3, 'mhseals_testing.ui.toasts:MAX_SHOWN'),
    ('health', 'tune.toasts.dedupe', 30.0, 'mhseals_testing.ui.toasts:DEDUPE'),
    ('health', 'probes.git.stale', 3600.0, 'mhseals_testing.ui.deploy:STALE'),
    ('fusion_panel', 'tune.perception.loss_warn', 0.15, 'mhseals_testing.ui.perception:LOSS_WARN'),
    ('input', 'tune.idle_s', 1.0, 'mhseals_testing.ui.app.view:idle_s'),
]


def shipped(name, key):
    path = config_path('config', f'{name}.yaml')
    node = load_strict(path.read_text(), path)
    for part in key.split('.'):
        node = node[part]
    return node


@pytest.mark.smoke
@pytest.mark.parametrize('name, key, old, reader', MOVED, ids=[f'{n}.{k}' for n, k, _, _ in MOVED])
def test_shipped_value_is_the_old_code_constant(name, key, old, reader):
    value = shipped(name, key)
    assert value == old and type(value) is type(old) or isinstance(old, float) and value == old
    module, attribute = reader.split(':')
    value = getattr(importlib.import_module(module), attribute)
    assert (value() if callable(value) else value) == old


def test_tune_reads_the_shipped_section_once_while_unchanged():
    assert tunables.tune('health') is tunables.tune('health')
    assert tunables.tune('health') == shipped('health', 'tune')


def test_tune_rereads_an_edited_file_and_keeps_the_last_good_one(tmp_path, monkeypatch):
    """atlas #35: an edit reaches the next tune() (mtime-keyed, keymap.current's rule); a broken edit keeps the last."""
    path = tmp_path / 'demo.yaml'
    path.write_text('tune: {idle_s: 1.0}\n')
    monkeypatch.setattr(tunables, 'config_path', lambda *parts: path)
    monkeypatch.setattr(tunables, 'LIVE', {})
    assert tunables.tune('demo') == {'idle_s': 1.0}
    path.write_text('tune: {idle_s: 2.5}\n')
    os.utime(path, ns=(1, 1))
    assert tunables.tune('demo') == {'idle_s': 2.5}
    path.write_text('tune: {idle_s: 3, idle_s: 4}\n')    # a duplicate key: strict refuses it
    os.utime(path, ns=(2, 2))
    assert tunables.tune('demo') == {'idle_s': 2.5}

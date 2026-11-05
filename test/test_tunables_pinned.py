"""A tunable moved out of code keeps its old code value: the shipped config equals the constant it replaced.

Each row is (config file, key, the code constant before the move, where the code reads it now). A move adds a
row; a deliberate retune edits the row's value in the same commit as the config.
"""
import importlib

import pytest

from mhseals_testing import tunables
from mhseals_testing.strict_yaml import load_strict

MOVED = [
    ('health', 'tune.toasts.lifetime', {'warn': 6.0, 'ok': 3.0, 'fail': None}, 'mhseals_testing.ui.toasts:LIFETIME'),
    ('health', 'tune.toasts.max_shown', 3, 'mhseals_testing.ui.toasts:MAX_SHOWN'),
    ('health', 'tune.toasts.dedupe', 30.0, 'mhseals_testing.ui.toasts:DEDUPE'),
    ('health', 'probes.git.stale', 3600.0, 'mhseals_testing.ui.deploy:STALE'),
    ('fusion_panel', 'tune.perception.loss_warn', 0.15, 'mhseals_testing.ui.perception:LOSS_WARN'),
]


def shipped(name, key):
    path = tunables.config_dir() / f'{name}.yaml'
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
    assert getattr(importlib.import_module(module), attribute) == old


def test_tune_reads_the_shipped_section_once():
    tunables.tune.cache_clear()
    assert tunables.tune('health') is tunables.tune('health')
    assert tunables.tune('health') == shipped('health', 'tune')

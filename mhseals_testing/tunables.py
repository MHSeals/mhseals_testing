"""Tunables out of code: the ``tune:`` section of one of this package's shipped config files.

``tune('health')`` is ``config/health.yaml``'s ``tune`` mapping (source tree, else the installed
share), read once through load_strict, so its schema fragment checks it. A key moved here keeps
its old code value in the shipped file (test/test_tunables_pinned.py).
"""
from functools import lru_cache

from mhseals_testing.health.config import config_path
from mhseals_testing.strict_yaml import load_strict


@lru_cache(maxsize=None)
def tune(name):
    """The ``tune`` mapping of config/<name>.yaml ({} when it has none)."""
    path = config_path('config', f'{name}.yaml')
    section = (load_strict(path.read_text(), path) or {}).get('tune') or {}
    if not isinstance(section, dict):
        raise ValueError(f'{path}: tune must be a mapping')
    return section

"""Tunables out of code: the ``tune:`` section of one of this package's shipped config files.

``tune('health')`` is ``config/health.yaml``'s ``tune`` mapping (source tree, else the installed
share), read through load_strict, so its schema fragment checks it, and read again when the file's
mtime changes (keymap.current's rule), so an edit reaches the next call. A file made invalid keeps
the last good section. A key moved here keeps its old code value in the shipped file
(test/test_tunables_pinned.py).
"""
import yaml

from mhseals_testing.health.config import config_path
from mhseals_testing.strict_yaml import load_strict

LIVE = {}    # name -> (mtime_ns, section)


def tune(name):
    """The ``tune`` mapping of config/<name>.yaml ({} when it has none)."""
    path = config_path('config', f'{name}.yaml')
    stamp = path.stat().st_mtime_ns
    if name in LIVE and LIVE[name][0] == stamp:
        return LIVE[name][1]
    try:
        section = (load_strict(path.read_text(), path) or {}).get('tune') or {}
        if not isinstance(section, dict):
            raise ValueError(f'{path}: tune must be a mapping')
    except (ValueError, yaml.YAMLError):
        if name not in LIVE:
            raise
        section = LIVE[name][1]
    LIVE[name] = (stamp, section)
    return section

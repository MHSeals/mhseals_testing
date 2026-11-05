"""Tunables out of code: the ``tune:`` section of one of this package's shipped config files.

``tune('health')`` is ``config/health.yaml``'s ``tune`` mapping (source tree, else the installed
share), read through load_strict, so its schema fragment checks it, and read again when the file's
mtime changes (keymap.current's rule, looked at every RECHECK_S), so an edit reaches the next call. A file
made invalid or missing keeps the last good section. A key moved here keeps its old code value in the
shipped file (test/test_tunables_pinned.py).
"""
import time

import yaml

from mhseals_testing.health.config import config_path
from mhseals_testing.strict_yaml import load_strict

LIVE = {}    # name -> (checked, mtime_ns, section)
RECHECK_S = 0.5    # how often tune() looks at a file's mtime (keymap.RECHECK_S)


def tune(name):
    """The ``tune`` mapping of config/<name>.yaml ({} when it has none). A missing, unreadable or invalid
    file keeps the last good section (it raises only when there is none yet)."""
    now = time.monotonic()
    if name in LIVE and now - LIVE[name][0] < RECHECK_S:
        return LIVE[name][2]
    path = config_path('config', f'{name}.yaml')
    try:
        stamp = path.stat().st_mtime_ns
        if name in LIVE and LIVE[name][1] == stamp:
            section = LIVE[name][2]
        else:
            section = (load_strict(path.read_text(), path) or {}).get('tune') or {}
            if not isinstance(section, dict):
                raise ValueError(f'{path}: tune must be a mapping')
    except (OSError, ValueError, yaml.YAMLError):
        if name not in LIVE:
            raise
        stamp, section = LIVE[name][1:]
    LIVE[name] = (now, stamp, section)
    return section

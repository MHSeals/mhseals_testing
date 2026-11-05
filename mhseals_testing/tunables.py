"""Tunables out of code: the ``tune:`` section of one of this package's shipped config files.

``tune('health')`` is ``config/health.yaml``'s ``tune`` mapping (source tree, else the installed
share), read once through load_strict, so its schema fragment checks it. A key moved here keeps
its old code value in the shipped file (test/test_tunables_pinned.py).
"""
from functools import lru_cache
from pathlib import Path

from mhseals_testing.strict_yaml import load_strict


def config_dir():
    """This package's config directory: the source tree's, else the installed share's."""
    source = Path(__file__).resolve().parents[1] / 'config'
    if source.is_dir():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('mhseals_testing')) / 'config'
    except (ImportError, KeyError, LookupError):
        return source


@lru_cache(maxsize=None)
def tune(name):
    """The ``tune`` mapping of config/<name>.yaml ({} when it has none)."""
    path = config_dir() / f'{name}.yaml'
    section = (load_strict(path.read_text(), path) or {}).get('tune') or {}
    if not isinstance(section, dict):
        raise ValueError(f'{path}: tune must be a mapping')
    return section

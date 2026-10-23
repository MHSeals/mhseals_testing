"""Which config files are versioned, and how each one is read into a history value.

``SUBSYSTEMS`` is the one table that knows the file subsystems. In-process writers call
``record_file`` after writing; ``observe_file`` is called first (and at start) so a hand
edit is logged as ``external`` instead of being merged into the next change. The driver's
arming copy of the hardware config is a derived file and is never passed here.
History never blocks a config write: failures go to ``warn`` and return None.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import yaml

_USER = Path.home() / '.config' / 'mhseals'


@dataclass(frozen=True)
class Adapter:
    path: Callable[[], Path]                          # where the operator's file lives
    effective: Optional[Callable[[Path], dict]] = None   # merged config, for subsystems with defaults


def _hardware_path():
    try:
        from mhseals_hardware.configuration import config_path
        return Path(config_path())
    except ImportError:
        return _USER / 'hardware.yaml'


def _hardware_effective(path):
    from mhseals_hardware.configuration import load_config
    return load_config(path)


def _rtk_path():
    from mhseals_testing import rtk
    return rtk.config_path()


def _health_path():
    from mhseals_testing.health.config import USER_FILE
    return USER_FILE


SUBSYSTEMS = {
    'hardware': Adapter(_hardware_path, _hardware_effective),
    'rtk': Adapter(_rtk_path),
    'health': Adapter(_health_path),
    'keymap': Adapter(lambda: _USER / 'keymap.yaml'),
}


def read(sub, path=None):
    """``(value, raw, meta)`` for the file, or None when it is missing or not a YAML mapping.

    A hardware value holds the file's ``overrides`` and the ``effective`` merged config, so a
    change to the shipped defaults shows up as a version too.
    """
    adapter = SUBSYSTEMS[sub]
    path = Path(path) if path else adapter.path()
    try:
        raw = path.read_text()
        parsed = yaml.safe_load(raw) or {}
    except (OSError, yaml.YAMLError):
        return None
    if not isinstance(parsed, dict):
        return None
    if adapter.effective is None:
        return parsed, raw, {}
    try:
        return {'overrides': parsed, 'effective': adapter.effective(path)}, raw, {}
    except Exception:       # an invalid file still has a version: its overrides
        return {'overrides': parsed}, raw, {}


def _capture(history, sub, path, warn, **kwargs):
    try:
        found = read(sub, path)
        if found is None:
            return None
        value, raw, meta = found
        if kwargs.get('source'):
            return history.record(sub, value, raw, meta=meta, **kwargs)
        return history.observe(sub, value, raw, meta)
    except Exception as error:      # the write already happened; history is best effort
        if warn:
            warn(f'Config history could not record {sub}: {error}')
        return None


def observe_file(history, sub, path=None, warn=None):
    """Log the file as ``external`` if it differs from the last recorded version."""
    return _capture(history, sub, path, warn)


def record_file(history, sub, source, path=None, warn=None):
    """Log the file as written by ``source`` (e.g. ``tui:pins``); None when nothing changed."""
    return _capture(history, sub, path, warn, source=source)


def observe_all(history, paths=None, warn=None):
    """Observe every file subsystem; ``paths`` overrides locations by name."""
    paths = paths or {}
    return [c for sub in SUBSYSTEMS if (c := observe_file(history, sub, paths.get(sub), warn))]

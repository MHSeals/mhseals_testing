"""Revert backends for the file subsystems (hardware, rtk, health, keymap).

A revert writes the stored ``raw`` text over the operator's file with ``data.atomic_write``,
runs the host's reload hook, then verifies by reading the file back through ``adapters.read``.
Hardware may also check the driver: ``driver(keys)`` returns the driver's live values (or None
when it cannot be read) and every requested key must be present and equal the version's effective value.
FCU and Nav2 need ROS clients and are not here.
"""

from pathlib import Path

import yaml

from mhseals_testing import data
from mhseals_testing import history as hist
from mhseals_testing.history import adapters
from mhseals_testing.history.revert import Backend


# Keys safe to change while armed: scalars and gains that move no pin, channel or direction.
# The driver's LIVE_KEYS also hot-applies channel_map and reversed, which would remap outputs.
ARMED_SAFE = frozenset({'command_timeout', 'decoupling', 'decoupling_negative'})


def hardware_live_keys():
    """Keys revertable while armed: the driver's hot-reloadable keys that are also ARMED_SAFE;
    None (nothing is live-safe) when the driver is not importable."""
    try:
        from mhseals_hardware.thruster_pwm_node import LIVE_KEYS
        return frozenset(LIVE_KEYS) & ARMED_SAFE
    except ImportError:
        return None


class FileBackend(Backend):
    """Revert one YAML file subsystem; ``reload()`` tells its consumer to re-read the file."""

    def __init__(self, sub, path=None, reload=None, live_keys=None, driver=None):
        self.sub, self.path, self.reload = sub, path, reload
        self.live_keys, self.driver = live_keys, driver

    def _path(self):
        return Path(self.path) if self.path else adapters.SUBSYSTEMS[self.sub].path()

    def read(self):
        found = adapters.read(self.sub, self._path())
        if found is None:
            raise OSError(f'{self.sub} config is unreadable')
        return found

    def apply(self, version):
        raw = version.get('raw')
        if raw is None:
            value = version['value']
            raw = yaml.safe_dump(value.get('overrides', value), sort_keys=False)
        data.atomic_write(self._path(), raw)
        if self.reload:
            change = self.reload()
            if change is not None and not getattr(change, 'ok', True):
                raise OSError(f'{self.sub} reload failed: {change.error}')

    def verify(self, version, timeout):
        if not self.persisted(version):
            return False
        return self._driver_agrees(version)

    def persisted(self, version):
        """File evidence only; hardware consumer verification is separate."""
        live = self.read()[0]
        key = (lambda v: v.get('overrides', v))
        if hist.digest(key(live)) != hist.digest(key(version['value'])):
            return False
        return True

    def _driver_agrees(self, version):
        if self.sub != 'hardware':
            return True
        keys = sorted(self.live_keys or ())
        seen = self.driver(keys) if self.driver and keys else None
        if not keys or not isinstance(seen, dict):
            return False
        effective = version['value'].get('effective', {})
        return all(hist.digest(seen[k]) == hist.digest(effective[k]) for k in keys) if set(keys) <= seen.keys() & effective.keys() else False


def file_backends(hardware_path=None, reload=None, driver=None):
    """One FileBackend per file subsystem; ``reload`` maps a subsystem to its reload hook."""
    reload = reload or {}
    return {sub: FileBackend(sub, hardware_path if sub == 'hardware' else None, reload.get(sub),
                             hardware_live_keys() if sub == 'hardware' else None,
                             driver if sub == 'hardware' else None)
            for sub in adapters.SUBSYSTEMS}

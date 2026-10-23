"""The one guarded path to FCU parameters: read, apply and the operator-confirmed compass write.

``FcuParams`` holds the rules and talks to the FCU only through a link (``MavrosLink`` on the boat,
a fake in tests). Reading never sets anything. ``apply`` refuses while armed, checks every name
and value against ``fcu_meta.json`` and the live FCU before the first set, reads the values back
at float32 precision and puts the old values back, newest first, if anything fails.
See config_history.md 2.5 and section 4.
"""

import json
import math
from pathlib import Path
import struct
import time

FIXED_MAG_CAL_YAW = 42006


def _meta_path():
    """config/history/fcu_meta.json in the source tree, else the installed share directory."""
    source = Path(__file__).resolve().parents[2] / 'config' / 'history' / 'fcu_meta.json'
    if source.exists():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('mhseals_testing')) / 'config' / 'history' / 'fcu_meta.json'
    except (ImportError, KeyError):
        return source


META = _meta_path()


def load_meta(path=META):
    """{'count': n, 'params': {NAME: {min, max, reboot, ...}}}; empty when the file is missing."""
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return {'count': 0, 'params': {}}


def same_float32(a, b):
    """True when two values are equal after rounding to float32, the FCU's own precision."""
    try:
        return struct.pack('<f', a) == struct.pack('<f', b)
    except (OverflowError, struct.error):
        return a == b


class FcuParams:
    """Guarded access to FCU parameters over ``link``.

    The link provides ``names()``, ``get(names) -> {name: value}`` (absent when unknown),
    ``set(name, value) -> bool``, ``armed() -> bool | None`` and ``command(id, params) -> bool``.
    """

    def __init__(self, link, meta=None, clock=time.monotonic, sleep=time.sleep,
                 stable=5.0, timeout=30.0, settle=1.0):
        self.link, self.clock, self.sleep = link, clock, sleep
        self.meta = load_meta() if meta is None else meta
        self.stable, self.timeout, self.settle = stable, timeout, settle

    def read_all(self, minimum=0):
        """Every parameter, or None while the FCU is still streaming its list.

        Accepted only once the parameter count has stayed the same for ``stable`` seconds and is at
        least ``minimum`` (the last known count); a partial read must never be recorded.
        """
        start = self.clock()
        count, since = -1, start
        while True:
            names = self.link.names()
            now = self.clock()
            if len(names) != count:
                count, since = len(names), now
            if count and count >= minimum and now - since >= self.stable:
                values = self.link.get(names)
                return values if len(values) == count else None
            if now - start >= self.timeout:
                return None
            self.sleep(0.5)

    def read(self, names):
        """The named parameters that exist; never sets or pulls."""
        return self.link.get(list(names))

    def check(self, changes):
        """Why ``changes`` must not be applied (a list of reasons, empty when they may be)."""
        if not changes:
            return ['nothing to change']
        problems = []
        known = self.meta.get('params', {})
        for name, value in changes.items():
            info = known.get(name)
            if info is None:
                problems.append(f'{name}: not a known FCU parameter')
            elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                problems.append(f'{name}: {value!r} is not a finite number')
            elif not info.get('min', -math.inf) <= value <= info.get('max', math.inf):
                problems.append(f'{name}: {value} outside {info.get("min")}..{info.get("max")}')
        return problems

    def apply(self, changes):
        """Set ``changes`` ({name: value}); returns dict(status, detail, written).

        status: 'refused' (nothing was set), 'applied', 'pending' (applied, a reboot makes it live),
        'rolled_back' (a set or the readback failed and the old values are back) or 'unknown'
        (rollback failed too: the caller must observe the FCU to learn its true state).
        """
        if self.link.armed() is not False:
            return self._result('refused', 'the FCU is armed or its state is unknown')
        problems = self.check(changes)
        before = {} if problems else self.link.get(list(changes))
        problems += [f'{name}: not on the FCU' for name in changes if name not in before and name in self.meta.get('params', {})]
        if problems:
            return self._result('refused', '; '.join(problems))
        written = []
        for name, value in changes.items():
            if not self.link.set(name, value):
                return self._rollback(written, before, f'{name} was not acknowledged')
            written.append(name)
        self.sleep(self.settle)
        now = self.link.get(written)
        wrong = [n for n in written if n not in now or not same_float32(now[n], changes[n])]
        if wrong:
            return self._rollback(written, before, f'readback differs for {", ".join(wrong)}')
        reboot = [n for n in written if self.meta['params'][n].get('reboot')]
        return self._result('pending' if reboot else 'applied',
                            f'reboot needed for {", ".join(reboot)}' if reboot else '', written)

    def _rollback(self, written, before, why):
        """Put back what was written, newest first, and check it took."""
        for name in reversed(written):
            self.link.set(name, before[name])
        self.sleep(self.settle)
        now = self.link.get(written)
        failed = [n for n in written if n not in now or not same_float32(now[n], before[n])]
        if failed:
            return self._result('unknown', f'{why}; rollback failed for {", ".join(failed)}', written)
        return self._result('rolled_back', why, written)

    @staticmethod
    def _result(status, detail, written=()):
        return dict(status=status, detail=detail, written=list(written))

    def fixed_mag_cal(self, yaw, mask=1, confirmed=False):
        """Send MAV_CMD_FIXED_MAG_CAL_YAW (42006) with the true yaw in degrees; lat/lon 0 means use GPS.

        The only compass write, and only when the operator has confirmed it and the FCU is disarmed.
        Returns True only when the FCU acknowledged it.
        """
        if not confirmed or self.link.armed() is not False:
            return False
        if not math.isfinite(yaw) or not 0 <= yaw < 360:
            return False
        return bool(self.link.command(FIXED_MAG_CAL_YAW, [float(yaw), float(mask), 0, 0, 0, 0, 0]))

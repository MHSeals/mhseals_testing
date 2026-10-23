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


def _wait(future, timeout, what):
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            raise TimeoutError(f'{what} timed out after {timeout:g}s')
        time.sleep(0.02)
    return future.result()


class MavrosLink:
    """The link over MAVROS 2.x services; the node is spun elsewhere, so calls poll their futures."""

    def __init__(self, node, namespace='/mavros', timeout=3.0):
        from mavros_msgs.msg import State
        from mavros_msgs.srv import CommandLong, ParamSetV2
        from rcl_interfaces.srv import GetParameters, ListParameters
        self.node, self.timeout, self.state = node, timeout, None
        base = namespace.rstrip('/')
        self.listing = node.create_client(ListParameters, base + '/param/list_parameters')
        self.getting = node.create_client(GetParameters, base + '/param/get_parameters')
        self.setting = node.create_client(ParamSetV2, base + '/param/set')
        self.commanding = node.create_client(CommandLong, base + '/cmd/command')
        node.create_subscription(State, base + '/state', self._on_state, 10)
        self.types = (ListParameters, GetParameters, ParamSetV2, CommandLong)

    def _on_state(self, message):
        self.state = message

    def _call(self, client, request, what):
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise TimeoutError(f'{what} service is not available')
        return _wait(client.call_async(request), self.timeout, what)

    def armed(self):
        return None if self.state is None or not self.state.connected else bool(self.state.armed)

    def names(self):
        request = self.types[0].Request()
        request.depth = 0
        return list(self._call(self.listing, request, 'list_parameters').result.names)

    def get(self, names):
        request = self.types[1].Request()
        request.names = list(names)
        values = self._call(self.getting, request, 'get_parameters').values
        # type 2 integer, 3 double; 0 means the name does not exist
        return {n: (v.integer_value if v.type == 2 else v.double_value)
                for n, v in zip(names, values) if v.type in (2, 3)}

    def set(self, name, value):
        request = self.types[2].Request()
        request.param_id = name
        if isinstance(value, int):
            request.value.type, request.value.integer_value = 2, value
        else:
            request.value.type, request.value.double_value = 3, float(value)
        return bool(self._call(self.setting, request, f'set {name}').success)

    def command(self, command, params):
        request = self.types[3].Request()
        request.broadcast, request.command = False, command
        for i, value in enumerate(params, 1):
            setattr(request, f'param{i}', float(value))
        return bool(self._call(self.commanding, request, f'command {command}').success)

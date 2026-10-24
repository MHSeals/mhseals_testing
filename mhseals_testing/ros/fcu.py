"""Private guarded FCU helper; production parameter apply remains unsupported/unwired.

Read methods expose cached values only. Apply requires a qualified fresh-observation and safety
adapter; the legacy MAVROS link deliberately cannot certify remote writes or compensation.
See config_history.md 2.5 and section 4. Actual transport qualification belongs to F00.
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


def _finite_number(value):
    """Receipt numbers must be plain, non-boolean and safely representable as finite."""
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _identity_token(value):
    """Identity tokens are nonempty strings or finite plain numbers (never bool)."""
    return (type(value) is str and bool(value)) or _finite_number(value)


def same_float32(a, b):
    """True when two values are equal after rounding to float32, the FCU's own precision."""
    try:
        return struct.pack('<f', a) == struct.pack('<f', b)
    except (OverflowError, struct.error):
        return a == b


class FcuParams:
    """Guarded access to FCU parameters over ``link``.

    Reads use ``names()``, ``get(names) -> {name: value}`` (absent when unknown),
    ``set(name, value) -> bool``, ``armed() -> bool | None`` and ``command(id, params) -> bool``.
    Apply additionally requires trusted ``safety_state()`` and
    ``observe_fresh(names, after_attempt, identity, request)`` receipts. Identity is
    (target, connection generation, boot). Timestamps use the injected monotonic clock.
    Each observation must have a unique ID and prove remote causality, not echo request fields.
    MavrosLink deliberately lacks these capabilities pending transport qualification (F00).
    """

    def __init__(self, link, meta=None, clock=time.monotonic, sleep=time.sleep,
                 stable=5.0, timeout=30.0, settle=1.0, safety_age=1.0, observation_age=1.0):
        self.link, self.clock, self.sleep = link, clock, sleep
        self.meta = load_meta() if meta is None else meta
        self.stable, self.timeout, self.settle = stable, timeout, settle
        self.safety_age, self.observation_age = safety_age, observation_age
        self._seen_observations = set()

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
            elif not _finite_number(value):
                problems.append(f'{name}: value is not a finite number')
            elif not info.get('min', -math.inf) <= value <= info.get('max', math.inf):
                problems.append(f'{name}: {value} outside {info.get("min")}..{info.get("max")}')
        return problems

    def _safe(self, identity=None):
        """Trusted adapter receipt; legacy cached MAVROS state cannot supply this."""
        state = self.link.safety_state()
        now = self.clock()
        if (not state or state.get('armed') is not False or state.get('connected') is not True
                or not _finite_number(state.get('received'))
                or not 0 <= now - state['received'] <= self.safety_age):
            raise ValueError('missing, stale or unsafe FCU state')
        current = tuple(state.get(k) for k in ('target', 'connection', 'boot'))
        if not all(_identity_token(x) for x in current) or (identity is not None and current != identity):
            raise ValueError('FCU target/connection/boot changed or unknown')
        return current

    def _observe(self, names, after, identity):
        # A qualified adapter must establish FCU-origin causal correlation itself. This envelope
        # only rejects inconsistent receipts; it cannot qualify a transport (F00 remains required).
        request = object()
        receipt = self.link.observe_fresh(list(names), after, identity, request)
        now = self.clock()
        if (not receipt or receipt.get('request') is not request
                or type(receipt.get('identity')) is not tuple
                or not all(_identity_token(x) for x in receipt['identity'])
                or receipt['identity'] != identity or receipt.get('source') != 'fcu'
                or set(receipt.get('values', {})) != set(names)
                or not all(_finite_number(v) for v in receipt['values'].values())
                or not _finite_number(receipt.get('observed'))
                or not _finite_number(receipt.get('received'))
                or not after <= receipt['observed'] <= receipt['received'] <= now
                or now - receipt['received'] > self.observation_age
                or not receipt.get('observation_id')
                or receipt['observation_id'] in self._seen_observations):
            raise ValueError('fresh FCU-origin observation unavailable')
        self._seen_observations.add(receipt['observation_id'])
        self._safe(identity)
        return receipt['values']

    def apply(self, changes):
        """Private, unwired helper. Incapable links refuse before any effect.

        ``written`` records attempts, including lost/false ACKs. ``rolled_back`` requires fresh
        post-compensation FCU evidence; unsafe transitions retain recovery_required instead.
        """
        problems = self.check(changes)
        if problems:
            return self._result('refused', '; '.join(problems))
        if not all(callable(getattr(self.link, n, None))
                   for n in ('safety_state', 'observe_fresh')):
            return self._result('refused', 'unsupported fresh FCU safety/observation capability')
        try:
            identity = self._safe()
            before = self._observe(changes, self.clock(), identity)
        except Exception as error:
            return self._result('refused', f'could not establish fresh FCU predecessor: {error}')
        written = []
        after = self.clock()
        for name, value in changes.items():
            try:
                self._safe(identity)
            except Exception as error:
                return self._result('recovery_required' if written else 'refused', str(error), written)
            written.append(name)  # intent precedes the externally uncertain set
            after = self.clock()
            try:
                acked = self.link.set(name, value)
            except Exception as error:
                return self._rollback(written, before, f'{name}: {error}', after, identity)
            if not acked:
                return self._rollback(written, before, f'{name} was not acknowledged', after, identity)
        self.sleep(self.settle)
        try:
            now = self._observe(written, after, identity)
        except Exception as error:
            return self._result('recovery_required', f'forward observation unknown: {error}', written)
        wrong = [n for n in written if not same_float32(now[n], changes[n])]
        if wrong:
            return self._rollback(written, before, f'readback differs for {", ".join(wrong)}', after, identity)
        reboot = [n for n in written if self.meta['params'][n].get('reboot')]
        return self._result('pending' if reboot else 'applied',
                            f'reboot needed for {", ".join(reboot)}' if reboot else '', written)

    def _rollback(self, written, before, why, after, identity):
        """Observe uncertainty first; never compensate through an unsafe identity transition."""
        try:
            self._observe(written, after, identity)
            for name in reversed(written):
                self._safe(identity)
                after = self.clock()
                self.link.set(name, before[name])
            self.sleep(self.settle)
            now = self._observe(written, after, identity)
        except Exception as error:
            return self._result('unknown', f'{why}; rollback_unverified; recovery_required: {error}', written)
        failed = [n for n in written if not same_float32(now[n], before[n])]
        if failed:
            return self._result('unknown', f'{why}; rollback_unverified for {", ".join(failed)}', written)
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
        response = self._call(self.commanding, request, f'command {command}')
        # MAVROS sets success for any COMMAND_ACK; only MAV_RESULT_ACCEPTED (0) means it was done
        return bool(response.success) and response.result == 0

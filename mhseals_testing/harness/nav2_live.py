"""Live tier-1 Nav2 apply (CFG-NAV-LIVE1).

One path behind the TUI popup and ``boat_trial config apply nav2``. ROS-free:
``Nav2Live`` is driven through a ``Ports`` object (ros/nav2_live.py is the ROS
one), so the sequence is tested with plain fakes. The owner accepted the
tradeoff of applying before T02-O1 owner fencing: the ownership check is the
session registry owner plus the authority state, both rechecked right before
the first write, and a failure partway may need a manual restart of Nav2.

The sequence, every step able to stop it:

    preview (natural diff against the live values, fences)
    -> intent journaled
    -> tokened mux hold (cause ``config``) when the mux is up -> fences again
    -> one atomic SetParameters per node, smoother first and at the more
       restrictive limits
    -> per-key GetParameters readback
    -> no ``not found``/``Invalid`` on /rosout for 2 s
    -> hold released with the same token
    -> outcome journaled, history ``applied: live-session``

A rejected set, a readback mismatch or a bad /rosout line rolls back in
reverse order, each step read back. A timeout is ``unknown``: nothing is
retried and nothing more is written. Scope: parameters the class table calls
``live``; a request that needs a reset or restart is refused as a whole.
"""

from dataclasses import dataclass, field
import json
import math
import sys
import time
import uuid

from mhseals_testing.harness import nav2_params as n2

HOLD_CAUSE = 'config'
HOLD_BY = 'nav2-live'
HOLD_CONFIRM_S = 2.0
ROSOUT_WINDOW_S = 2.0
# Nodes whose parameters the lowering (n2.plan) reads: a limits change always
# needs both.
LOWERING_NODES = ('controller_server', 'velocity_smoother')
# What --now may change while a goal runs: the motion limits and the values
# derived from them.
LIMIT_KEYS = frozenset(
    [f'velocity_smoother.{k}'
     for k in ('max_velocity', 'min_velocity', 'max_accel', 'max_decel')]
    + [n2.FOLLOW + k for k in (
        'vx_max', 'vy_max', 'wz_max', 'vx_min', 'ax_max', 'ax_min', 'ay_max',
        'ay_min', 'az_max', 'az_min', 'vx_std', 'vy_std', 'wz_std',
        'prune_distance', 'rotate_to_heading_angular_vel',
        'max_angular_accel')])
# The smoother value that is the more restrictive of two (closer to zero).
RESTRICT = {'max_velocity': min, 'max_accel': min,
            'min_velocity': max, 'max_decel': max}
# Exit codes, as boat_trial config.
DONE, NOTHING, REFUSED, ROLLED_BACK, UNSAFE = 0, 1, 2, 3, 4


class Ports:
    """What Nav2Live needs from the world; tests fake it."""

    def owner(self):
        """Return the live session-registry owner, or None."""
        raise NotImplementedError

    def authority(self):
        """Return the latest fresh authority record, None if unknown."""
        raise NotImplementedError

    def goals(self):
        """Return running navigation goal names; None when unknown."""
        raise NotImplementedError

    def mux(self):
        """Return the fresh /command_mux/state, or None."""
        raise NotImplementedError

    def mux_expected(self):
        """Say whether the mux must be up (muxed and armed)."""
        raise NotImplementedError

    def mux_request(self, request):
        """Publish ``request`` on /command_mux/request."""
        raise NotImplementedError

    def read(self, node):
        """Return ({param: value}, declared names) of ROS node ``node``.

        Raises TimeoutError when the node is silent.
        """
        raise NotImplementedError

    def get(self, node, names):
        """Return {param: value} for the names ``node`` has set."""
        raise NotImplementedError

    def put(self, node, values):
        """Return (successful, reason) of one SetParametersAtomically.

        Raises TimeoutError, or ValueError for a value of the wrong type.
        """
        raise NotImplementedError

    def rosout_open(self):
        """Start collecting /rosout and return a watcher (``errors``, ``close``)."""
        raise NotImplementedError

    def journal(self, phase, **fields):
        """Append one event to the session journal; OSError if it cannot."""
        raise NotImplementedError


def normalize(requested):
    """Map each raw key to (flat ``node.param`` key, value).

    The ``ros__parameters`` level is not part of a parameter name.
    """
    out = {}
    for key, value in requested.items():
        parts = [p for p in str(key).split('.') if p != 'ros__parameters']
        out[key] = ('.'.join(n2.split('.'.join(parts))), value)
    return out


def same(left, right):
    """Compare parameter values: numbers by value, bools only with bools."""
    if isinstance(left, bool) or isinstance(right, bool):
        return type(left) is type(right) and left == right
    if isinstance(left, (list, tuple)) and isinstance(right, (list, tuple)):
        return len(left) == len(right) and all(map(same, left, right))
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return left == right or (math.isnan(left) and math.isnan(right))
    return type(left) is type(right) and left == right


def restrictive(param, old, new):
    """Return the smoother value to send first: the more restrictive one."""
    pick = RESTRICT.get(param)
    lists = isinstance(old, list) and isinstance(new, list)
    if pick is None or not lists or len(old) != len(new):
        return new
    return [pick(a, b) for a, b in zip(old, new)]


@dataclass
class Step:
    """One atomic set on one node."""

    node: str                    # flat node name (controller_server, ...)
    values: dict                 # {param: value} sent in one request
    before: dict = field(default_factory=dict)   # in effect before the step


@dataclass
class Preview:
    """What an apply would do, and what would stop it."""

    requested: dict              # the raw request, as given
    owner: dict | None = None    # the registry owner it was made under
    rows: list = field(default_factory=list)    # key, old, new, derived
    steps: list = field(default_factory=list)
    refusals: list = field(default_factory=list)
    live: dict = field(default_factory=dict)    # flat live values read
    wanted: dict = field(default_factory=dict)  # flat key -> value to hold
    names: dict = field(default_factory=dict)   # raw key -> flat key
    allow_goal: bool = False

    def signature(self):
        """Return a comparable text of the rows."""
        return json.dumps([[r['key'], r['old'], r['new']]
                           for r in self.rows], default=str)


def target(node):
    """Return the ROS node path of flat node name ``node``."""
    return n2.target(node + '.x')[0]


class Nav2Live:
    """Preview and apply a requested Nav2 parameter set on the live stack.

    ``me`` is the pid that must own the session (the TUI), or None for a
    guest such as the CLI, which only needs a live owner. ``record()`` writes
    the config-history entry once the apply held.
    """

    def __init__(self, ports, *, me=None, record=None, sleep=time.sleep,
                 clock=time.monotonic):
        self.ports, self.me, self.record = ports, me, record
        self.sleep, self.clock = sleep, clock
        self.last = None

    # ---- preview

    def preview(self, requested, now=False):
        """Return what applying ``requested`` changes, and what refuses it."""
        found = self.last = Preview(dict(requested),
                                    owner=self.ports.owner())
        if found.owner is None:
            found.refusals.append('no live session owner')
            return found
        named = normalize(requested)
        found.names = {raw: flat for raw, (flat, _) in named.items()}
        changes = dict(named.values())
        if not changes:
            found.refusals.append('nothing is requested')
        elif len(changes) != len(named):
            found.refusals.append('two keys name the same parameter')
        if found.refusals:
            return found
        declared = self._read(found, changes)
        if found.refusals:
            return found
        try:
            plan = n2.plan(changes, found.live, declared)
        except (ValueError, KeyError, TypeError, ImportError, OSError) as error:
            found.refusals.append(f'cannot derive the request: {error}')
            return found
        for cls in ('reset', 'restart'):
            keys = sorted(f'{node}.{param}'
                          for node, values in plan[cls].items()
                          for param in values)
            if keys:
                found.refusals.append(
                    f'needs a {cls} of Navigation, not a live set: '
                    + ', '.join(keys))
        sent = {f'{node}.{param}': value
                for node, values in plan['live'].items()
                for param, value in values.items()}
        unchanged = {key: found.live[key] for key in plan['unchanged']
                     if key in found.live}
        found.wanted = {**unchanged, **sent}
        found.rows = [{'key': key, 'old': found.live.get(key),
                       'new': sent[key], 'derived': key not in changes}
                      for key in sorted(sent)]
        found.allow_goal = bool(now) and all(k in LIMIT_KEYS for k in sent)
        found.refusals += self.fences(found.owner, found.allow_goal)
        if not found.refusals:
            found.steps = self._steps(plan['live'], found.live)
        return found

    def _read(self, found, changes):
        """Read the nodes a request touches; return their declared names."""
        nodes = {n2.split(key)[0] for key in changes}
        if nodes & set(LOWERING_NODES):
            nodes |= set(LOWERING_NODES)
        declared = {}
        for node in sorted(nodes):
            try:
                values, names = self.ports.read(target(node))
            except (TimeoutError, OSError) as error:
                found.refusals.append(f'{node} did not answer ({error})')
                continue
            declared[node] = set(names)
            found.live.update({f'{node}.{param}': value
                               for param, value in values.items()})
        return declared

    @staticmethod
    def _steps(live_groups, live):
        """Order the sets: smoother (restrictive) first, the rest, smoother."""
        steps, state, later = [], dict(live), None
        for node, values in live_groups.items():
            if node == 'velocity_smoother':
                first = {p: restrictive(p, live.get(f'{node}.{p}'), v)
                         for p, v in values.items()}
                later = {p: v for p, v in values.items()
                         if not same(first[p], v)} or None
                values = {p: v for p, v in first.items()
                          if not same(v, live.get(f'{node}.{p}'))}
            if values:
                steps.append(Step(node, dict(values)))
        if later:
            steps.append(Step('velocity_smoother', later))
        for step in steps:
            step.before = {p: state.get(f'{step.node}.{p}')
                           for p in step.values}
            state.update({f'{step.node}.{p}': v
                          for p, v in step.values.items()})
        return steps

    # ---- fences

    def fences(self, owner, allow_goal=False):
        """Return the reasons not to touch the boat now (empty: clear).

        Rechecked right before the first write.
        """
        if owner is None:
            return ['no live session owner']
        reasons = []
        if self.me is not None and owner.get('pid') != self.me:
            reasons.append('this instance does not own the session')
        record = self.ports.authority()
        if record is None:
            return reasons + ['authority state unknown (no fresh record '
                              'from the session owner)']
        who = record.get('owner')
        if record.get('input') in ('rc', 'manual') or who in (
                'rc', 'keyboard'):
            reasons.append(f'someone is driving ({who})')
        if record.get('procedure'):
            reasons.append(
                f'procedure running: {record["procedure"].get("name")}')
        if (record.get('task') or {}).get('status') == 'RUNNING':
            reasons.append('a task is running')
        claims = [c for c in record.get('claims') or []
                  if c.get('state') != 'revoked']
        if claims:
            reasons.append(f'claim held: {claims[0].get("who", "?")} '
                           f'({claims[0].get("id", "?")})')
        if who == 'unknown':
            reasons.append('authority owner is unknown')
        if record.get('conflict'):
            reasons.append('authority sources conflict')
        running = self.ports.goals()
        if running is None:
            reasons.append('cannot tell whether a goal is active')
        elif running and not allow_goal:
            reasons.append(f'a goal is active ({", ".join(running)}); '
                           'a limits-only change can use --now')
        elif who in ('nav', 'external') and not running:
            reasons.append(f'something else is driving ({who})')
        return reasons

    # ---- apply

    def applier(self, requested):
        """Serve the Nav2File seam: apply, checked against the last preview."""
        last = self.last
        same_request = last is not None and last.requested == requested
        return self.apply(requested, last if same_request else None)

    def apply(self, requested, previewed=None, now=False):
        """Apply and verify; return the receipt dict.

        ``status`` is verified, refused, rolled_back, rollback_failed or
        unknown.
        """
        now = now or bool(previewed and previewed.allow_goal)
        found = self.preview(requested, now=now)
        if found.refusals:
            return self._refused(found, found.refusals)
        if previewed is not None:
            if (previewed.owner or {}).get('pid') != found.owner.get('pid'):
                return self._refused(
                    found, ['the session owner changed since the preview'])
            if previewed.signature() != found.signature():
                return self._refused(
                    found, ['live values changed since the preview; '
                            'open it again'])
        if not found.steps:
            return self._receipt(found, 'verified', steps=0)
        token = uuid.uuid4().hex
        try:
            self.ports.journal(
                'nav2_live_intent', hold=token[:8],
                steps=[s.node for s in found.steps],
                keys={r['key']: r['new'] for r in found.rows},
                before={r['key']: r['old'] for r in found.rows})
        except OSError as error:
            return self._refused(
                found, [f'the intent could not be journaled ({error})'])
        hold = {'needed': False, 'held': False, 'released': None}
        try:
            receipt = self._held(found, token, hold)
        finally:
            if hold['held']:
                hold['released'] = self._release(token)
        receipt['hold'] = hold
        if hold['held'] and not hold['released']:
            receipt['warning'] = ('the mux hold was not confirmed released; '
                                  'press U to release it')
        try:
            self.ports.journal(
                'nav2_live_outcome', hold=token[:8], status=receipt['status'],
                detail=receipt.get('error') or receipt.get('refused') or '')
        except OSError as error:
            receipt['warning'] = (f'{receipt.get("warning", "")} outcome not '
                                  f'journaled ({error})').strip()
        if receipt['status'] == 'verified' and self.record:
            self.record()
        return receipt

    def _held(self, found, token, hold):
        """Hold the mux, recheck the fences, then write."""
        why = self._hold(token, hold)
        if why:
            return self._refused(found, [why])
        again = self.fences(self.ports.owner(), found.allow_goal)
        if again:
            return self._refused(found, again)
        return self._write(found)

    def _refused(self, found, reasons):
        return self._receipt(found, 'refused', refused='; '.join(reasons),
                             reasons=list(reasons))

    def _receipt(self, found, status, **extra):
        receipt = {'status': status, 'live': True, 'rows': found.rows,
                   **extra}
        if status == 'verified':
            # Every requested key holds its requested value, or the value
            # the launch would derive for it.
            receipt['verified'] = {raw: found.requested[raw]
                                   for raw in found.names}
            receipt['adjusted'] = {
                raw: found.wanted[flat] for raw, flat in found.names.items()
                if not same(found.wanted[flat], found.requested[raw])}
        return receipt

    # ---- mux hold

    def _hold(self, token, hold):
        """Place and confirm the tokened hold; return a refusal or ''."""
        state = self.ports.mux()
        hold['needed'] = state is not None
        if state is None:
            if self.ports.mux_expected():
                return 'the mux state is unknown while armed'
            return ''
        if any(h.get('cause') == HOLD_CAUSE
               for h in state.get('holds') or []):
            return f'a {HOLD_CAUSE} hold is already in place'
        self.ports.mux_request({'op': 'hold', 'cause': HOLD_CAUSE,
                                'by': HOLD_BY, 'token': token})
        hold['held'] = True
        if self._mux_has(True):
            return ''
        return 'the mux did not confirm the config hold'

    def _mux_has(self, present):
        deadline = self.clock() + HOLD_CONFIRM_S
        while True:
            holds = (self.ports.mux() or {}).get('holds') or []
            held = any(h.get('cause') == HOLD_CAUSE and h.get('has_token')
                       for h in holds)
            if held == present:
                return True
            if self.clock() >= deadline:
                return False
            self.sleep(0.05)

    def _release(self, token):
        self.ports.mux_request({'op': 'release', 'cause': HOLD_CAUSE,
                                'by': HOLD_BY, 'token': token})
        return self._mux_has(False)

    # ---- writes

    def _write(self, found):
        done, watch = [], self.ports.rosout_open()
        try:
            for step in found.steps:
                try:
                    ok, reason = self.ports.put(target(step.node),
                                                step.values)
                except TimeoutError as error:
                    return self._unknown(
                        found, done, f'{step.node} set: {error}; nothing '
                        'was retried; restart Navigation if the values '
                        'look wrong')
                except ValueError as error:
                    return self._back(found, done,
                                      f'{step.node} set refused: {error}')
                if not ok:
                    return self._back(
                        found, done,
                        f'{step.node} rejected the set: {reason}')
                done.append(step)
                try:
                    bad = self._mismatch(step.node, step.values)
                except TimeoutError as error:
                    return self._unknown(
                        found, done, f'{step.node} readback: {error}; '
                        'nothing was retried')
                if bad:
                    return self._back(
                        found, done,
                        f'{step.node} readback differs: {", ".join(bad)}')
            deadline = self.clock() + ROSOUT_WINDOW_S
            while self.clock() < deadline:
                self.sleep(0.05)
            lines = watch.errors({s.node for s in found.steps},
                                 [p for s in found.steps for p in s.values])
            if lines:
                return self._back(found, done, f'/rosout: {lines[0]}')
            return self._receipt(found, 'verified', steps=len(done))
        finally:
            watch.close()

    def _unknown(self, found, done, error):
        return self._receipt(found, 'unknown', error=error,
                             done=[s.node for s in done])

    def _mismatch(self, node, values):
        got = self.ports.get(target(node), list(values))
        return [p for p, v in values.items()
                if p not in got or not same(got[p], v)]

    def _back(self, found, done, error):
        """Roll back the steps that went through, in reverse, each read."""
        failed = []
        for step in reversed(done):
            try:
                ok, reason = self.ports.put(target(step.node), step.before)
                bad = (self._mismatch(step.node, step.before) if ok
                       else [reason])
            except (TimeoutError, ValueError) as problem:
                return self._unknown(
                    found, done,
                    f'{error}; rollback of {step.node}: {problem}')
            if bad:
                failed.append(f'{step.node} ({", ".join(map(str, bad))})')
        if failed:
            return self._receipt(
                found, 'rollback_failed', done=[s.node for s in done],
                error=f'{error}; could not restore {"; ".join(failed)}; '
                      'restart Navigation')
        return self._receipt(found, 'rolled_back',
                             done=[s.node for s in done], error=error)


# ---- boat_trial config apply nav2

def text(value):
    """Return ``value`` as it is shown in a diff row."""
    return 'unset' if value is None else json.dumps(value)


def rows_text(rows):
    """Return the preview rows as indented lines."""
    return [f'  {r["key"]}  {text(r["old"])} -> {text(r["new"])}'
            + ('  (derived)' if r['derived'] else '')
            for r in rows] or ['  no differences']


def cli_apply(requested, live, *, yes=False, now=False, as_json=False,
              out=None):
    """Preview ``requested`` and, with ``yes``, apply it; return the exit code.

    0 applied, 1 nothing to apply, 2 refused or preview only, 3 failed and
    rolled back, 4 unknown or the rollback failed (Navigation may need a
    restart).
    """
    out = out or sys.stdout

    def emit(doc, lines):
        print(json.dumps(doc, default=str) if as_json
              else '\n'.join(lines), file=out)

    found = live.preview(requested, now=now)
    plan = {'outcome': 'refused' if found.refusals else 'preview',
            'changes': found.rows, 'reasons': found.refusals}
    lines = [f'nav2: {len(found.rows)} change(s)', *rows_text(found.rows),
             *[f'refused: {r}' for r in found.refusals]]
    if found.refusals:
        emit(plan, lines)
        return REFUSED
    if not found.rows:
        emit({**plan, 'outcome': 'nothing to do'},
             [*lines, 'nothing to apply: the nodes already hold these '
                      'values'])
        return NOTHING
    if not yes:
        emit(plan, [*lines, 'preview only: pass --yes to apply live'])
        return REFUSED
    receipt = live.apply(requested, found, now=now)
    status = receipt['status']
    doc = {'outcome': {'verified': 'applied'}.get(status, status),
           'changes': receipt['rows'], 'reasons': receipt.get('reasons', []),
           'error': receipt.get('error', ''), 'hold': receipt.get('hold'),
           'warning': receipt.get('warning', ''),
           'live': 'until Navigation restarts'}
    if status == 'verified':
        verdict = ('applied live: lasts until Navigation restarts; '
                   'nav2.yaml holds it for the next start')
    else:
        verdict = f'{status}: {receipt.get("error") or receipt.get("refused")}'
    emit(doc, [*lines, verdict,
               *([receipt['warning']] if receipt.get('warning') else [])])
    return {'verified': DONE, 'refused': REFUSED,
            'rolled_back': ROLLED_BACK}.get(status, UNSAFE)

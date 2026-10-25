"""Journal lines folded into the spans and points the Activity strip draws.

Pure, no Rich. Live and review use the same code: feed `Timeline.add` one
`events.jsonl` line at a time. Times are seconds (`t_ns` / 1e9). In-order lines
fold in O(1); an out-of-order line is bisect-inserted and refolds the journal.
"""

import bisect
import math
from dataclasses import dataclass, replace
from typing import Optional

from mhseals_testing.ui.timeaxis import clock

SEP = ' · '

# Fallback key letters for journals written before `key` rode on the start line.
KEYS = {'characterize': 'f', 'magcal': 'g', 'controller_check': 'v', 'manual': 'm',
        'thruster_set': 'i', 'nav2_bringup': 'n', 'square': 'w', 'station': 's',
        'mission': 't', 'perception': 'p'}
LEVELS = {'ok': 0, 'warn': 1, 'fail': 2}
RESULT_LEVEL = {'TRUST': 0, 'PASS': 0, 'WARN': 1}     # any other verdict is a failure
END_WORD = {1: 'warn', 2: 'fail'}
HOLD_JOIN = 1.0            # a hold set this close to a stop is that stop's cause
VIEWS = ('results', 'events', 'markers')


@dataclass(frozen=True)
class Span:
    kind: str                      # proc manual armed hold recoff health
    t0: float
    t1: Optional[float] = None     # None while open
    key: str = ''
    label: str = ''
    end: str = ''                  # '' normal, 'warn' or 'fail': abnormal end, drawn with a cap
    slices: tuple = ()             # (t0, t1, level) per direction, plus the cut slice of an abort
    level: int = 0                 # worst of the slices, the end and (health) the check level
    run_id: Optional[str] = None
    by: Optional[str] = None
    cause: Optional[str] = None
    verdict: str = ''              # last result verdict


@dataclass(frozen=True)
class Point:
    kind: str                      # mark config health auth agent
    t: float
    sev: int
    label: str
    system: bool = False
    ref: Optional[dict] = None     # the journal line, for popup routing


@dataclass(frozen=True)
class Item:
    """One list row: `kind` picks the strip glyph, so the list is the legend."""
    kind: str                      # a Span kind, 'abort', or a Point kind
    t: float
    sev: int
    what: str
    when: str
    result: str
    ref: Optional[dict] = None


def _level(verdict):
    return RESULT_LEVEL.get(verdict.split()[0] if verdict else '', 2)


def _end_level(line):
    """Severity of an abnormal stop: operator Space/takeover warns, faults and guards fail."""
    cause = str(line.get('cause') or '')
    if line.get('by', 'operator') != 'operator' or cause.startswith(('guard', 'health')):
        return 2
    return 1


class _Fold:
    """Single forward pass over time-ordered lines."""

    def __init__(self):
        self.done, self.points, self.open = [], [], {}
        self.agent_run = False     # an agent run act is open: the procedure starting now is the agent's

    def feed(self, e):
        phase, t = e.get('phase', ''), e['t']
        handler = getattr(self, 'on_' + phase, None)
        if handler:
            handler(e, t)
        elif phase.endswith('_start'):
            self.start(e, t, e.get('procedure') or phase[:-6])
        elif phase.endswith('_stop'):
            self.stop(e, t, e.get('procedure') or phase[:-5])

    def close(self, slot, t, **changes):
        span = self.open.pop(slot, None)
        if span:
            self.done.append(replace(span, t1=t, **changes))
        return span

    def on_armed(self, e, t):
        self.open.setdefault('armed', Span('armed', t, label=str(e.get('profile', ''))))

    def on_disarmed(self, e, t):
        by = e.get('by', 'operator')
        if by != 'operator':       # a fault disarm cuts whatever bars are running
            for slot, span in self.open.items():
                if span.kind in ('proc', 'manual'):
                    self.open[slot] = replace(span, end='fail', level=2)
        self.close('armed', t, by=by, cause=e.get('reason'),
                   end='' if by == 'operator' else 'fail')

    def on_hold_set(self, e, t):
        by = e.get('by', 'operator')
        self.open.setdefault('hold', Span('hold', t, label=by, by=by, cause=e.get('reason')))
        self.points.append(Point('auth', t, 1, f'hold · {by}', ref=e))

    def on_hold_release(self, e, t):
        self.close('hold', t)

    def on_guest_attach(self, e, t):
        self.points.append(Point('auth', t, 0, f"guest {e.get('procedure', '')} {e.get('pid', '')}".strip(), ref=e))

    def on_guest_detach(self, e, t):
        pass

    def on_takeover(self, e, t):
        self.points.append(Point('auth', t, 1, f"takeover · {e.get('from', '')}".strip(' ·'), ref=e))

    def on_handover(self, e, t):
        self.points.append(Point('auth', t, 0, f"handover · {e.get('to', '')}".strip(' ·'), ref=e))

    def on_rec_stop(self, e, t):
        self.open.setdefault('recoff', Span('recoff', t, cause=e.get('reason')))

    def on_rec_start(self, e, t):
        self.close('recoff', t)

    def on_marker(self, e, t):
        agent = e.get('kind') == 'agent'
        self.points.append(Point('agent' if agent else 'mark', t, 0, str(e.get('note', '')),
                                 e.get('kind') == 'system', e))

    def _agent(self, e, t, what, sev=0):
        """An agent point (agent-control 5): claim, act edge, revoke, lease expiry."""
        if e.get('owner', 'agent') == 'agent':
            self.points.append(Point('agent', t, sev, f"{what}{SEP}{e.get('who', '?')}", ref=e))

    def on_claim(self, e, t):
        self._agent(e, t, 'claim')

    def on_claim_release(self, e, t):
        self._agent(e, t, 'release')

    def on_claim_revoke(self, e, t):
        self._agent(e, t, 'revoked', 1)

    def on_claim_expire(self, e, t):
        self._agent(e, t, 'lease expired', 1)

    def on_agent_act(self, e, t):
        if e.get('op') == 'run':
            self.agent_run = e.get('state') == 'started' and not e.get('error')
        if e.get('state') in ('started', 'done') or e.get('error'):
            what = f"{e.get('op', 'act')} {e.get('error') or e.get('state')}"
            self._agent(e, t, what, 1 if e.get('error') else 0)

    def on_config_changed(self, e, t):
        sev = {'rolled_back': 2, 'pending': 1, 'external': 1}.get(e.get('applied'), 0)
        self.points.append(Point('config', t, sev, str(e.get('summary', '')), ref=e))

    def on_health(self, e, t):
        if 'check' not in e:
            return                 # pre-ask lines carry no check name
        slot, level = ('health', e['check']), LEVELS.get(e.get('level'), 0)
        self.close(slot, t)
        if level:
            self.open[slot] = Span('health', t, label=e['check'], level=level)
            self.points.append(Point('health', t, level, str(e.get('title') or e['check']), ref=e))

    def on_result(self, e, t):
        span = self.open.get(e.get('run_id') or e.get('procedure'))
        try:
            w0, w1 = (float(x) / 1e9 for x in e['window'])
        except (KeyError, TypeError, ValueError):
            return                 # no window, or not two numbers
        if span is None or not (math.isfinite(w0) and math.isfinite(w1)):
            return
        level = _level(str(e.get('verdict', '')))
        self.open[e.get('run_id') or e.get('procedure')] = replace(
            span, slices=span.slices + ((w0, w1, level),), level=max(span.level, level),
            verdict=str(e.get('verdict', '')))

    def start(self, e, t, name):
        key = e.get('key') or KEYS.get(name)
        if not key:
            return                 # not a procedure phase (rec_start, session_start, ...)
        self.open[e.get('run_id') or name] = Span(
            'manual' if name == 'manual' else 'proc', t, key=key,
            label=f'agent:{name}' if self.agent_run else str(e.get('label', name)),
            run_id=e.get('run_id'))

    def stop(self, e, t, name):
        slot = e.get('run_id') or name
        span = self.open.get(slot)
        if span is None or span.kind not in ('proc', 'manual'):
            return
        outcome = str(e.get('outcome', 'complete'))
        end_level = 0 if outcome.startswith('complete') else _end_level(e)
        end_level = max(end_level, 2 if span.end else 0)
        slices = span.slices
        if end_level and slices:   # colour only the slice known to be cut; else just the cap
            slices += ((slices[-1][1], t, end_level),)
        self.close(slot, t, end=END_WORD.get(end_level, ''), slices=slices,
                   level=max(span.level, end_level), by=e.get('by'), cause=e.get('cause'))


class Timeline:
    """Incremental fold of the journal; `version` bumps on every accepted line."""

    def __init__(self):
        self.version = 0
        self._lines, self._times = [], []
        self._fold = _Fold()
        self._cache = None

    def add(self, line):
        """Fold one journal line. Lines without a finite `t_ns` are ignored."""
        try:
            t = float(line['t_ns']) / 1e9
        except (KeyError, TypeError, ValueError):
            return
        if not math.isfinite(t):
            return
        e = dict(line, t=t)
        self.version += 1
        self._cache = None
        if not self._times or t >= self._times[-1]:
            self._times.append(t)
            self._lines.append(e)
            self._fold.feed(e)
            return
        at = bisect.bisect_right(self._times, t)
        self._times.insert(at, t)
        self._lines.insert(at, e)
        self._fold = _Fold()
        for old in self._lines:
            self._fold.feed(old)

    def _built(self):
        if self._cache is None:
            spans = sorted(self._fold.done + list(self._fold.open.values()), key=lambda s: s.t0)
            self._cache = (spans, sorted(self._fold.points, key=lambda p: p.t))
        return self._cache

    def spans(self):
        return list(self._built()[0])

    def points(self):
        return list(self._built()[1])

    def unrecorded(self):
        """Intervals where the vehicle was armed and the recorder off (the deck hazard)."""
        spans = self._built()[0]
        armed = [s for s in spans if s.kind == 'armed']
        off = [s for s in spans if s.kind == 'recoff']
        out = []
        for a in armed:
            for r in off:
                lo = max(a.t0, r.t0)
                hi = min(math.inf if a.t1 is None else a.t1, math.inf if r.t1 is None else r.t1)
                if lo < hi:
                    out.append((lo, hi))
        return sorted(out)

    def busy(self):
        """Intervals the time axis must keep readable; open spans run to infinity."""
        spans, points = self._built()
        return ([(s.t0, math.inf if s.t1 is None else s.t1)
                 for s in spans if s.kind in ('armed', 'proc', 'manual', 'health')]
                + [(p.t, p.t) for p in points])

    def items(self, view):
        """List rows for `view`, newest first; a hold set by an abort folds into its stop row."""
        spans, points = self._built()
        if view == 'markers':
            return [Item(p.kind, p.t, 0, p.label, clock(p.t), '', p.ref) for p in reversed(points)
                    if p.kind == 'mark' or (p.ref or {}).get('phase') == 'marker']
        if view not in VIEWS:
            raise ValueError(f'unknown view {view!r}')
        rows = [self._run_item(s, spans) for s in spans if s.kind in ('proc', 'manual')]
        if view == 'events':
            held = {id(p) for s in spans if s.kind in ('proc', 'manual') and s.end
                    for p in points if p.kind == 'auth' and p.label.startswith('hold')
                    and abs(p.t - s.t1) <= HOLD_JOIN}
            rows += [Item(p.kind, p.t, p.sev, p.label, clock(p.t), self._point_result(p), p.ref)
                     for p in points if id(p) not in held]
        return sorted(rows, key=lambda r: r.t, reverse=True)

    @staticmethod
    def _point_result(p):
        return (p.ref or {}).get('applied', '') if p.kind == 'config' else ''

    @staticmethod
    def _run_item(span, spans):
        if span.t1 is None:
            kind, result = 'run', span.verdict or 'running'
        elif span.end:
            kind, result = 'abort', 'ABORTED'
            if span.cause == 'hold' or any(h.kind == 'hold' and abs(h.t0 - span.t1) <= HOLD_JOIN
                                           for h in spans):
                result += '+hold'
        else:
            kind, result = span.kind, span.verdict.split()[0] if span.verdict else 'done'
        return Item(kind, span.t0, span.level, f'{span.key} {span.label}', clock(span.t0), result)

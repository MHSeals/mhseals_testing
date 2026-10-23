"""The health engine: evaluates checks over probe facts and keeps their shown status steady.

Probes collect facts, kinds judge them, and this module adds what neither can:
staleness, the `when` and `missing` policies, the severity cap and hysteresis.
It does no I/O and takes the clock as an argument, so tests drive time by hand.
"""

import dataclasses
import time

from .model import STATUSES, Result
from .registry import KINDS

STALE_PERIODS = 3        # a probe sample older than this many periods is not evidence
RAISE_DEFAULT = {'fail': 0.0, 'unknown': 10.0}     # seconds a band must persist; others use 2 s
RAISE_OTHER = 2.0


@dataclasses.dataclass(frozen=True)
class Transition:
    id: str
    old: str
    new: str
    result: Result
    time: float


@dataclasses.dataclass(frozen=True)
class HealthReport:
    results: dict            # check id -> Result carrying the shown (debounced) status
    transitions: tuple = ()
    muted: frozenset = frozenset()    # ids shown dim: they never toast or gate


class Engine:
    """Turns `tick(facts, probes)` calls into a HealthReport."""

    def __init__(self, specs, periods=None, clock=time.monotonic):
        self.specs = [s for s in specs if s.enabled]
        self.periods = periods or {}             # probe -> seconds between samples
        self.clock = clock
        self.muted = set()                       # session mutes; the TUI popup edits them
        self.last = None                         # shown results of the latest tick; None before one
        self._shown = {}                         # id -> status currently shown
        self._pending = {}                       # id -> (candidate status, since)

    def tick(self, facts, probes, session=None):
        """Evaluate every check.

        facts:   {name: Fact} from all probes.
        probes:  {probe: (sample time, error or None)} for each probe that has run.
        session: values the `when` conditions compare against.
        """
        now = self.clock()
        results, transitions = {}, []
        for spec in self.specs:
            result = self._evaluate(spec, facts, probes, session or {}, now)
            if result is None:                   # skipped: hidden, and its history forgotten
                self._shown.pop(spec.id, None)
                self._pending.pop(spec.id, None)
                continue
            shown = self._debounce(spec, result.status, now)
            old = self._shown.get(spec.id, 'ok')
            if shown != old:
                transitions.append(Transition(spec.id, old, shown, result, now))
            self._shown[spec.id] = shown
            results[spec.id] = dataclasses.replace(result, status=shown)
        self.last = results
        transitions = [t for t in transitions if t.id not in self.muted]
        return HealthReport(results, tuple(transitions), frozenset(self.muted))

    def mute(self, check_id, muted=True):
        """Silence (or restore) one check for the session: no toasts, no gating."""
        (self.muted.add if muted else self.muted.discard)(check_id)

    def gate(self, procedure_key, bench=False, motion=True):
        """(blocking reasons, warnings) from the last tick for starting ``procedure_key``.

        A check gates a procedure when its `gates` names the key, or `motion`
        (only when ``motion``, i.e. the procedure moves the boat, and skipped
        under the bench override, which runs without sensors). fail
        blocks; warn and unknown (nothing known, e.g. no session) only warn.
        Muted checks never gate.
        """
        if self.last is None:
            return [], ['health has not been evaluated yet']
        blocking, warnings = [], []
        for spec in self.specs:
            wanted = procedure_key in spec.gates or ('motion' in spec.gates and motion and not bench)
            result = self.last.get(spec.id)
            if not wanted or result is None or spec.id in self.muted:
                continue
            reason = f'{spec.title}: {result.detail or result.value or result.status}'
            if result.status == 'fail':
                blocking.append(reason)
            elif result.status in ('warn', 'unknown'):
                warnings.append(reason)
        return blocking, warnings

    def _fresh(self, kind, facts, now):
        """Facts without those whose own collection time is older than the kind's probes allow.

        A fact with time 0 carries no timestamp and is trusted; the probe-level
        check in _evaluate still covers it.
        """
        periods = [self.periods[p] for p in kind.needs if self.periods.get(p)]
        if not periods:
            return facts
        limit = STALE_PERIODS * max(periods)
        return {name: fact for name, fact in facts.items()
                if not fact.time or now - fact.time <= limit}

    def _evaluate(self, spec, facts, probes, session, now):
        """The undebounced Result, capped at the check's severity; None when skipped."""
        if any(session.get(k) != v for k, v in spec.when.items()):
            return Result('ok')
        kind = KINDS[spec.kind]
        for probe in kind.needs + ((spec.params.probe,) if getattr(spec.params, 'probe', '') else ()):
            sample = probes.get(probe)
            if sample is None:
                return self._missing(spec, f'no {probe} sample')
            taken, error = sample
            if error:
                return self._missing(spec, f'{probe} probe failed: {error}')
            period = self.periods.get(probe)
            if period and now - taken > STALE_PERIODS * period:
                return self._missing(spec, f'no {probe} sample for {now - taken:.1f} s')
        try:
            result = kind.evaluate(spec.params, self._fresh(kind, facts, now))
            if result.status not in STATUSES:
                raise ValueError(f'unknown status {result.status!r}')
        except Exception as error:               # a buggy kind or odd fact must not stop the tick
            result = Result('unknown', detail=f'{spec.kind} failed: {error!r}')
        if result.status == 'unknown':
            return self._missing(spec, result.detail, result)
        return self._cap(spec, result)

    def _missing(self, spec, detail, result=None):
        """Apply the check's `missing` policy to absent or stale evidence."""
        if spec.missing == 'skip':
            return None
        status = 'fail' if spec.missing == 'fail' else 'unknown'
        base = result or Result('unknown')
        return self._cap(spec, dataclasses.replace(base, status=status, detail=detail))

    @staticmethod
    def _cap(spec, result):
        """The result's status never exceeds the check's severity (unknown is not capped)."""
        if result.status == 'unknown':
            return result
        if STATUSES.index(result.status) > STATUSES.index(spec.severity):
            return dataclasses.replace(result, status=spec.severity)
        return result

    def _debounce(self, spec, raw, now):
        """The status to show: a change must persist for its raise or clear time.

        A change is a band of results on one side of the shown status. The band
        keeps its start while results stay in it, so alternating or blipping
        results still count as persisting, and it resolves to the worst status
        seen in it. A raise band survives clear_after of better results before
        it is dropped; a clear band is dropped by the first result not better.
        """
        shown = self._shown.get(spec.id, 'ok')
        rank = STATUSES.index(raw) - STATUSES.index(shown)
        band = self._pending.get(spec.id)          # [worse, since, worst status, off since]
        if band and not (rank > 0 if band[0] else rank < 0):
            if band[3] is None:
                band[3] = now                      # first result outside the band
            if now - band[3] >= (spec.clear_after if band[0] else 0):
                band = None                        # it stayed outside long enough
        elif band:
            band[3] = None
        if rank and band is None:
            band = [rank > 0, now, raw, None]
        if band and (rank > 0) == band[0] and rank:
            band[2] = max(band[2], raw, key=STATUSES.index)
        if band is None:
            self._pending.pop(spec.id, None)
            return shown
        self._pending[spec.id] = band
        worse, since, peak, _ = band
        if now - since >= (self._raise_after(spec, peak) if worse else spec.clear_after):
            self._pending.pop(spec.id, None)
            return peak
        return shown

    @staticmethod
    def _raise_after(spec, status):
        if spec.raise_after is not None:
            return spec.raise_after
        return RAISE_DEFAULT.get(status, RAISE_OTHER)

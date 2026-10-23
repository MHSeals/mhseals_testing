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


class Engine:
    """Turns `tick(facts, probes)` calls into a HealthReport."""

    def __init__(self, specs, periods=None, clock=time.monotonic):
        self.specs = [s for s in specs if s.enabled]
        self.periods = periods or {}             # probe -> seconds between samples
        self.clock = clock
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
        return HealthReport(results, tuple(transitions))

    def _evaluate(self, spec, facts, probes, session, now):
        """The undebounced Result, capped at the check's severity; None when skipped."""
        if any(session.get(k) != v for k, v in spec.when.items()):
            return Result('ok')
        kind = KINDS[spec.kind]
        for probe in kind.needs:
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
            result = kind.evaluate(spec.params, facts)
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
        """The status to show: a change must persist for its raise or clear time."""
        shown = self._shown.get(spec.id, 'ok')
        if raw == shown:
            self._pending.pop(spec.id, None)
            return shown
        candidate, since = self._pending.get(spec.id, (None, now))
        if candidate != raw:
            since = now
            self._pending[spec.id] = (raw, now)
        worse = STATUSES.index(raw) > STATUSES.index(shown)
        delay = self._raise_after(spec, raw) if worse else spec.clear_after
        if now - since >= delay:
            self._pending.pop(spec.id, None)
            return raw
        return shown

    @staticmethod
    def _raise_after(spec, status):
        if spec.raise_after is not None:
            return spec.raise_after
        return RAISE_DEFAULT.get(status, RAISE_OTHER)

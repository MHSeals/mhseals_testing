"""Run the engine over the probes and fold its results into dashboard advisory rows.

Checks that share an ``advisory:`` name become one row: the level is the worst
of their results; the detail is the first check's detail, then the rest after
a ``;`` (``panels.status.short`` shows what precedes it, so the first check leads). A row with no
evidence yet is left out, so a boat without an FCU shows no battery row; once a
row has shown a value, a silent probe turns it N/A instead of hiding it.
"""

import time

from mhseals_testing import calibration
from mhseals_testing.health import probes as probe_module
from mhseals_testing.health.config import ConfigError, load
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.model import STATUSES

LEVELS = {'ok': 'OK', 'info': 'OK', 'unknown': 'N/A', 'warn': 'WARN', 'fail': 'FAIL'}


def rows(specs, report, seen):
    """{advisory name: (level, detail)} from the engine's results.

    ``seen`` is the set of advisory names that have ever carried a value; it grows here.
    """
    grouped = {}
    for spec in specs:
        result = report.results.get(spec.id)
        if spec.advisory and result is not None:
            grouped.setdefault(spec.advisory, []).append(result)
    out = {}
    for name, results in grouped.items():
        seen.update([name] if any(r.value for r in results) else [])
        if name not in seen:
            continue
        worst = max((r.status for r in results), key=STATUSES.index)
        known = [r for r in results if r.status != 'unknown'] or results
        details = [r.detail for r in known if r.detail]
        out[name] = (LEVELS[worst], '; '.join(filter(None, (details[:1] and details[0],
                                                           ', '.join(details[1:])))))
    return out


class Health:
    """The engine, the shared fact store and the App-tick probes."""

    def __init__(self, profile, bag_directory, facts=None, clock=None, directory=None, host=None):
        self.facts = facts or probe_module.Facts(clock or time.monotonic)
        self.host = host or (lambda: probe_module.host_facts(bag_directory))
        self.error = ''
        try:
            config = load(directory, profile)
        except ConfigError as error:
            self.error, config = str(error), None
        self.specs = tuple(s for s in config.specs if s.enabled) if config else ()
        periods = {n: p['period'] for n, p in (config.probes if config else {}).items()
                   if isinstance(p, dict) and 'period' in p}
        self.engine = Engine(self.specs, periods, self.facts.clock)
        self.periods = periods
        self._ran, self._seen = {}, set()
        self.transitions = []           # engine transitions not yet taken by drain()

    def tick(self, snapshot):
        """Run the due App-tick probes, evaluate, and return the advisory rows."""
        now = self.facts.clock()
        for name, probe in (('snapshot', lambda: probe_module.snapshot_facts(snapshot)),
                            ('calibration', lambda: probe_module.staleness_facts(calibration.load())),
                            ('host', self.host)):
            if now - self._ran.get(name, float('-inf')) >= self.periods.get(name, 1):
                self._ran[name] = now
                try:
                    self.facts.publish(name, probe())
                except Exception as error:      # a failing probe reads as unknown, not a crash
                    self.facts.publish(name, {}, error=repr(error))
        facts, probes = self.facts.read()
        report = self.engine.tick(facts, probes)
        self.transitions.extend(report.transitions)
        return rows(self.specs, report, self._seen)

    def drain(self):
        """The transitions since the last call, oldest first."""
        taken, self.transitions = self.transitions, []
        return taken

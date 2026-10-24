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
from mhseals_testing.health.probes import git as git_probe
from mhseals_testing.ui.deploy import deploy_text

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

    def __init__(self, profile, bag_directory, facts=None, clock=None, directory=None, host=None, git=None,
                 hardware=None):
        self.facts = facts or probe_module.Facts(clock or time.monotonic)
        self.host = host or (lambda: probe_module.host_facts(bag_directory))
        self.hardware = hardware or (lambda: None)      # the hardware config now, for calibration.invalid
        self.error = ''
        self.directory, self.profile, self.pending = directory, profile, None
        try:
            config = load(directory, profile)
        except ConfigError as error:
            self.error, config = str(error), None
        self.adopt(config)
        self._ran, self._seen = {}, set()
        # The git worker (health.md 5.1: 30 s, fetch off) fills its own `latest`; tick() hands that to the facts.
        self.git = git if git is not None else git_probe.from_specs(
            self.specs, (config.probes if config else {}).get('git'))
        self.git_thread = None
        self._git_published = None      # the last `latest` handed to the facts
        self.running_hash = None        # the testing checkout's commit when the first sample landed
        self.transitions = []           # engine transitions not yet taken by drain()

    def adopt(self, config):
        """Run the engine on ``config``'s checks (a new engine, keeping the session mutes)."""
        self.specs = tuple(s for s in config.specs if s.enabled) if config else ()
        self.periods = {n: p['period'] for n, p in (config.probes if config else {}).items()
                        if isinstance(p, dict) and 'period' in p}
        muted = self.engine.muted if hasattr(self, 'engine') else ()
        self.engine = Engine(self.specs, self.periods, self.facts.clock)
        self.engine.muted.update(muted)

    def reload(self):
        """Parse the rules again (raises ConfigError); ``adopt_pending`` takes them when free."""
        self.pending = load(self.directory, self.profile)

    def adopt_pending(self):
        """Take the reloaded rules, if any."""
        if self.pending is not None:
            config, self.pending, self.error = self.pending, None, ''
            self.adopt(config)

    def tick(self, snapshot):
        """Run the due App-tick probes, evaluate, and return the advisory rows."""
        now = self.facts.clock()
        for name, probe in (('snapshot', lambda: probe_module.snapshot_facts(snapshot)),
                            ('calibration', self.calibration),
                            ('host', self.host)):
            if now - self._ran.get(name, float('-inf')) >= self.periods.get(name, 1):
                self._ran[name] = now
                try:
                    self.facts.publish(name, probe())
                except Exception as error:      # a failing probe reads as unknown, not a crash
                    self.facts.publish(name, {}, error=repr(error))
        self.sample_git()
        facts, probes = self.facts.read()
        report = self.engine.tick(facts, probes)
        self.transitions.extend(report.transitions)
        return rows(self.specs, report, self._seen)

    def calibration(self):
        """calibration.stale and calibration.invalid from calibration/current.json."""
        current = calibration.load()
        return {**probe_module.staleness_facts(current), **probe_module.invalid_facts(current, self.hardware())}

    def sample_git(self):
        """Start the git worker on first use and publish whatever it has sampled."""
        if not self.git.repos:
            return
        if self.git_thread is None:
            self.git_thread = self.git.start(self.periods.get('git', git_probe.PERIOD))
        latest = self.git.latest
        if latest and latest is not self._git_published:    # republishing would restamp old facts as fresh
            self._git_published = latest
            self.facts.publish('git', latest)
            hash_ = getattr(latest.get('git.testing'), 'value', {}).get('hash')
            self.running_hash = self.running_hash or hash_

    def deploy(self):
        """The header deploy Text from the git_state results (None until the probe has run)."""
        report = self.engine.last or {}
        states = [r.data for i, r in report.items() if i.startswith('git.') and isinstance(r.data, dict)]
        testing = report.get('git.testing')
        hash_ = testing.data.get('hash') if testing and isinstance(testing.data, dict) else None
        newer = bool(self.running_hash and hash_ and hash_ != self.running_hash)
        return deploy_text(states, code_newer=newer)

    def close(self):
        self.git.stop()

    def drain(self):
        """The transitions since the last call, oldest first."""
        taken, self.transitions = self.transitions, []
        return taken

"""What health was doing around a procedure: transitions, snapshots and trial-window validity.

Reports used to show every sensor STALE because health was read after teardown.
The journal is fed each tick's transitions, takes a snapshot at the start and
again before teardown, and answers whether a check that gates a procedure was
not ok at any moment inside its window. Whoever ticks the engine must pass each
report to ``record`` (App.health_tick does); a snapshot's own time is stored as ``at``.
"""

from .model import STATUSES

NOT_OK = ('unknown', 'warn', 'fail')


class Journal:
    def __init__(self):
        self.changes = {}        # check id -> [(time, status)] in time order
        self.snapshots = []      # {label, time, counts, bad}

    def record(self, report):
        """Remember a tick's transitions."""
        for t in report.transitions:
            self.changes.setdefault(t.id, []).append((t.time, t.new))

    def status(self, check_id, at):
        """The shown status of ``check_id`` at ``at`` ('ok' before any change)."""
        shown = 'ok'
        for time, status in self.changes.get(check_id, ()):
            if time > at:
                break
            shown = status
        return shown

    def during(self, check_id, since, until):
        """The worst status ``check_id`` showed anywhere in [since, until]."""
        seen = [self.status(check_id, since)]
        seen += [s for t, s in self.changes.get(check_id, ()) if since < t <= until]
        return max(seen, key=STATUSES.index)

    def snapshot(self, label, results, now):
        """Record and return the shown statuses (``results``: id -> Result) at ``now``."""
        counts = {}
        for result in results.values():
            counts[result.status] = counts.get(result.status, 0) + 1
        entry = {'label': label, 'at': now, 'counts': counts,
                 'bad': {i: r.status for i, r in results.items() if r.status in NOT_OK}}
        self.snapshots.append(entry)
        return entry

    def window_problems(self, engine, procedure_key, since, until, motion=True):
        """Gated checks of ``procedure_key`` that were not ok inside the window, as strings.

        An empty list means the window is valid; anything else means analysis
        must not fit through it. `motion` gates count only when ``motion``. Muted checks are ignored, as they are for gating.
        """
        problems = []
        for spec in engine.specs:
            if not {procedure_key, 'motion' if motion else None} & set(spec.gates) or spec.id in engine.muted:
                continue
            worst = self.during(spec.id, since, until)
            if worst in NOT_OK:
                problems.append(f'{spec.title} was {worst}')
        return problems

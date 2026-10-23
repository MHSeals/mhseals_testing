"""Toast feed: the engine's transitions become toasts (health.md 5.4).

For the first GRACE seconds nothing toasts, since start-up is noisy; then one summary toast
counts what is still non-ok. After that each transition goes to ``app.toasts.transition``
(which ignores info and unknown, dedupes and shows recoveries).
"""

from mhseals_testing.ui import theme

GRACE = 10.0


class ToastFeed:
    """``health`` offers ``drain()``, ``specs`` (with ``id`` and ``title``) and ``engine.last`` and ``engine.muted``."""

    def __init__(self, health, grace=GRACE):
        self.health, self.grace = health, grace
        self.start = None
        self.summarised = False

    def tick(self, app, snapshot, now):
        transitions = self.health.drain()
        if self.start is None:
            self.start = now
        if not self.summarised:
            if now - self.start < self.grace:
                return                      # transitions inside the grace are summarised, not toasted
            self.summarised = True
            self.summarise(app, now)
            return
        titles = {s.id: s.title for s in self.health.specs}
        for transition in transitions:
            app.toasts.transition(transition, titles.get(transition.id, transition.id), now)

    def summarise(self, app, now):
        engine = self.health.engine
        shown = [r for id, r in (engine.last or {}).items() if id not in engine.muted]
        parts = [f'{n} {status}' for status in ('fail', 'warn')
                 if (n := sum(r.status == status for r in shown))]
        if parts:
            app.toasts.summary(f"Health: {theme.SEPARATOR.join(parts)}{theme.SEPARATOR}! details", now)

    def view(self):
        return {}

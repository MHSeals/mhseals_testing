"""Health feed: the engine's latest report as the counts the header and Status row 0 show."""

SEVERITY = {'fail': 'bad', 'warn': 'warn'}   # engine status -> view key; ok and info count as ok


def summary(engine):
    """``{ok, warn, bad, worst}`` of ``engine.last``, or None before the first tick.

    Muted checks are left out (they never toast or gate); ``unknown`` ones count nowhere.
    ``worst`` is the title of the first check in the worst band, '' when nothing is wrong.
    """
    results = engine.last
    if results is None:
        return None
    muted = engine.muted
    counts = {'ok': 0, 'warn': 0, 'bad': 0, 'worst': ''}
    first = {}                                   # view key -> title of its first check
    for spec in engine.specs:
        result = results.get(spec.id)
        if result is None or spec.id in muted:
            continue
        key = SEVERITY.get(result.status, 'ok' if result.status in ('ok', 'info') else None)
        if key:
            counts[key] += 1
            first.setdefault(key, spec.title or spec.id)
    counts['worst'] = first.get('bad') or first.get('warn') or ''
    return counts


class HealthFeed:
    """Samples ``app.engine`` each live tick; ``view()['health']`` is absent without an engine."""

    def __init__(self):
        self.counts = None

    def tick(self, app, snapshot, now):
        self.counts = summary(app.engine)

    def view(self):
        return {'health': self.counts} if self.counts else {}

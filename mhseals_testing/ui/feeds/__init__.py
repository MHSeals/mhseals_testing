"""Feeds: one small object per data source the dashboard shows.

A feed has ``tick(app, snapshot, now)``, called on each live tick, and ``view() -> dict``, the
keys it adds to ``App.view()``. ``App.feeds`` holds them, so a new data source is a new feed,
not an edit of ``App.view()``.
"""


class Feed:
    """The interface; a feed need not subclass it."""

    def tick(self, app, snapshot, now):
        """Sample the source (no-op by default)."""

    def view(self):
        """The view keys this feed contributes."""
        return {}


def merged(feeds):
    """The union of every feed's view; a later feed wins on a shared key."""
    out = {}
    for feed in feeds:
        out.update(feed.view())
    return out

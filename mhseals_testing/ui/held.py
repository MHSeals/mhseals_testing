"""Panels that change slowly, laid out at their own rate while the screen redraws continuously (perf.yaml ``hold``).

While the screen redraws continuously (driving, a run, a popup) most of a frame's cost is Rich laying out panels
whose text has not changed: the Status column, Procedures, Activity, Thrusters. In those frames (``continuous``)
``Held`` lays its panel out at most once per its ``hold`` period and draws the lines it laid out last in between.
Every other frame (a key's, a popup's, a still screen's change, any render outside the loop) lays every panel out,
so what a key changes shows in its own frame. A panel missing from ``hold`` (or 0) is laid out every frame.
"""

import time
from contextlib import contextmanager

from rich.segment import Segment

ACTIVE = False          # inside a continuous frame: held panels may draw their last layout
_cache = {}             # name -> (width, height, time, lines)


@contextmanager
def continuous():
    """The frame drawn inside is a continuous one (perf.draw with force=False)."""
    global ACTIVE
    ACTIVE = True
    try:
        yield
    finally:
        ACTIVE = False


def periods():
    """config/perf.yaml ``hold``: panel name -> s between layouts."""
    from mhseals_testing.tunables import tune
    return tune('perf').get('hold') or {}


class Held:
    """``panel`` (a renderable, or a callable making one) laid out at most every ``period`` s in continuous frames."""

    def __init__(self, name, panel, period, clock=time.monotonic):
        self.name, self.panel, self.period, self.clock = name, panel, period, clock

    def __rich_console__(self, console, options):
        width, height, now = options.max_width, options.height, self.clock()
        held = _cache.get(self.name)
        if not ACTIVE or held is None or held[:2] != (width, height) or now - held[2] >= self.period:
            panel = self.panel() if callable(self.panel) else self.panel
            held = _cache[self.name] = (width, height, now, console.render_lines(panel, options, pad=height is not None))
        new_line = Segment.line()
        for line in held[3]:
            yield from line
            yield new_line


def hold(name, panel, table=None):
    """``panel`` held at its ``hold`` period, else the panel itself (None stays None: an empty slot)."""
    if panel is None:
        return None
    period = (periods() if table is None else table).get(name)
    if period:
        return Held(name, panel, period)
    return panel() if callable(panel) else panel

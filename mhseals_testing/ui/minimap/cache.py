"""Map layer caches and the redraw scheduler (map.md section 9); pure and ROS-free.

A slow layer (costmap, referee, truth, course) is built once per ``(view, version)`` key and
kept as a `Canvas`. A frame starts from copies of those and paints only the per-frame things
(boat, tracks, labels, cursor). `Redraw` decides when a frame is due: only on change, at most
`MAP_HZ` on the Map tab or `DRIVE_HZ` on Drive, never while hidden, and at `DRIVE_HZ` when the
measured frame cost stays over `BUDGET_MS`.
"""

from mhseals_testing.ui.minimap.canvas import Canvas

MAP_HZ = 5.0
DRIVE_HZ = 2.0
BUDGET_MS = 8.0  # cached path on the boat (map.md 9); the x86 regression guard is 3 ms
SMOOTH = 0.3     # weight of the newest frame cost in the running average


class LayerCache:
    """Canvases for named layers, each valid for one ``(view, version)`` key."""

    def __init__(self):
        self._items = {}  # name -> (key, canvas)

    def get(self, name, key, build):
        """The layer ``name`` for ``key``; ``build()`` runs only when the key changed."""
        held = self._items.get(name)
        if held is None or held[0] != key:
            held = self._items[name] = (key, build())
        return held[1]

    def frame(self, cols, rows, layers):
        """A fresh canvas holding every ``(name, key, build)`` layer, ready for per-frame paint."""
        out = Canvas(cols, rows)
        for name, key, build in layers:
            layer = self.get(name, key, build)
            out.overlay(layer)
        return out


class Redraw:
    """Redraw-on-change with a rate cap and the over-budget fallback."""

    def __init__(self, map_hz=MAP_HZ, drive_hz=DRIVE_HZ, budget_ms=BUDGET_MS):
        self.map_hz, self.drive_hz, self.budget_ms = map_hz, drive_hz, budget_ms
        self.cost_ms = 0.0   # running average of the frame cost, shown in the footer
        self._last = None    # time of the last frame
        self._dirty = True

    def mark(self):
        """Something the frame shows changed (a stamp, the view, a key)."""
        self._dirty = True

    @property
    def slow(self):
        """True while the average cost is over budget; the subtitle then shows `map 2 Hz`."""
        return self.cost_ms > self.budget_ms

    def hz(self, on_map_tab):
        return self.drive_hz if self.slow or not on_map_tab else self.map_hz

    def due(self, now, visible=True, on_map_tab=True):
        """True when a frame should be drawn at ``now`` (seconds); a hidden view never draws."""
        if not visible or not self._dirty:
            return False
        return self._last is None or now - self._last >= 1.0 / self.hz(on_map_tab) - 1e-9

    def done(self, now, cost_ms):
        """Record that a frame was drawn at ``now`` and took ``cost_ms``."""
        self._last, self._dirty = now, False
        self.cost_ms = cost_ms if self.cost_ms == 0.0 else (
            self.cost_ms + SMOOTH * (cost_ms - self.cost_ms))

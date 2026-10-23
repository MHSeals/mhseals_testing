"""Time axis of the activity strip: maps session time onto terminal columns.

Pure arithmetic, no Rich. Idle stretches fold into one column only when that
saves columns, every other column covers the same span, and columns are whole
and anchored at the right edge so the picture shifts a column at a time.
"""

import math
import time

# Seconds per column the scale may take; adjacent steps differ by at most 1.5x.
STEPS = [1, 1.5, 2, 3, 4, 5, 7.5, 10, 15, 20, 30, 45, 60, 90, 120, 180, 240, 300,
         450, 600, 900, 1200, 1800, 2700, 3600]
MIN_SCALE = 5.0
FOLD_MIN = 180.0           # shortest idle gap worth folding
PAD = 30.0                 # busy margin around every busy interval
TAIL = 120.0               # the most recent seconds are never folded
LEAD_MIN = 180.0           # leading idle beyond this is trimmed
TICK_MINUTES = (1, 2, 5, 10, 15, 30, 60, 120)
TICK_GAP = 9               # columns between tick labels
FOLD = 'fold'


def clock(t):
    """Wall-clock hh:mm of a session time."""
    return time.strftime('%H:%M', time.localtime(t))


def duration(seconds):
    """Short length of a folded gap: 47m, 3h."""
    return f'{round(seconds / 60)}m' if seconds < 5940 else f'{seconds / 3600:.0f}h'


def quantise(scale):
    """Smallest step that is at least `scale`; whole hours beyond the last step."""
    step = next((step for step in STEPS if step >= scale - 1e-9), None)
    return step or math.ceil(scale / STEPS[-1] - 1e-9) * STEPS[-1]


def merge(intervals):
    """Union of (a, b) intervals, sorted."""
    merged = []
    for a, b in sorted(intervals):
        if merged and a <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], b))
        else:
            merged.append((a, b))
    return merged


class Axis:
    """Columns for the session from t0 to end, `width` wide.

    busy is a list of (a, b) intervals that must stay readable. window=(a, b)
    gives a linear axis without folds (zoom, loupe, popup); exact=True fills the
    width with an unquantised scale for a static session (review).
    """

    def __init__(self, t0, end, width, busy, window=None, exact=False):
        self.t0, self.end, self.width = t0, end, width
        self._right = end          # layout edge: `end`, or the window's right side
        self.start, self.folds, self.scale = t0, [], MIN_SCALE
        if width < 1:
            self._cols = []
            return
        if window:
            self.start, self._right = window
            self.scale = max(window[1] - window[0], 1e-9) / width
            self._cols = self._layout([], self.scale)
            assert self._cols, 'window layout must fit its own width'
            return
        self._plan(busy, exact)

    def _plan(self, busy, exact):
        """Pick the trimmed start, the fold set and the scale together."""
        edge = [(max(self.t0, a - PAD), min(self.end, b + PAD)) for a, b in busy]
        busy = merge(edge + [(max(self.t0, self.end - TAIL), self.end)])
        gaps = [(busy[i][1], busy[i + 1][0]) for i in range(len(busy) - 1)]
        if busy[0][0] - self.t0 > LEAD_MIN:
            self.start = busy[0][0]
        floor = 1e-6 if exact else MIN_SCALE   # review fills the width, live grows from the right
        scale = quantise(max(floor, (self.end - self.start) / self.width))
        folds = []
        for _ in range(3):
            folds = [g for g in gaps if g[0] >= self.start
                     and g[1] - g[0] > max(FOLD_MIN, 2 * scale)]
            # Each fold and each segment around it needs a column of its own.
            keep = sorted(sorted(folds, key=lambda g: g[0] - g[1])[:(self.width - 1) // 2])
            folds = keep
            active = self.end - self.start - sum(b - a for a, b in folds)
            scale = max(floor, active / max(1, self.width - len(folds)))
            scale = scale if exact else quantise(scale)
        # Whole columns can overflow by one per segment: coarsen until it fits.
        # A coarser scale can make a fold worthless: drop it and settle again.
        while True:
            cols = self._layout(folds, scale)
            while cols is None:
                scale = scale * 1.01 if exact else quantise(scale * 1.01)
                cols = self._layout(folds, scale)
            kept = [g for g in folds if g[1] - g[0] > max(FOLD_MIN, 2 * scale)]
            if len(kept) == len(folds):
                break
            folds = kept
            active = self.end - self.start - sum(b - a for a, b in folds)
            grown = max(floor, active / max(1, self.width - len(folds)))
            scale = max(scale, grown if exact else quantise(grown))
        self.scale, self.folds, self._cols = scale, folds, cols

    def _layout(self, folds, scale):
        """Whole columns right to left from the right edge; None when they do not fit."""
        cols = [None] * self.width
        pos, edge = self.width, self._right
        for a, b in reversed(folds):
            pos = self._fill(cols, pos, edge, b, scale)
            if pos < 1:
                return None
            pos -= 1
            cols[pos] = (FOLD, b - a)
            edge = a
        pos = self._fill(cols, pos, edge, self.start, scale)
        return cols if pos >= 0 else None

    @staticmethod
    def _fill(cols, pos, edge, floor, scale):
        """Fill columns left of pos covering floor..edge; return the new pos."""
        count = max(1, math.ceil((edge - floor) / scale - 1e-9))
        for k in range(1, count + 1):
            if pos - k < 0:
                return -1
            cols[pos - k] = (max(floor, edge - k * scale), edge - (k - 1) * scale)
        return pos - count

    def cols(self):
        """Per column: (ta, tb), (FOLD, seconds), or None before the session."""
        return self._cols

    def col(self, t):
        """Column holding time t, or None when it is folded or off the axis."""
        for i, c in enumerate(self._cols):
            if c and c[0] != FOLD and c[0] <= t < c[1]:
                return i
        return None

    def extent(self, t0, t1):
        """Columns (a, b) covered by t0..t1, clamped to the visible content."""
        real = [i for i, c in enumerate(self._cols) if c and c[0] != FOLD]
        if not real:
            return None
        a = self.col(t0) if self.col(t0) is not None else (
            real[0] if t0 < self._cols[real[0]][0] else real[-1])
        b = self.col(t1 - 1e-6) if self.col(t1 - 1e-6) is not None else (
            real[0] if t1 < self._cols[real[0]][0] else real[-1])
        return a, max(a, b)

    def labels(self, end_text='now', sel=None, fmt=clock):
        """Non-touching (column, text, role) labels in placement priority.

        Order: selection time, end label, fold lengths, start, ticks. A label
        that would touch an earlier one (or the selection bracket) is dropped.
        """
        placed, taken = [], []

        def put(col, text, role):
            if col < 0 or col + len(text) > self.width:
                return
            if any(col - 1 < b and col + len(text) + 1 > a for a, b in taken):
                return
            taken.append((col, col + len(text)))
            placed.append((col, text, role))

        if sel:
            a, b = self.extent(*sel) or (0, 0)
            taken.append((a, b + 1))
            text = fmt(sel[0])
            right = b + 2
            put(right if right + len(text) <= self.width else a - len(text) - 1, text, 'sel')
        put(self.width - len(end_text), end_text, 'end')
        for i, c in enumerate(self._cols):
            if c and c[0] == FOLD:
                text = duration(c[1])
                put(max(0, i - len(text) // 2), text, 'fold')
        real = [i for i, c in enumerate(self._cols) if c and c[0] != FOLD]
        if real:
            put(real[0], fmt(max(self._cols[real[0]][0], self.t0)), 'start')
            step = next((m * 60 for m in TICK_MINUTES if m * 60 / self.scale >= TICK_GAP),
                        TICK_MINUTES[-1] * 60)
            for i in real:
                tick = math.ceil(self._cols[i][0] / step) * step
                if self._cols[i][0] <= tick < self._cols[i][1]:
                    put(i - 2, fmt(tick), 'tick')
        return placed

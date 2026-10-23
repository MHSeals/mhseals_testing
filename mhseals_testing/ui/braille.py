"""A braille dot canvas for the motion plots: pure, no terminal access.

A cell is 2 dots wide and 4 high. Series are binned to one dot column each
(mean), gaps longer than ``GAP`` seconds stay empty, and samples outside the
scale are drawn on the edge row, non-finite samples are skipped and a flat or
non-finite scale draws on the centre row. Two layers share a canvas: ``meas`` in the
accent color, ``pred`` muted; a cell holding both takes the accent.
"""

import math

from rich.text import Text

from mhseals_testing.ui import theme

GAP = 0.5
LAYERS = {'pred': theme.MUTED, 'meas': theme.ACCENT}

# Dot bit per (column, row) inside a cell, as in the Unicode braille block.
_BITS = ((0x01, 0x02, 0x04, 0x40), (0x08, 0x10, 0x20, 0x80))
# ASCII fallback, by dots in the cell's upper and lower halves: none, top, bottom, both.
_ASCII = ' \'.:'


def _row(value, low, high, rows):
    """Dot row of ``value``; a flat or non-finite scale or value puts it mid-height."""
    if not (math.isfinite(low) and math.isfinite(high) and math.isfinite(value)) or high <= low:
        return (rows - 1) // 2
    return round((high - min(max(value, low), high)) / (high - low) * (rows - 1))


class Canvas:
    def __init__(self, width, height):
        self.width, self.height = width, height
        self.dots = {layer: set() for layer in LAYERS}
        self.lines = []

    def line(self, series, t0, t1, low, high, layer='meas'):
        """Plot ``[(t, v)]`` over the window t0..t1 on the scale low..high.

        Returns the clip marks as a set of ``'▲'`` / ``'▼'`` for the label.
        """
        columns, rows = self.width * 2, self.height * 4
        span = t1 - t0
        bins = {}
        # A non-finite window, or one whose span overflows, plots nothing.
        for t, v in series if math.isfinite(t0) and math.isfinite(span) and span > 0 else ():
            if math.isfinite(v) and t0 <= t <= t1:
                x = (t - t0) / span
                if math.isfinite(x):
                    bins.setdefault(min(columns - 1, int(x * columns)), []).append((t, v))
        clipped, last = set(), None
        for x in sorted(bins):
            v = sum(v for _, v in bins[x]) / len(bins[x])
            if v > high:
                clipped.add('^' if theme.ASCII else '▲')
            elif v < low:
                clipped.add('v' if theme.ASCII else '▼')
            y = _row(v, low, high, rows)
            self.dots[layer].add((x, y))
            # Join to the previous bin unless the time between its last and
            # this bin's first sample is a hole in the data.
            first, end = bins[x][0][0], bins[x][-1][0]
            if last and first - last[2] <= GAP:
                self._join(layer, last[0], last[1], x, y)
            last = (x, y, end)
        return clipped

    def _join(self, layer, x0, y0, x1, y1):
        """Connect two dots: linear across skipped columns, vertical within each step."""
        prev = y0
        for x in range(x0 + 1, x1 + 1):
            cur = round(y0 + (y1 - y0) * (x - x0) / (x1 - x0))
            self.dots[layer].add((x, cur))
            for fill in range(min(cur, prev) + 1, max(cur, prev)):
                self.dots[layer].add((x if fill > (cur + prev) / 2 else x - 1, fill))
            prev = cur

    def hline(self, value, low, high, char=None):
        """A reference line (the zero line), drawn only in cells without dots.

        ``char`` defaults to a dotted rule, or ``-`` in ASCII mode.
        """
        row = _row(value, low, high, self.height * 4) // 4
        self.lines.append((row, char))

    def rows(self):
        """One ``Text`` per cell row."""
        out = []
        for row in range(self.height):
            text = Text()
            for col in range(self.width):
                text.append(*self._cell(col, row))
            out.append(text)
        return out

    def _cell(self, col, row):
        mask, layers = {}, set()
        for layer, dots in self.dots.items():
            bits = 0
            for dx in range(2):
                for dy in range(4):
                    if (col * 2 + dx, row * 4 + dy) in dots:
                        bits |= _BITS[dx][dy]
            mask[layer] = bits
            if bits:
                layers.add(layer)
        if not layers:
            for line_row, char in self.lines:
                if line_row == row:
                    return char or ('-' if theme.ASCII else '┈'), theme.MUTED
            return ' ', None
        bits = mask['pred'] | mask['meas']
        style = LAYERS['meas' if 'meas' in layers else 'pred']
        if theme.ASCII:
            upper, lower = bits & 0x1B, bits & 0xE4
            return _ASCII[bool(upper) + 2 * bool(lower)], style
        return chr(0x2800 + bits), style


def canvas(width, height):
    return Canvas(width, height)

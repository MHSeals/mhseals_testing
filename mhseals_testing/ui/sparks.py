"""Small plots for popups: a braille dot grid, a 2-row spark strip, an eighth-block bar and a ladder.

Pure: values in, Rich ``Text`` rows out, no terminal access and no state between calls. Braille
cells are 2 dots wide and 4 high; ASCII mode draws each cell as ``' .:`` by its upper and lower
dots (as ``braille.py`` does) and the bar with ``#``. Styles are theme names read at call time.
"""

import math

from rich.text import Text

from mhseals_testing.ui import theme

_BITS = ((0x01, 0x02, 0x04, 0x40), (0x08, 0x10, 0x20, 0x80))
_ASCII = ' \'.:'
EIGHTHS = ' ▏▎▍▌▋▊▉'
RUNGS = '▁▂▃▄▅▆▇█'


class Dots:
    """A ``cols`` x ``rows`` cell grid of braille dots in layers; a later layer wins a shared cell."""

    def __init__(self, cols, rows):
        self.cols, self.rows = cols, rows
        self.layers, self.labels = [], {}

    def layer(self, style):
        dots = set()
        self.layers.append((style, dots))
        return dots

    def label(self, col, row, text, style=None):
        """Text over cells, starting at (col, row); clipped at the right edge."""
        for i, char in enumerate(text):
            if 0 <= col + i < self.cols and 0 <= row < self.rows:
                self.labels[(col + i, row)] = (char, style)

    def segment(self, dots, x0, y0, x1, y1):
        steps = max(1, int(max(abs(x1 - x0), abs(y1 - y0)) + .5))
        for i in range(steps + 1):
            dots.add((round(x0 + (x1 - x0) * i / steps), round(y0 + (y1 - y0) * i / steps)))

    def ring(self, dots, cx, cy, r, every=1):
        """A circle of radius ``r`` dots; ``every`` > 1 leaves it dotted."""
        n = max(8, int(2 * math.pi * r))
        for i in range(0, n, every):
            a = 2 * math.pi * i / n
            dots.add((round(cx + r * math.cos(a)), round(cy + r * math.sin(a))))

    def rows_text(self):
        out = []
        for row in range(self.rows):
            text = Text()
            for col in range(self.cols):
                if (col, row) in self.labels:
                    text.append(*self.labels[(col, row)])
                    continue
                bits, style = 0, None
                for layer_style, dots in self.layers:
                    mine = 0
                    for dx in range(2):
                        for dy in range(4):
                            if (col * 2 + dx, row * 4 + dy) in dots:
                                mine |= _BITS[dx][dy]
                    if mine:
                        bits, style = bits | mine, layer_style
                if theme.ASCII:
                    char = _ASCII[bool(bits & 0x1B) + 2 * bool(bits & 0xE4)]
                else:
                    char = chr(0x2800 + bits) if bits else ' '
                text.append(char, style=style)
            out.append(text)
        return out


def _y(value, lo, hi, dots):
    if hi <= lo:
        return dots // 2
    return round((hi - min(max(value, lo), hi)) / (hi - lo) * (dots - 1))


def spark(values, width, lo, hi, band=None, gaps=None, height=2, style=None):
    """``height`` rows of a braille line over ``width`` cells; one value per dot column, resampled.

    None values draw nothing. ``band`` (low, high) is drawn as two dotted faint lines; ``gaps``
    (bools, one per value) marks a cell ``┄`` in the fail hue on the bottom row where data is missing.
    """
    grid, n, columns = Dots(width, height), len(values), width * 2
    band_dots, line = grid.layer(theme.PALETTE.style('faint')), grid.layer(style or theme.ACCENT)
    dots = height * 4
    if band:
        for edge in band:
            if lo <= edge <= hi:
                y = _y(edge, lo, hi, dots)
                band_dots.update((x, y) for x in range(0, columns, 2))
    prev, gap_cells = None, set()
    for x in range(columns):
        i = int(x * n / columns) if n else None
        value = values[i] if i is not None else None
        if gaps and i is not None and gaps[i]:
            gap_cells.add(x // 2)
        if value is None or not math.isfinite(value):
            prev = None
            continue
        y = _y(value, lo, hi, dots)
        for fill in range(min(y, prev if prev is not None else y), max(y, prev if prev is not None else y) + 1):
            line.add((x, fill))
        prev = y
    for col in gap_cells:
        grid.label(col, height - 1, theme.pick('┄', '.'), theme.BAD)
    return grid.rows_text()


def hbar(value, lo, hi, width, band=None, style=None):
    """``value`` on lo..hi as eighth blocks; unfilled cells inside ``band`` (low, high) show ``░``."""
    frac = 0.0 if hi <= lo or value is None else min(1.0, max(0.0, (value - lo) / (hi - lo)))
    eighths = round(frac * width * 8)
    text = Text()
    for col in range(width):
        fill = min(8, max(0, eighths - col * 8))
        if fill:
            char = theme.FILL if theme.ASCII else ('█' if fill == 8 else EIGHTHS[fill])
            text.append(char, style=style or theme.ACCENT)
            continue
        mid = lo + (hi - lo) * (col + .5) / width
        inside = band and band[0] <= mid <= band[1]
        text.append(theme.pick('░', '=') if inside else theme.pick('─', '-'), style=theme.PALETTE.style('faint'))
    return text


def ladder(steps, current, styles=None):
    """``▁ none  ▃ 3D  ...``: one rising rung per step, the ``current`` one lit, the rest faint."""
    text, faint = Text(), theme.PALETTE.style('faint')
    for i, step in enumerate(steps):
        rung = RUNGS[round(i * (len(RUNGS) - 1) / max(1, len(steps) - 1))]
        lit = step == current
        style = f"bold {(styles or {}).get(step, theme.ACCENT)}" if lit else faint
        if i:
            text.append('  ')
        text.append(f'{theme.FOCUS if lit else " "}' if theme.ASCII else '', style=style)
        text.append(theme.pick(rung, '#' if lit else '.') + ' ' + step, style=style)
    return text

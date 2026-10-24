"""Raster of the activity timeline strip (timeline.md sections 2-6).

Takes spans, points and bands on session time plus a `timeaxis.Axis`, and
returns equal-width Rich lines: events, skyline top, skyline bottom, axis and
labels (form 'panel'), or one skyline row (form 'one'). Height says who is
driving, the head letter which procedure, colour only how bad (U11: a hold is
slate, never yellow); a NO_COLOR run keeps bad slices visible with a texture.
"""

import bisect
import os
from dataclasses import dataclass

from rich.text import Text

from . import theme
from .timeaxis import FOLD, clock

WARN, FAIL = 'warn', 'fail'
SEVERITY = {'ok': 0, WARN: 1, FAIL: 2}
POINT_PRIORITY = {'health': 3, 'auth': 2, 'config': 1, 'mark': 0}
HOLD = 'bold bright_black'      # slate, bold (U11)
CLUSTER_GAP = 2                 # event columns this close form one cluster
MANUAL = 'manual'


@dataclass
class Span:
    """A procedure or manual run. end is None, 'abort' (yellow cap) or 'fault' (red cap).

    bad is the (a, b, level) slice known to be cut or failed, if any.
    """
    a: float
    b: float
    key: str
    kind: str = 'proc'
    end: str = None
    running: bool = False
    bad: tuple = None


@dataclass
class Point:
    """A point event: kind is mark, config, health or auth; level is ok/warn/fail."""
    t: float
    kind: str
    level: str = 'ok'


@dataclass
class Band:
    """An axis state over a time interval: warn, fail, recoff or armrecoff."""
    a: float
    b: float
    state: str


class Line:
    """A row of single-cell glyphs, each with its own style."""

    def __init__(self, width):
        self.cells = [(' ', '')] * width

    def put(self, col, text, style=''):
        for i, ch in enumerate(text):
            if 0 <= col + i < len(self.cells):
                self.cells[col + i] = (ch, style)

    def text(self):
        out = Text(no_wrap=True)
        for ch, style in self.cells:
            out.append(ch, style=style or None)
        return out


def _hits(col, a, b):
    """Does the interval a..b overlap the axis column?"""
    return col and col[0] != FOLD and a < col[1] and b > col[0]


def _reach(cols):
    """Real (unfolded) column indices with their start and end times, for bisecting."""
    real = [i for i, c in enumerate(cols) if c and c[0] != FOLD]
    return real, [cols[i][0] for i in real], [cols[i][1] for i in real]


def _span_range(reach, a, b):
    """Positions (lo, hi) in `real` of the columns that a..b overlaps, or None (O(log cols))."""
    _, starts, ends = reach
    lo, hi = bisect.bisect_right(ends, a), bisect.bisect_left(starts, b) - 1
    return (lo, hi) if lo <= hi else None


def _level_style(level, bold=False):
    base = {WARN: 'yellow', FAIL: 'red'}.get(level, '')
    return f'bold {base}'.strip() if bold else base


def _skyline(axis, spans, armed, holds, g, color):
    """Two rows (top, bottom) of the skyline; folded columns are filled by the caller."""
    top, bottom = Line(axis.width), Line(axis.width)
    cols = axis.cols()
    reach = _reach(cols)
    real = reach[0]
    ranges = {id(sp): _span_range(reach, sp.a, sp.b) for sp in spans}
    begins = {}         # real position -> number of procedures whose first column is there
    for sp in spans:
        if sp.kind != MANUAL and ranges[id(sp)]:
            begins[ranges[id(sp)][0]] = begins.get(ranges[id(sp)][0], 0) + 1
    for sp in spans:
        if not ranges[id(sp)]:
            continue
        lo, hi = ranges[id(sp)]
        idx = real[lo:hi + 1]
        first, last = idx[0], idx[-1]
        proc = sp.kind != MANUAL
        for i in idx:
            level = sp.bad[2] if sp.bad and _hits(cols[i], sp.bad[0], sp.bad[1]) else 'ok'
            body = g['bad'] if level != 'ok' and not color else g['body']
            style = _level_style(level) if color else ''
            if proc:
                top.put(i, body, style)
            bottom.put(i, body, style)
        row = top if proc else bottom
        head = sp.key if proc else 'm'
        if sp.a < axis.start:       # started before the window
            row.put(first, g['clip'], 'dim')
        else:
            row.put(first, head, 'reverse bold')
        if sp.end and (sp.b <= cols[last][1]):
            cap = _level_style(WARN if sp.end == 'abort' else FAIL, bold=True)
            top.put(last, g['cap'], cap)
        elif sp.running:
            top.put(last, g['tip'], 'bold')
        elif proc and hi + 1 < len(real) and real[hi + 1] == last + 1 and begins.get(hi + 1):
            top.put(last, g['notch'], '')
    for i, c in enumerate(cols):
        if c and c[0] != FOLD and bottom.cells[i][0] == ' ':
            hold = any(_hits(c, a, b) for a, b in holds)
            if hold or any(_hits(c, a, b) for a, b in armed):
                bottom.put(i, g['idle'], HOLD if hold else 'dim')
    return top, bottom


def _events(axis, points, g, color):
    """Event row: glyphs at their true column, clusters as glyph + count, off-left stub."""
    line = Line(axis.width)
    by_col = {}
    for p in points:
        col = axis.col(p.t)
        if col is not None:
            by_col.setdefault(col, []).append(p)
    off = sum(1 for p in points if p.t < axis.start)
    groups, cur = [], []
    for col in sorted(by_col):
        if cur and col - cur[-1] > CLUSTER_GAP:
            groups.append(cur)
            cur = []
        cur.append(col)
    groups.append(cur)

    def glyph(p):
        if p.kind == 'health':
            return g['health_fail'] if p.level == FAIL else g['health_warn']
        return g[{'mark': 'mark', 'config': 'config', 'auth': 'auth'}[p.kind]]

    def rank(p):
        return (SEVERITY[p.level], POINT_PRIORITY[p.kind])

    for cols in filter(None, groups):
        members = [p for c in cols for p in by_col[c]]
        singles = len(members) <= 3 and all(len(by_col[c]) == 1 for c in cols) \
            and all(b - a >= 2 for a, b in zip(cols, cols[1:]))
        if singles:
            for c in cols:
                p = by_col[c][0]
                line.put(c, glyph(p), _level_style(p.level, True) if color and p.level != 'ok' else 'bold')
        else:
            worst = max(members, key=rank)
            count = '+' if len(members) >= 10 else str(len(members))
            style = _level_style(worst.level, True) if color and worst.level != 'ok' else 'bold'
            line.put(cols[0], glyph(worst) + count, style)
    if off:
        line.put(0, g['off_left'] + str(min(off, 9)), 'dim')
    return line


def _axis_row(axis, bands, g):
    """Axis line: health and recording over time, folds as a broken line."""
    line = Line(axis.width)
    for i, c in enumerate(axis.cols()):
        if c is None:
            continue
        if c[0] == FOLD:
            line.put(i, g['fold'], 'dim')
            continue
        states = {b.state for b in bands if _hits(c, b.a, b.b)}
        for state, style in (('armrecoff', 'bold yellow'), (FAIL, 'bold red'),
                             (WARN, 'bold yellow'), ('recoff', 'dim')):
            if state in states:
                line.put(i, g[state], style)
                break
        else:
            line.put(i, g['ok'], 'dim')
    return line


def _labels(axis, g, end_text, sel, fmt):
    line = Line(axis.width)
    if sel:
        ext = axis.extent(*sel)
        if ext:
            a, b = ext
            n = b - a + 1
            bracket = g['single'] if n == 1 else g['bracket'][0] + g['bracket'][1] * (n - 2) + g['bracket'][2]
            line.put(a, bracket, 'bold cyan')
    for col, text, role in axis.labels(end_text, sel, fmt):
        line.put(col, text, 'bold cyan' if role == 'sel' else ('bold' if role == 'end' else 'dim'))
    return line


def _loupe_row(axis, bands, g, window, play):
    """Axis row of the review scrubber: `├ ┤` bracket the loupe window, `┃` is the playhead."""
    line = _axis_row(axis, bands, g)
    ext = axis.extent(*window) if window else None
    if ext:
        line.put(ext[0], g['loupe_l'], 'bold cyan')
        line.put(ext[1], g['loupe_r'], 'bold cyan')
    col = axis.col(play) if play is not None else None
    if col is not None:
        line.put(col, g['play'], 'bold cyan')
    return line


def rows(axis, spans=(), points=(), armed=(), holds=(), bands=(), form='panel', end_text='now',
         sel=None, fmt=clock, ascii_mode=None, color=None, window=None, play=None):
    """Rich lines of the strip, each exactly axis.width cells wide.

    form 'footer' (review): events, one skyline row and the axis with the loupe
    `window` bracket and the `play` head. `play` also puts the head on the panel
    form's axis row (the Activity loupe).
    """
    g = theme.timeline(ascii_mode)
    color = not os.environ.get('NO_COLOR') if color is None else color
    top, bottom = _skyline(axis, spans, armed, holds, g, color)
    for line in (top, bottom):
        for i, c in enumerate(axis.cols()):
            if c and c[0] == FOLD:
                line.put(i, g['fold'], 'dim')
    if form in ('one', 'footer'):
        one = Line(axis.width)
        for i in range(axis.width):
            if top.cells[i][0] != ' ':
                one.cells[i] = top.cells[i]
            elif bottom.cells[i][0] != ' ':
                ch, style = bottom.cells[i]
                one.cells[i] = (g['manual1'] if ch == g['body'] else ch, style)
        if form == 'footer':
            return [_events(axis, points, g, color).text(), one.text(),
                    _loupe_row(axis, bands, g, window, play).text()]
        return [one.text()]
    axis_row = _loupe_row(axis, bands, g, None, play) if play is not None else _axis_row(axis, bands, g)
    return [_events(axis, points, g, color).text(), top.text(), bottom.text(), axis_row.text(), _labels(axis, g, end_text, sel, fmt).text()]

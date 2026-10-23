"""Draw referee marks onto a map canvas (referee.md section 6); pure, no colour codes.

Gates and lines are braille segments, buoys and marks dots, bays rectangle outlines. The
colour is the layer ``ref_<state>``; with ``ascii`` the state is also a glyph so colour is
never the only cue. ``to_dot`` maps a course-frame point to canvas dot coordinates.
"""

from mhseals_testing.ui import theme

STATES = ('pending', 'next', 'passed', 'failed', 'touched', 'skipped')
MARKER = {'pending': '·', 'next': '>', 'passed': '+', 'failed': 'x', 'touched': '✕'}
ASCII_MARKER = {'pending': '.', 'next': '>', 'passed': '+', 'failed': 'x', 'touched': 'x'}


def _line(canvas, a, b, layer):
    """Bresenham between two dot coordinates."""
    (x0, y0), (x1, y1) = a, b
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx + dy
    while True:
        canvas.dot(x0, y0, layer)
        if (x0, y0) == (x1, y1):
            return
        e2 = 2 * err
        if e2 >= dy:
            err, x0 = err + dy, x0 + sx
        if e2 <= dx:
            err, y0 = err + dx, y0 + sy


def _text(canvas, col, row, text, layer):
    for i, char in enumerate(text):
        canvas.glyph(col + i, row, char, layer)


def draw(canvas, marks, to_dot, ascii_only=None):
    """Paint ``marks`` (objects with kind, points, state, text) onto ``canvas``."""
    ascii_only = theme.ASCII if ascii_only is None else ascii_only
    glyphs = ASCII_MARKER if ascii_only else MARKER
    for mark in marks:
        state = mark.state if mark.state in STATES else 'pending'
        layer = 'ref_' + state
        dots = [(round(x), round(y)) for x, y in (to_dot(p) for p in mark.points)]
        if not dots:
            continue
        if mark.kind == 'bay' and len(dots) == 4:
            for a, b in zip(dots, dots[1:] + dots[:1]):
                _line(canvas, a, b, layer)
        elif len(dots) >= 2:
            for a, b in zip(dots, dots[1:]):
                _line(canvas, a, b, layer)
        for x, y in dots[:2] if mark.kind != 'bay' else []:
            canvas.dot(x, y, layer)
            canvas.dot(x + 1, y, layer)
        cx = sum(x for x, _ in dots) // len(dots) // 2
        cy = sum(y for _, y in dots) // len(dots) // 4
        if state in glyphs and (state != 'pending' or ascii_only):
            canvas.glyph(cx, cy, glyphs[state], layer)
        if mark.text:
            _text(canvas, cx + 1, cy, mark.text, layer)

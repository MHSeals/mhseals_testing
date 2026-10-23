"""Map objects and truth onto a Canvas (map.md sections 2.2, 2.3); pure, no ROS.

`Projection` maps world metres to canvas dots and cells. Layers may carry a colour token
after a colon (``object:obj.red``); `canvas.rank` ignores it and the emitter reads it.
"""

import math
from collections import namedtuple

from mhseals_testing.ui.minimap.view import rotate

Obj = namedtuple('Obj', 'x y label score contact', defaults=(False,))
SCORE_FLOOR = 0.3   # default detection floor D (0, .3, .5, .7); below it the object is hidden
RING_HALF = 5.0     # rings show at or below this half-width, labels at LABEL_HALF
LABEL_HALF = 10.0


class Projection:
    """World (x, y) m to canvas dots; ``yaw`` set means heading-up about ``centre``."""

    def __init__(self, canvas, centre, half, yaw=None):
        self.canvas, self.centre, self.half, self.yaw = canvas, centre, half, yaw
        self.scale = canvas.dots_wide / (2 * half) if half > 0 else 0.0  # dots per metre

    def dot(self, point):
        dx, dy = rotate(point, self.centre, self.yaw)
        return (math.floor(self.canvas.dots_wide / 2 + dx * self.scale),
                math.floor(self.canvas.dots_high / 2 - dy * self.scale))

    def cell(self, point):
        x, y = self.dot(point)
        return x // 2, y // 4


def line(canvas, a, b, layer):
    """Dots strictly between dot points ``a`` and ``b`` (Bresenham)."""
    (x, y), (x1, y1) = a, b
    dx, dy, sx, sy = abs(x1 - x), -abs(y1 - y), 1 if x < x1 else -1, 1 if y < y1 else -1
    err = dx + dy
    while (x, y) != (x1, y1):
        if (x, y) != a:
            canvas.dot(x, y, layer)
        e2 = 2 * err
        if e2 >= dy:
            err, x = err + dy, x + sx
        if e2 <= dx:
            err, y = err + dx, y + sy


def strength(score):
    """Colour strength percent: full from .7, 78 from .5, 50 below (map.md 2.2)."""
    return 100 if score >= 0.7 else 78 if score >= 0.5 else 50


def draw_objects(proj, objects, table, boat=None, ascii_=False, floor=SCORE_FLOOR):
    """Class glyphs (best score keeps a cell), contact chips, rings and labels.

    A faded glyph's layer ends ``@<percent>`` (like ts115 tints); ``floor`` hides lower scores.

    ``boat`` orders label placement nearest first; without it the view centre is used.
    """
    canvas, taken, placed = proj.canvas, set(), []
    for o in sorted((o for o in objects if o.score >= floor), key=lambda o: -o.score):
        sym, pos = table.symbol(o.label), proj.cell((o.x, o.y))
        if pos in taken or not canvas._inside(*pos):
            continue
        taken.add(pos)
        char = sym.ascii if ascii_ else sym.glyph
        if o.contact:
            canvas.glyph(*pos, 'X' if ascii_ else char, 'contact', chip=True)
        else:
            fade = strength(o.score)
            canvas.glyph(*pos, char, f'object:{sym.colour}' + (f'@{fade}' if fade < 100 else ''))
        placed.append((o, sym, pos))
        if proj.half <= RING_HALF:
            _ring(proj, o, sym.radius)
    if proj.half <= LABEL_HALF:
        here = boat or proj.centre
        for o, sym, (col, row) in sorted(placed, key=lambda p: math.dist((p[0].x, p[0].y), here)):
            text = f'{sym.ascii}{min(int(o.score * 100), 99)}'
            for start in (col + 1, col - len(text)):
                span = [(c, row) for c in range(start, start + len(text))]
                if all(canvas._inside(*p) and p not in taken for p in span):
                    taken.update(span)
                    for p, ch in zip(span, text):
                        canvas.glyph(*p, ch, 'label')
                    break


def _ring(proj, o, radius, steps=16):
    """A faint ring at the class radius, skipped when under two dots across."""
    r = radius * proj.scale
    if r < 1.0:
        return
    x, y = proj.dot((o.x, o.y))
    for i in range(steps):
        a = 2 * math.pi * i / steps
        proj.canvas.dot(round(x + r * math.cos(a)), round(y - r * math.sin(a)), 'ring')


def draw_truth(proj, truth):
    """``truth`` is (point, matched track point or None) pairs: a dot, and a faint line to the match."""
    for point, match in truth:
        a = proj.dot(point)
        proj.canvas.dot(*a, 'truth_dot')
        if match is not None:
            line(proj.canvas, a, proj.dot(match), 'truth_line')

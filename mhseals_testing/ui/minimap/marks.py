"""Map objects and truth onto a Canvas (map.md sections 2.2, 2.3); pure, no ROS.

`Projection` maps world metres to canvas dots and cells. Layers may carry a colour token
after a colon (``object:obj.red``); `canvas.rank` ignores it and the emitter reads it.
"""

import math
from collections import namedtuple

from mhseals_testing.analysis.classes import TRACKS
from mhseals_testing.ui.minimap.view import rotate

# A map object or tracker track (perception-v2 7.1). ``motion`` is static, moving or unknown,
# ``source`` live or prior (course map); ``vx vy`` m/s and ``sx sy`` size in m, map frame.
Obj = namedtuple('Obj', 'x y label score contact motion vx vy sx sy source',
                 defaults=(False, 'static', 0.0, 0.0, 0.0, 0.0, 'live'))
SCORE_FLOOR = 0.3   # default detection floor D (0, .3, .5, .7); below it the object is hidden
RING_HALF = 10.0    # class-radius rings show at or below this half-width, labels at LABEL_HALF
LABEL_HALF = 10.0
ARROWS = ('→↗↑↖←↙↓↘', '>/^\\</v\\')
HEAD_CELLS = 3      # a velocity line this many cells long gets an arrowhead glyph
PRED_PERIOD, FOV_PERIOD = 3, 2   # every Nth dot of the predicted path and of the camera cone


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


def arrow(yaw_deg, ascii_=False):
    """The 8-way arrow for a screen heading in degrees counter-clockwise from east."""
    return ARROWS[ascii_][round(yaw_deg / 45) % 8]


def line(canvas, a, b, layer, period=1):
    """Dots strictly between dot points ``a`` and ``b`` (Bresenham), every ``period``-th one."""
    (x, y), (x1, y1) = a, b
    dx, dy, sx, sy = abs(x1 - x), -abs(y1 - y), 1 if x < x1 else -1, 1 if y < y1 else -1
    err, n = dx + dy, 0
    while (x, y) != (x1, y1):
        if (x, y) != a:
            if n % period == 0:
                canvas.dot(x, y, layer)
            n += 1
        e2 = 2 * err
        if e2 >= dy:
            err, x = err + dy, x + sx
        if e2 <= dx:
            err, y = err + dx, y + sy


def strength(score):
    """Colour strength percent: full from .7, 78 from .5, 50 below (map.md 2.2)."""
    return 100 if score >= 0.7 else 78 if score >= 0.5 else 50


def _fade(o, tracks):
    """Strength percent of an object: its score band, capped for unknown motion and priors."""
    fade = strength(o.score)
    if o.motion == 'unknown':
        fade = min(fade, tracks['unknown_strength'])
    if o.source == 'prior':
        fade = min(fade, tracks['prior_strength'])
    return fade


def draw_motion(proj, o, colour, tracks, ascii_=False):
    """A moving track's velocity line (solid, class hue) to ``p + v*arrow_s`` and its predicted
    path on to ``p + v*predict_s`` (every 3rd dot); returns the arrowhead cell or None."""
    if o.motion != 'moving' or not (math.isfinite(o.vx) and math.isfinite(o.vy)) or not (o.vx or o.vy):
        return None
    canvas, p = proj.canvas, proj.dot((o.x, o.y))
    tip = proj.dot((o.x + o.vx * tracks['arrow_s'], o.y + o.vy * tracks['arrow_s']))
    end = proj.dot((o.x + o.vx * tracks['predict_s'], o.y + o.vy * tracks['predict_s']))
    line(canvas, p, tip, f'track.vel:{colour}')
    canvas.dot(*tip, f'track.vel:{colour}')
    line(canvas, tip, end, 'track.pred', PRED_PERIOD)
    if math.hypot((tip[0] - p[0]) / 2, (tip[1] - p[1]) / 4) < HEAD_CELLS:
        return None
    cell = (tip[0] // 2, tip[1] // 4)
    canvas.glyph(*cell, arrow(math.degrees(math.atan2(p[1] - tip[1], tip[0] - p[0])), ascii_),
                 f'track.vel:{colour}')
    return cell


def draw_objects(proj, objects, table, boat=None, ascii_=False, floor=SCORE_FLOOR):
    """Class glyphs (best score keeps a cell), contact chips, motion, rings and labels.

    A faded glyph's layer ends ``@<percent>`` (like ts115 tints): unknown motion and priors are
    capped at the minimap.yaml strengths, and a prior's label gets a ``·``. ``floor`` hides lower
    scores; objects with a non-finite position are skipped (floor() would raise and freeze the map).

    ``boat`` orders label placement nearest first; without it the view centre is used.
    """
    canvas, taken, placed = proj.canvas, set(), []
    tracks = getattr(table, 'tracks', TRACKS)
    for o in sorted((o for o in objects
                     if o.score >= floor and math.isfinite(o.x) and math.isfinite(o.y)), key=lambda o: -o.score):
        sym, pos = table.symbol(o.label), proj.cell((o.x, o.y))
        if pos in taken or not canvas._inside(*pos):
            continue
        taken.add(pos)
        char = sym.ascii if ascii_ else sym.glyph
        if o.contact:
            canvas.glyph(*pos, 'X' if ascii_ else char, 'contact', chip=True)
        else:
            fade = _fade(o, tracks)
            canvas.glyph(*pos, char, f'object:{sym.colour}' + (f'@{fade}' if fade < 100 else ''))
            head = draw_motion(proj, o, sym.colour, tracks, ascii_)
            if head:
                taken.add(head)
        placed.append((o, sym, pos))
        if proj.half <= RING_HALF:
            _ring(proj, o, sym.radius)
    if proj.half <= LABEL_HALF:
        here = boat or proj.centre
        for o, sym, (col, row) in sorted(placed, key=lambda p: math.dist((p[0].x, p[0].y), here)):
            text = f'{sym.ascii}{min(int(o.score * 100), 99)}' + (('.' if ascii_ else '·') if o.source == 'prior' else '')
            for start in (col + 1, col - len(text)):
                span = [(c, row) for c in range(start, start + len(text))]
                if all(canvas._inside(*p) and p not in taken for p in span):
                    taken.update(span)
                    for p, ch in zip(span, text):
                        canvas.glyph(*p, ch, 'label')
                    break


def draw_fov(proj, apex, yaw, hfov_deg, range_m):
    """The camera cone: two dotted edge rays from ``apex`` (world x, y) along ``yaw`` (rad, map)
    +- hfov/2 and a dotted arc at ``range_m``; never a tint (the costmap owns cell backgrounds)."""
    if not all(map(math.isfinite, (*apex, yaw, hfov_deg, range_m))) or range_m <= 0 or hfov_deg <= 0:
        return
    canvas, half = proj.canvas, math.radians(min(hfov_deg, 359.0)) / 2
    start = proj.dot(apex)
    edge = lambda a: (apex[0] + range_m * math.cos(a), apex[1] + range_m * math.sin(a))
    for side in (-1, 1):
        line(canvas, start, proj.dot(edge(yaw + side * half)), 'fov', FOV_PERIOD)
    steps = max(2, int(range_m * proj.scale * 2 * half / FOV_PERIOD))
    for i in range(steps + 1):
        canvas.dot(*proj.dot(edge(yaw - half + 2 * half * i / steps)), 'fov')


def _ring(proj, o, radius, steps=16):
    """A faint ring at the class radius, skipped when under two dots across."""
    r = radius * proj.scale
    if r < 1.0:
        return
    x, y = proj.dot((o.x, o.y))
    for i in range(steps):
        a = 2 * math.pi * i / steps
        proj.canvas.dot(round(x + r * math.cos(a)), round(y - r * math.sin(a)), 'grid')


def draw_truth(proj, truth):
    """``truth`` is (point, matched track point or None) pairs: a dot, and a faint line to the match."""
    for point, match in truth:
        a = proj.dot(point)
        proj.canvas.dot(*a, 'truth_dot')
        if match is not None:
            line(proj.canvas, a, proj.dot(match), 'truth_line')

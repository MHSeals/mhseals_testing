"""Dock banners and boat marks onto a Canvas (map.md section 2.2); pure, no ROS.

Tiers: ``colour`` (banner colour cells), ``plain`` (NO_COLOR or 16-colour: reverse chips
with a colour letter) and ``ascii``.
"""

import math
from collections import namedtuple

from mhseals_testing.ui.minimap.marks import LABEL_HALF, line

Banner = namedtuple('Banner', 'x y colour shape')       # colour red|green|blue, shape c|t|s|p
Bay = namedtuple('Bay', 'a b banners occupied')         # a, b: bay mouth ends in world metres
SHAPES = {'c': '●', 't': '▲', 's': '■', 'p': '+'}
ARROWS = ('→↗↑↖←↙↓↘', '>/^\\</v\\')
HULL = (1.34, 0.9)  # m, length and beam
RAY_DOTS = 11
WIDE = 3.0          # cells a bay needs for a two-cell banner chip
BRACKET = 4.5       # cells a bay needs for the ASCII [bs] form


def bay_cells(proj, bay):
    """The bay mouth's length in cells (cells are 2 dots wide, 4 high), at any rotation."""
    (xa, ya), (xb, yb) = proj.dot(bay.a), proj.dot(bay.b)
    return math.hypot((xb - xa) / 2, (yb - ya) / 4)


def draw_dock(proj, bay, tier='colour'):
    """Bay walls as dots and each banner as a chip, never a bare object glyph.

    An occupied bay narrower than 3 cells draws no banners: only the vessel shows.
    """
    canvas, width = proj.canvas, bay_cells(proj, bay)
    line(canvas, proj.dot(bay.a), proj.dot(bay.b), 'wall')
    if bay.occupied and width < WIDE:
        return
    for b in bay.banners:
        col, row = proj.cell((b.x, b.y))
        layer = f'dock_target:obj.{b.colour}'
        if tier == 'ascii':
            text = b.colour[0] + b.shape
            text = f'[{text}]' if width >= BRACKET else text
            for i, ch in enumerate(text):
                canvas.glyph(col + i, row, ch, 'dock_target')
        elif tier == 'plain' and width >= WIDE:
            canvas.glyph(col, row, b.colour[0], layer, chip=True)
            canvas.glyph(col + 1, row, SHAPES[b.shape], layer, chip=True)
        else:
            canvas.glyph(col, row, SHAPES[b.shape], layer, chip=True)


def arrow(yaw_deg, ascii_=False):
    """The 8-way arrow for a screen heading in degrees counter-clockwise from east."""
    return ARROWS[ascii_][round(yaw_deg / 45) % 8]


def draw_boat(proj, pos, yaw, reliable=True, ascii_=False):
    """Accent chip arrow above 10 m, hull outline at 10 m or less, plus a heading ray.

    An unreliable yaw (or a non-finite one, e.g. from a diverged EKF) draws the arrow
    in warn and no heading ray.
    """
    finite = math.isfinite(yaw)
    canvas, layer = proj.canvas, 'boat:accent' if reliable and finite else 'boat:warn'
    shown = 90.0 if proj.yaw is not None or not finite else yaw  # heading-up points straight up
    a = math.radians(shown)
    x, y = proj.dot(pos)
    for i in range(1, RAY_DOTS + 1 if finite else 1):
        canvas.dot(round(x + i * math.cos(a)), round(y - i * math.sin(a)), layer)
    col, row = proj.cell(pos)
    if proj.half > LABEL_HALF:
        canvas.glyph(col, row, arrow(shown, ascii_), layer, chip=True)
        return
    half_l, half_w = HULL[0] / 2 * proj.scale, HULL[1] / 2 * proj.scale
    c, s = math.cos(a), math.sin(a)
    corners = [(x + u * c * half_l - v * s * half_w, y - (u * s * half_l + v * c * half_w))
               for u, v in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
    for p, q in zip(corners, corners[1:] + corners[:1]):
        line(canvas, (round(p[0]), round(p[1])), (round(q[0]), round(q[1])), layer)
    canvas.glyph(col, row, arrow(shown, ascii_), layer)

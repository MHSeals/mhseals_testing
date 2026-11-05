"""Dock banners and boat marks onto a Canvas (map.md section 2.2); pure, no ROS.

Tiers: ``colour`` (banner colour cells), ``plain`` (NO_COLOR or 16-colour: reverse chips
with a colour letter) and ``ascii``.
"""

import math
from collections import namedtuple

from mhseals_testing.ui.minimap import hull

from mhseals_testing.ui.minimap.marks import arrow, line

# The smooth hull in an image tier (hull_sprite.py): yaw (map deg), length, beam and wake in dots, the body and
# bow layers whose colours it wears; it ranks as ``layer`` among the canvas's dots.
Hull = namedtuple('Hull', 'yaw length beam body bow wake layer')
Banner = namedtuple('Banner', 'x y colour shape')       # colour red|green|blue, shape c|t|s|p
Bay = namedtuple('Bay', 'a b banners occupied')         # a, b: bay mouth ends in world metres
SHAPES = {'c': '●', 't': '▲', 's': '■', 'p': '+'}
VMAX = 2.0          # m/s at which the wake reaches its full length
WAKE_SPEED = 0.15   # m/s below which no wake or speed leader is drawn
WAKE_DEG, WAKE_DOTS = 18.0, (4, 10)   # wake lines off the reverse heading; length 4 + 10*speed/vmax
WAKE_FADE = 3       # smooth braille: every WAKE_FADE-th wake dot the spacing grows by one, so the wake thins aft
LEAD_S, LEAD_MAX, CRAB_DEG = 3.0, 16.0, 15.0
WIDE = 3.0          # cells a bay needs for a two-cell banner chip
BRACKET = 4.5       # cells a bay needs for the ASCII [bs] form


def bay_cells(proj, bay):
    """The bay mouth's length in cells (cells are 2 dots wide, 4 high), at any rotation."""
    (xa, ya), (xb, yb) = proj.dot(bay.a), proj.dot(bay.b)
    return math.hypot((xb - xa) / proj.canvas.sx, (yb - ya) / proj.canvas.sy)


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


def draw_boat(proj, pos, yaw, reliable=True, ascii_=False, speed=None, cog=None, wake=True):
    """The filled hull rotated to heading (tui-visual 2.3), a heading leader and the wake.

    ``yaw`` and ``cog`` are map degrees CCW from east; ``speed`` m/s. The leader is the distance
    covered in 3 s; a COG more than 15 deg off the heading gets its own muted leader (crabbing).
    An unreliable or non-finite yaw fills the hull in warn and draws no leader. Heading-up points up.
    ASCII draws the hull cells as ``#`` and the bow cell as an 8-way arrow. theme.yaml ``hull: smooth`` traces
    the hull's outline in braille (clearing its inside) and paints it anti-aliased in an image tier, the wake
    there a soft V (hull_sprite.py); theme.yaml ``wake: false`` draws no wake.
    """
    finite = math.isfinite(yaw)
    good = reliable and finite
    up = proj.yaw is not None
    shown = 90.0 if up or not finite else yaw
    x, y = proj.dot(pos)
    canvas = proj.canvas
    k = canvas.k                      # sizes below are braille dots; an image canvas has k times more
    minimum = (hull.MIN_DOTS[0] * k, hull.MIN_DOTS[1] * k)
    look = 'classic' if ascii_ else hull.style()
    sprite = look == 'smooth' and k > 1
    moving = speed is not None and math.isfinite(speed) and speed > WAKE_SPEED
    wake = wake and moving and hull.wake_shown()
    length = (WAKE_DOTS[0] + WAKE_DOTS[1] * min(speed / VMAX, 1.0)) * k if wake else 0.0
    cells = {}
    if sprite:
        canvas.sprite(x, y, Hull(shown, *hull.size(proj.scale, minimum), 'boat' if good else 'boat:warn',
                                 'bow' if good else 'boat:warn', length, 'boat'))
    for dx, dy, part in () if sprite else hull.mask(shown, proj.scale, minimum, look):
        layer = part if good else 'boat:warn'
        if ascii_:
            cell = canvas.cell(x + dx, y + dy)
            cells[cell] = 'bow' if part == 'bow' or cells.get(cell) == 'bow' else 'boat'
        else:
            canvas.dot(x + dx, y + dy, layer)
    for dx, dy in () if sprite or ascii_ else hull.interior(shown, proj.scale, minimum, look):
        canvas.clear(x + dx, y + dy, 'boat')
    for cell, part in cells.items():
        canvas.glyph(*cell, arrow(shown, True) if part == 'bow' else '#', part if good else 'boat:warn')
    bow, port, starboard = hull.points(shown, proj.scale, minimum, look)
    if good:
        reach = min(max(3.0 * k, (speed or 0.0) * LEAD_S * proj.scale), LEAD_MAX * k) if moving else 3.0 * k
        _ray(canvas, (x + bow[0], y + bow[1]), shown, reach, 'bow', 2)
        if moving and cog is not None and math.isfinite(cog) and abs((cog - yaw + 180) % 360 - 180) > CRAB_DEG:
            _ray(canvas, (x, y), shown + cog - yaw, reach + math.hypot(*bow), 'wake:muted', 2)
    if wake and not sprite:
        for side, corner in ((1, port), (-1, starboard)):
            _ray(canvas, (x + corner[0], y + corner[1]), shown + 180 - side * WAKE_DEG, length, 'wake', 2,
                 WAKE_FADE if look == 'smooth' else 0)


def _ray(canvas, start, angle_deg, length, layer, period, fade=0):
    """Every ``period``-th dot of a ray from ``start`` (dots) at a screen angle CCW from right; with ``fade``
    the spacing grows by one every ``fade`` dots."""
    a = math.radians(angle_deg)
    i, n = 1, 0
    while i <= int(length):
        canvas.dot(math.floor(start[0] + i * math.cos(a)), math.floor(start[1] - i * math.sin(a)), layer)
        n += 1
        i += period + (n // fade if fade else 0)

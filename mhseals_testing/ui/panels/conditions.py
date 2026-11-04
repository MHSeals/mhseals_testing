"""Conditions · Command panel (tui-visual 3): one north-up rose, one command arrow, labelled bars.

Pure and ROS-free. The rose is a world-frame picture: the hull (the map's pentagon in braille dots, slate with its
foremost quarter in the bow token, as the image tier paints it) turns to the exact heading, the command is one arrow
icon at its bearing in the hue of whoever drives (16 bearings, hand-drawn in braille), yaw is four curved arrows (⤴ ⤵ ⤶ ⤷) round the corners and the world drift a dashed muted
line. The right column names the driver, then surge, sway and yaw as bipolar bars, then the heading.
"""

import math
import os
from functools import lru_cache

import numpy as np
from rich.text import Text

from mhseals_testing.ui import theme
from mhseals_testing.ui import imaging as pixels
from mhseals_testing.ui.minimap import hull
from mhseals_testing.ui.minimap.layers import SPEED_BANDS
from mhseals_testing.ui.minimap.structure import VMAX
from mhseals_testing.ui.panels import TITLES

ROSE_COLS, ROSE_ROWS = 21, 9
RADIUS = 17                 # dots; the canvas is 42 x 36 dots and dots are square
CENTRE = (ROSE_COLS, ROSE_ROWS * 2)
HULL_DOTS = (9, 6)          # length, beam of the rose's hull
REACH = (HULL_DOTS[0] / 2 + 1, HULL_DOTS[1] / 2 + 1)   # the command line starts this far out
ICON_REACH = RADIUS - 3     # dots; no command-icon dot lies farther out, clear of the ticks and letters
FULL_SPEED = 0.3            # m/s at the full radius; faster is clipped to the ring
STALE_S = 60.0              # an older drift estimate is drawn dim
ARROWS = '↑↗→↘↓↙←↖'         # screen directions, clockwise from up
ARROWS_ASCII = '^/>\\v/<\\'
# The command icon: a filled head on a short stem, in braille dots (x right, y down) about its middle,
# hand-drawn for 0, 22.5 and 45 deg; the other 13 bearings are exact flips and transposes of these
# (dots are square), so all 16 have one weight. ICON_SYM maps a master's offsets onto a bearing.
ICONS = {
    0: ((0, -3), (-1, -2), (0, -2), (1, -2), (-2, -1), (-1, -1), (0, -1), (1, -1), (2, -1),
        (0, 0), (0, 1), (0, 2), (0, 3)),
    1: ((1, -3), (-1, -2), (0, -2), (1, -2), (0, -1), (1, -1), (2, -1), (0, 0), (0, 1), (-1, 2), (-1, 3)),
    2: ((3, -3), (1, -3), (2, -3), (3, -2), (3, -1), (2, -2), (1, -1), (0, 0), (-1, 1), (-2, 2)),
}
ICON_SYM = ((1, 0, 0, 1), (-1, 0, 0, 1), (1, 0, 0, -1), (-1, 0, 0, -1),
            (0, -1, -1, 0), (0, 1, -1, 0), (0, -1, 1, 0), (0, 1, 1, 0))   # (x, y) -> (ax + by, cx + dy)
SPIN = ('↺', '↻')           # +yaw turns counter-clockwise (REP 103); the readout label
TEXT = '\ufe0e'             # text presentation: keeps ⤴ from rendering as a two-cell emoji
# one curved arrow in each corner, together tracing the turn: (col, row) -> glyph, counter-clockwise first
SPIN_CORNERS = (
    {(1, 0): '⤴', (0, 0): '⤶', (0, 1): '⤵', (1, 1): '⤷'},    # +yaw: TR up, TL left, BL down, BR right
    {(1, 0): '⤵', (0, 0): '⤷', (0, 1): '⤴', (1, 1): '⤶'},    # -yaw: TR down, TL right, BL up, BR left
)
SPIN_ASCII = ('ccw', 'cw')
POINTS = ('N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW')
EIGHTHS = ' ▏▎▍▌▋▊▉█'       # left-aligned fills; a bar growing left shows them in reverse video
BITS = {(0, 0): 0x01, (0, 1): 0x02, (0, 2): 0x04, (0, 3): 0x40,
        (1, 0): 0x08, (1, 1): 0x10, (1, 2): 0x20, (1, 3): 0x80}
# Rose layers, highest first: a cell takes the hue of its strongest dot.
BOW, BOAT, COMMAND, YAW, DRIFT, GLOW, TICK = range(7)
ASCII_DOTS = '#o.(:*.'      # the ASCII cell per layer
# The proximity glow (/perception/nearby): an arc on the ring toward the nearest lidar point, inside GLOW_FAR m;
# inside GLOW_NEAR it is wider, two dots deep and bold. Half-widths in degrees.
GLOW_NEAR, GLOW_FAR = 3.0, 8.0
GLOW_SPAN = {True: 30.0, False: 15.0}
BOW_SHARE = 0.25            # the hull's foremost quarter wears the bow hue (both tiers)


def _polar(bearing_deg, radius):
    """Dot offset (right, down) of a screen bearing (0 = up, clockwise) at ``radius``."""
    rad = math.radians(bearing_deg)
    return radius * math.sin(rad), -radius * math.cos(rad)


def _arrow(bearing):
    return theme.pick(ARROWS, ARROWS_ASCII)[round(bearing / 45) % 8]


def _shape(k):
    a, b, c, d = next(m for m in ICON_SYM if _maps(m, k))
    return tuple((a * x + b * y, c * x + d * y) for x, y in ICONS[k % 4 if k % 4 < 3 else 1])


def _fits(k, tenths):
    """The icon's (shape, anchor) at ``tenths`` / 10 dots out, or None if a dot passes ICON_REACH or
    lands in a letter cell."""
    letters = {((CENTRE[0] + round(dx)) // 2, (CENTRE[1] + round(dy)) // 4)
               for dx, dy in (_polar(t, RADIUS) for t in (0, 90, 180, 270))}
    shape = _shape(k)
    ox, oy = (round(v) for v in _polar(k * 22.5, tenths / 10))
    spots = [(ox + x, oy + y) for x, y in shape]
    if all(math.hypot(x + 0.5, y + 0.5) <= ICON_REACH for x, y in spots) and not any(
            ((CENTRE[0] + x) // 2, (CENTRE[1] + y) // 4) in letters for x, y in spots):
        return shape, (ox, oy)
    return None


@lru_cache(maxsize=16)
def command_icon(k):
    """(dot offsets, anchor) of the icon for screen bearing ``k`` * 22.5 deg. The anchor is the
    farthest out that fits every bearing of the same kind (cardinal, diagonal, in-between), so
    mirrored bearings sit at one radius."""
    kin = [j for j in range(16) if (j % 4 == 0) == (k % 4 == 0) and (j % 4 == 2) == (k % 4 == 2)]
    for tenths in range(120, 40, -5):
        if all(_fits(j, tenths) for j in kin):
            return _fits(k, tenths)
    raise AssertionError(k)


def _maps(m, k):
    """Whether symmetry ``m`` turns the master's bearing (0, 22.5 or 45) into bearing ``k``."""
    base = {0: 0, 1: 1, 2: 2, 3: 1}[k % 4]
    bx, by = _polar(base * 22.5, 1.0)
    wx, wy = _polar(k * 22.5, 1.0)
    return abs(m[0] * bx + m[1] * by - wx) < 1e-6 and abs(m[2] * bx + m[3] * by - wy) < 1e-6


def cardinal(heading):
    return POINTS[round(heading / 22.5) % 16]


def _reach(rel):
    """Dots from the centre to just outside the hull, ``rel`` radians off the bow."""
    return math.ceil(1 / math.hypot(math.cos(rel) / REACH[0], math.sin(rel) / REACH[1]))


def hull_dots(up):
    """``[(dx, dy, layer)]``: the rose's hull at screen bearing ``up`` in dots about CENTRE (the map's
    mask, 3 deg steps), its foremost quarter BOW as in the image tier, the rest BOAT."""
    fx, fy = _polar(up, 1.0)
    bow = HULL_DOTS[0] * (0.5 - BOW_SHARE)
    return [(dx, dy, BOW if dx * fx + dy * fy > bow else BOAT)
            for dx, dy, _ in hull.mask(90.0 - up, minimum=HULL_DOTS)]


def _token(name, fallback):
    """A palette token's style, or ``fallback`` under a theme overlay that lacks it."""
    try:
        return theme.PALETTE.style(name)
    except KeyError:
        return fallback


def glow(nearby, up):
    """``(screen bearing, near)`` of the proximity glow, or None: nothing within GLOW_FAR, or no bearing.

    ``nearby`` is ``{range, bearing}`` (degrees clockwise from the bow); ``up`` is the bow's screen bearing."""
    if not nearby or nearby.get('bearing') is None or nearby.get('range') is None or nearby['range'] > GLOW_FAR:
        return None
    return (up + nearby['bearing']) % 360, nearby['range'] <= GLOW_NEAR


def _glow_bearings(shown, step):
    centre, near = shown
    span = GLOW_SPAN[near]
    return [centre + offset for offset in np.arange(-span, span + step / 2, step)]


def rose(drift=None, heading=None, yaw_ok=True, age=0.0, command=(0.0, 0.0, 0.0), amplitude=1.0, hue='',
         nearby=None):
    """The 9 x 21 rose as Text rows: north up, or bow up (``↑bow``) while ``heading`` is None.

    ``drift`` is the world velocity (east, north) in m/s, ``heading`` the compass bearing in degrees
    and ``command`` the body (surge, sway, yaw) against ``amplitude``. Without a heading the command
    is drawn in the body frame and the world drift is hidden, never mixed in. ``nearby`` (the nearest lidar
    point) lights the ring toward it in the fail hue (``glow``).
    """
    if pixels.active():
        return _rose_image(drift, heading, yaw_ok, age, command, amplitude, hue, nearby)
    dots, glyphs = {}, {}
    cx, cy = CENTRE
    up = heading if heading is not None else 0.0     # the bow's screen bearing

    def put(x, y, layer):
        pos = (x // 2, y // 4)
        if 0 <= pos[0] < ROSE_COLS and 0 <= pos[1] < ROSE_ROWS:
            bits, best = dots.get(pos, (0, layer))
            dots[pos] = (bits | BITS[(x % 2, y % 4)], min(best, layer))

    def ray(bearing, start, stop, layer, step=1):
        for radius in range(start, stop + 1, step):
            dx, dy = _polar(bearing, radius)
            put(cx + round(dx), cy + round(dy), layer)

    for step in range(16):
        ray(step * 22.5, RADIUS - (step % 4 == 2), RADIUS, TICK)   # a longer tick on the intercardinals
    lit = glow(nearby, up)
    if lit is not None:
        for bearing in _glow_bearings(lit, 3.0):
            ray(bearing, RADIUS - 1 if lit[1] else RADIUS, RADIUS, GLOW)
    if heading is None:
        for col, char in enumerate(theme.pick('↑bow', '^bow')):
            glyphs[(ROSE_COLS // 2 - 1 + col, 0)] = (char, 'bold')
    else:
        for bearing, letter in ((0, 'N'), (90, 'E'), (180, 'S'), (270, 'W')):
            dx, dy = _polar(bearing, RADIUS)
            glyphs[((cx + round(dx)) // 2, (cy + round(dy)) // 4)] = (letter, 'bold')
        if drift is not None and any(drift):
            bearing = math.degrees(math.atan2(drift[0], drift[1]))
            start = _reach(math.radians(bearing - up))
            ray(bearing, start, start + round(min(math.hypot(*drift) / FULL_SPEED, 1.0) * (RADIUS - 1 - start)),
                DRIFT, 2)
    for dx, dy, layer in hull_dots(up):
        put(cx + dx, cy + dy, layer)

    surge, sway, yaw = command
    scale = amplitude or 1.0
    if surge or sway:                                # one icon at the commanded bearing, no shaft
        bearing = up + math.degrees(-math.atan2(sway, surge))   # +sway is to port: counter-clockwise
        k = round(bearing / 22.5) % 16
        dots_, (ox, oy) = command_icon(k)
        if theme.ASCII:
            glyphs[((cx + ox) // 2, (cy + oy) // 4)] = (ARROWS_ASCII[round(k / 2) % 8], hue)
        else:
            for dx, dy in dots_:
                put(cx + ox + dx, cy + oy + dy, COMMAND)
    if yaw:                                        # the turn: curved arrows round the corners, dim below half
        strong = abs(yaw) / scale >= 0.5
        look = f'{"bold" if strong else "dim"} {hue}'.strip()
        if theme.ASCII:
            word = SPIN_ASCII[yaw < 0]
            for i, char in enumerate(word):
                glyphs[(ROSE_COLS - 1 - len(word) + i, 0)] = (char, look)
        else:
            for (right, bottom), char in SPIN_CORNERS[yaw < 0].items():
                glyphs[(right * (ROSE_COLS - 1), bottom * (ROSE_ROWS - 1))] = (char + TEXT, look)

    # the body in slate sits with the frames, as in the image tier; ASCII keeps the brighter boat token
    boat = (_token('boat', 'bold') if theme.ASCII else _token('slate', '')) if yaw_ok else theme.WARN
    near = lit is not None and lit[1]
    styles = (_token('bow', theme.ACCENT) if yaw_ok else theme.WARN, boat, hue, hue,
              f'dim {theme.MUTED}' if age > STALE_S else theme.MUTED, f"{'bold' if near else 'dim'} {theme.BAD}".strip(),
              theme.MUTED)
    ascii_dots = ASCII_DOTS
    rows = []
    for row in range(ROSE_ROWS):
        line = Text()
        for col in range(ROSE_COLS):
            if (col, row) in glyphs:
                line.append(*glyphs[(col, row)])
            elif (col, row) in dots:
                bits, layer = dots[(col, row)]
                line.append(ascii_dots[layer] if theme.ASCII else chr(0x2800 + bits), styles[layer])
            else:
                line.append(' ' if theme.ASCII else '⠀')
        rows.append(line)
    return rows


ROSE_Q = 2.0                # deg: the image rose's hull, command and drift turn in 2 deg steps
ROSE_TILE = (7, 3)          # the hull's tile is the centre one; a turn resends only it
ICON_SPAN = (7.0, 13.0)     # dots from the centre: the image command icon's tail and tip (the braille icon's span)
ICON_HEAD = (3.5, 2.5)      # dots: the image icon's head length and half width


def _rose_image(drift, heading, yaw_ok, age, command, amplitude, hue, nearby=None):
    """The rose as image cells (ui/imaging/), the braille rose's look in pixels: muted ticks, the
    hull's pentagon in slate with its foremost quarter in the bow token at the exact heading (no
    bowsprit needed), the command icon (a stem and head) in the driver's hue, the dashed drift.
    Letters, ``↑bow`` and the corner yaw arrows stay text. Inputs are quantized to ROSE_Q, so the
    image (and its tiles) repeat as the boat turns."""
    q = lambda deg: round(deg / ROSE_Q) * ROSE_Q % 360
    up = q(heading) if heading is not None else 0.0
    glyphs = {}
    cx, cy = CENTRE
    if heading is None:
        for col, char in enumerate(theme.pick('↑bow', '^bow')):
            glyphs[(ROSE_COLS // 2 - 1 + col, 0)] = (char, 'bold')
    else:
        for bearing, letter in ((0, 'N'), (90, 'E'), (180, 'S'), (270, 'W')):
            dx, dy = _polar(bearing, RADIUS)
            glyphs[((cx + round(dx)) // 2, (cy + round(dy)) // 4)] = (letter, 'bold')
    surge, sway, yaw = command
    scale = amplitude or 1.0
    icon = q(up + math.degrees(-math.atan2(sway, surge))) if surge or sway else None
    if yaw:                                        # the turn stays text: curved arrows round the corners
        look = f'{"bold" if abs(yaw) / scale >= 0.5 else "dim"} {hue}'.strip()
        for (right, bottom), char in SPIN_CORNERS[yaw < 0].items():
            glyphs[(right * (ROSE_COLS - 1), bottom * (ROSE_ROWS - 1))] = (char + TEXT, look)
    shown = None
    if heading is not None and drift is not None and any(drift):
        bearing = math.degrees(math.atan2(drift[0], drift[1]))
        start = _reach(math.radians(bearing - up))
        shown = (q(bearing), start, start + round(min(math.hypot(*drift) / FULL_SPEED, 1.0) * (RADIUS - 1 - start)))
    stale = age > STALE_S
    tokens = (_token('slate', ''), _token('bow', theme.ACCENT), theme.MUTED, theme.WARN, hue)
    lit = glow(nearby, up)
    lit = None if lit is None else (q(lit[0]), lit[1])
    key = (up, heading is None, icon, shown, stale, yaw_ok, tokens, theme.PALETTE.mode, lit)

    def paint(r):
        sx, sy = r.cw / 2, r.ch / 4                   # px per dot
        at = lambda x, y: ((x + 0.5) * sx, (y + 0.5) * sy)
        pen = max(1, round(min(sx, sy) / 2.5))
        muted = r.ink(pixels.rgb(theme.MUTED, pixels.token('muted')))
        for step in range(16):                        # as the braille ticks: longer on the intercardinals
            a, b = _polar(step * 22.5, RADIUS - 0.25 - (step % 4 == 2)), _polar(step * 22.5, RADIUS + 0.25)
            r.line(*at(cx + a[0], cy + a[1]), *at(cx + b[0], cy + b[1]), muted, pen)
        if lit is not None:                           # the proximity glow: the ring's arc toward the nearest point
            fail = pixels.rgb(theme.BAD, pixels.token('fail'))
            ink = r.ink(fail if lit[1] else tuple((f + b) // 2 for f, b in zip(fail, pixels.token('bg'))))
            arc = [_polar(bearing, RADIUS - 0.5 * lit[1]) for bearing in _glow_bearings(lit, ROSE_Q)]
            for (ax, ay), (bx, by) in zip(arc, arc[1:]):
                r.line(*at(cx + ax, cy + ay), *at(cx + bx, cy + by), ink, pen * (2 if lit[1] else 1))
        if shown:
            ink = r.ink(tuple((m + b) // 2 for m, b in zip(pixels.rgb(theme.MUTED), pixels.token('bg'))) if stale
                        else pixels.rgb(theme.MUTED))
            a, b = _polar(shown[0], shown[1]), _polar(shown[0], shown[2])
            r.line(*at(cx + a[0], cy + a[1]), *at(cx + b[0], cy + b[1]), ink, pen, dash=max(2, round(sx)))
        warn = pixels.rgb(theme.WARN, pixels.token('warn'))
        body = r.ink(pixels.rgb(tokens[0], pixels.token('slate')) if yaw_ok else warn)
        poly = hull.outline(90.0 - up, *HULL_DOTS)
        r.polygon([at(cx + x, cy + y) for x, y in poly], body)
        # the bow: the hull's foremost quarter, as hull_dots splits it, at the exact heading
        fx, fy = _polar(up, 1.0)
        ys, xs = np.nonzero(r.px == body)
        ahead = (xs + 0.5 - (cx + 0.5) * sx) / sx * fx + (ys + 0.5 - (cy + 0.5) * sy) / sy * fy
        front = ahead > HULL_DOTS[0] * (0.5 - BOW_SHARE)
        r.px[ys[front], xs[front]] = r.ink(pixels.rgb(tokens[1], pixels.token('bow')) if yaw_ok else warn)
        if icon is not None:
            drive = r.ink(pixels.rgb(hue, pixels.token('fg')))
            (tail, tip), (head, half) = ICON_SPAN, ICON_HEAD
            a, b = _polar(icon, tail), _polar(icon, tip - head)
            r.line(*at(cx + a[0], cy + a[1]), *at(cx + b[0], cy + b[1]), drive, pen + 1)
            base, side = _polar(icon, tip - head), _polar(icon + 90, half)
            r.polygon([at(cx + x, cy + y) for x, y in (
                _polar(icon, tip), (base[0] + side[0], base[1] + side[1]), (base[0] - side[0], base[1] - side[1]))],
                drive)

    grid = pixels.surface('conditions.rose', paint, ROSE_COLS, ROSE_ROWS, key=key, holes=glyphs, tile=ROSE_TILE)
    return pixels.text_rows(grid, lambda col, row: glyphs.get((col, row), (' ', None)))


# --- the command column -----------------------------------------------------------------------

def mode_label(chords, tmux=None, mode=None, unavailable=None):
    """How the keyboard drives, in plain words: ``combos`` (keys held together) or ``one key``; ``mode``
    (``proc · C=I``, ``rc · mux``) wins outside manual; ``unavailable`` (why the terminal cannot hold keys
    together, ui/input.py) shows ``one key (tmux)``."""
    if mode:
        return mode
    if chords:
        return 'combos'
    if unavailable:
        return f'one key ({unavailable})'
    tmux = bool(os.environ.get('TMUX')) if tmux is None else tmux
    return 'one key (tmux)' if tmux else 'one key'


def _figure(value, amplitude):
    """``+60%``, or a muted ``0``."""
    if not value or not amplitude:
        return Text('0', style=theme.MUTED)
    return Text(f'{value / amplitude:+.0%}')


def speed_style(fraction):
    """The spd.* band of ``fraction`` of full (spd.0 calm .. spd.4, spd.5 past full): the minimap trail's speed
    palette, so one colour means one speed everywhere (D-20261101-27)."""
    band = SPEED_BANDS if fraction > 1 else min(int(max(fraction, 0.0) * SPEED_BANDS), SPEED_BANDS - 1)
    return _token('spd.%d' % band, theme.FG)


def bar(value, amplitude, cells, hue=None, grows_left=False):
    """A bipolar bar of ``cells`` per side about a centre ``│``, at 1/8-cell resolution.

    A positive value grows right, or left when ``grows_left`` (sway and yaw: port is left, as on
    the water). Each cell wears the spd.* band of its own distance out, so a harder command reaches
    hotter cells. Past ``amplitude`` the end cell shows ``▶``/``◀`` in spd.5. ASCII: ``#`` per whole
    cell. ``hue`` is kept for callers; the gradient replaced it.
    """
    frac = value / amplitude if amplitude else 0.0
    left = (frac > 0) == grows_left
    eighths = round(min(abs(frac), 1.0) * cells * 8)
    look = lambda i: speed_style((i + 0.5) / cells)       # the band of cell i, counted outward
    side = Text()
    if theme.ASCII:
        for i in range(round(eighths / 8)):
            side.append('#', look(i))
    else:
        for i in range(eighths // 8):
            side.append('█', look(i))
        if eighths % 8:
            part, cell = eighths % 8, look(eighths // 8)
            # Unicode has right-aligned blocks only for 1/8 and 1/2; the rest is a left fill reversed.
            side.append(*(('▕', cell) if part == 1 and left else ('▐', cell) if part == 4 and left
                          else (EIGHTHS[8 - part], f'reverse {cell}') if left else (EIGHTHS[part], cell)))
    if abs(frac) > 1:
        side = side[:-1]
        side.append(theme.pick('◀' if left else '▶', '<' if left else '>'), style=speed_style(abs(frac)))
    pad = Text(' ' * (cells - len(side)))
    empty = Text(' ' * cells)
    lhs, rhs = (Text.assemble(pad, _flip(side)), empty) if left else (empty, side + pad)
    if not frac:
        lhs, rhs = empty, empty
    return Text.assemble(lhs, (theme.BAR, theme.MUTED), rhs)


def _flip(side):
    """The cells of ``side`` in reverse order, styles kept: a left bar runs outward from the centre."""
    out = Text()
    for i in range(len(side) - 1, -1, -1):
        out.append_text(side[i])
    return out


def _labelled(label, value, amplitude, width):
    figure = _figure(value, amplitude)
    return Text.assemble(label, ' ' * max(width - len(label) - len(figure), 1), figure)


def _yaw_label(yaw):
    if not yaw:
        return 'yaw'
    return 'yaw ' + theme.pick('↺' if yaw > 0 else '↻', 'ccw' if yaw > 0 else 'cw')


def heading_row(info):
    """``hdg 247° WSW`` (``247° WSW ±4°`` with a sigma), warn with ``!`` while yaw is unreliable."""
    heading = info.get('heading')
    if heading is None:
        return Text(f'hdg {theme.DASH}', style=theme.MUTED)
    deg = theme.pick('°', '')      # ascii_safe's `deg` would not fit the column
    text = f'hdg {heading % 360:03.0f}{deg} {cardinal(heading)}'
    if info.get('heading_sd') is not None:
        text = f"{text[4:]} {theme.pick('±', '+-')}{info['heading_sd']:.0f}{deg}"
    return Text(f'! {text}', style=theme.WARN) if not info.get('yaw_ok', True) else Text(text)


# --- the panel (ui.md 6.3) --------------------------------------------------------------------

NOTABLE_STATES = ('trending', 'changed')   # drift states worth a word in the subtitle
OWNERS = {'human': 'keyboard', 'ours': 'procedure', 'nav': 'nav', 'agent': 'agent', 'hold': 'hold'}
VERDICT_HUES = {'marginal': 'WARN', 'poor': 'BAD'}   # good and unknown stay plain
BAR_CELLS = 6               # per side in the full panel; the 100 column panel fits 5
MICRO_CELLS = 2             # per side in the compact line


def _short(value, digits=2):
    """``.12`` for 0.12: the panel drops the leading zero to fit 36 columns."""
    return f'{value:.{digits}f}'.replace('0.', '.', 1) if abs(value) < 1 else f'{value:.{digits}f}'


def _deg(rad_per_s):
    return math.degrees(rad_per_s)


def conditions_title(info, full=True):
    """The subtitle: ``good 94% · 4 min`` (plus ``· trending`` or ``· changed``), or the measuring reason."""
    verdict = info.get('verdict', 'unknown')
    if verdict == 'unknown':
        return Text(info.get('reason') or 'measuring', style=theme.MUTED)
    trust = info.get('ceiling')
    trust = info.get('drift_trust') if trust is None else trust
    title = Text.assemble((verdict, getattr(theme, VERDICT_HUES.get(verdict, ''), '')))
    title.append(f' {trust:.0%}' if trust is not None else '', style=theme.MUTED)
    if not full:      # the compact line has no room for the minutes
        return title
    title.append(f"{theme.SEPARATOR}{info.get('neutral_s', 0) / 60:.0f} min", style=theme.MUTED)
    if info.get('state') in NOTABLE_STATES:
        title.append(f"{theme.SEPARATOR}{info['state']}", style=theme.WARN)
    return title


def _drift_figures(info):
    """``(speed m/s, bearing degrees toward)`` of the world drift, or None before one exists."""
    drift = info.get('drift')
    return None if drift is None else (math.hypot(*drift), math.degrees(math.atan2(drift[0], drift[1])) % 360)


def drift_row(info):
    """``drift .12 m/s →045° ±.02 · trust 88%``; a dash until a drift exists."""
    figures = _drift_figures(info)
    if figures is None:
        return Text(f'drift {theme.DASH}', style=theme.MUTED)
    return Text(theme.ascii_safe(f"drift {_short(figures[0])} m/s {theme.ARROW}{figures[1]:03.0f}° "
                                 f"±{_short(info['se'])}{theme.SEPARATOR}trust {info['drift_trust']:.0%}"))


def sog_text(sog):
    """``sog .42`` (m/s over ground, from odometry) in its spd.* band against the minimap's VMAX; None hides it."""
    if sog is None:
        return Text()
    return Text.assemble(('sog ', theme.MUTED), (f'{_short(sog)}', speed_style(sog / VMAX)), (' m/s', theme.MUTED))


def sea_row(info, sog=None):
    """``sog .42 m/s · floor .14°/s · sea .9°/s`` (no heave figure until an a_z source exists), or ``! yaw 4°``
    in warn while yaw is unreliable; ``sog`` leads while odometry reports one."""
    if not info.get('yaw_ok', True):
        body = Text(f"! yaw {info.get('yaw_note', '')}".rstrip(), style=theme.WARN)
    elif info.get('floor') is None or info.get('sea') is None:
        body = Text(f'floor {theme.DASH}', style=theme.MUTED)
    else:
        body = Text(theme.ascii_safe(
            f"floor {_short(_deg(info['floor']))}°/s{theme.SEPARATOR}sea {_short(_deg(info['sea']), 1)}°/s"))
    head = sog_text(sog)
    return Text.assemble(head, (theme.SEPARATOR, theme.MUTED), body) if head else body


def driver(view):
    """``(label, hue)`` of who drives: ``keys·combos``, ``nav``, ``proc · C=I``, ``rc · mux``."""
    kind = (view.get('authority') or ('', ''))[0]
    owner = OWNERS.get(kind)
    hue = theme.auth_style(kind) if owner else theme.MUTED
    mode = view.get('command_mode')
    label = mode_label(view.get('chords', False), mode=mode,
                       unavailable=view.get('chords_unavailable')).split(' (')[0]
    if owner == 'keyboard' and not mode:
        label = f'keys{theme.MIDDOT}{label}'
    elif owner and not mode:
        label = owner
    return label, hue


GAUGE_CELLS = 10


def speed_gauge(speed, cells=GAUGE_CELLS, label=True):
    """``≫ ▰▰▰▰▰▰▰▱▱▱ 75%``: the manual speed as a gauge whose lit cells run calm to hot.

    Each cell wears the spd.* band of its own position, so a higher speed reaches hotter cells, in the
    speed palette of the bars and the minimap trail. ASCII: ``>> #######--- 75%``; NO_COLOR keeps the
    count of lit cells and the percentage.
    """
    speed = min(1.0, max(0.0, float(speed)))
    lit = int(speed * cells + 0.5)
    on, off = theme.pick('▰', '#'), theme.pick('▱', '-')
    out = Text.assemble((theme.PALETTE.glyph('speed'), speed_style(speed)), ' ')
    for i in range(cells):
        out.append(on, speed_style((i + 0.5) / cells)) if i < lit else out.append(off, theme.MUTED)
    if label:
        out.append(f' {speed:.0%}', speed_style(speed))
    return out


def command_column(view, info, width):
    """The 9 right-hand rows: driver, speed gauge, surge/sway/yaw figures over their bars, heading."""
    label, hue = driver(view)
    surge, sway, yaw = view['command']
    amplitude = view.get('amplitude', 1.0)
    cells = min(BAR_CELLS, (width - 3) // 2)
    speed = view.get('speed')
    gauge = Text() if speed is None else speed_gauge(speed, min(GAUGE_CELLS, max(3, width - 9)))
    rows = [Text(label, style=hue), gauge]
    for name, value, left in (('surge', surge, False), ('sway', sway, True), (_yaw_label(yaw), yaw, True)):
        rows.append(_labelled(name, value, amplitude, 3 + 2 * cells))
        rows.append(Text.assemble((theme.pick('▕', '['), theme.MUTED), bar(value, amplitude, cells, hue, left),
                                  (theme.pick('▏', ']'), theme.MUTED)))
    rows.append(heading_row(info))
    return rows


def _fit(segments, width, sep=None):
    """Join ``segments`` (most important first) with separators while they fit in ``width``."""
    out = Text()
    for i, segment in enumerate(segments):
        piece = Text.assemble((theme.SEPARATOR, theme.MUTED), segment) if i and sep is None else segment
        if i and len(out) + len(piece) > width:
            break
        out.append_text(piece)
    out.truncate(width, overflow='ellipsis')
    return out


def compact_lines(view, info, width=46):
    """The 60-89 column form: ``good 94% · hdg 030° NNE · drift .12→045° ±.02`` over
    ``Cmd ↑ surge  │█▊+60 · yaw ▐│  +20 · keys·combos``; segments that do not fit are dropped."""
    cond = [conditions_title(info, full=False), heading_row(info)]
    figures = _drift_figures(info)
    if figures:
        cond.append(Text(theme.ascii_safe(f"drift {_short(figures[0])}{theme.ARROW}{figures[1]:03.0f}° "
                                          f"±{_short(info['se'])}")))
    label, hue = driver(view)
    surge, sway, yaw = view['command']
    amplitude = view.get('amplitude', 1.0)
    rel = math.degrees(-math.atan2(sway, surge))
    head = Text.assemble(('Cmd ', theme.MUTED), (_arrow(rel) if surge or sway else theme.MIDDOT, hue))
    axes = [Text.assemble(name, ' ', bar(value, amplitude, MICRO_CELLS, hue, left), _figure(value, amplitude))
            for name, value, left in (('surge', surge, False), ('sway', sway, True), (_yaw_label(yaw), yaw, True))
            if value] or [Text('0', style=theme.MUTED)]
    speed = view.get('speed')
    tail = [] if speed is None else [speed_gauge(speed, 5)]
    command = [Text.assemble(head, ' ', axes[0]), *axes[1:], Text(label, style=hue), *tail]
    return [_fit(cond, width), _fit(command, width)]


class _Body:
    """The panel body, laid out for the width Rich gives it (38 at 100 columns, 39 at 120)."""

    def __init__(self, view, info, compact):
        self.view, self.info, self.compact = view, info, compact

    def __rich_console__(self, console, options):
        width = options.max_width
        if self.compact:
            lines = compact_lines(self.view, self.info, width)
        else:
            info, command = self.info, self.view['command']
            _, hue = driver(self.view)
            left = rose(info.get('drift'), info.get('heading'), info.get('yaw_ok', True), info.get('age', 0.0),
                        command, self.view.get('amplitude', 1.0), hue, info.get('nearby'))
            right = command_column(self.view, info, width - ROSE_COLS - 3)
            lines = [Text.assemble(a, ' ', (theme.BAR, theme.MUTED), ' ', b) for a, b in zip(left, right)]
            lines += [drift_row(info), sea_row(info, self.view.get('sog'))]
        for line in lines:
            line.truncate(width, overflow='ellipsis')
            yield line


def conditions_panel(view, compact=False):
    """The rose beside the command column over the drift and sea rows (13 rows), or two lines when ``compact``.

    ``view['conditions']`` is ``Conditions.summary()`` plus ``heading`` (compass degrees), ``yaw_ok`` and
    ``yaw_note`` (the health ``rose`` anchor); absent means nothing measured yet.
    """
    info = view.get('conditions') or {}
    subtitle = None if compact else conditions_title(info)
    if subtitle is not None and pixels.active():     # the rose is an image: name the tier, quietly
        subtitle.append(f'{theme.SEPARATOR}{pixels.TIER}', style=theme.MUTED)
    return theme.block(_Body(view, info, compact), TITLES['conditions'], view['focus'] == 'conditions',
                       subtitle=subtitle)

"""Prototype boat glyphs and command arrows for the gallery (tools/glyph_gallery.py); not wired in.

Every option is a pure function of a bearing that returns cells, cached per heading bucket, so a
frame costs a dict lookup and a few cell writes. Coordinates: a cell is 1 unit wide and 2 high,
a braille dot is half a unit; screen bearings are 0 = up, clockwise.
"""

import math
from functools import lru_cache

from rich.text import Text

from mhseals_testing.ui import theme
from mhseals_testing.ui.minimap import hull
from mhseals_testing.ui.panels import conditions as cond

BITS = cond.BITS
QUAD = ' ▘▝▀▖▌▞▛▗▚▐▜▄▙▟█'          # by TL=1 TR=2 BL=4 BR=8
# A rounder hull than the map's pentagon: (forward, starboard) fractions of (length, beam).
SHAPE = ((0.5, 0.0), (0.38, 0.24), (0.2, 0.44), (0.0, 0.5), (-0.5, 0.45),
         (-0.5, -0.45), (0.0, -0.5), (0.2, -0.44), (0.38, -0.24))
LENGTH, BEAM = 4.6, 2.9           # units: the rose hull's 9 x 6 dots, a little longer
BOW_FRAC = 0.32                   # the foremost third wears the bow hue


# --- colour helpers ------------------------------------------------------------------------

def hexof(token):
    return theme.PALETTE.resolve(token, 'dark', 'truecolor')


def mix(a, b, t):
    """``a`` blended toward ``b`` by ``t`` (hex strings)."""
    pa = [int(a[i:i + 2], 16) for i in (1, 3, 5)]
    pb = [int(b[i:i + 2], 16) for i in (1, 3, 5)]
    return '#' + ''.join(f'{round(x + (y - x) * t):02X}' for x, y in zip(pa, pb))


def truecolor():
    return theme.PALETTE.style('accent').startswith('#')


# --- geometry -------------------------------------------------------------------------------

def _axes(bearing):
    r = math.radians(bearing)
    return (math.sin(r), -math.cos(r)), (math.cos(r), math.sin(r))   # forward, starboard


def hull_poly(bearing, length=LENGTH, beam=BEAM):
    (fx, fy), (sx, sy) = _axes(bearing)
    return [(u * length * fx + v * beam * sx, u * length * fy + v * beam * sy) for u, v in SHAPE]


def _inside(x, y, poly):
    hit = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            hit = not hit
    return hit


def coverage(poly, half, pw, ph, ss=4):
    """{(px, py): fraction} over a (2*half+1)-cell square window centred on the middle of the
    centre cell, at ``pw`` x ``ph`` pixels per cell; pixel (0, 0) is the window's top-left."""
    cols, rows = 2 * half[0] + 1, 2 * half[1] + 1
    ox, oy = -(half[0] + 0.5), -(2 * half[1] + 1)
    sw, sh = 1 / pw, 2 / ph
    out = {}
    for py in range(rows * ph):
        for px in range(cols * pw):
            hits = sum(_inside(ox + (px + (i + .5) / ss) * sw, oy + (py + (j + .5) / ss) * sh, poly)
                       for i in range(ss) for j in range(ss))
            if hits:
                out[(px, py)] = hits / ss / ss
    return out, (ox, oy, sw, sh)


def _is_bow(px, py, frame, bearing, length=LENGTH):
    ox, oy, sw, sh = frame
    (fx, fy), _ = _axes(bearing)
    x, y = ox + (px + .5) * sw, oy + (py + .5) * sh
    return x * fx + y * fy > length * (0.5 - BOW_FRAC)


# --- boat options: each returns {(dc, dr): (char, part, fg_hex_or_None, bg_hex_or_None)} --------
# part is 'bow' | 'boat'; fg/bg are truecolor overrides (antialiased), None = the part's token.

HALF = (3, 2)          # the window: 7 x 5 cells about the centre


@lru_cache(maxsize=64)
def boat_quadrants(bucket, n=16):
    """A: the hull rasterised to quadrant blocks (2 x 2 per cell), solid, one hue per cell."""
    bearing = bucket * 360 / n
    cov, frame = coverage(hull_poly(bearing), HALF, 2, 2)
    cells = {}
    for (px, py), c in cov.items():
        if c < 0.5:
            continue
        key = (px // 2 - HALF[0], py // 2 - HALF[1])
        bits, bow, all_ = cells.get(key, (0, 0, 0))
        cells[key] = (bits | (1 << ((px % 2) + 2 * (py % 2))), bow + _is_bow(px, py, frame, bearing), all_ + 1)
    return {k: (QUAD[b], 'bow' if 2 * w >= a else 'boat', None, None) for k, (b, w, a) in cells.items()}


# B: a geometric pointer and a stern block, hand-placed for 8 headings.
POINTER = {
    0: ((0, -1, '▲', 'bow'), (0, 0, '█', 'boat')),
    1: ((1, -1, '◥', 'bow'), (0, 0, '◢', 'boat'), (0, -1, '▗', 'boat'), (1, 0, '▘', 'boat')),
    2: ((1, 0, '▶', 'bow'), (0, 0, '█', 'boat'), (-1, 0, '▐', 'boat')),
    3: ((1, 1, '◢', 'bow'), (0, 0, '◥', 'boat'), (0, 1, '▝', 'boat'), (1, 0, '▖', 'boat')),
    4: ((0, 1, '▼', 'bow'), (0, 0, '█', 'boat')),
    5: ((-1, 1, '◣', 'bow'), (0, 0, '◤', 'boat'), (0, 1, '▘', 'boat'), (-1, 0, '▗', 'boat')),
    6: ((-1, 0, '◀', 'bow'), (0, 0, '█', 'boat'), (1, 0, '▌', 'boat')),
    7: ((-1, -1, '◤', 'bow'), (0, 0, '◣', 'boat'), (0, -1, '▖', 'boat'), (-1, 0, '▝', 'boat')),
}


def boat_pointer(bucket, n=8):
    return {(dc, dr): (ch, part, None, None) for dc, dr, ch, part in POINTER[bucket % 8]}


CHEVRON = ((0.5, 0.0), (-0.5, 0.5), (-0.22, 0.0), (-0.5, -0.5))   # the chart plotter's own-ship mark


def _halfblock(poly, bearing, length, bow_frac, aa):
    """Half blocks (1 x 2 square pixels per cell) of ``poly``; in truecolor a pixel's coverage blends
    its hue into the panel bg (antialiased edges), and a cell's two pixels are its fg and bg."""
    cov, frame = coverage(poly, HALF, 1, 2, ss=6)
    bg, boat, bow = hexof('bg'), hexof('boat'), hexof('bow')
    cut = 0.22 if aa else 0.5
    (fx, fy), _ = _axes(bearing)
    ox, oy, sw, sh = frame

    def pix(px, py):
        c = cov.get((px, py), 0.0)
        if c < cut:
            return None
        x, y = ox + (px + .5) * sw, oy + (py + .5) * sh
        is_bow = x * fx + y * fy > length * (0.5 - bow_frac)
        hue = bow if is_bow else boat
        return (mix(bg, hue, min(1.0, (c - cut) / (0.85 - cut))) if aa else hue), is_bow

    cells = {}
    for col in range(2 * HALF[0] + 1):
        for row in range(2 * HALF[1] + 1):
            top, bot = pix(col, 2 * row), pix(col, 2 * row + 1)
            if not (top or bot):
                continue
            part = 'bow' if (top and top[1]) or (bot and bot[1]) else 'boat'
            key = (col - HALF[0], row - HALF[1])
            if top and bot:
                cells[key] = ('▀', part, top[0], bot[0]) if top[0] != bot[0] else ('█', part, top[0], None)
            elif top:
                cells[key] = ('▀', part, top[0], None)
            else:
                cells[key] = ('▄', part, bot[0], None)
    return cells


@lru_cache(maxsize=256)
def boat_halfblock(bucket, n=72, aa=True):
    """C: the hull in antialiased half blocks, the foremost fifth in the bow hue."""
    bearing = bucket * 360 / n
    return _halfblock(hull_poly(bearing, LENGTH + 0.4, BEAM + 0.2), bearing, LENGTH + 0.4, 0.2, aa)


@lru_cache(maxsize=256)
def boat_chevron(bucket, n=72, aa=True):
    """E: a notched chevron (a chart plotter's own-ship mark) in antialiased half blocks."""
    bearing = bucket * 360 / n
    (fx, fy), (sx, sy) = _axes(bearing)
    length, beam = 5.0, 4.2
    poly = [(u * length * fx + v * beam * sx, u * length * fy + v * beam * sy) for u, v in CHEVRON]
    return _halfblock(poly, bearing, length, 0.3, aa)


@lru_cache(maxsize=256)
def boat_hybrid(bucket, n=72):
    """D: the quadrant body of A at 16 headings plus a braille bowsprit at the exact heading:
    the body reads at a glance, the 3-dot line gives the fine angle. Returns cells and dots."""
    bearing = bucket * 360 / n
    body = boat_quadrants(round(bearing / 22.5) % 16)
    (fx, fy), _ = _axes(bearing)
    dots = {}
    for k in range(3):
        r = LENGTH / 2 + 0.6 + k * 0.5                   # units ahead of the centre
        x, y = round(r * fx * 2), round(r * fy * 2)      # dots about the centre dot of the centre cell
        dots[(x, y)] = 'bow'
    return body, dots


# --- tiles: a boat option on its own 7 x 5 window --------------------------------------------

def _style(part, fg, bg):
    if fg and truecolor():
        return f'{fg} on {bg}' if bg else fg
    return cond._token(part, 'bold') if part == 'boat' else cond._token('bow', theme.ACCENT)


def _ascii_boat(bearing):
    """The ASCII fallback for every option: ``#`` hull cells and the bow as ``^ / > \\ v / < \\``."""
    cells = {k: ('#', p, None, None) for k, (_, p, _, _) in boat_quadrants(round(bearing / 22.5) % 16).items()}
    bow = [k for k, v in cells.items() if v[1] == 'bow']
    (fx, fy), _ = _axes(bearing)
    if bow:
        tip = max(bow, key=lambda k: k[0] * fx + 2 * k[1] * fy)
        cells[tip] = (cond.ARROWS_ASCII[round(bearing / 45) % 8], 'bow', None, None)
    return cells


def boat_cells(option, bearing):
    """(cells, dots) of a boat option at a screen bearing; dots are (dx, dy) about the centre dot."""
    if theme.ASCII:
        return _ascii_boat(bearing), {}
    if option == '0':                                 # today's map/rose mask, 9 x 6 dots
        return {}, {(dx, dy): part for dx, dy, part in hull.mask(90.0 - bearing, minimum=cond.HULL_DOTS)}
    if option == 'A':
        return boat_quadrants(round(bearing / 22.5) % 16), {}
    if option == 'B':
        return boat_pointer(round(bearing / 45) % 8), {}
    if option == 'C':
        return boat_halfblock(round(bearing / 5) % 72, aa=truecolor()), {}
    if option == 'D':
        return boat_hybrid(round(bearing / 5) % 72)
    if option == 'E':
        return boat_chevron(round(bearing / 5) % 72, aa=truecolor()), {}
    raise KeyError(option)


def tile(option, bearing, cols=9, rows=5):
    """The option drawn alone on a ``cols`` x ``rows`` window, as Text rows."""
    cells, dots = boat_cells(option, bearing)
    cc, cr = cols // 2, rows // 2
    braille = {}
    for (dx, dy), part in dots.items():
        x, y = cc * 2 + 1 + dx, cr * 4 + 2 + dy        # the centre dot of the centre cell
        pos = (x // 2, y // 4)
        bits, best = braille.get(pos, (0, part))
        braille[pos] = (bits | BITS[(x % 2, y % 4)], 'bow' if 'bow' in (part, best) else 'boat')
    out = []
    for r in range(rows):
        line = Text()
        for c in range(cols):
            key = (c - cc, r - cr)
            if key in cells:
                ch, part, fg, bg = cells[key]
                line.append(ch, _style(part, fg, bg))
            elif (c, r) in braille:
                bits, part = braille[(c, r)]
                line.append(chr(0x2800 + bits), _style(part, None, None))
            else:
                line.append(' ')
        out.append(line)
    return out


# --- arrows on the rose -----------------------------------------------------------------------

HEAD_LEN, HEAD_HALF = 4.5, 3.0     # dots: a filled head's length and half-width


def _tri(tip, bearing, length, half):
    (fx, fy), (sx, sy) = _axes(bearing)
    bx, by = tip[0] - fx * length, tip[1] - fy * length
    return [tip, (bx + sx * half, by + sy * half), (bx - sx * half, by - sy * half)]


def _fill_dots(poly):
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    return [(x, y) for x in range(math.floor(min(xs)), math.ceil(max(xs)) + 1)
            for y in range(math.floor(min(ys)), math.ceil(max(ys)) + 1) if _inside(x, y, poly)]


def _line(a, b, offset=0.0):
    """Dots on a..b (dot coordinates), shifted ``offset`` dots to starboard; with t along it."""
    (ax, ay), (bx, by) = a, b
    n = max(1, round(math.hypot(bx - ax, by - ay) * 1.5))
    length = math.hypot(bx - ax, by - ay) or 1
    px, py = -(by - ay) / length, (bx - ax) / length
    return [(round(ax + (bx - ax) * i / n + px * offset), round(ay + (by - ay) * i / n + py * offset), i / n)
            for i in range(n + 1)]


GLYPH_HEADS = '▲◥▶◢▼◣◀◤'

# Today's head at 0 deg is a filled 1-3-5 dot wedge; these are hand-drawn wedges for 22.5 deg
# (tip at (0, 0), x right, y down), mapped to the other seven in-between bearings by the square's
# symmetries (dots are square, so a flip or transpose is exact).
HEAD_22 = {
    'T': ((0, 0), (-1, 1), (0, 1), (-2, 2), (-1, 2), (0, 2)),                       # 6 dots, 3 rows
    'U': ((0, 0), (-1, 1), (0, 1), (-2, 2), (-1, 2), (0, 2), (-3, 3), (-2, 3), (-1, 3), (0, 3)),
}
_SYM = [lambda x, y: (x, y), lambda x, y: (-x, y), lambda x, y: (x, -y), lambda x, y: (-x, -y),
        lambda x, y: (-y, -x), lambda x, y: (y, -x), lambda x, y: (-y, x), lambda x, y: (y, x)]


def today_head(option, bearing):
    """Head dots for an in-between bearing (22.5 + 45 k), or None to draw today's head."""
    k = round(bearing / 22.5) % 16
    if k % 2 == 0 or abs(bearing - k * 22.5) % 360 > 5.625:
        return None
    if option == 'S':                              # snap: today's head at the nearer cardinal
        snapped = (k - 1) * 22.5 if k in (1, 7, 9, 15) else (k + 1) * 22.5
        out = set()
        for barb in (snapped + 180 - cond.HEAD_SPREAD, snapped + 180 + cond.HEAD_SPREAD):
            for length in range(1, cond.HEAD_LEN + 1):
                dx, dy = cond._polar(barb, length)
                out.add((round(dx), round(dy)))
        return tuple(out | {(0, 0)})
    want = cond._polar(k * 22.5, 1.0)
    base = cond._polar(22.5, 1.0)
    for f in _SYM:
        fx, fy = f(*base)
        if abs(fx - want[0]) < 1e-6 and abs(fy - want[1]) < 1e-6:
            return tuple(f(x, y) for x, y in HEAD_22[option])
    raise AssertionError(k)


@lru_cache(maxsize=2048)
def arrow(option, bearing_q, start, stop):
    """The command arrow from ``start`` to ``stop`` dots out at a screen bearing (cached per degree).

    Returns (dots, cells): dots {(x, y): (role, t)} about the rose centre, role 'shaft'|'head'|'tail',
    t in 0..1 from tail to head; cells {(col, row): (char, role, fg, bg)} in rose cells."""
    bearing = bearing_q
    (fx, fy), _ = _axes(bearing)
    a, b = (start * fx, start * fy), (stop * fx, stop * fy)
    dots, cells = {}, {}

    def add(x, y, role, t):
        old = dots.get((x, y))
        if old is None or (role == 'head') > (old[0] == 'head'):
            dots[(x, y)] = (role, t)

    if option == '0':                              # compass-hires: a 1-dot line and two barbs
        tip = stop + cond.HEAD_LEN
        for r in range(start, tip + 1):
            dx, dy = cond._polar(bearing, r)
            add(round(dx), round(dy), 'shaft', (r - start) / max(1, tip - start))
        tx, ty = cond._polar(bearing, tip)
        for barb in (bearing + 180 - cond.HEAD_SPREAD, bearing + 180 + cond.HEAD_SPREAD):
            for length in range(1, cond.HEAD_LEN + 1):
                dx, dy = cond._polar(barb, length)
                add(round(tx + dx), round(ty + dy), 'head', 1.0)
    elif option in ('T', 'U', 'S'):                # today's arrow; only the in-between heads change
        head = today_head(option, bearing)
        tip = stop + cond.HEAD_LEN
        tx, ty = cond._polar(bearing, tip)
        if head is None:                           # 0, 45, 90 ...: exactly today's drawing
            return arrow('0', bearing_q, start, stop)
        for r in range(start, tip - 2):
            dx, dy = cond._polar(bearing, r)
            add(round(dx), round(dy), 'shaft', (r - start) / max(1, tip - start))
        for hx, hy in head:
            add(round(tx) + hx, round(ty) + hy, 'head', 1.0)
    elif option == 'A':                            # a 2-dot shaft and a filled wedge
        tip = (b[0] + fx * HEAD_LEN, b[1] + fy * HEAD_LEN)
        for off in (-0.5, 0.5):
            for x, y, t in _line(a, b, off):
                add(x, y, 'shaft', t)
        for x, y in _fill_dots(_tri(tip, bearing, HEAD_LEN + 0.5, HEAD_HALF)):
            add(x, y, 'head', 1.0)
    elif option == 'B':                            # a 1-dot shaft and a triangle glyph at the tip
        for x, y, t in _line(a, b):
            add(x, y, 'shaft', t)
        tx, ty = b[0] + fx * 2.5, b[1] + fy * 2.5
        cells[((cond.CENTRE[0] + round(tx)) // 2, (cond.CENTRE[1] + round(ty)) // 4)] = (
            GLYPH_HEADS[round(bearing / 45) % 8], 'head', None, None)
    elif option == 'C':                            # half-block AA silhouette, gradient tail->head
        cells.update(_halfblock_arrow(round(bearing), start, stop))
    elif option == 'E':                            # glow: gradient shaft, wedge, tinted cells
        tip = (b[0] + fx * HEAD_LEN, b[1] + fy * HEAD_LEN)
        for x, y, t in _line(a, b):
            add(x, y, 'shaft', t)
        for x, y in _fill_dots(_tri(tip, bearing, HEAD_LEN + 0.5, HEAD_HALF - 0.5)):
            add(x, y, 'head', 1.0)
    elif option == 'D':                            # comet: gradient 1-dot shaft, wedge, fletching
        tip = (b[0] + fx * HEAD_LEN, b[1] + fy * HEAD_LEN)
        for x, y, t in _line(a, b):
            add(x, y, 'shaft', t)
        for x, y in _fill_dots(_tri(tip, bearing, HEAD_LEN + 0.5, HEAD_HALF - 0.5)):
            add(x, y, 'head', 1.0)
        for side in (-1, 1):                       # two feathers swept back from the tail
            for k in (1, 2):
                fb = bearing + 180 + side * 35
                dx, dy = cond._polar(fb, k)
                add(round(a[0] + fx * 2 + dx), round(a[1] + fy * 2 + dy), 'tail', 0.0)
    return dots, cells


@lru_cache(maxsize=512)
def _halfblock_arrow(bearing, start, stop):
    """C: shaft 1 unit wide and a 3-unit head as one polygon, rasterised to half blocks with
    coverage blended into the bg; the shaft fades from 45 % at the tail to full at the head."""
    (fx, fy), (sx, sy) = _axes(bearing)
    head, hw, sw = 2.4, 1.5, 0.5
    u0, u1 = start / 2 - 0.3, (stop + cond.HEAD_LEN) / 2 - head   # units; the tip where 0's is
    pts = [(u0, sw), (u1, sw), (u1, hw), (u1 + head, 0), (u1, -hw), (u1, -sw), (u0, -sw)]
    poly = [(u * fx + v * sx, u * fy + v * sy) for u, v in pts]
    half = (cond.ROSE_COLS // 2, cond.ROSE_ROWS // 2)
    cov, frame = coverage(poly, half, 1, 2, ss=5)
    ox, oy, pw, ph = frame
    bg, hue = hexof('bg'), _hue_hex()
    cut = 0.2 if truecolor() else 0.5

    def pix(px, py):
        c = cov.get((px, py), 0.0)
        if c < cut:
            return None
        x, y = ox + (px + .5) * pw, oy + (py + .5) * ph
        t = min(1.0, max(0.0, (x * fx + y * fy - u0) / (u1 - u0)))
        role = 'head' if x * fx + y * fy > u1 - 0.2 else 'shaft'
        if not truecolor():
            return hue, role
        return mix(bg, hue, min(1.0, (c - cut) / (0.85 - cut)) * (0.5 + 0.5 * t if role == 'shaft' else 1.0)), role

    cells = {}
    for col in range(cond.ROSE_COLS):
        for row in range(cond.ROSE_ROWS):
            top, bot = pix(col, 2 * row), pix(col, 2 * row + 1)
            if not (top or bot):
                continue
            role = 'head' if 'head' in ((top or ('', ''))[1], (bot or ('', ''))[1]) else 'shaft'
            if top and bot:
                cells[(col, row)] = (('▀', role, top[0], bot[0]) if top[0] != bot[0] and truecolor()
                                     else ('█', role, top[0], None))
            elif top:
                cells[(col, row)] = ('▀', role, top[0], None)
            else:
                cells[(col, row)] = ('▄', role, bot[0], None)
    return cells


def _hue_hex():
    return hexof('accent')     # the gallery's driver hue (task / procedure): accent


def rose(boat='0', arrow_opt='0', heading=0.0, command_bearing=None, magnitude=0.8, drift=(0.06, 0.1)):
    """The 9 x 21 Conditions rose of conditions.rose, with pluggable boat and arrow painters."""
    if theme.ASCII:     # the ASCII rose is unchanged across options: the real path
        return cond.rose(drift, heading, command=_command(heading, command_bearing, magnitude), hue='bold')
    cx, cy = cond.CENTRE
    dots, glyphs, solid = {}, {}, {}
    order = ('head', 'tail', 'shaft', 'bow', 'boat', 'yaw', 'drift', 'tick')

    def put(x, y, layer, t=0.0):
        pos = (x // 2, y // 4)
        if 0 <= pos[0] < cond.ROSE_COLS and 0 <= pos[1] < cond.ROSE_ROWS:
            bits, best, bt = dots.get(pos, (0, layer, t))
            win = layer if order.index(layer) < order.index(best) else best
            dots[pos] = (bits | BITS[(x % 2, y % 4)], win, max(bt, t) if win == layer else bt)

    def ray(bearing, a, b, layer, step=1):
        for radius in range(a, b + 1, step):
            dx, dy = cond._polar(bearing, radius)
            put(cx + round(dx), cy + round(dy), layer)

    for step in range(16):
        ray(step * 22.5, cond.RADIUS - (step % 4 == 2), cond.RADIUS, 'tick')
    for bearing, letter in ((0, 'N'), (90, 'E'), (180, 'S'), (270, 'W')):
        dx, dy = cond._polar(bearing, cond.RADIUS)
        glyphs[((cx + round(dx)) // 2, (cy + round(dy)) // 4)] = (letter, 'bold')
    if drift and any(drift):
        bearing = math.degrees(math.atan2(drift[0], drift[1]))
        a = cond._reach(math.radians(bearing - heading))
        ray(bearing, a, a + round(min(math.hypot(*drift) / cond.FULL_SPEED, 1.0) * (cond.RADIUS - 1 - a)), 'drift', 2)

    cells, bdots = boat_cells(boat, heading)
    centre_cell = (cx // 2, cy // 4)
    for (dc, dr), cell in cells.items():
        solid[(centre_cell[0] + dc, centre_cell[1] + dr)] = cell
    for (dx, dy), part in bdots.items():
        # option 0's mask is about dot (0, 0) = the rose centre; D's dots too
        put(cx + dx, cy + dy, part)

    if command_bearing is not None:
        rel = math.radians(command_bearing - heading)
        start = cond._reach(rel) + (1 if boat in 'AC' else 0)
        stop = start + round(magnitude * (cond.COMMAND_END - start))
        adots, acells = arrow(arrow_opt, round(command_bearing) % 360, start, stop)
        for (x, y), (role, t) in adots.items():
            put(cx + x, cy + y, role, t)
        for pos, cell in acells.items():
            solid.setdefault(pos, cell)

    hue = _hue_hex()
    bg = hexof('bg')
    tc = truecolor()

    def layer_style(layer, t):
        if layer in ('head', 'shaft', 'tail'):
            if not tc:
                return theme.ACCENT
            if arrow_opt in ('A',) and layer == 'shaft':
                return mix(bg, hue, 0.62)
            if arrow_opt == 'E':
                if layer == 'head':
                    return f'bold {hue} on {mix(bg, hue, 0.2)}'
                return mix(bg, hue, 0.45 + 0.55 * t)
            if arrow_opt == 'D':
                return hue if layer == 'head' else mix(bg, hue, 0.35 + 0.65 * t) if layer == 'shaft' else mix(bg, hue, 0.4)
            return hue
        return {'bow': cond._token('bow', theme.ACCENT), 'boat': cond._token('boat', 'bold'),
                'yaw': hue, 'drift': theme.MUTED, 'tick': theme.MUTED}[layer]

    rows = []
    for row in range(cond.ROSE_ROWS):
        line = Text()
        for col in range(cond.ROSE_COLS):
            if (col, row) in glyphs:
                line.append(*glyphs[(col, row)])
            elif (col, row) in solid:
                ch, part, fg, bgc = solid[(col, row)]
                if part in ('head', 'shaft', 'tail'):
                    style = (f'{fg} on {bgc}' if bgc else fg) if fg and tc else (theme.ACCENT if part != 'head' or tc else f'bold {theme.ACCENT}')
                    if arrow_opt == 'B' and part == 'head' and tc:
                        style = f'bold {hue}'
                else:
                    style = _style(part, fg, bgc)
                line.append(ch, style)
            elif (col, row) in dots:
                bits, layer, t = dots[(col, row)]
                line.append(chr(0x2800 + bits), layer_style(layer, t))
            else:
                line.append('⠀')
        rows.append(line)
    return rows


def _command(heading, bearing, magnitude):
    if bearing is None:
        return (0.0, 0.0, 0.0)
    rel = math.radians(bearing - heading)
    return (magnitude * math.cos(rel), -magnitude * math.sin(rel), 0.0)


# --- owner's direction: an arrow icon only, no shaft, and a rotation icon in a corner ---------

# Braille arrow icons, dots about the icon centre (x right, y down), hand-drawn for 0, 22.5 and
# 45 deg; the other 13 bearings are exact flips/transposes (dots are square). Each is a 1-3-5 style
# filled head on a short shaft, about 7 dots long, so all 16 carry the same weight.
ICON_MASTERS = {
    0: ((0, -3), (-1, -2), (0, -2), (1, -2), (-2, -1), (-1, -1), (0, -1), (1, -1), (2, -1),
        (0, 0), (0, 1), (0, 2), (0, 3)),
    22.5: ((1, -3), (-1, -2), (0, -2), (1, -2), (0, -1), (1, -1), (2, -1),   # base slants with the axis
           (0, 0), (0, 1), (-1, 2), (-1, 3)),
    45: ((3, -3), (1, -3), (2, -3), (3, -2), (3, -1), (2, -2),
         (1, -1), (0, 0), (-1, 1), (-2, 2)),
}
ICON_RADIUS = 11            # dots from the rose centre to the icon centre
LIGHT = '↑↗→↘↓↙←↖'
HEAVY = '⬆⬈➡⬊⬇⬋⬅⬉'
SPIN_CCW, SPIN_CW = '↺', '↻'     # +yaw is counter-clockwise (REP 103)


@lru_cache(maxsize=16)
def icon_dots(k):
    """The icon for bearing ``k`` * 22.5 deg as dot offsets about its centre."""
    base = 0 if k % 4 == 0 else 45 if k % 4 == 2 else 22.5
    want, have = cond._polar(k * 22.5, 1.0), cond._polar(base, 1.0)
    for f in _SYM:
        fx, fy = f(*have)
        if abs(fx - want[0]) < 1e-6 and abs(fy - want[1]) < 1e-6:
            return tuple(f(x, y) for x, y in ICON_MASTERS[base])
    raise AssertionError(k)


def rose_icon(style='braille', heading=0.0, bearing=None, yaw=0.0, magnitude=0.8, amplitude=1.0):
    """The rose with the command as one icon at a fixed radius and yaw as ↻/↺ in the top-right corner.

    ``style``: 'braille' (16 hand-drawn icons), 'light' (↑↗ snapped to 8), 'heavy' (⬆⬈ snapped to 8),
    'mixed' (light glyphs on the eight main bearings, braille icons between)."""
    hue = _hue_hex()
    tc = truecolor()
    rows = cond.rose(None, heading, command=(0.0, 0.0, 0.0), hue=hue)   # ticks, letters, hull
    grid = [[(row.plain[c], row.get_style_at_offset(None, c) if False else _span_style(row, c))
             for c in range(cond.ROSE_COLS)] for row in rows]
    cx, cy = cond.CENTRE
    if bearing is not None and magnitude:
        k = round(bearing / 22.5) % 16
        dx, dy = cond._polar(bearing, ICON_RADIUS)
        use_glyph = theme.ASCII or style in ('light', 'heavy') or (style == 'mixed' and k % 2 == 0)
        if use_glyph:
            table = cond.ARROWS_ASCII if theme.ASCII else HEAVY if style == 'heavy' else LIGHT
            pos = ((cx + round(dx)) // 2, (cy + round(dy)) // 4)
            grid[pos[1]][pos[0]] = (table[round(bearing / 45) % 8], f'bold {hue}')
        else:
            cells = {}
            for ix, iy in icon_dots(k):
                x, y = cx + round(dx) + ix, cy + round(dy) + iy
                pos = (x // 2, y // 4)
                cells[pos] = cells.get(pos, 0) | BITS[(x % 2, y % 4)]
            for (col, row), bits in cells.items():
                if 0 <= col < cond.ROSE_COLS and 0 <= row < cond.ROSE_ROWS:
                    old = grid[row][col][0]
                    if 0x2800 <= ord(old) <= 0x28FF:
                        bits |= ord(old) - 0x2800
                    grid[row][col] = (chr(0x2800 + bits), f'bold {hue}' if tc else f'bold {theme.ACCENT}')
    if yaw:
        share = min(abs(yaw) / (amplitude or 1.0), 1.0)
        if theme.ASCII:
            word = 'ccw' if yaw > 0 else ' cw'
            for i, ch in enumerate(word):
                grid[0][cond.ROSE_COLS - 3 + i] = (ch, 'bold')
        else:
            style_ = (f'bold {mix(hexof("bg"), hue, 0.35 + 0.65 * share)}' if tc
                      else 'bold' if share >= 0.5 else '')
            grid[0][cond.ROSE_COLS - 2] = (SPIN_CCW if yaw > 0 else SPIN_CW, style_)
    out = []
    for cells in grid:
        line = Text()
        for ch, st in cells:
            line.append(ch, st)
        out.append(line)
    return out


def _span_style(row, col):
    style = ''
    for span in row.spans:
        if span.start <= col < span.end:
            style = span.style
    return style

"""Terminal image tiers for the TUI's drawings (graphics-proto): kitty, sixel, or the braille text.

One module behind every dot drawing (the Conditions rose, the Motion plots, the popup sparks and
dials, the minimap with its hull): a widget paints into a `Raster` at the terminal's cell pixel
size in flat palette colours, and `surface()` cuts it into tiles of whole cells and returns the
cells to print. Labels stay terminal text: a widget passes them as holes.

- ``kitty``: Unicode placeholder cells (U+10EEEE with row, column and id-byte diacritics, the id's
  low byte in the 256-colour foreground). A tile is transmitted once, under an id picked by its
  content, so a hull back at a heading it showed before, or a plot tile that did not change,
  costs only its placeholder text. Placeholders flow, clip and move with the Rich layout.
- ``sixel``: private-use marker cells. After Rich lays out a frame, `frame()` finds them and
  splits each tile's visible cells (a label hole or a popup may hide some) into rectangles drawn
  cropped; the terminal stream turns the markers into cursor-forward so text never paints over
  the image, and `after()` draws the parts that are new at their place (all after a resize).
- ``text``: widgets draw their braille as before; nothing here runs.

Detection (`start`) asks the terminal once: XTVERSION, the cell size and DA1 (``4`` = sixel),
then the kitty graphics query only if XTVERSION names kitty or ghostty; each round ends at the
DA1 reply or ``PROBE_S``. ``auto`` stays text under tmux (it keeps a few sixel images per pane).
theme.yaml ``graphics`` forces a tier; NO_COLOR and ASCII force text. numpy and stdlib only.
"""

import base64
import hashlib
import os
import re
import select
import time
import zlib
from collections import OrderedDict, deque

import numpy as np
from rich.cells import cell_len
from rich.segment import Segment
from rich.style import Style

SETTINGS = ('off', 'auto', 'kitty', 'sixel')   # theme.yaml ``graphics``; off (the default) is the text
PROBE_ID = 4242
PROBE = ('\x1b[>0q'                                              # XTVERSION
         '\x1b[16t'                                              # cell size in pixels
         '\x1b[c')                                               # DA1: everyone answers, so it ends the probe
# Asked only of a terminal that named itself kitty or ghostty: tmux takes an unknown APC for a
# window title, so the query never goes out blind.
PROBE_KITTY = f'\x1b_Gi={PROBE_ID},s=1,v=1,a=q,t=d,f=24;AAAA\x1b\\\x1b[c'
PROBE_S = 0.5               # s; the probe normally ends at the DA1 reply, one round trip
DEFAULT_CELL = (10, 20)     # px per cell when the terminal does not say (kitty; sixel then stays text)
POOL = 480                  # tile ids alive at once (sixel markers have room for 511)
TILE = (8, 4)               # cells per tile, at most 16 x 8 (the marker code)
KITTY_NAMES = ('kitty', 'ghostty')
_REPLY = re.compile(rb'\x1b_G([^\x1b]*)\x1b\\|\x1bP>\|([^\x1b]*)\x1b\\|\x1b\[6;(\d+);(\d+)t|\x1b\[\?([\d;]*)c')

ESC, ST = '\x1b', '\x1b\\'
PLACEHOLDER = '\U0010EEEE'
# kitty's row/column diacritics (the first 32 of its table)
DIACRITICS = tuple(chr(c) for c in (
    0x305, 0x30D, 0x30E, 0x310, 0x312, 0x33D, 0x33E, 0x33F, 0x346, 0x34A, 0x34B, 0x34C, 0x350, 0x351, 0x352,
    0x357, 0x35B, 0x363, 0x364, 0x365, 0x366, 0x367, 0x368, 0x369, 0x36A, 0x36B, 0x36C, 0x36D, 0x36E, 0x36F,
    0x483, 0x484))
BLANK = (' ', None)         # the cell of an empty tile
MARK0 = 0xF0000            # sixel marker: MARK0 + id * 128 + row * 16 + col (Supplementary PUA-A)
_MARKS = re.compile('[\U000F0000-\U000FFFFD]+')

TIER = 'text'
SETTING = 'off'             # the setting start() was given
_caps = {}                  # the session's probe replies: start() asks the terminal once
clock = time.monotonic      # s; surface()'s repaint rate (tests run a simulated clock)
# Over SSH (the boat radio: about 1 Mbit/s) image bytes get a budget, a fifth of that link; above
# it every image repaints at THROTTLE_HZ at most. Text costs more than sixel, so never fall back to it.
SSH_BUDGET = 25_000         # B/s
THROTTLE_HZ = 2.0
METER_S = 2.0               # s of image bytes the rate is measured over
BUDGET = None               # B/s, or None: no budget (a local terminal)
_sent = deque()             # (time, bytes) of image writes within METER_S
CELL = DEFAULT_CELL
K = 1                       # dot canvases use 2K x 4K dots per cell in an image tier (1: braille)
GENERATION = 0              # bumped by start(): cached surfaces repaint for a new terminal
STATS = {'upload': 0, 'sixel': 0, 'tiles': 0}   # bytes and tiles sent since start
_pool = OrderedDict()       # digest -> Tile, least recently used first
_by_id = {}
_free = []
_pending = []               # kitty escapes to send before the next frame
_surfaces = {}              # name -> (key, time, cells)
_live = {}                  # sixel: (row, col) -> (id, crop) of the tile parts visible this frame
_drawn = {}                 # sixel: (row, col) -> id painted there
_size = None
_stream = None


# --- detection ---------------------------------------------------------------------------------

def parse(data):
    """``(caps, rest)`` from probe reply bytes: caps has ``done`` (DA1 seen), ``kitty``, ``sixel``,
    ``cell`` (w, h px or None) and ``xtversion``; ``rest`` is what was not a reply (typed keys)."""
    caps = dict(done=False, kitty=False, sixel=False, cell=None, xtversion=None)
    for m in _REPLY.finditer(data):
        if m[1] is not None:
            caps['kitty'] = caps['kitty'] or m[1].startswith(b'i=%d;OK' % PROBE_ID)
        elif m[2] is not None:
            caps['xtversion'] = m[2].decode('utf-8', 'replace')
        elif m[3] is not None:
            if int(m[3]) > 0 and int(m[4]) > 0:
                caps['cell'] = (int(m[4]), int(m[3]))
        else:
            caps['done'] = True
            caps['sixel'] = '4' in m[5].decode().split(';')[1:]
    return caps, _REPLY.sub(b'', data)


def _ask(fd, write, query, timeout):
    write(query)
    data, deadline = b'', time.monotonic() + timeout
    while True:
        caps, rest = parse(data)
        left = deadline - time.monotonic()
        if caps['done'] or left <= 0:
            return caps, rest
        if select.select([fd], [], [], left)[0]:
            chunk = os.read(fd, 4096)
            if not chunk:
                return caps, rest
            data += chunk


def _kitty_env(env):
    return env.get('TERM') == 'xterm-kitty' or 'KITTY_WINDOW_ID' in env


def probe(fd, write, timeout=PROBE_S, env=None):
    """Ask the terminal on ``fd`` (XTVERSION, cell size, DA1), then the kitty graphics query if it
    is kitty or ghostty; each round ends at the DA1 reply or ``timeout`` s. ``(caps, rest)``."""
    env = os.environ if env is None else env
    caps, rest = _ask(fd, write, PROBE, timeout)
    name = (caps['xtversion'] or '').lower()
    if caps['done'] and (name.startswith(KITTY_NAMES) or (not name and _kitty_env(env))):
        more, extra = _ask(fd, write, PROBE_KITTY, timeout)
        caps['kitty'], rest = more['kitty'], rest + extra
    return caps, rest


def choose(setting, caps, env=None, colours='truecolor'):
    """The tier for theme.yaml's ``graphics`` setting, the probe's ``caps`` and the environment.

    No setting picks an image tier under tmux or screen (named by XTVERSION even through ssh and
    docker, or by $TMUX / $STY): tmux keeps only a few sixel images per pane and drops the oldest
    tiles. Nor in a dumb terminal, nor sixel unless the terminal gave its cell size: a guessed cell
    lets tiles spill past their cells, over the arm and stop popups."""
    from mhseals_testing.ui import palette
    env = os.environ if env is None else env
    name = (caps.get('xtversion') or '').lower()
    if (setting == 'off' or env.get('NO_COLOR') or palette.ascii_mode(env) or env.get('TERM') == 'dumb'
            or env.get('TMUX') or env.get('STY') or name.startswith(('tmux', 'screen'))):
        return 'text'
    if setting == 'kitty' or (setting == 'auto' and caps.get('kitty')
                              and colours in ('256', 'truecolor', 'eight_bit')
                              and (name.startswith(KITTY_NAMES) or (not name and _kitty_env(env)))):
        return 'kitty'
    return 'sixel' if caps.get('cell') and (setting == 'sixel' or caps.get('sixel')) else 'text'


def start(keys, console, write, setting='off', env=None):
    """Probe the terminal on the key reader's fd (once a session) and pick the tier; returns it.

    ``off`` probes nothing. Bytes that were not probe replies (keys typed meanwhile) go back to the
    reader's buffer. In sixel the console's stream is wrapped so markers become cursor-forward.
    Without a tty on the reader's fd every setting is text."""
    global TIER, CELL, K, GENERATION, SETTING, BUDGET, _stream
    _reset()
    GENERATION += 1
    SETTING = setting
    env = os.environ if env is None else env
    fd = getattr(keys, 'fd', None)
    caps = _caps
    try:
        tty = fd is not None and os.isatty(fd)
    except OSError:
        tty = False
    if not caps and tty and choose(setting, {'sixel': True, 'cell': DEFAULT_CELL}, env) != 'text':
        try:
            caps, rest = probe(fd, write, env=env)
            _caps.update(caps)
            if rest and hasattr(keys, 'buffer'):
                keys.buffer.extend(rest)
                keys.escape_started = time.monotonic()
        except OSError:
            caps = {}
    system = getattr(console, 'color_system', None) or 'truecolor'
    TIER = choose(setting, caps, env, system) if tty else 'text'
    BUDGET = SSH_BUDGET if env.get('SSH_CONNECTION') else None
    CELL = caps.get('cell') or DEFAULT_CELL
    K = 1 if TIER == 'text' else max(1, min(4, round(CELL[0] / 3.5)))
    if TIER == 'sixel' and console is not None and not isinstance(console.file, _Stream):
        _stream = console.file = _Stream(console.file)
    return TIER


def stop(console, write):
    """Free kitty's images and unwrap the stream; widgets draw braille again."""
    global TIER, K
    if TIER == 'kitty' and _by_id:
        write(''.join(_delete(i) for i in _by_id))
    if console is not None and isinstance(console.file, _Stream):
        console.file = console.file.inner
    TIER, K = 'text', 1
    _reset()


def restart(keys, console, write, setting, env=None):
    """A new ``graphics`` setting mid-session: free the old tier, start the new one (the terminal
    is probed at most once a session). Clears the screen when images were up; returns the tier."""
    shown = TIER != 'text'
    stop(console, write)
    tier = start(keys, console, write, setting, env)
    if shown:
        write('\x1b[2J')
    return tier


def forget():
    """Drop the cached probe (a new terminal: the next start asks again)."""
    _caps.clear()


def use(tier, cell=DEFAULT_CELL, budget=None):
    """Set the tier without a terminal (tests)."""
    global TIER, CELL, K, GENERATION, BUDGET
    _reset()
    GENERATION += 1
    TIER, CELL, BUDGET = tier, cell, budget
    K = 1 if tier == 'text' else max(1, min(4, round(cell[0] / 3.5)))


def _reset():
    global _size
    _pool.clear()
    _by_id.clear()
    _free[:] = list(range(POOL, 0, -1))
    _pending.clear()
    _surfaces.clear()
    _live.clear()
    _drawn.clear()
    _sent.clear()
    _size = None
    STATS.update(upload=0, sixel=0, tiles=0)


def active():
    return TIER != 'text'


def _meter(size):
    now = clock()
    _sent.append((now, size))
    while _sent[0][0] < now - METER_S:      # trimmed here too: rate() runs only under a budget
        _sent.popleft()


def rate(now=None):
    """Image bytes per second written over the last METER_S."""
    now = clock() if now is None else now
    while _sent and _sent[0][0] < now - METER_S:
        _sent.popleft()
    return sum(size for _, size in _sent) / METER_S


# Kitty placeholders are text Rich rewrites every frame (about 4-10 B a cell): for the large,
# boat-centred minimap that is 2-3x its braille, so under kitty it stays braille (sixel draws it).
KITTY_TEXT = ('minimap',)


def images(widget):
    """Whether ``widget`` draws as image cells in the active tier."""
    return TIER == 'sixel' or (TIER == 'kitty' and widget not in KITTY_TEXT)


def k(widget):
    """Dots per braille dot of ``widget``'s canvas: K as an image, 1 as braille."""
    return K if images(widget) else 1


# --- colours -----------------------------------------------------------------------------------

def rgb(style, fallback=(213, 219, 229)):
    """The (r, g, b) of a Rich style string or palette colour (``#hex``, ``color(N)``, a name)."""
    try:
        colour = Style.parse(style or '').color
    except Exception:  # an unparsable style: the fallback
        colour = None
    return tuple(colour.get_truecolor()) if colour is not None else fallback


def token(name, fallback=(213, 219, 229)):
    """A theme token's truecolor (r, g, b) for the current light/dark mode."""
    from mhseals_testing.ui import palette, theme
    try:
        mode = palette.detect_theme() if theme.PALETTE.mode == 'auto' else theme.PALETTE.mode
        return rgb(theme.PALETTE.resolve(name, mode, 'truecolor'), fallback)
    except (KeyError, ValueError):
        return fallback


# --- raster ------------------------------------------------------------------------------------

class Raster:
    """An indexed image (0 = transparent) with flat-colour primitives: no anti-aliasing."""

    def __init__(self, cols, rows, cell=None):
        self.cw, self.ch = cell or CELL
        self.cols, self.rows = cols, rows
        self.w, self.h = cols * self.cw, rows * self.ch
        self.px = np.zeros((self.h, self.w), np.uint8)
        self.inks = {}
        self.palette = [(0, 0, 0)]

    def ink(self, colour):
        """The index of an (r, g, b) colour, added on first use."""
        index = self.inks.get(colour)
        if index is None:
            index = self.inks[colour] = len(self.palette)
            self.palette.append(colour)
        return index

    def rect(self, x0, y0, x1, y1, ink):
        x0, y0, x1, y1 = max(0, int(x0)), max(0, int(y0)), min(self.w, int(x1)), min(self.h, int(y1))
        if x0 < x1 and y0 < y1:
            self.px[y0:y1, x0:x1] = ink

    def squares(self, xs, ys, size, ink, high=None):
        """A ``size`` px square (``size`` x ``high``) at each top-left (xs, ys), in one numpy pass."""
        xs, ys = np.asarray(xs, int), np.asarray(ys, int)
        for dy in range(high or size):
            for dx in range(size):
                x, y = xs + dx, ys + dy
                ok = (x >= 0) & (x < self.w) & (y >= 0) & (y < self.h)
                self.px[y[ok], x[ok]] = ink

    def line(self, x0, y0, x1, y1, ink, width=1, dash=0):
        """A straight line of ``width`` px squares; ``dash`` px on, ``dash`` off when set."""
        n = max(1, int(max(abs(x1 - x0), abs(y1 - y0)) + 0.5))
        t = np.arange(n + 1) / n
        if dash:
            t = t[(np.arange(n + 1) // dash) % 2 == 0]
        off = (width - 1) / 2
        self.squares(np.floor(x0 + (x1 - x0) * t - off), np.floor(y0 + (y1 - y0) * t - off), width, ink)

    def polyline(self, points, ink, width=1):
        for a, b in zip(points, points[1:]):
            self.line(*a, *b, ink, width)

    def polygon(self, points, ink):
        """Even-odd scanline fill, sampled at pixel centres."""
        if len(points) < 3:
            return
        ys = [p[1] for p in points]
        edges = list(zip(points, points[1:] + points[:1]))
        for y in range(max(0, int(min(ys))), min(self.h, int(max(ys)) + 1)):
            yc = y + 0.5
            xs = sorted(x0 + (yc - y0) * (x1 - x0) / (y1 - y0)
                        for (x0, y0), (x1, y1) in edges if (y0 <= yc) != (y1 <= yc))
            for a, b in zip(xs[::2], xs[1::2]):
                self.rect(round(a), y, round(b), y + 1, ink)

    def rgba(self):
        return _rgba(self.px, self.palette)


def _rgba(px, palette):
    lut = np.array([(r, g, b, 255) for r, g, b in palette], np.uint8)
    lut[0, 3] = 0
    return lut[px]


def dots_to_px(raster, layers, sx, sy):
    """Paint dot layers ``[(ink, xs, ys)]`` (dot coordinates, ``sx`` x ``sy`` dots per cell), lowest
    priority first; each dot is a crisp block of at least 2 x 2 px that leaves no seam to the next."""
    fx, fy = raster.cw / sx, raster.ch / sy
    wide, high = max(2, int(np.ceil(fx))), max(2, int(np.ceil(fy)))
    for ink, xs, ys in layers:
        if len(xs):
            raster.squares(np.floor(np.asarray(xs) * fx), np.floor(np.asarray(ys) * fy), wide, ink, high)


# --- tiles and ids -----------------------------------------------------------------------------

class Tile:
    __slots__ = ('id', 'digest', 'cols', 'rows', 'px', 'palette', 'sent')

    def __init__(self, digest, cols, rows, px, palette):
        self.id, self.digest, self.cols, self.rows = 0, digest, cols, rows
        self.px, self.palette, self.sent = px, palette, False


def _tile(raster, c0, r0, cols, rows):
    """The pooled Tile for a cell rectangle of ``raster``, made and queued for kitty when new."""
    ys, xs = slice(r0 * raster.ch, (r0 + rows) * raster.ch), slice(c0 * raster.cw, (c0 + cols) * raster.cw)
    return _pooled(raster.px[ys, xs], raster.palette, cols, rows)


def _pooled(px, palette, cols, rows):
    h = hashlib.blake2b(bytes(c for rgb in palette for c in rgb), digest_size=12,
                        person=b'%dx%d' % (cols, rows))
    h.update(np.ascontiguousarray(px))
    digest = h.digest()
    tile = _pool.get(digest)
    if tile is not None:
        _pool.move_to_end(digest)
        return tile
    if not _free:
        _evict()
    tile = Tile(digest, cols, rows, px.copy(), list(palette))
    tile.id = _free.pop()
    _pool[digest] = tile
    _by_id[tile.id] = tile
    STATS['tiles'] += 1
    if TIER == 'kitty':
        _pending.append(tile)
    return tile


def _dim(n):
    """The id of tile ``n`` dimmed halfway to the theme bg, as Rich dims the text under a modal
    backdrop (layout.Overlay): an image never outshines the popup over it."""
    tile = _by_id.get(n)
    if tile is None:
        return n
    bg = token('bg', (0, 0, 0))
    palette = [tile.palette[0]] + [tuple((c + b) // 2 for c, b in zip(rgb, bg)) for rgb in tile.palette[1:]]
    return _pooled(tile.px, palette, tile.cols, tile.rows).id


def _evict():
    _, tile = _pool.popitem(last=False)
    del _by_id[tile.id]
    for place in [p for p, (n, _) in _drawn.items() if n == tile.id]:
        del _drawn[place]
    if TIER == 'kitty' and tile.sent:
        _pending.append(_delete(_kitty_id(tile.id)))
    _free.append(tile.id)


def _kitty_id(n):
    """Pool slot ``n`` (1..POOL) as a kitty id: low byte 1..255 in the colour, high byte >= 1."""
    return ((1 + (n - 1) // 255) << 24) | (1 + (n - 1) % 255)


def _delete(image_id):
    return f'{ESC}_Ga=d,d=I,i={image_id},q=2{ST}'


def cell(tile, r, c):
    """``(text, style)`` of cell (r, c) of ``tile`` in the active tier."""
    if TIER == 'kitty':
        kid = _kitty_id(tile.id)
        return PLACEHOLDER + DIACRITICS[r] + DIACRITICS[c] + DIACRITICS[kid >> 24], f'color({kid & 0xFF})'
    return chr(MARK0 + tile.id * 128 + r * 16 + c), ''


def surface(name, raster_or_paint, cols, rows, key=None, hz=None, holes=(), tile=TILE):
    """Cells of an image widget: ``rows`` lists of ``cols`` ``(text, style)``, None at ``holes``.

    ``raster_or_paint`` is a Raster or a ``paint(raster)`` callable (called only when due: ``key``
    changed, or ``hz`` allows a repaint). Cells listed in ``holes`` are left to the widget's text."""
    now = clock()
    held = _surfaces.get(name)
    if held is not None and held[0] == (key, cols, rows, GENERATION) and key is not None:
        return held[2]
    if held is not None and hz and now - held[1] < 1.0 / hz and held[0][1:] == (cols, rows, GENERATION):
        return held[2]
    if (held is not None and BUDGET and now - held[1] < 1.0 / THROTTLE_HZ
            and held[0][1:] == (cols, rows, GENERATION) and rate(now) > BUDGET):
        return held[2]        # over the link budget: every image repaints at THROTTLE_HZ at most
    raster = raster_or_paint
    if callable(raster_or_paint):
        raster = Raster(cols, rows)
        raster_or_paint(raster)
    holes = set(holes)
    grid = [[None] * cols for _ in range(rows)]
    tw, th = tile
    for r0 in range(0, rows, th):
        for c0 in range(0, cols, tw):
            w, h = min(tw, cols - c0), min(th, rows - r0)
            if all((c0 + c, r0 + r) in holes for r in range(h) for c in range(w)):
                continue
            if TIER == 'kitty' and not raster.px[r0 * raster.ch:(r0 + h) * raster.ch,
                                                 c0 * raster.cw:(c0 + w) * raster.cw].any():
                t = None      # kitty: an empty tile is plain spaces, no upload or placeholders
            else:
                t = _tile(raster, c0, r0, w, h)
            for r in range(h):
                for c in range(w):
                    if (c0 + c, r0 + r) not in holes:
                        grid[r0 + r][c0 + c] = cell(t, r, c) if t is not None else BLANK
    _surfaces[name] = ((key, cols, rows, GENERATION), now, grid)
    return grid


def text_rows(grid, hole=lambda col, row: (' ', None)):
    """Rich Text rows from a surface grid; ``hole(col, row)`` gives ``(text, style)`` of a hole."""
    from rich.text import Text
    out = []
    for row, cells in enumerate(grid):
        line, run, look = Text(), [], None
        for col, spec in enumerate(cells):
            text, style = spec if spec is not None else hole(col, row)
            if style != look and run:      # one span (one SGR) per run of a look, not per cell
                line.append(''.join(run), look)
                run = []
            run.append(text)
            look = style
        if run:
            line.append(''.join(run), look)
        out.append(line)
    return out


def ansi_rows(grid, hole):
    """ANSI strings from a surface grid; ``hole(col, row)`` gives a hole's already-styled string."""
    out = []
    for row, cells in enumerate(grid):
        parts, colour = [], None
        for col, spec in enumerate(cells):
            if spec is None:
                if colour is not None:
                    parts.append('\x1b[0m')
                    colour = None
                parts.append(hole(col, row))
                continue
            if spec[1] != colour:
                parts.append(f'\x1b[0;38;5;{spec[1][6:-1]}m' if spec[1] else '\x1b[0m')
                colour = spec[1]
            parts.append(spec[0])
        out.append(''.join(parts) + ('\x1b[0m' if colour else ''))
    return out


# --- the frame ---------------------------------------------------------------------------------

def _scan(segments):
    """``({id: {(r, c): (row, col)}}, segments)``: the tile cells in rendered ``segments`` (screen
    rows from 0; kitty ids only), and the segments with kitty placeholders shortened: a cell that
    continues the one to its left on screen goes out bare (4 bytes, not 10)."""
    found, row, col, out = {}, 0, 0, []
    last = last_at = None
    for seg in segments:
        text = seg.text
        if seg.control:
            out.append(seg)
            continue
        if '\n' in text:
            for i, part in enumerate(text.split('\n')):
                if i:
                    row, col = row + 1, 0
                col += cell_len(part)
            out.append(seg)
            continue
        if TIER == 'sixel' and _MARKS.search(text):
            dim, pos = bool(seg.style and seg.style.dim), 0    # dim: under a modal backdrop
            for match in _MARKS.finditer(text):
                here = col + cell_len(text[pos:match.start()])
                codes = [ord(ch) - MARK0 for ch in match.group()]
                i = 0
                while i < len(codes):       # one run per tile row: its codes count up by one
                    n, j = codes[i], i + 1
                    while j < len(codes) and codes[j] == codes[j - 1] + 1 and (codes[j] ^ n) >> 4 == 0:
                        j += 1
                    r, c = (n >> 4) & 7, n & 15     # by the tile's screen origin: a tile may show twice
                    found.setdefault(n >> 7, {}).setdefault((row - r, here - c, dim), []).append((r, c, c + j - i))
                    here, i = here + j - i, j
                col, pos = here, match.end()
            col += cell_len(text[pos:])
            out.append(seg)
            continue
        elif TIER == 'kitty' and PLACEHOLDER in text and seg.style is not None and seg.style.color is not None:
            if seg.style.dim:
                seg = _kitty_dim(seg)
                text = seg.text
            low = seg.style.color.number or 0
            parts = text.split(PLACEHOLDER)
            kept, here = [parts[0]], col + cell_len(parts[0])
            for part in parts[1:]:
                d = [DIACRITICS.index(ch) if ch in DIACRITICS else -1 for ch in part[:3]]
                if len(d) == 3 and min(d) >= 0:
                    found.setdefault((d[2] - 1) * 255 + low, {})
                    cur = (low, d[0], d[1], d[2])
                    # kitty infers row, column and id byte from the cell to the left: drop them
                    bare = last == (low, d[0], d[1] - 1, d[2]) and last_at == (row, here)
                    kept.append(PLACEHOLDER + (part[3:] if bare else part))
                    last, last_at = cur, (row, here + 1)
                else:
                    kept.append(PLACEHOLDER + part)
                here += 1 + cell_len(part[3:] if len(d) == 3 and min(d) >= 0 else part)
            seg = Segment(''.join(kept), seg.style, seg.control)
        out.append(seg)
        col += cell_len(text)
    return found, out


def _kitty_dim(seg):
    """A dimmed placeholder run (under a modal backdrop) re-pointed at its tile's dimmed copy; a
    run naming more than one tile is left as it is."""
    low = seg.style.color.number or 0
    parts = seg.text.split(PLACEHOLDER)
    highs = {p[2] for p in parts[1:] if len(p) >= 3 and p[2] in DIACRITICS}
    if len(highs) != 1:
        return seg
    kid = _kitty_id(_dim((DIACRITICS.index(highs.pop()) - 1) * 255 + low))
    high = DIACRITICS[kid >> 24]
    text = parts[0] + ''.join(PLACEHOLDER + (p[:2] + high + p[3:] if len(p) >= 3 else p) for p in parts[1:])
    return Segment(text, seg.style + Style(color=f'color({kid & 0xFF})'), seg.control)


def before(write):
    """Kitty: send tiles made since the last frame (and the deletes of evicted ones)."""
    if TIER != 'kitty' or not _pending:
        return
    out = []
    for item in _pending:
        if isinstance(item, str):
            out.append(item)
        elif item.id in _by_id and _by_id[item.id] is item and not item.sent:
            out.append(kitty_upload(_kitty_id(item.id), item))
            item.sent = True
    _pending.clear()
    data = ''.join(out)
    STATS['upload'] += len(data)
    if data:
        _meter(len(data))
        write(data)


def frame(segments, size):
    """After layout, before the write: touch the tiles on screen; in sixel, find complete tiles.
    Returns the segments to write (kitty's shortened)."""
    global _size
    if TIER == 'text':
        return segments
    found, segments = _scan(segments)
    for n in found:
        tile = _by_id.get(n)
        if tile is not None:
            _pool.move_to_end(tile.digest)
    if TIER != 'sixel':
        return segments
    if size != _size:
        _drawn.clear()
        _size = size
    _live.clear()
    for n, places in found.items():
        if n not in _by_id:
            continue
        # A tile's visible cells (a label hole, a popup's edge or corner may hide some) go out as
        # the fewest rectangles of whole rows: each is drawn cropped, never over text.
        for (top, left, dim), cells in places.items():
            shown = _dim(n) if dim else n
            for r0, c0, r1, c1 in _rects(cells):
                _live[(top + r0, left + c0)] = (shown, (r0, c0, r1, c1))
    if _stream is not None:      # the markers carry the undimmed id
        _stream.live = frozenset(n for n in found if n in _by_id)
    return segments


def _rects(spans):
    """``(r0, c0, r1, c1)`` rectangles covering the ``(r, c0, c1)`` row spans: a row's touching
    spans joined, then merged with the same span on the rows below."""
    rows = {}
    for r, a, b in sorted(spans):
        line = rows.setdefault(r, [])
        if line and line[-1][1] == a:
            line[-1] = (line[-1][0], b)
        else:
            line.append((a, b))
    first = next(iter(rows.values()))
    if all(line == first for line in rows.values()) and len(first) == 1 and len(rows) == max(rows) - min(rows) + 1:
        return [(min(rows), first[0][0], max(rows) + 1, first[0][1])]   # the usual case: one rectangle
    rects, open_ = [], {}
    for r in sorted(rows):
        runs = rows[r]
        still = {}
        for run in runs:
            if run in open_ and open_[run][1] == r:
                still[run] = (open_.pop(run)[0], r + 1)
            else:
                still[run] = (r, r + 1)
        rects.extend((r0, c0, r1, c1) for (c0, c1), (r0, r1) in open_.items())
        open_ = still
    rects.extend((r0, c0, r1, c1) for (c0, c1), (r0, r1) in open_.items())
    return sorted(rects)


def after(write):
    """Sixel: clear and paint each visible tile part not already painted there."""
    global _drawn
    if TIER != 'sixel':
        return
    out, drawn = [], {}
    for (row, col), (n, crop) in _live.items():
        tile = _by_id.get(n)
        if tile is None:            # evicted while this frame made a dimmed copy: drawn next frame
            continue
        drawn[(row, col)] = (n, crop)
        if _drawn.get((row, col)) == (n, crop):
            continue
        r0, c0, r1, c1 = crop
        ch, cw = tile.px.shape[0] // tile.rows, tile.px.shape[1] // tile.cols
        out.append(''.join(f'\x1b[{row + r + 1};{col + 1}H' + ' ' * (c1 - c0) for r in range(r1 - r0)))
        part = tile.px[r0 * ch:r1 * ch, c0 * cw:c1 * cw]
        if part.any():               # an empty tile only clears what was under it, once
            out.append(f'\x1b[{row + 1};{col + 1}H' + sixel(part, tile.palette))
    _drawn = drawn
    if out:
        data = '\x1b7\x1b[0m' + ''.join(out) + '\x1b8'
        STATS['sixel'] += len(data)
        _meter(len(data))
        write(data)


class _Stream:
    """The console's file in sixel: marker runs go out as cursor-forward over a painted tile, or
    spaces where the tile is not drawn (covered, cut, or its image is gone)."""

    def __init__(self, inner):
        self.inner, self.live = inner, frozenset()

    def _swap(self, match):
        out, skip = [], 0
        for ch in match.group():
            if (ord(ch) - MARK0) >> 7 in self.live:
                skip += 1
                continue
            if skip:
                out.append(f'\x1b[{skip}C')
                skip = 0
            out.append(' ')
        if skip:
            out.append(f'\x1b[{skip}C')
        return ''.join(out)

    def write(self, text):
        return self.inner.write(_MARKS.sub(self._swap, text))

    def __getattr__(self, name):
        return getattr(self.inner, name)


# --- encoders ----------------------------------------------------------------------------------

def kitty_upload(image_id, tile, chunk=4096):
    """Transmit a tile as zlib RGBA with a virtual placement for its Unicode placeholders."""
    h, w = tile.px.shape
    data = base64.standard_b64encode(zlib.compress(_rgba(tile.px, tile.palette).tobytes(), 6)).decode('ascii')
    parts = [data[i:i + chunk] for i in range(0, len(data), chunk)] or ['']
    head = f'a=T,f=32,s={w},v={h},o=z,U=1,i={image_id},c={tile.cols},r={tile.rows},q=2'
    return ''.join(f'{ESC}_G{head if i == 0 else "q=2"},m={int(i < len(parts) - 1)};{p}{ST}'
                   for i, p in enumerate(parts))


def sixel(px, palette):
    """A sixel image of indexed pixels ``px`` (0 transparent, P2=1) with run-length encoding."""
    h, w = px.shape
    used = [int(i) for i in np.unique(px) if i]
    head = f'{ESC}P0;1;0q"1;1;{w};{h}' + ''.join(
        f'#{i};2;{r * 100 // 255};{g * 100 // 255};{b * 100 // 255}' for i in used for r, g, b in (palette[i],))
    bands = -(-h // 6)
    stack = np.zeros((bands * 6, w), np.uint8)
    stack[:h] = px
    stack = stack.reshape(bands, 6, w)
    weights = (1 << np.arange(6, dtype=np.uint8))[None, :, None]
    # every colour's sixel characters for every band at once: (colour, band, x)
    chars = np.stack([((stack == i) * weights).sum(1, dtype=np.uint8) + 63 for i in used]) if used else []
    out = []
    for b in range(bands):
        rows = []
        for c, i in enumerate(used):
            line = chars[c, b].tobytes().rstrip(b'?')      # no trailing blanks; a colour absent here: none
            if line:
                rows.append(f'#{i}' + _RUN.sub(_run, line.decode('ascii')))
        out.append('$'.join(rows))
    while out and not out[-1]:       # no empty bands, and no line feed, below the last pixels
        out.pop()
    return head + '-'.join(out) + ST


_RUN = re.compile(r'(.)\1{3,}')     # four or more of a character go out as !N<char>


def _run(match):
    return f'!{len(match.group())}{match.group(1)}'

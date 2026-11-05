"""The Fusion tab's shared pieces: tokens and sizes, and one paint for every tier (Inks, braille cells, the
throttled draw and its rate over a link budget)."""

import numpy as np

from mhseals_testing.ui import palette, theme
from mhseals_testing.ui import imaging as pixels


RAMP = tuple(f'spd.range.{band}' for band in range(5, -1, -1))   # near (warm) to far; spd.range.0 past the last break
# theme.yaml spd.range.*: the spd tones with one 16-colour hue per band
STALE = 'warn'         # the whole scan in one ink no band uses while it cannot be read against the boxes
GREYS = 6              # grey levels of the frame behind the points
LINK_BLOCKS = (64, 32, 16, 8, 4)   # px: over a link budget the frame is drawn in blocks this size (runs that sixel/kitty pack),
#                                coarsest first: it starts coarse and refines while the budget has room
CLASS_TOKENS = tuple(t[4:] for t in theme.PALETTE.tokens if t.startswith('obj.') and t != 'obj.unknown')   # obj.red: red
SOURCE_MARK = {'lidar': 'L', 'split': 'L', 'lidar_ray': 'R', 'held': 'H', 'stereo': 'S', 'pass': '-', 'fused': '?'}
READOUT_ROWS = 8       # Alignment's six lines and its border
AROUND_MIN = 24        # Around's least width beside the camera column
AROUND_SHARE = 0.3     # and its least share of the width
TEXT_CELL = (2, 4)     # braille dots per cell
BITS = ((0x01, 0x02, 0x04, 0x40), (0x08, 0x10, 0x20, 0x80))
ASCII = {'point': '.', 'stale': ':', 'ring': '.', 'fov': '.', 'stereo': 'o', 'bar': '|', 'box': '#', 'ray': '*', 'det': '@', 'boat': '^'}
DEG = theme.pick('°', 'deg')
PM = theme.pick('±', '+-')
SPARK_COLUMNS = 12
STAMP_MATCH_S = 0.005  # frame and boxes this close count as one capture (no note)


def class_token(label):
    """The obj.* token of a class label (``red_buoy`` -> obj.red); obj.unknown otherwise."""
    return next((f'obj.{c}' for c in CLASS_TOKENS if c in label.lower()), 'obj.unknown')


def ramp_index(r, breaks):
    return int(np.searchsorted(breaks, r, side='right')) if r is not None else len(breaks)


def range_label(det):
    """``7.2L`` (fused range and source), with ``/6.8S`` when the stereo range differs by >= 0.1 m."""
    mark = SOURCE_MARK.get(det['source'], '?')
    if det['range'] is None:
        return mark
    out = f"{det['range']:.1f}{mark}"
    if det['stereo'] is not None and mark != 'S' and abs(det['stereo'] - det['range']) >= 0.1:
        out += f"/{det['stereo']:.1f}S"
    return out


def optical_xz(det, pos):
    """(x right, z ahead) of a position in the det's frame: optical when the frame says so, else body."""
    x, y, z = pos
    return (x, z) if 'optical' in det['frame'] else (-y, x)


# --- one paint for every tier ---------------------------------------------------------------------

def ink_style(token):
    """The text style of a layer token (points and legend alike)."""
    return theme.PALETTE.style(token)


class Inks:
    """Layer name -> ink index, allocated in priority order so a text cell takes its highest layer."""

    def __init__(self, raster, layers):
        self.ids = {}
        for name, token in layers:
            self.ids[name] = raster.ink(pixels.token(token))
        self.styles = {name: ink_style(token) for name, token in layers}
        self.order = [name for name, _ in layers]

    def __getitem__(self, name):
        return self.ids[name]


_WEIGHTS = np.array([[BITS[dx][dy] for dx in range(2)] for dy in range(4)], np.int32)


def text_cells(raster, inks, chars):
    """Braille (or ASCII) cells of a TEXT_CELL raster: ``rows`` lists of (char, style)."""
    blocks = raster.px.reshape(raster.rows, 4, raster.cols, 2).transpose(0, 2, 1, 3)
    top = blocks.max(axis=(2, 3))
    bits = ((blocks > 0) * _WEIGHTS).sum(axis=(2, 3))
    looks = {}
    for name, ink in inks.ids.items():
        looks[ink] = (chars.get(name.split('.')[0], '.'), inks.styles.get(name))
    out = []
    for row in range(raster.rows):
        cells = []
        for col in range(raster.cols):
            ink = int(top[row, col])
            if not ink:
                cells.append((' ', None))
                continue
            char, style = looks.get(ink, ('.', None))
            cells.append((char if theme.ASCII else chr(0x2800 + int(bits[row, col])), style))
        out.append(cells)
    return out


_TEXT = {}    # drawing name -> (key, painted at, rows): the text tier repaints only when its data changes
_LABELS = {}  # drawing name -> (surface grid, labels): an image tile's labels, held until the tile repaints


def draw(name, paint, layers, cols, rows, key, labels, hz=None):
    """Text rows of a drawing: image tiles (labels as holes) or braille/ASCII with the labels written in.

    A changed ``key`` repaints at ``hz`` at most (the boxes change every frame); a new size at once. Labels go with
    the drawing they were made for: between repaints the held tile keeps its own (its holes are where they were)."""
    if pixels.active() and pixels.images(name):
        def image(r):
            paint(r, Inks(r, layers))
        grid = pixels.surface(name, image, cols, rows, key=key, hz=hz, holes=labels)
        held = _LABELS.get(name)
        if held is not None and held[0] is grid:
            labels = held[1]
        else:
            _LABELS[name] = (grid, labels)
        return pixels.text_rows(grid, lambda col, row: labels.get((col, row), (' ', None)))
    held, now, tier = _TEXT.get(name), pixels.clock(), palette.detect_tier()
    if held is not None and held[0][1:] == (cols, rows, theme.ASCII, tier) and (
            held[0][0] == key or (hz and now - held[1] < 1.0 / hz)):
        return held[2]
    raster = pixels.Raster(cols, rows, cell=TEXT_CELL)
    inks = Inks(raster, layers)
    paint(raster, inks)
    grid = text_cells(raster, inks, ASCII)
    for (col, row), cell in labels.items():
        if 0 <= row < rows and 0 <= col < cols:
            grid[row][col] = cell
    out = pixels.text_rows(grid)
    _TEXT[name] = ((key, cols, rows, theme.ASCII, tier), now, out)
    return out


LINK_SHARE = 0.9      # of the link budget the drawings aim at
RATE_S = 5.0          # s between rate updates over a link budget: a few repaints of ~10 kB each, so one is not noise
START = 0.25          # of the top rate a drawing starts at, then steps up by at most STEP each update
STEP = 1.5
OVER_S = 0.5          # s between rate updates while the link is over its budget (a burst is answered at once, not each tick)
_HZ = {}              # drawing name -> (repaints per second, set at, image bytes sent by then, index in LINK_BLOCKS)


def draw_rate(cfg, name, budgeted=1.0):
    """Repaints per second: draw_rate, and over a link budget at most THROTTLE_HZ times ``budgeted`` (two big surfaces
    share the link, so Around takes half the camera's rate), resized every RATE_S from the image bytes the held rate
    sent (at once when the link is over its budget; a drawing starts at START of the top rate and rises at most STEP
    times per update): ``hz * LINK_SHARE * BUDGET / sent``. A map from the rate alone (rate / BUDGET, or its square)
    settles above the budget; this settles near LINK_SHARE of it (a dense scan left 1.1-2 times the budget)."""
    if not pixels.BUDGET:
        return cfg['draw_rate']
    top, now, total = min(cfg['draw_rate'], pixels.THROTTLE_HZ) * budgeted, pixels.clock(), pixels.STATS['upload'] + pixels.STATS['sixel']
    held = _HZ.get(name)
    over = pixels.rate(now) > pixels.BUDGET
    if held is not None and total >= held[2] and now - held[1] < (OVER_S if over else RATE_S):
        return held[0]
    hz, level = top * START, 0
    if held is not None and total >= held[2]:
        raw = held[0] * LINK_SHARE * pixels.BUDGET * (now - held[1]) / max(total - held[2], 1.0)
        hz, level = min(raw, held[0] * STEP, held[0] if over else top), held[3]
        # the frame's blocks follow the size of a repaint, which no 1 s window may go over: a repaint over its share
        # coarsens them, one that fits four times refines them (every tile of a finer frame is new: a step sends several repaints' worth)
        size = pixels.paint_size(name)
        if size is not None and size > pixels.PAINT_SHARE * pixels.BUDGET:
            level = max(0, level - 1)
        elif size is not None and 4 * size <= pixels.PAINT_SHARE * pixels.BUDGET and not over:
            level = min(len(LINK_BLOCKS) - 1, level + 1)
    hz = min(top, max(top / 16, hz))
    _HZ[name] = (hz, now, total, level)
    return hz


def link_block(name='fusion.camera'):
    """px of a block of the frame: 1 locally; over a link budget the drawing's level in LINK_BLOCKS."""
    return LINK_BLOCKS[_HZ[name][3] if name in _HZ else 0] if pixels.BUDGET else 1


def link_stride(n, cfg, framed=False):
    """Every k-th point so at most ``points_link`` (``points_link_framed`` over the camera image) are drawn per paint
    over a link budget (all of them locally)."""
    return max(1, -(-n // cfg['feed']['points_link_framed' if framed else 'points_link'])) if pixels.BUDGET else 1


def scan_stale(model, cfg):
    """Whether the drawn scan cannot be read against the boxes: the boat is turning (lidar_fusion skipped its frame
    for motion) or the scan is more than scan_dt_max from the boxes' stamp. Not motion compensated, a lagged scan
    sits off its boxes like an extrinsic error, so it is drawn in the STALE ink and the subtitle leads with it."""
    dt = model.get('scan_dt')
    return bool(model.get('motion')) or (dt is not None and abs(dt) > cfg['feed']['scan_dt_max'])


def label_cells(text, col, row, cols, rows, style):
    """Hole cells for ``text`` starting at (col, row), shifted left to stay inside ``cols``."""
    col = max(0, min(col, cols - len(text)))
    if not 0 <= row < rows:
        return {}
    return {(col + i, row): (ch, style) for i, ch in enumerate(text[:cols])}

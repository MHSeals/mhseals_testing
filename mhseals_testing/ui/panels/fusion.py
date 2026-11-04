"""The Fusion tab: camera-lidar alignment at a glance (perception-calibration.md 2, lidar-fusion-design).

Three panels from ``ros.fusion_feed.FusionFeed.snapshot()`` plus the fusion status facts:
- **Camera**: the scan projected into the image (with lidar_fusion's applied correction) coloured by
  range on the spd ramp, over the camera frame in grey while the image is on (``i``; image tiers only: the
  frame is drawn with the boxes and scan of its own stamp, so all three are what the camera saw), with each box in its class
  colour and its range label (``L`` lidar, ``R`` ray mode, ``H`` held, ``S`` stereo, then the stereo
  range when it differs). A misaligned extrinsic shows as points sitting off their boxes.
- **Around**: top-down in camera axes (x right, z ahead): FOV, range rings, the scan, each box at its
  fused position with a +-2 sigma range bar, a line to its stereo position and its range +-2 sigma;
  ray hits are diamonds.
- **Alignment**: the live check (fusion_calib's check rule over the live samples), the applied and
  estimated correction, the du/dv pixel residuals with their trend, box sources and fusion health.

One paint per drawing serves every tier: an image tier paints a pixels.Raster at the cell size and
shows its tiles; text paints the same Raster at braille resolution (2 x 4 dots a cell), each cell in
its highest layer's colour; ASCII uses one character per layer. Labels are terminal text (holes).
Pure: no ROS, no clock.
"""

import math

import numpy as np
from rich.console import Group
from rich.layout import Layout
from rich.text import Text

from mhseals_testing.ui import palette, theme
from mhseals_testing.ui import imaging as pixels

RAMP = ('spd.5', 'spd.4', 'spd.3', 'spd.2', 'spd.1', 'spd.0')   # near (warm) to far; spd.0 past the last break
# 16 colours: spd.5/spd.4 are both magenta (near a red buoy's box) and spd.3/spd.2 both yellow, so one hue per band;
# yellow is left to the stale scan (warn) and green to the buoys; every ANSI hue is taken, so the 5-10 m band is the
# bright cyan beside the plain cyan 10-20 m band
RAMP16 = dict(zip(RAMP, ('bright_magenta', 'bright_cyan', 'cyan', 'blue', 'white', 'bright_black')))
STALE = 'warn'         # the whole scan in one ink no band uses while it cannot be read against the boxes
GREYS = 6              # grey levels of the frame behind the points
LINK_BLOCKS = (64, 32, 16, 8, 4)   # px: over a link budget the frame is drawn in blocks this size (runs that sixel/kitty pack),
#                                coarsest first: it starts coarse and refines while the budget has room
CLASS_TOKENS = ('red', 'green', 'yellow', 'blue', 'black')
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
    """The text style of a layer token: the panel's own ramp in 16 colours (points and legend alike)."""
    return (RAMP16.get(token) if palette.detect_tier() == '16' else None) or theme.PALETTE.style(token)


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


# --- Camera ---------------------------------------------------------------------------------------

def camera_layers(dets):
    """Boxes under the points in braille (a cell takes its highest layer), so a cluster beside its box stays in
    range colours; in ASCII over them, since a box there is a few '#' and points on its corners hide it. Images
    draw boxes over the points either way."""
    boxes = [(f'box.{token}', token) for token in sorted({class_token(d['cls']) for d in dets})]
    ramp = [('stale', STALE)] + [(f'ramp{i}', token) for i, token in enumerate(reversed(RAMP))]
    return ramp + boxes if theme.ASCII else boxes + ramp


def frame_shown(model):
    """Whether the Camera tile draws the frame: the image on, an image tier for the tile, a frame with camera_info's
    aspect (the points and boxes are normalised by that size, so another crop would not line up). Over a link budget
    it joins after the tile's first repaint, so the link does not take both at once."""
    return (model.get('frame') is not None and model.get('frame_aspect') is not False
            and pixels.active() and pixels.images('fusion.camera')
            and (not pixels.BUDGET or not model.get('image') or _STARTED.get('fusion.camera') == pixels.GENERATION))


def camera_model(model):
    """The model the Camera tile draws: with the frame, its own stamp's boxes and scan (the feed's ``at_frame``)."""
    view = model.get('at_frame') if frame_shown(model) else None
    return {**model, **view} if view else model


def block_mean(frame, h, w, fy, fx):
    """``frame`` as h x w means, the block (i, j) covering rows i*fy.. of the frame's height and columns j*fx.. of its width
    (fy, fx: the block's share of the raster)."""
    edges = [np.minimum((np.arange(n) * f * size).astype(int), size - 1)
             for n, f, size in ((h, fy, frame.shape[0]), (w, fx, frame.shape[1]))]
    total = np.add.reduceat(np.add.reduceat(frame.astype(np.float32), edges[0], axis=0), edges[1], axis=1)
    return total / (np.maximum(np.diff(edges[0], append=frame.shape[0]), 1)[:, None]
                    * np.maximum(np.diff(edges[1], append=frame.shape[1]), 1)[None, :])


def paint_frame(r, frame, share=1.0):
    """The frame in GREYS levels from bg towards muted, stretched to the raster (the camera_info aspect); over a link
    budget in blocks of link_block() px; ``share`` of its height from the top (the rest is left clear)."""
    bg, mid = np.asarray(pixels.token('bg'), float), np.asarray(pixels.token('muted'), float)
    greys = [r.ink(tuple(int(v) for v in bg + (mid - bg) * (i / (GREYS - 1)) * 0.8)) for i in range(GREYS)]
    block = link_block()
    h, w = -(-r.h // block), -(-r.w // block)
    if block > 1:       # each block takes its mean, so sensor noise does not flip a block's level (a repaint resends it)
        level = (block_mean(frame, h, w, block / r.h, block / r.w).astype(np.uint16) * GREYS // 256).astype(np.uint8)
        level = np.repeat(np.repeat(level, block, axis=0), block, axis=1)[:r.h, :r.w]
    else:
        ys = np.minimum(((np.arange(r.h) + 0.5) * frame.shape[0] / r.h).astype(int), frame.shape[0] - 1)
        xs = np.minimum(((np.arange(r.w) + 0.5) * frame.shape[1] / r.w).astype(int), frame.shape[1] - 1)
        level = (frame[ys][:, xs].astype(np.uint16) * GREYS // 256).astype(np.uint8)
    rows = -(-int(r.h * share) // block) * block if share < 1 else r.h
    r.px[:rows] = np.asarray(greys, np.uint8)[level][:rows]


_STARTED = {}    # drawing name -> pixels.GENERATION it first painted in (an image tile, over a link budget)
JOIN_STEPS = 4   # over a link budget a frame at a new block size comes in this many repaints, top rows first
_JOINED = {}     # drawing name -> (block size of its frame, repaints of it so far)


def paint_camera(model, cfg):
    breaks = np.asarray(cfg['ramp'], float)

    def paint(r, inks):
        if r.cw > TEXT_CELL[0] and frame_shown(model):
            share = 1.0
            if pixels.BUDGET:       # all of a frame's tiles are new at once (a repaint of 1-1.5 x the budget)
                block, steps = _JOINED.get('fusion.camera', (None, 0))
                steps = steps + 1 if block == link_block() else 1
                _JOINED['fusion.camera'] = (link_block(), steps)
                share = min(1.0, steps / JOIN_STEPS)
            paint_frame(r, model['frame'], share)
        if (pixels.BUDGET and model.get('image') and pixels.active() and pixels.images('fusion.camera')
                and _STARTED.get('fusion.camera') != pixels.GENERATION):
            _STARTED['fusion.camera'] = pixels.GENERATION
            return          # the tile's first repaint over a link is empty: its neighbours' first ones are all new tiles too
        scan = model.get('points')
        if scan is not None and len(scan[1]):
            step = link_stride(len(scan[1]), cfg, frame_shown(model))
            uv, rng = scan[1][::step], scan[2][::step]
            size = max(1, r.cw // 4)
            if scan_stale(model, cfg):
                r.squares(uv[:, 0] * r.w - size // 2, uv[:, 1] * r.h - size // 2, size, inks['stale'])
            else:
                bucket = np.searchsorted(breaks, rng, side='right')     # 0: nearer than the first break
                for b in reversed(range(len(RAMP))):                     # far first, so near points draw on top
                    sel = bucket == b
                    if sel.any():
                        r.squares(uv[sel, 0] * r.w - size // 2, uv[sel, 1] * r.h - size // 2, size,
                                  inks[f'ramp{len(RAMP) - 1 - b}'])
        width = max(1, r.cw // 5)
        for det in model['dets']:
            if det['box'] is None:
                continue
            u0, v0, u1, v1 = det['box']
            x0, y0, x1, y1 = (min(u0, u1) * r.w, min(v0, v1) * r.h, max(u0, u1) * r.w - 1, max(v0, v1) * r.h - 1)
            # ray mode dashed in images; text keeps a solid outline (a dash leaves a braille box a few dots)
            ray = det['source'] == 'lidar_ray' and r.cw > TEXT_CELL[0]
            ink, dash = inks[f"box.{class_token(det['cls'])}"], (3 * width if ray else 0)
            for a, b in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
                r.line(*a, *b, ink, width, dash)
    return paint


def camera_rows(model, cfg, cols, rows):
    model = camera_model(model)
    labels = {}
    for det in sorted((d for d in model['dets'] if d['box'] is not None), key=lambda d: d['range'] or 1e9):
        u0, v0, u1, v1 = det['box']
        style = theme.PALETTE.style(class_token(det['cls']))
        text, col = range_label(det), int(min(u0, u1) * cols)
        # over the box, else under it; a label that would touch another drops the stereo range, then itself
        for row, label in [(r, t) for t in (text, text.split('/')[0]) for r in
                           (int(min(v0, v1) * rows) - 1, int(max(v0, v1) * rows) + 1)]:
            cells = label_cells(label + ' ', col, row, cols, rows, style)
            if cells and not any((c + dc, rr) in labels for c, rr in cells for dc in (-1, 0)):
                labels.update(label_cells(label, col, row, cols, rows, style))
                break
    layers = camera_layers(model['dets'])
    hz = draw_rate(cfg, 'fusion.camera')
    block = link_block()
    return draw('fusion.camera', paint_camera(model, cfg), layers, cols, rows,
                (model['version'], frame_shown(model), scan_stale(model, cfg), block), labels, hz)


# --- Around (top-down) ----------------------------------------------------------------------------

def reach(model, rings):
    """The outer ring drawn: the first ring past every box (and 20 m without boxes)."""
    far = max((d['range'] for d in model['dets'] if d['range']), default=0.0)
    return next((r for r in rings if r >= far * 1.1), rings[-1]) if far else min(rings[-1], 20.0)


FAN_MARGIN = math.radians(10)    # rings drawn this far past the camera's half FOV


def half_fov(model):
    K = model.get('K')
    return math.atan2(K[2], K[0]) if K else math.radians(55)


def fan(w, h, span, half):
    """``at(x, z)`` -> raster (px, py) of a top-down point, the boat at the bottom centre; the rings' arc
    (half FOV plus FAN_MARGIN) fills the width or the height, whichever binds."""
    arc = min(math.pi / 2, half + FAN_MARGIN)
    scale = min((h - 2) / span, w / 2 * 0.98 / (span * math.sin(arc)))
    cx, base = w / 2, h - 1 - max(0.0, (h - 2 - span * scale) / 2)

    def at(x, z):
        return cx + x * scale, base - z * scale
    return at, scale, arc


def paint_around(model, cfg, span):
    def paint(r, inks):
        half = half_fov(model)
        at, scale, arc = fan(r.w, r.h, span, half)
        cx, base = at(0, 0)
        for sign in (-1, 1):
            r.line(cx, base, *at(sign * span * math.sin(half), span * math.cos(half)), inks['fov'], 1, 2)
        for ring in [x for x in cfg['rings'] if x <= span]:
            a = np.linspace(-arc, arc, max(16, int(ring * scale * 3 * arc)))
            r.squares(cx + ring * scale * np.sin(a), base - ring * scale * np.cos(a), 1, inks['ring'])
        scan = model.get('points')
        if scan is not None and len(scan[3]):
            xz = scan[3][::max(1, len(scan[3]) // 1500, link_stride(len(scan[3]), cfg))]
            xz = xz[np.hypot(xz[:, 0], xz[:, 1]) <= span]
            r.squares(cx + xz[:, 0] * scale, base - xz[:, 1] * scale, 1, inks['point'])
        dot = max(2, r.cw // 2)
        for det in model['dets']:
            if det['range'] is None:
                continue
            x, z = optical_xz(det, det['pos'])
            px, pz = at(x, z)
            ink = inks[f"det.{class_token(det['cls'])}"]
            if det['stereo_pos'] is not None and det['source'] not in ('stereo', 'pass'):
                sx, sz = at(*optical_xz(det, det['stereo_pos']))
                r.line(px, pz, sx, sz, inks['stereo'])
                r.squares([sx - 1], [sz - 1], max(1, dot // 2), inks['stereo'])
            if det['sigma']:
                ux, uz = x / det['range'], z / det['range']
                k = 2 * det['sigma']
                r.line(*at(x - ux * k, z - uz * k), *at(x + ux * k, z + uz * k), inks['bar'], max(1, dot // 3))
            if det['source'] == 'lidar_ray':
                h = dot
                r.polygon([(px, pz - h), (px + h, pz), (px, pz + h), (px - h, pz)], ink)
            else:
                r.rect(px - dot / 2, pz - dot / 2, px + dot / 2 + 1, pz + dot / 2 + 1, ink)
        b = max(2, r.cw // 2)
        r.polygon([(cx, base - b), (cx + b / 2, base), (cx - b / 2, base)], inks['boat'])
    return paint


def around_labels(model, span, cols, rows, cell):
    """Hole labels: the outer ring's range at the top right, each box's ``7.0+-0.2`` beside its mark."""
    labels = label_cells(f'{span:g}m', cols - len(f'{span:g}m'), 0, cols, rows, theme.MUTED)
    at, _, _ = fan(cols * cell[0], rows * cell[1], span, half_fov(model))
    for det in sorted((d for d in model['dets'] if d['range']), key=lambda d: d['range']):
        px, pz = at(*optical_xz(det, det['pos']))
        text = f"{det['range']:.1f}" + (f"{PM}{2 * det['sigma']:.1f}" if det['sigma'] else '')
        col, row = int(px / cell[0]) + 2, int(pz / cell[1])
        if theme.ASCII:     # the mark spans to the cell of px + 1; one blank cell after it, as braille leaves
            col, text = int((px + 1) / cell[0]) + 1, ' ' + text
        if col + len(text) > cols:
            col = int(px / cell[0]) - 1 - len(text)
        cells = label_cells(text, col, row, cols, rows, theme.PALETTE.style(class_token(det['cls'])))
        if cells and not any((c + dc, rr) in labels for c, rr in cells for dc in (-1, 0, 1)):
            labels.update(cells)
    return labels


def around_rows(model, cfg, cols, rows):
    span = reach(model, cfg['rings'])
    tokens = sorted({class_token(d['cls']) for d in model['dets']})
    layers = ([('ring', 'grid'), ('fov', 'fov'), ('point', 'muted'), ('stereo', 'muted'), ('bar', 'slate')]
              + [(f'det.{t}', t) for t in tokens] + [('boat', 'boat')])
    cell = pixels.CELL if pixels.active() and pixels.images('fusion.around') else TEXT_CELL
    return draw('fusion.around', paint_around(model, cfg, span), layers, cols, rows,
                (model['version'], span), around_labels(model, span, cols, rows, cell), draw_rate(cfg, 'fusion.around', 0.5))


# --- Alignment ------------------------------------------------------------------------------------

def median(values):
    values = sorted(v for v in values if v is not None and math.isfinite(v))
    return values[len(values) // 2] if values else None


def check(model, cfg):
    """``(level, reason)``: fusion_calib's check rule over the live samples; level None while collecting.

    Hits and L/S come from the feed's ``grades``: the near boxes lidar_fusion samples (stereo < near, a sample
    class, clear of the edge, isolated) of the frames the fusion processed, pooled per object over the residual
    window; an object fails only if it never got a hit. With no graded box at all the check WARNs (``near 0``)."""
    c = cfg['check']
    res = model['residuals']
    if len(res) < c['min_samples']:
        return None, f'collecting {len(res)}/{c["min_samples"]} samples'
    du, dv = median([r[1] for r in res]), median([r[2] for r in res])
    rank = ['PASS', 'WARN', 'FAIL']
    level = 'PASS'

    def worse(lv):
        nonlocal level
        level = max(level, lv, key=rank.index)
    for name, off in (('du', du), ('dv', dv)):     # dv: vertical (pitch) offset, same limits
        if abs(off) > c[name + '_warn_px']:
            worse('FAIL')
        elif abs(off) > c[name + '_pass_px']:
            worse('WARN')
    why = [f'du {du:+.1f} dv {dv:+.1f} px']
    objects = {}                  # backend id -> ([hit], [L/S]) over the window, as fusion_calib's per-object medians
    for _, key, hit, ratio in model.get('grades', ()):
        hits, ratios = objects.setdefault(key, ([], []))
        hits.append(hit)
        if ratio is not None:
            ratios.append(ratio)
    if not objects:               # no near box paired with its stereo array: du/dv alone is not fusion_calib's check
        why.append('near 0')
        worse('WARN')
    else:
        hit = [o for o in objects.values() if any(o[0])]
        why.append(f'near {len(hit)}/{len(objects)}')
        if len(hit) < len(objects):
            worse('FAIL')         # an object that never got a hit
        elif any(sum(h) / len(h) < c['hit_rate_min'] for h, _ in hit):
            worse('WARN')
    ratio = max((median(r) for _, r in objects.values() if r), key=lambda v: abs(v - 1), default=None)
    if ratio is not None:
        why.append(f'L/S {ratio:.2f}')
        if abs(ratio - 1) > c['range_fail_tol']:
            worse('FAIL')
        elif abs(ratio - 1) > c['range_tol']:
            worse('WARN')
    return level, theme.SEPARATOR.join(why)


def spark(values, scale):
    bars = theme.SPARK
    values = values[-SPARK_COLUMNS:]
    if not values:
        return theme.MIDDOT * SPARK_COLUMNS
    return ''.join(bars[min(len(bars) - 1, int(abs(v) / scale * (len(bars) - 1)))] for v in values).rjust(SPARK_COLUMNS)


def trend(res, index, buckets=SPARK_COLUMNS):
    """Median |residual| per time bucket over the window, oldest first."""
    if not res:
        return []
    t0 = min(r[0] for r in res)
    step = (0 - t0) / buckets or 1.0
    groups = [[] for _ in range(buckets)]
    for r in res:
        groups[min(buckets - 1, int((r[0] - t0) / step))].append(r[index])
    return [median(g) for g in groups if g]


def readout(model, cfg, status, width=None):
    """Alignment's lines; at ``width`` the check drops ``live`` then its du/dv (the rows below show them), and the
    correction drops its axis letters, then puts the estimate on its own line. Memoised on the data it reads (the
    window's ends, not the version, which every scan bumps) and recomputed each second for the trend's buckets."""
    res, grades = model['residuals'], model.get('grades') or [()]
    key = (len(res), res[-1][1:] if res else None, res[0][1:] if res else None, len(grades), grades[-1][1:],
           grades[0][1:], model.get('applied'), width, tuple(sorted(status.items())), theme.ASCII,
           palette.detect_tier(), int(pixels.clock()))
    if _READOUT and _READOUT[0] == key:
        return _READOUT[1]
    lines = _readout(model, cfg, status, width)
    _READOUT[:] = [key, lines]
    return lines


_READOUT = []


def _readout(model, cfg, status, width):
    width = width or 1 << 30
    level, why = check(model, cfg)
    style = {'PASS': theme.GOOD, 'WARN': theme.WARN, 'FAIL': theme.BAD}.get(level, theme.MUTED)
    head = Text.assemble(('check ', theme.MUTED), (level or theme.MIDDOT, f'bold {style}'.strip()), ' ')
    short = why.split(theme.SEPARATOR, 1)[1] if level and theme.SEPARATOR in why else why
    for tail in (f'{why}{theme.SEPARATOR}live', why, short):
        if len(head) + len(tail) <= width:
            break
    lines = [Text.assemble(head, (tail, theme.MUTED))]
    applied, est = status.get('applied_deg'), status.get('extrinsic_deg')
    stale = not applied and any(model.get('applied') or ())     # the feed keeps the last one for the points
    applied = applied or (model.get('applied') if stale else None)
    if applied:
        est_text = f'est {" ".join(f"{v:+.2f}" for v in est)}' if est else ''
        for names in (('ox', 'oy', 'oz'), ('', '', '')):      # optical axes: +oy is down, so +oy turns starboard
            corr = Text('corr  ', style=theme.MUTED)
            corr.append(' '.join(f'{n} {v:+.2f}'.strip() for n, v in zip(names, applied)) + DEG)
            if stale:
                corr.append(' stale', style=theme.WARN)
            if len(corr) + len(theme.SEPARATOR) + len(est_text) <= width:
                break
        if est_text and len(corr) + len(theme.SEPARATOR) + len(est_text) <= width:
            corr.append(f'{theme.SEPARATOR}{est_text}', style=theme.MUTED)
            lines.append(corr)
        else:
            lines += [corr, Text(f'      {est_text}', style=theme.MUTED)] if est_text else [corr]
    else:
        lines.append(Text('corr  none applied', style=theme.MUTED))
    res = model['residuals']
    scale = cfg['check']['du_warn_px']
    for name, index in (('du', 1), ('dv', 2)):
        values = trend(res, index)
        mid = median([r[index] for r in res])
        hot = mid is not None and abs(mid) > cfg['check'][name + '_pass_px']
        lines.append(Text.assemble((f'{name}    ', theme.MUTED), (spark(values, scale), theme.WARN if hot else theme.FG),
                                   f' {mid:+5.1f} px' if mid is not None else f' {theme.MIDDOT}',
                                   (' raw' if mid is not None else '', theme.MUTED)))   # before the applied corr
    counts = {}
    for det in model['dets']:
        counts[det['source']] = counts.get(det['source'], 0) + 1
    lines.append(Text.assemble(('boxes ', theme.MUTED), theme.SEPARATOR.join(
        f'{"ray" if s == "lidar_ray" else s} {n}' for s, n in sorted(counts.items())) or theme.MIDDOT))
    if not model['fused']:
        lines.append(Text('fusion off: stereo ranges only', style=theme.MUTED))
    else:
        state = status.get('state') or 'stale'
        hue = {'ok': theme.GOOD, 'warn': theme.WARN, 'error': theme.BAD}.get(state, theme.MUTED)
        parts = [f"{status['hz']:.1f} Hz"] if status.get('hz') is not None else []
        parts += [f"lidar {status['share']:.0%}"] if status.get('share') is not None else []
        lines.append(Text.assemble(('fusion ', theme.MUTED), (state, hue), (theme.SEPARATOR, theme.MUTED),
                                   theme.SEPARATOR.join(parts)))
    for line in lines:
        line.no_wrap, line.overflow = True, theme.OVERFLOW
    return lines


def fusion_status(facts):
    """The applied/estimated correction, state, rate and lidar share from ``perc.<src>.fusion_*`` facts."""
    out = {}
    for name, fact in (facts or {}).items():
        if not name.startswith('perc.'):
            continue
        key = name.rsplit('.', 1)[-1]
        if key == 'fusion_state':
            out['state'] = fact.value
            for field in ('applied_deg', 'extrinsic_deg'):
                try:
                    out[field] = tuple(float(v) for v in str(fact.data.get(field, '')).split())[:3] or None
                except ValueError:
                    pass
        elif key == 'fusion_hz':
            out['hz'] = fact.value
        elif key == 'fusion_share':
            out['share'] = fact.value
    return out


# --- the tab --------------------------------------------------------------------------------------

def camera_size(model, max_cols, max_rows):
    """(cols, rows) of the camera drawing that keeps the image aspect in a terminal cell's shape."""
    w, h = model.get('size') or (16, 9)
    cw, ch = pixels.CELL if pixels.active() else (1, 2)
    rows = max(3, max_rows)
    cols = round(rows * ch * w / (h * cw))
    if cols > max_cols:
        cols, rows = max_cols, max(3, min(rows, round(max_cols * h * cw / (w * ch))))
    return max(8, cols), rows


_CFG = []


def config():
    """fusion_panel.yaml, loaded once."""
    if not _CFG:
        from mhseals_testing.fusion import load_config
        _CFG.append(load_config())
    return _CFG[0]


def camera_subtitle(model, cfg):
    """The range legend (in the points' own styles), the point count, the scan's offset from the boxes and ``no frame``
    in an image tier without one. A stale scan leads with ``TURNING``/``STALE`` and its offset (the title row cuts
    the tail at 80 columns; the word carries it without colour)."""
    model = camera_model(model)
    dt, stale = model.get('scan_dt'), scan_stale(model, cfg)
    offset = f'scan {dt:+.2f}s' if dt is not None else ''
    sub = Text()
    if stale:
        sub.append('TURNING' if model.get('motion') else 'STALE', style=f'bold {ink_style(STALE)}'.strip())
        sub.append(f' {offset}{theme.SEPARATOR}' if offset else theme.SEPARATOR, style=theme.MUTED)
    note = image_note(model, cfg)
    if note:
        sub.append(note[0], style=note[1])
        sub.append(theme.SEPARATOR, style=theme.MUTED)
    ramp = Text()
    for i, token in enumerate(RAMP[:-1]):
        ramp.append(theme.pick('⣿', '#'), style=ink_style(token))
        ramp.append(f"{cfg['ramp'][i]:g} ", style=theme.MUTED)
    ramp.append('m', style=theme.MUTED)
    sub.append_text(Text(' ').join([ramp, Text(f"{len(model['points'][1]) if model.get('points') else 0} pts",
                                               style=theme.MUTED)]))
    if offset and not stale:
        sub.append(f'{theme.SEPARATOR}{offset}', style=theme.MUTED)
    return sub


def image_note(model, cfg):
    """``(text, style)`` of the image toggle's state for the Camera title, or None while the frame is drawn in time
    with its boxes: off, needs graphics (the text tiers), no frame yet, loading (over a link), another aspect than camera_info, or the
    frame's boxes ``frame +0.40s`` from it (no array of its own stamp)."""
    if not model.get('image'):
        return 'image off', theme.MUTED
    if not (pixels.active() and pixels.images('fusion.camera')):
        return 'image needs graphics', theme.MUTED
    if model.get('frame') is None:
        return 'no frame', theme.MUTED
    if model.get('frame_aspect') is False:
        return 'frame aspect != camera_info', theme.WARN
    if pixels.BUDGET:       # over a link the frame starts after an empty repaint and comes in by rows
        block, steps = _JOINED.get('fusion.camera', (None, 0))
        if _STARTED.get('fusion.camera') != pixels.GENERATION or block != link_block() or steps < JOIN_STEPS:
            return 'image loading', theme.MUTED
    dt = model.get('frame_dt')
    if dt is not None and abs(dt) > STAMP_MATCH_S:
        return f'frame {dt:+.2f}s', theme.WARN if abs(dt) > cfg['feed']['scan_dt_max'] else theme.MUTED
    return None


def body(view, width, height, cfg=None):
    """The tab's body; ``view['fusion']`` is the feed snapshot (None: no ROS node in this session)."""
    cfg = cfg or config()
    model = view.get('fusion')
    focused = view.get('focus') == 'fusion'
    if model is None:
        why = ('Fusion is live only (not in review)' if view.get('fusion_review')
               else 'Fusion needs a ROS node (none in this session)')
        return theme.block(Text(why, style=theme.MUTED), 'Camera', focused)
    status = fusion_status(view.get('fusion_facts'))
    readout_rows = READOUT_ROWS
    side = max(AROUND_MIN, int(width * AROUND_SHARE))
    cam_cols, cam_rows = camera_size(model, width - side - 8, max(3, height - readout_rows - 2))
    around = (width - cam_cols - 8, height - 2)
    sub = camera_subtitle(model, cfg)
    camera = theme.block(Text('\n').join(camera_rows(model, cfg, cam_cols, cam_rows)), 'Camera', focused, sub)
    around_panel = theme.block(Text('\n').join(around_rows(model, cfg, *around)), 'Around', False)
    align = theme.block(Group(*readout(model, cfg, status, cam_cols)), 'Alignment', False)
    left = Layout(size=cam_cols + 4)
    left.split_column(Layout(camera, size=cam_rows + 2), Layout(align))
    page = Layout()
    page.split_row(left, Layout(around_panel))
    return page

"""Map layers drawn onto a `Canvas`: trail, costmap tint, lidar hits and plans (map.md 4.2).

Pure and ROS-free. Lines are walked in dot space with one global step counter, so a
"every Nth dot" pattern holds when the source points are closer than a dot. Colour is
symbolic: a dot's layer is ``trail.<mode>.<band>`` and a tint is ``<token>@<percent>``;
the emitter maps them to the palette and to the colourless tiers.
"""

import math
from collections import namedtuple

import numpy as np

from mhseals_testing.ui.minimap import view
from mhseals_testing.ui.minimap.canvas import BITS

# Every Nth dot is drawn per GPS quality; None is no fix (a gap mark, no dots).
GPS_PERIOD = {'fix': 1, 'float': 2, 'dgps': 3, '3d': 4, 'none': None}
# GPSRAW fix_type (what MapState and the trail file keep) to a GPS_PERIOD band; others are 'none'.
FIX_BANDS = {6: 'fix', 5: 'float', 4: 'dgps', 3: '3d'}
TRACK_BANDS = ((0.25, 'ok', 1), (0.6, 'mid', 2))  # (cross-track limit m, band, period)
TRACK_FAR = ('bad', 3)
SPEED_BANDS = 5  # spd.0 .. spd.4 up to vmax, spd.5 above it
PLAN_PERIOD, LOCAL_PLAN_PERIOD = 2, 3
DETAIL_HALF = 10.0  # m; local plan and the full costmap ramp only at or below this half-width
COST_MIN, INSCRIBED, LETHAL = 50, 99, 100
HITS_RANGE, HITS_NEAR = 20.0, 3.0  # m from the boat

# xte is the cross-track error against the goal's first plan, None when unknown.
# ``t`` (s, the pose clock) ages the crumb into the mid and old bands.
Crumb = namedtuple('Crumb', 'x y fix xte speed epoch t', defaults=(0, None, 0.0, 0, None))
AGE_BANDS = ((30.0, None), (120.0, 'mid'))   # fresh up to 30 s keeps its mode band, then mid, then old


class Frame:
    """World to canvas-dot mapping for a view: centre, half-width and optional heading-up yaw."""

    def __init__(self, canvas, centre, half, yaw=None):
        self.canvas, self.centre, self.yaw = canvas, centre, yaw
        self.metres_per_dot = 2 * half / max(canvas.dots_wide, 1)
        self.half = half

    def dots(self, point):
        """Float (x, y) in dots of a world point; y grows downward."""
        dx, dy = view.rotate(point, self.centre, self.yaw)
        return (self.canvas.dots_wide / 2 + dx / self.metres_per_dot,
                self.canvas.dots_high / 2 - dy / self.metres_per_dot)


def _steps(points):
    """``(segment, xs, ys)`` arrays: one step per dot along the polyline, one dot apart.

    ``segment`` is the index of the later point of the segment holding the step; a step that
    lands on a vertex belongs to the segment ending there. Numpy, so a long trail costs one pass.
    """
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(p) < 2:
        return np.zeros(0, dtype=int), np.zeros(0), np.zeros(0)
    seg = np.hypot(*np.diff(p, axis=0).T)
    d = np.concatenate(([0.0], np.cumsum(seg)))
    s = np.arange(0.0, math.floor(d[-1]) + 1) if d[-1] > 0 else np.zeros(0)
    i = np.clip(np.searchsorted(d, s, side='left'), 1, len(p) - 1)
    f = (s - d[i - 1]) / np.where(seg[i - 1] > 0, seg[i - 1], 1.0)
    xy = p[i - 1] + f[:, None] * (p[i] - p[i - 1])
    return i, xy[:, 0], xy[:, 1]


def _draw_line(frame, points, layer, period):
    pts = [frame.dots(p) for p in points if math.isfinite(p[0]) and math.isfinite(p[1])]
    _, xs, ys = _steps(pts)
    for x, y in zip(np.floor(xs[::period]).astype(int).tolist(), np.floor(ys[::period]).astype(int).tolist()):
        frame.canvas.dot(x, y, layer)


def draw_plan(frame, points, local=False):
    """`/plan` every 2nd dot, `/local_plan` every 3rd and only at or below the detail zoom."""
    if local and frame.half > DETAIL_HALF:
        return
    _draw_line(frame, points, 'local_plan' if local else 'plan',
               LOCAL_PLAN_PERIOD if local else PLAN_PERIOD)


def _quality(crumb, mode, vmax):
    """(band, period) of a crumb in a trail mode; a None period is a gap."""
    if mode == 'gps':
        band = crumb.fix if isinstance(crumb.fix, str) else FIX_BANDS.get(crumb.fix, 'none')
        return band, GPS_PERIOD.get(band)
    if mode == 'track' and crumb.xte is not None:
        for limit, band, period in TRACK_BANDS:
            if crumb.xte < limit:
                return band, period
        return TRACK_FAR
    if mode == 'speed':
        if not vmax or not math.isfinite(vmax) or crumb.speed is None or math.isnan(crumb.speed):
            return 'spd.0', 1
        band = SPEED_BANDS if crumb.speed > vmax else min(int(crumb.speed / vmax * SPEED_BANDS), SPEED_BANDS - 1)
        return 'spd.%d' % band, 1
    return 'plain', 1


def draw_trail(frame, crumbs, mode='plain', epoch=None, vmax=0.0, colourless=False, now=None):
    """Draw the trail; ``mode`` is plain, gps, speed or track.

    Crumbs of an epoch other than ``epoch`` (the current map datum) are ``trail.old``, and with
    ``now`` crumbs fade by age: ``trail.mid`` after 30 s, ``trail.old`` after 120 s.
    Segments never join two epochs. Colourless tiers dash the over-vmax speed parts.
    Projection, ages and stepping are numpy; Python touches only the painted dots.
    """
    if not crumbs:
        return
    columns = list(zip(*crumbs))   # x y fix xte speed epoch t; a None t becomes nan
    a = np.array((columns[0], columns[1], columns[5], columns[6]), dtype=float).T
    ok = np.isfinite(a[:, 0]) & np.isfinite(a[:, 1])
    if not ok.all():
        a, crumbs = a[ok], [c for c, keep in zip(crumbs, ok.tolist()) if keep]
        if not crumbs:
            return
    x, y = frame.dots((a[:, 0], a[:, 1]))
    age = now - a[:, 3] if now is not None else np.zeros(len(a))
    with np.errstate(invalid='ignore'):     # an unstamped crumb (nan) stays fresh
        fade = np.where(age > AGE_BANDS[1][0], 2, np.where(age > AGE_BANDS[0][0], 1, 0))
    if epoch is not None:
        fade[a[:, 2] != epoch] = 2
    if mode == 'plain':
        period, fresh = np.ones(len(crumbs), dtype=int), 'trail.plain.plain'
    else:
        looks = [_quality(c, mode, vmax) for c in crumbs]
        period = np.array([2 if mode == 'speed' and colourless and band == 'spd.5' else p or 0 for band, p in looks])
        fresh = np.array(['trail.%s.%s' % (mode, band) for band, _ in looks])
    layer = np.where(fade == 2, 'trail.old', np.where(fade == 1, 'trail.mid', fresh))
    breaks = (np.flatnonzero(np.diff(a[:, 2])) + 1).tolist()
    n, canvas = 0, frame.canvas  # one global step counter keeps patterns continuous across runs
    for lo, hi in zip([0, *breaks], [*breaks, len(a)]):
        seg, sx, sy = _steps(np.stack([x[lo:hi], y[lo:hi]], axis=1))
        k = seg + lo
        step = period[k]
        gap = step == 0
        keep = ~gap & ((n + np.arange(len(k))) % np.maximum(step, 1) == 0)
        n += len(k)
        cx, cy = np.floor(sx).astype(int), np.floor(sy).astype(int)
        for col, row in zip((cx[gap] // 2).tolist(), (cy[gap] // 4).tolist()):
            canvas.gap(col, row)
        canvas.dots(cx[keep].tolist(), cy[keep].tolist(), layer[k[keep]].tolist())


TINTS = ((66, 'cost.1'), (83, 'cost.2'), (INSCRIBED, 'cost.3'))  # cost below the bound -> ramp step
DENSITY = {'lethal': 0xFF, 'inscribed': 0x55, 'cost.3': 0x02}  # colourless tiers: braille bits per band


def _tint_for(cost, far):
    """Costmap tint token for a cost, or None when it is not drawn: the 5-stop ramp binned at
    50/66/83/99/100 (tui-visual 2.3); the far zoom keeps only inscribed and lethal."""
    if cost >= LETHAL:
        return 'lethal'
    if cost == INSCRIBED:
        return 'inscribed'
    if cost < COST_MIN or far:
        return None
    return next(token for bound, token in TINTS if cost < bound)


def _cells(grid, resolution, origin, floor=COST_MIN):
    """World centres (xs, ys) and costs of the costmap cells at or above ``floor``, as arrays."""
    g = np.asarray(grid)
    rows, cols = np.nonzero(g >= floor)
    return (origin[0] + (cols + 0.5) * resolution, origin[1] + (rows + 0.5) * resolution,
            g[rows, cols].astype(int))


def draw_costmap(frame, grid, resolution, origin, density=False):
    """Tint canvas cells from a costmap (rows grow north from ``origin``), max over the cells.

    A costmap cell covers every canvas cell its footprint touches, so a lethal cell
    survives pooling whichever of the two is coarser. Each canvas cell keeps its highest cost.
    Pooling is a numpy max into a canvas-sized array; Python only walks the tinted cells.
    ``density`` (16-colour and NO_COLOR tiers, which have no tint) draws bands as braille dots.
    """
    far = frame.half > DETAIL_HALF
    reach = resolution / frame.metres_per_dot / 2  # footprint half-size in dots
    xs, ys, costs = _cells(grid, resolution, origin)
    x, y = frame.dots((xs, ys))
    c0, c1 = np.floor((x - reach) / 2).astype(int), np.floor((x + reach) / 2).astype(int)
    r0, r1 = np.floor((y - reach) / 4).astype(int), np.floor((y + reach) / 4).astype(int)
    canvas = frame.canvas
    pooled = np.zeros((canvas.cols, canvas.rows), dtype=int)
    if pooled.size and len(costs):
        for dc in range(int((c1 - c0).max()) + 1):
            for dr in range(int((r1 - r0).max()) + 1):
                c, r = c0 + dc, r0 + dr
                ok = (c <= c1) & (r <= r1) & (c >= 0) & (c < canvas.cols) & (r >= 0) & (r < canvas.rows)
                np.maximum.at(pooled, (c[ok], r[ok]), costs[ok])
    for col, row in zip(*np.nonzero(pooled)):
        token = _tint_for(int(pooled[col, row]), far)
        if token and not density:
            canvas.tint(int(col), int(row), token)
        elif token in DENSITY:
            for (dx, dy), bit in BITS.items():
                if DENSITY[token] & bit:
                    canvas.dot(int(col) * 2 + dx, int(row) * 4 + dy, 'costmap')


def draw_hits(frame, grid, resolution, origin, boat):
    """Lethal costmap cells within 20 m of the boat as single dots; `hits.near` inside 3 m."""
    xs, ys, _ = _cells(grid, resolution, origin, LETHAL)
    d = np.hypot(xs - boat[0], ys - boat[1])
    keep = d <= HITS_RANGE
    x, y = frame.dots((xs[keep], ys[keep]))
    # far sorts after near, so a dot shared by both keeps the near warning (first layer wins).
    dots = np.unique(np.stack([np.floor(x), np.floor(y), d[keep] > HITS_NEAR]).T, axis=0)
    for px, py, far in dots:
        frame.canvas.dot(int(px), int(py), 'hits' if far else 'hits.near')

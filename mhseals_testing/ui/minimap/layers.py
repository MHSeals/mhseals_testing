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
Crumb = namedtuple('Crumb', 'x y fix xte speed epoch', defaults=(0, None, 0.0, 0))


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
    """One (segment, x, y) per dot step along the polyline, one dot apart, counted globally.

    ``segment`` is the index of the later point of the segment holding the step.
    """
    nxt, done = 0.0, 0.0
    for i in range(1, len(points)):
        (x0, y0), (x1, y1) = points[i - 1], points[i]
        length = math.hypot(x1 - x0, y1 - y0)
        while nxt <= done + length and length > 0:
            f = (nxt - done) / length
            yield i, x0 + f * (x1 - x0), y0 + f * (y1 - y0)
            nxt += 1.0
        done += length


def _plot(canvas, x, y, layer):
    canvas.dot(math.floor(x), math.floor(y), layer)


def _draw_line(frame, points, layer, period):
    pts = [frame.dots(p) for p in points if math.isfinite(p[0]) and math.isfinite(p[1])]
    for n, (_, x, y) in enumerate(_steps(pts)):
        if n % period == 0:
            _plot(frame.canvas, x, y, layer)


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


def draw_trail(frame, crumbs, mode='plain', epoch=None, vmax=0.0, colourless=False):
    """Draw the trail; ``mode`` is plain, gps, speed or track.

    Crumbs of an epoch other than ``epoch`` (the current map datum) are ``trail.old``.
    Segments never join two epochs. Colourless tiers dash the over-vmax speed parts.
    """
    crumbs = [c for c in crumbs if math.isfinite(c.x) and math.isfinite(c.y)]
    runs, cur = [], []
    for c in crumbs:
        if cur and c.epoch != cur[-1].epoch:
            runs.append(cur)
            cur = []
        cur.append(c)
    runs.append(cur)
    n = 0  # one global step counter keeps patterns continuous across runs
    for run in runs:
        pts = [frame.dots((c.x, c.y)) for c in run]
        for i, x, y in _steps(pts):
            band, period = _quality(run[i], mode, vmax)
            if mode == 'speed' and colourless and band == 'spd.5':
                period = 2
            if period is None:
                frame.canvas.gap(math.floor(x) // 2, math.floor(y) // 4)
            elif n % period == 0:
                tail = 'old' if epoch is not None and run[i].epoch != epoch else mode + '.' + band
                _plot(frame.canvas, x, y, 'trail.' + tail)
            n += 1


def _tint_for(cost, far):
    """Costmap tint token for a cost, or None when it is not drawn (map.md 4.2)."""
    if cost >= LETHAL:
        return 'lethal_bg@%d' % (75 if far else 100)
    if cost == INSCRIBED:
        return 'lethal_bg@%d' % (45 if far else 70)
    if cost < COST_MIN or far:
        return None
    return 'cost_bg@%d' % round(45 + (cost - COST_MIN) / (INSCRIBED - 1 - COST_MIN) * 55)


def _cells(grid, resolution, origin, floor=COST_MIN):
    """World centres (xs, ys) and costs of the costmap cells at or above ``floor``, as arrays."""
    g = np.asarray(grid)
    rows, cols = np.nonzero(g >= floor)
    return (origin[0] + (cols + 0.5) * resolution, origin[1] + (rows + 0.5) * resolution,
            g[rows, cols].astype(int))


def draw_costmap(frame, grid, resolution, origin):
    """Tint canvas cells from a costmap (rows grow north from ``origin``), max over the cells.

    A costmap cell covers every canvas cell its footprint touches, so a lethal cell
    survives pooling whichever of the two is coarser. Each canvas cell keeps its highest cost.
    Pooling is a numpy max into a canvas-sized array; Python only walks the tinted cells.
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
        if token:
            canvas.tint(int(col), int(row), token)


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

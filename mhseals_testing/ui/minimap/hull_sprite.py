"""The hull as an anti-aliased image (theme.yaml ``hull: smooth`` in an image tier); pure numpy, no ROS.

`paint` composites onto a `Raster` (ui/imaging/): the faint wakes first, then the shape's decks in the body
colour mixed DECK_ALPHA into the background, then its hulls (hull_shapes.py) in the body colour. Coverage is
sampled SUPERSAMPLE x each way and box-filtered down, so edges are anti-aliased without a stroke. Each hull trails
the dashboard boat's V: two arms off its transom corners at WAKE_DEG (shared among the hulls) off the reverse
heading, as long as the caller's speed makes them, fading from WAKE_ALPHA at the stern to nothing and tapering
from the caller's line weight to TAPER of it. Every part is cached per heading bucket, size in px and shape, so a
frame costs one small blend.
"""

import math
from functools import lru_cache

import numpy as np

from mhseals_testing.ui.minimap import hull, hull_shapes

SUPERSAMPLE = 4             # samples per px each way (the hull)
WAKE_SUPERSAMPLE = 2        # the wake's: a faint thin line needs fewer
DECK_ALPHA = 0.45           # a deck or crossbeam reads as a lighter bridge between the hulls
WAKE_ALPHA = 0.7            # the wake's opacity at the stern; it fades to 0 at the arm's end
TAPER = 0.35                # the wake's width at its end, as a share of the width at the stern


def _grid(x0, y0, w, h, s=SUPERSAMPLE):
    """Sample centres (x, y) of a ``w`` x ``h`` px box at top-left (x0, y0), ``s`` x each way."""
    xs = x0 + (np.arange(w * s) + 0.5) / s
    ys = y0 + (np.arange(h * s) + 0.5) / s
    return np.meshgrid(xs, ys)


def _down(samples, h, w, s=SUPERSAMPLE):
    """Box-filter supersampled values to px."""
    return samples.reshape(h, s, w, s).mean(axis=(1, 3)).astype(np.float32)


@lru_cache(maxsize=256)
def silhouette(yaw_q, length, beam, form):
    """``(x0, y0, hulls, decks)``: the coverage (0..1 per px) of the shape's hulls and of its decks alone, top-left
    at (x0, y0) px about the centre. ``yaw_q`` is degrees CCW from screen right, ``length`` and ``beam`` px."""
    reach = int(math.ceil(max(length, beam) / 2)) + 1
    xs, ys = _grid(-reach, -reach, 2 * reach, 2 * reach)
    c, s = math.cos(math.radians(yaw_q)), math.sin(math.radians(yaw_q))
    along, across = (xs * c - ys * s) / length, -(xs * s + ys * c) / beam     # body fractions: ahead, to port
    hulls, decks = hull_shapes.coverage(form, hull.GEOMETRY, along, across)
    return -reach, -reach, _down(hulls, 2 * reach, 2 * reach), _down(decks, 2 * reach, 2 * reach)


@lru_cache(maxsize=256)
def wake(yaw_q, length, beam, reach, pen, spread_deg, form):
    """``(x0, y0, alpha)`` of each hull's two wake arms ``reach`` px long off its transom corners, or None."""
    if reach < 1:
        return None
    arms, sterns = [], hull.sterns(yaw_q, (length, beam), 'smooth', form)
    for corners in sterns:
        for side, (sx, sy) in zip((1, -1), corners):     # the hulls share the spread: slim Vs that never cross
            a = math.radians(yaw_q + 180 - side * spread_deg / len(sterns))
            arms.append((sx, sy, sx + reach * math.cos(a), sy - reach * math.sin(a)))
    pad = pen + 1
    x0 = int(math.floor(min(min(a[0], a[2]) for a in arms) - pad))
    y0 = int(math.floor(min(min(a[1], a[3]) for a in arms) - pad))
    w = int(math.ceil(max(max(a[0], a[2]) for a in arms) + pad)) - x0
    h = int(math.ceil(max(max(a[1], a[3]) for a in arms) + pad)) - y0
    xs, ys = _grid(x0, y0, w, h, WAKE_SUPERSAMPLE)
    alpha = np.zeros(xs.shape)
    for ax, ay, bx, by in arms:     # each arm only over its own box of samples
        box = tuple(slice(int(max(0, (min(a, b) - pad - o) * WAKE_SUPERSAMPLE)),
                          int((max(a, b) + pad - o) * WAKE_SUPERSAMPLE) + 1)
                    for a, b, o in ((ay, by, y0), (ax, bx, x0)))
        px, py = xs[box], ys[box]
        dx, dy = bx - ax, by - ay
        t = np.clip(((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy), 0.0, 1.0)
        far = np.hypot(px - ax - t * dx, py - ay - t * dy)
        half = pen * (1 - (1 - TAPER) * t) / 2
        alpha[box] = np.maximum(alpha[box], (far <= half) * WAKE_ALPHA * (1 - t))
    return x0, y0, _down(alpha, h, w, WAKE_SUPERSAMPLE)


def paint(raster, x, y, yaw, length, beam, inks, bg, form='monohull', wake_px=0.0, pen=2, spread_deg=18.0,
          step=3.0):
    """The hull of shape ``form`` centred on px (x, y) at ``yaw`` (deg CCW from screen right, bucketed to
    ``step``), ``length`` x ``beam`` px, with ``inks`` (body, wake) as (r, g, b) over ``bg`` where nothing is
    drawn; wakes when ``wake_px`` > 0, their arms ``pen`` px wide at the stern."""
    yaw_q = (round(yaw / step) * step) % 360
    size = (round(length * 2) / 2, round(beam * 2) / 2)
    x, y = int(round(x)), int(round(y))
    body, trail = inks
    held = wake(yaw_q, *size, round(wake_px), pen, spread_deg, form) if wake_px >= 1 else None
    if held is not None:
        raster.blend(x + held[0], y + held[1], held[2], trail, bg)
    x0, y0, cover, decks = silhouette(yaw_q, *size, form)
    if decks.any():         # opaque, so what lies under (the trail) stays hidden as under the hulls
        raster.blend(x + x0, y + y0, decks, tuple(round(g + (c - g) * DECK_ALPHA) for c, g in zip(body, bg)), bg)
    raster.blend(x + x0, y + y0, cover, body, bg)

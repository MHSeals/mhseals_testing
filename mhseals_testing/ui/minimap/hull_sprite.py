"""The smooth hull as an anti-aliased image (theme.yaml ``hull: smooth`` in an image tier); pure numpy, no ROS.

`paint` composites onto a `Raster` (ui/imaging/): the faint wake first, then the hull's silhouette (hull.py's
smooth outline) in the body colour with its foremost ``bow_share`` in the bow colour, the two meeting over
FEATHER px. Coverage is sampled SUPERSAMPLE x each way and box-filtered down, so edges are anti-aliased
without a stroke. The wake is the dashboard boat's V: two arms off the transom corners at WAKE_DEG off the
reverse heading, as long as the caller's speed makes them, fading from WAKE_ALPHA at the stern to nothing and
tapering from the caller's line weight to TAPER of it. Every part is cached per heading bucket and size in px,
so a frame costs one small blend.
"""

import math
from functools import lru_cache

import numpy as np

from mhseals_testing.ui.minimap import hull

SUPERSAMPLE = 4             # samples per px each way (the hull)
WAKE_SUPERSAMPLE = 2        # the wake's: a faint thin line needs fewer
FEATHER = 1.5               # px over which the bow colour fades into the body's
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


# the smooth outline's port side by length fraction, aft to bow: its half-beam at any station
_SIDE = sorted(p for p in hull.SMOOTH if p[1] >= 0)
_U, _V = np.array([u for u, _ in _SIDE]), np.array([v for _, v in _SIDE])


@lru_cache(maxsize=256)
def silhouette(yaw_q, length, beam, bow_share):
    """``(x0, y0, body, bow)``: the hull's coverage and its bow's (0..1 per px), top-left at (x0, y0) px about
    the centre. ``yaw_q`` is degrees CCW from screen right, ``length`` and ``beam`` px."""
    reach = int(math.ceil(max(length, beam) / 2)) + 1
    xs, ys = _grid(-reach, -reach, 2 * reach, 2 * reach)
    c, s = math.cos(math.radians(yaw_q)), math.sin(math.radians(yaw_q))
    along, across = xs * c - ys * s, -(xs * s + ys * c)      # body frame: ahead and to port, px
    inside = np.abs(across) <= np.interp(along / length, _U, _V, left=-1.0, right=-1.0) * beam
    ahead = along - length * (0.5 - bow_share)
    bowness = np.clip(ahead / FEATHER + 0.5, 0.0, 1.0) * inside
    return -reach, -reach, _down(inside, 2 * reach, 2 * reach), _down(bowness, 2 * reach, 2 * reach)


@lru_cache(maxsize=256)
def wake(yaw_q, length, beam, reach, pen, spread_deg):
    """``(x0, y0, alpha)`` of the two wake arms ``reach`` px long off the transom corners, or None."""
    if reach < 1:
        return None
    _, port, starboard = hull.points(yaw_q, 0.0, (length, beam), 'smooth')
    arms = []
    for side, (sx, sy) in ((1, port), (-1, starboard)):
        a = math.radians(yaw_q + 180 - side * spread_deg)
        arms.append((sx, sy, sx + reach * math.cos(a), sy - reach * math.sin(a)))
    pad = pen + 1
    x0 = int(math.floor(min(min(a[0], a[2]) for a in arms) - pad))
    y0 = int(math.floor(min(min(a[1], a[3]) for a in arms) - pad))
    w = int(math.ceil(max(max(a[0], a[2]) for a in arms) + pad)) - x0
    h = int(math.ceil(max(max(a[1], a[3]) for a in arms) + pad)) - y0
    xs, ys = _grid(x0, y0, w, h, WAKE_SUPERSAMPLE)
    alpha = np.zeros(xs.shape)
    for ax, ay, bx, by in arms:
        dx, dy = bx - ax, by - ay
        t = np.clip(((xs - ax) * dx + (ys - ay) * dy) / (dx * dx + dy * dy), 0.0, 1.0)
        far = np.hypot(xs - ax - t * dx, ys - ay - t * dy)
        half = pen * (1 - (1 - TAPER) * t) / 2
        alpha = np.maximum(alpha, (far <= half) * WAKE_ALPHA * (1 - t))
    return x0, y0, _down(alpha, h, w, WAKE_SUPERSAMPLE)


def paint(raster, x, y, yaw, length, beam, inks, bg, bow_share, wake_px=0.0, pen=2, spread_deg=18.0, step=3.0):
    """The hull centred on px (x, y) at ``yaw`` (deg CCW from screen right, bucketed to ``step``), ``length`` x
    ``beam`` px, with ``inks`` (body, bow, wake) as (r, g, b) over ``bg`` where nothing is drawn; a wake when
    ``wake_px`` > 0, its arms ``pen`` px wide at the stern."""
    yaw_q = (round(yaw / step) * step) % 360
    size = (round(length * 2) / 2, round(beam * 2) / 2)
    x, y = int(round(x)), int(round(y))
    body, bow, trail = inks
    held = wake(yaw_q, *size, round(wake_px), pen, spread_deg) if wake_px >= 1 else None
    if held is not None:
        raster.blend(x + held[0], y + held[1], held[2], trail, bg)
    x0, y0, cover, front = silhouette(yaw_q, *size, bow_share)
    raster.blend(x + x0, y + y0, cover, body, bg)
    raster.blend(x + x0, y + y0, front, bow, bg)

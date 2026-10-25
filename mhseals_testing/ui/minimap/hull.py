"""The boat glyph (tui-visual 2.3): a filled pentagon hull as braille dot offsets; pure, no ROS.

`mask(yaw, dots_per_m, minimum)` gives the dots of a hull centred on dot (0, 0), dot y down,
each tagged ``boat`` or ``bow`` (the 2-3 foremost). The hull is true scale, never smaller than
``minimum`` (length, beam) dots, and cached per 3 deg of yaw and half-dot of size, so a frame
costs one loop over the cached offsets. The Conditions rose uses the same mask at 9x6 dots.
"""

import math
from functools import lru_cache

LENGTH, BEAM = 1.34, 0.9    # m, the boat's hull
# Body-frame outline as fractions of (length, beam): bow, shoulders, sterns.
HULL = ((0.5, 0.0), (0.2, 0.5), (-0.5, 0.47), (-0.5, -0.47), (0.2, -0.5))
MIN_DOTS = (12, 7)          # length, beam: a readable icon at +-20 m, true size at +-5 m
YAW_STEP = 3.0              # deg of the mask cache


def _size(dots_per_m, minimum):
    """(length, beam) in dots, rounded to half a dot for the cache key."""
    length = max(LENGTH * dots_per_m, minimum[0])
    beam = max(BEAM * dots_per_m, minimum[1])
    return round(length * 2) / 2, round(beam * 2) / 2


def outline(yaw_deg, length, beam):
    """The hull vertices in dots about the centre (x right, y down); ``yaw_deg`` is CCW from right."""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return [((u * length) * c - (v * beam) * s, -((u * length) * s + (v * beam) * c)) for u, v in HULL]


def _inside(x, y, poly):
    hit = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            hit = not hit
    return hit


@lru_cache(maxsize=512)
def _mask(yaw_q, length, beam):
    poly = outline(yaw_q, length, beam)
    reach = int(math.ceil(max(length, beam) / 2)) + 1
    dots = [(dx, dy) for dy in range(-reach, reach + 1) for dx in range(-reach, reach + 1)
            if _inside(dx, dy, poly)]
    c, s = math.cos(math.radians(yaw_q)), math.sin(math.radians(yaw_q))
    ahead = sorted(dots, key=lambda d: -(d[0] * c - d[1] * s))
    bow = set(ahead[:3 if len(dots) > 24 else 2])
    return tuple((dx, dy, 'bow' if (dx, dy) in bow else 'boat') for dx, dy in dots)


def mask(yaw_deg, dots_per_m=0.0, minimum=MIN_DOTS):
    """Dot offsets ``(dx, dy, part)`` of the hull at ``yaw_deg`` (CCW from screen right; 90 is up)."""
    yaw_q = (round(yaw_deg / YAW_STEP) * YAW_STEP) % 360 if math.isfinite(yaw_deg) else 90.0
    return _mask(yaw_q, *_size(dots_per_m if math.isfinite(dots_per_m) else 0.0, minimum))


def points(yaw_deg, dots_per_m=0.0, minimum=MIN_DOTS):
    """``(bow, port_stern, starboard_stern)`` in dots about the centre, for the leader and wake."""
    bow, _, port, starboard, _ = outline(yaw_deg, *_size(dots_per_m, minimum))
    return bow, port, starboard

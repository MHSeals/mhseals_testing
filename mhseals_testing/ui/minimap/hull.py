"""The boat glyph (tui-visual 2.3): the hull as braille dot offsets and as an outline; pure, no ROS.

`mask(yaw, dots_per_m, minimum, style)` gives the dots of a hull centred on dot (0, 0), dot y down,
each tagged ``boat`` or ``bow`` (the 2-3 foremost). The hull is true scale (minimap.yaml ``hull``),
never smaller than ``minimum`` (length, beam) dots, and cached per 3 deg of yaw and half-dot of size,
so a frame costs one loop over the cached offsets. The Conditions rose uses the same mask at 9x6 dots.

Two looks (theme.yaml ``hull``): ``classic`` fills a pentagon; ``smooth`` traces a sleek outline (a
pointed ogive bow, a straight run, a transom with rounded quarters) as a one-dot stroke, 4-connected so
it never doubles up, and `interior` names the dots inside it that the hull clears of what lies under.
The image tier draws the same outline anti-aliased (hull_sprite.py).
"""

import math
from functools import lru_cache

from mhseals_testing.analysis.classes import load_section

SIZE = load_section('hull', {'length_m': 1.34, 'beam_m': 0.9, 'min_dots': [12, 7]})
LENGTH, BEAM = SIZE['length_m'], SIZE['beam_m']   # m, the boat's hull
MIN_DOTS = tuple(SIZE['min_dots'])                 # length, beam: a readable icon at +-20 m, true size at +-5 m
YAW_STEP = 3.0              # deg of the mask cache
BOW_SHARE = 0.25            # the smooth hull's foremost quarter wears the bow hue (the image tier, the rose)
# Body-frame outlines as fractions of (length, beam), bow first, port (+v) side then starboard.
CLASSIC = ((0.5, 0.0), (0.2, 0.5), (-0.5, 0.47), (-0.5, -0.47), (0.2, -0.5))
# smooth: the bow's half-beam is 0.5 * (1 - t ** FULLNESS) from BOW_START (t = 0) to the stem (t = 1), the run
# narrows to RUN_AFT at -0.5 + QUARTER, and each transom quarter is a QUARTER-radius arc
BOW_START, FULLNESS, RUN_AFT, QUARTER, SEGMENTS = -0.15, 1.6, 0.49, 0.1, 16


def _smooth():
    side = [(BOW_START + (0.5 - BOW_START) * t, 0.5 * (1 - t ** FULLNESS))
            for t in (1 - i / SEGMENTS for i in range(SEGMENTS + 1))]
    side += [(-0.5 + QUARTER - QUARTER * math.sin(a), RUN_AFT - QUARTER + QUARTER * math.cos(a))
             for a in (math.pi / 2 * i / 4 for i in range(5))]
    return tuple(side + [(u, -v) for u, v in reversed(side[1:])])


SMOOTH = _smooth()
OUTLINES = {'classic': CLASSIC, 'smooth': SMOOTH}
STERN = {'classic': CLASSIC[2], 'smooth': (-0.5, RUN_AFT - QUARTER)}   # the port transom corner, for the wake


def style():
    """The theme's hull look (theme.yaml ``hull``)."""
    from mhseals_testing.ui import theme
    return theme.PALETTE.hull


def wake_shown():
    """Whether the hull trails a wake while moving (theme.yaml ``wake``)."""
    from mhseals_testing.ui import theme
    return theme.PALETTE.wake


def size(dots_per_m, minimum=MIN_DOTS):
    """(length, beam) in dots at ``dots_per_m``, never under ``minimum``."""
    return _size(dots_per_m if math.isfinite(dots_per_m) else 0.0, minimum)


def _size(dots_per_m, minimum):
    """(length, beam) in dots, rounded to half a dot for the cache key."""
    length = max(LENGTH * dots_per_m, minimum[0])
    beam = max(BEAM * dots_per_m, minimum[1])
    return round(length * 2) / 2, round(beam * 2) / 2


def _turn(yaw_deg, length, beam, shape):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return [((u * length) * c - (v * beam) * s, -((u * length) * s + (v * beam) * c)) for u, v in shape]


def outline(yaw_deg, length, beam, look='classic'):
    """The hull vertices in dots about the centre (x right, y down); ``yaw_deg`` is CCW from right."""
    return _turn(yaw_deg, length, beam, OUTLINES[look])


def _inside(x, y, poly):
    hit = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            hit = not hit
    return hit


@lru_cache(maxsize=512)
def _fill(yaw_q, length, beam, look):
    poly = outline(yaw_q, length, beam, look)
    reach = int(math.ceil(max(length, beam) / 2)) + 1
    return frozenset((dx, dy) for dy in range(-reach, reach + 1) for dx in range(-reach, reach + 1)
                     if _inside(dx, dy, poly))


@lru_cache(maxsize=512)
def _mask(yaw_q, length, beam, look):
    dots = _fill(yaw_q, length, beam, look)
    if look == 'smooth':      # the fill's edge: a dot with a 4-neighbour outside
        dots = {(x, y) for x, y in dots
                if any(n not in dots for n in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)))}
    dots = sorted(dots, key=lambda d: (d[1], d[0]))
    c, s = math.cos(math.radians(yaw_q)), math.sin(math.radians(yaw_q))
    ahead = sorted(dots, key=lambda d: -(d[0] * c - d[1] * s))
    bow = set(ahead[:3 if len(dots) > 24 else 2])
    return tuple((dx, dy, 'bow' if (dx, dy) in bow else 'boat') for dx, dy in dots)


def _key(yaw_deg, dots_per_m, minimum):
    yaw_q = (round(yaw_deg / YAW_STEP) * YAW_STEP) % 360 if math.isfinite(yaw_deg) else 90.0
    return (yaw_q, *_size(dots_per_m if math.isfinite(dots_per_m) else 0.0, minimum))


def mask(yaw_deg, dots_per_m=0.0, minimum=MIN_DOTS, look=None):
    """Dot offsets ``(dx, dy, part)`` of the hull at ``yaw_deg`` (CCW from screen right; 90 is up)."""
    return _mask(*_key(yaw_deg, dots_per_m, minimum), look or style())


@lru_cache(maxsize=512)
def _interior(yaw_q, length, beam, look):
    edge = {(x, y) for x, y, _ in _mask(yaw_q, length, beam, look)}
    return tuple(sorted(_fill(yaw_q, length, beam, look) - edge))


def interior(yaw_deg, dots_per_m=0.0, minimum=MIN_DOTS, look=None):
    """The dots inside an outline hull (none for a filled one): the hull clears them of the trail beneath."""
    return _interior(*_key(yaw_deg, dots_per_m, minimum), look or style())


def points(yaw_deg, dots_per_m=0.0, minimum=MIN_DOTS, look=None):
    """``(bow, port_stern, starboard_stern)`` in dots about the centre, for the leader and wake."""
    (u, v) = STERN[look or style()]
    return tuple(_turn(yaw_deg, *_size(dots_per_m, minimum), ((0.5, 0.0), (u, v), (u, -v))))

"""The boat glyph (tui-visual 2.3): the hull as braille dot offsets and as an outline; pure, no ROS.

`mask(yaw, dots_per_m, minimum, style)` gives the dots of a hull centred on dot (0, 0), dot y down,
each tagged ``boat`` or ``bow`` (the 2-3 foremost: the ASCII tier's arrow cell). The hull is true scale, never
smaller than ``minimum`` (length, beam) dots, and cached per 3 deg of yaw and half-dot of size, so a frame costs
one loop over the cached offsets. The Conditions rose uses the same mask at 9x6 dots.

The shape (hull_shapes.py: monohull, catamaran) and its metres are the vessel profile's ``hull:`` over
minimap.yaml's; each tier draws the vessel's shape when its ``draws`` list has it, else the list's first (`shape`).
Two looks (theme.yaml ``hull``): ``classic`` fills a pentagon (a monohull look, whatever the shape); ``smooth``
traces each part's outline as a one-dot stroke, 4-connected so it never doubles up, and `interior` names the
dots inside it that the hull clears of what lies under. The image tier draws the shape anti-aliased
(hull_sprite.py).
"""

import math
from functools import lru_cache

from mhseals_hardware import vessel
from mhseals_testing.analysis.classes import _section, load_section
from mhseals_testing.ui.minimap import hull_shapes

GEOMETRY_KEYS = ('shape', 'length', 'beam', 'pontoon', 'deck')    # vessel facts; the rest is how the TUI draws
DEFAULTS = {'shape': 'monohull', 'length': 1.34, 'beam': 0.9, 'pontoon': 0.0, 'deck': 0.0,
            'min_dots': [12, 7], 'draws': {'image': ['monohull'], 'dots': ['monohull']}}


def _settings():
    found = vessel.section('hull')
    if not set(found) <= set(GEOMETRY_KEYS):
        extra = sorted(set(found) - set(GEOMETRY_KEYS))
        raise ValueError(f'vessel profile hull: takes {", ".join(GEOMETRY_KEYS)}, not {extra}')
    settings = _section('vessel profile hull', found, load_section('hull', DEFAULTS))
    hull_shapes.parts(settings['shape'], settings)                # an unknown shape refuses here
    for tier, shapes in settings['draws'].items():
        if not shapes or not all(name in hull_shapes.SHAPES for name in shapes):
            raise ValueError(f'hull.draws.{tier}: a list of shapes from {", ".join(hull_shapes.SHAPES)}')
    return settings


SIZE = _settings()
GEOMETRY = {key: SIZE[key] for key in GEOMETRY_KEYS}
LENGTH, BEAM = SIZE['length'], SIZE['beam']        # m, the boat's overall hull
MIN_DOTS = tuple(SIZE['min_dots'])                 # length, beam: a readable icon at +-20 m, true size at +-5 m
YAW_STEP = 3.0              # deg of the mask cache
# Body-frame outlines as fractions of (length, beam), bow first, port (+v) side then starboard.
CLASSIC = ((0.5, 0.0), (0.2, 0.5), (-0.5, 0.47), (-0.5, -0.47), (0.2, -0.5))
SMOOTH = hull_shapes.outline(hull_shapes.MONO_SIDE)


def shape(tier):
    """The shape ``tier`` (``image`` or ``dots``) draws: the vessel's if its draws list has it, else the first."""
    draws = SIZE['draws'].get(tier) or ['monohull']
    return GEOMETRY['shape'] if GEOMETRY['shape'] in draws else draws[0]


def _polygons(look, form):
    """``[(outline, deck)]`` in body fractions: the classic pentagon, else the shape's parts."""
    return [(CLASSIC, False)] if look == 'classic' else hull_shapes.polygons(form, GEOMETRY)


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
    """The monohull vertices in dots about the centre (x right, y down); ``yaw_deg`` is CCW from right."""
    return _turn(yaw_deg, length, beam, CLASSIC if look == 'classic' else SMOOTH)


def _inside(x, y, poly):
    hit = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            hit = not hit
    return hit


@lru_cache(maxsize=512)
def _fill(yaw_q, length, beam, look, form):
    reach = int(math.ceil(max(length, beam) / 2)) + 1
    grid = [(dx, dy) for dy in range(-reach, reach + 1) for dx in range(-reach, reach + 1)]
    dots = set()
    for outline_, _ in _polygons(look, form):
        poly = _turn(yaw_q, length, beam, outline_)
        dots.update((x, y) for x, y in grid if _inside(x, y, poly))
    return frozenset(dots)


@lru_cache(maxsize=512)
def _mask(yaw_q, length, beam, look, form):
    dots = _fill(yaw_q, length, beam, look, form)
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
    return _mask(*_key(yaw_deg, dots_per_m, minimum), look or style(), shape('dots'))


@lru_cache(maxsize=512)
def _interior(yaw_q, length, beam, look, form):
    edge = {(x, y) for x, y, _ in _mask(yaw_q, length, beam, look, form)}
    return tuple(sorted(_fill(yaw_q, length, beam, look, form) - edge))


def interior(yaw_deg, dots_per_m=0.0, minimum=MIN_DOTS, look=None):
    """The dots inside an outline hull (none for a filled one): the hull clears them of the trail beneath."""
    return _interior(*_key(yaw_deg, dots_per_m, minimum), look or style(), shape('dots'))


def sterns(yaw_deg, size, look='smooth', form='monohull'):
    """Each hull's ``(port, starboard)`` transom corners in dots about the centre, ``size`` (length, beam) dots:
    a wake trails each hull."""
    pairs = [((-0.5, 0.47), (-0.5, -0.47))] if look == 'classic' else hull_shapes.sterns(form, GEOMETRY)
    return tuple(tuple(_turn(yaw_deg, *size, pair)) for pair in pairs)


def points(yaw_deg, dots_per_m=0.0, minimum=MIN_DOTS, look=None):
    """``(bow, [(port, starboard) per hull])`` in dots about the centre, for the leader and the dots tier's wake."""
    size = _size(dots_per_m, minimum)
    return _turn(yaw_deg, *size, ((0.5, 0.0),))[0], sterns(yaw_deg, size, look or style(), shape('dots'))

"""The hull shape library: generic planforms the vessel profile picks by name; pure, no ROS.

A shape is a list of parts in body fractions (u ahead, v to port, both over the hull's length and beam):
``Part(offset, width, deck, side)``. A hull part is a body on the centreline ``offset`` (v), ``width`` wide,
whose port half-width along u follows ``side`` (`side`); a deck part is a flat rectangle ``width`` across
and ``deck`` long that bridges hulls. Each hull trails a wake from its transom corners (``sterns``). A new shape
is an entry in SHAPES: a function of the geometry to its parts.

Geometry (minimap.yaml ``hull:``, the vessel profile's ``hull:`` over it), metres:
  monohull   length, beam
  catamaran  length, beam (overall), pontoon (each hull's width), deck (the bridge's length); the pontoons'
             centrelines lie beam - pontoon apart
"""

import math
from collections import namedtuple

import numpy as np

Part = namedtuple('Part', 'offset width deck side')   # deck: 0 for a hull, else the bridge's length fraction
# A hull's port side, aft to bow, as (u, v) over its own length and width: a pointed ogive bow (half-width
# 0.5 * (1 - t ** FULLNESS) from BOW_START at t = 0 to the stem at t = 1), a straight run narrowing to RUN_AFT
# at -0.5 + QUARTER, and a transom with QUARTER-radius rounded corners.
BOW_START, FULLNESS, RUN_AFT, QUARTER, SEGMENTS = -0.15, 1.6, 0.49, 0.1, 16
# a pontoon is a long parallel tube: its bow starts farther forward and is fuller, its quarters tighter
PONTOON = {'bow_start': 0.2, 'fullness': 1.8, 'run_aft': 0.5, 'quarter': 0.12}


def side(bow_start=BOW_START, fullness=FULLNESS, run_aft=RUN_AFT, quarter=QUARTER):
    """A hull's port side bow first: the stem, the ogive, the run, the transom quarter (u, v fractions)."""
    points = [(bow_start + (0.5 - bow_start) * t, 0.5 * (1 - t ** fullness))
              for t in (1 - i / SEGMENTS for i in range(SEGMENTS + 1))]
    points += [(-0.5 + quarter - quarter * math.sin(a), run_aft - quarter + quarter * math.cos(a))
               for a in (math.pi / 2 * i / 4 for i in range(5))]
    return tuple(points)


def outline(port):
    """The closed outline (bow, port side aft, starboard side forward) of a port side."""
    return tuple(list(port) + [(u, -v) for u, v in reversed(port[1:])])


MONO_SIDE = side()
PONTOON_SIDE = side(**PONTOON)


def _monohull(geometry):
    return (Part(0.0, 1.0, 0.0, MONO_SIDE),)


def _catamaran(geometry):
    beam = geometry['beam'] or 1.0
    width = min(max(geometry['pontoon'] / beam, 0.05), 0.5)      # a pontoon of 0 draws as a thin one
    centre = (1 - width) / 2
    bridge = min(max(geometry['deck'] / (geometry['length'] or 1.0), 0.0), 1.0)
    hulls = (Part(centre, width, 0.0, PONTOON_SIDE), Part(-centre, width, 0.0, PONTOON_SIDE))
    return ((Part(0.0, 2 * centre, bridge, ()),) if bridge else ()) + hulls


SHAPES = {'monohull': _monohull, 'catamaran': _catamaran}


def parts(shape, geometry):
    """The parts of ``shape`` for ``geometry`` (decks first, so hulls draw over them)."""
    if shape not in SHAPES:
        raise ValueError(f'hull shape {shape!r} must be one of {", ".join(SHAPES)}')
    return SHAPES[shape](geometry)


def profile(part):
    """``(u, v)`` arrays of a hull part's half-width (over its own width) by station, aft to bow."""
    port = sorted(part.side)
    return np.array([u for u, _ in port]), np.array([v for _, v in port])


def sterns(shape, geometry):
    """Each hull part's transom corners ``((u, v_port), (u, v_starboard))`` in body fractions: where the
    side's last quarter arc meets the transom."""
    return tuple(((u, part.offset + v * part.width), (u, part.offset - v * part.width))
                 for part in parts(shape, geometry) if not part.deck for u, v in (part.side[-1],))


def polygons(shape, geometry):
    """``[(outline, deck)]``: each part's closed outline in body fractions (u, v), decks first."""
    out = []
    for part in parts(shape, geometry):
        if part.deck:
            u, v = part.deck / 2, part.width / 2
            out.append((((u, v), (-u, v), (-u, -v), (u, -v)), True))
        else:
            out.append((tuple((u, part.offset + v * part.width) for u, v in outline(part.side)), False))
    return out


def coverage(shape, geometry, along, across):
    """A boolean mask: which of the body-frame points (``along`` and ``across`` as length and beam fractions)
    lie on a hull part, and which on a deck part only."""
    hulls = np.zeros(along.shape, bool)
    decks = np.zeros(along.shape, bool)
    halves = {}             # hulls that share a side share its half-width
    for part in parts(shape, geometry):
        if part.deck:
            decks |= (np.abs(along) <= part.deck / 2) & (np.abs(across) <= part.width / 2)
            continue
        if part.side not in halves:
            halves[part.side] = np.interp(along, *profile(part), left=-1.0, right=-1.0)
        hulls |= np.abs(across - part.offset) <= halves[part.side] * part.width
    return hulls, decks & ~hulls

"""Propose a course from tracked objects for the confirm popup .

Pure. The proposal only labels and assigns tracked objects; it never invents one.
``pack`` maps challenge name to its params, in pack order::

    {'nav_channel': {'red': 'red_buoy', 'green': 'green_buoy', 'max_width': 4.0,
                     'count': 2, 'min_score': 0.5, 'approach': (x, y)}}

A challenge with ``red``/``green`` takes gates; one with only ``labels`` takes single marks.
Objects are dicts with ``label``, ``x``, ``y``, ``score`` and optional ``sigma``.
"""

import math

from mhseals_testing.referee import geometry
from mhseals_testing.referee.core import Course, Element

MIN_SCORE = 0.5


def _element(challenge, n, obj):
    return Element(f'{challenge}/{n}', challenge, obj['label'], obj['x'], obj['y'],
                   obj.get('radius', 0.2), sigma=obj.get('sigma', 0.0))


def _gates(params, free):
    """Gate pairs of ``free`` objects, each as a tuple of two objects."""
    by_xy = {(o['x'], o['y']): o for o in free}
    segments = geometry.gates(free, params['red'], params['green'], params['max_width'])
    return [(by_xy[a], by_xy[b]) for a, b in segments]


def _marks(params, free):
    return [(o,) for o in free if o['label'] in params['labels']]


def _near(anchor, group):
    cx = sum(o['x'] for o in group) / len(group)
    cy = sum(o['y'] for o in group) / len(group)
    return math.dist(anchor, (cx, cy))


def course(pack, objects, pose, approach=None, cursor=None, name='proposed'):
    """Return ``(Course(source='proposed'), short)``; ``short`` lists missing counts.

    Shared-class objects go to the challenge whose ``approach`` (else ``approach[name]``,
    else the ``cursor``, else the ``pose``) they are nearest, and no object is used twice.
    """
    taken, elements, short = set(), [], []
    for challenge, params in pack.items():
        low = params.get('min_score', MIN_SCORE)
        labels = {params.get('red'), params.get('green'), *params.get('labels', ())} - {None}
        free = [o for o in objects if id(o) not in taken
                and o['label'] in labels and o.get('score', 0.0) >= low]
        groups = _gates(params, free) if 'red' in params else _marks(params, free)
        anchor = (params.get('approach') or (approach or {}).get(challenge)
                  or cursor or pose[:2])
        groups.sort(key=lambda g: _near(anchor, g))
        want = params.get('count')
        chosen = groups[:want] if want else groups
        n = 0
        for group in chosen:
            for obj in group:
                taken.add(id(obj))
                elements.append(_element(challenge, n, obj))
                n += 1
        if want and len(chosen) < want:
            unit = 'gates' if 'red' in params else 'marks'
            short.append(f'{challenge}: {len(chosen)}/{want} {unit} (expect {want})')
    return Course(name, tuple(elements), 'proposed', frame='map'), short

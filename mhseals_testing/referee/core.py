"""Course model shared by the challenge judges: elements, the course, and a fast contact test."""

import math
from dataclasses import dataclass, field

from mhseals_testing.referee.geometry import HULL, contacts


@dataclass(frozen=True)
class Element:
    """A course object in the course frame; bays carry yaw, width, depth, banner and occupied in ``data``."""
    id: str  # 'nav_channel/3', stable within a course
    challenge: str  # truth ``task`` tag or the operator's assignment
    label: str  # detection class, '' for structures (dock bays)
    x: float
    y: float
    radius: float = 0.2
    sigma: float = 0.0  # m; 0 for truth, tracker covariance for confirmed
    data: dict = field(default_factory=dict)


@dataclass(frozen=True)
class Course:
    name: str
    frame: str  # 'world' (sim) | 'map' (boat)
    source: str  # 'truth' | 'confirmed' | 'proposed'
    elements: tuple
    day: dict = field(default_factory=dict)  # e.g. {'docking': {'banner': 'blue_square'}}
    datum: dict | None = None  # {'lat', 'lon', 'yaw'} for 'map' courses; None for sim

    def by_id(self, element_id):
        return next(e for e in self.elements if e.id == element_id)

    def of(self, challenge):
        """The elements assigned to one challenge, in course order."""
        return tuple(e for e in self.elements if e.challenge == challenge)


def contacts_near(track, elements, hull=HULL):
    """Same answer as ``geometry.contacts`` on the elements as dicts, but skips far-away objects.

    Items outside the track's bounding box grown by the hull's half diagonal plus their radius are
    dropped, and each sample is tested only against items within that reach of it.
    """
    items = [dict(x=e.x, y=e.y, radius=e.radius, label=e.label) for e in elements]
    poses = [p for p in track if math.isfinite(p[1] + p[2])]
    if not poses or not items:
        return []
    half = math.hypot(*hull) / 2
    xs, ys = [p[1] for p in poses], [p[2] for p in poses]
    lo_x, hi_x, lo_y, hi_y = min(xs), max(xs), min(ys), max(ys)
    touched = []
    for index, item in enumerate(items):
        reach = half + item['radius']
        if not (lo_x - reach <= item['x'] <= hi_x + reach and lo_y - reach <= item['y'] <= hi_y + reach):
            continue
        near = [p for p in poses if math.hypot(p[1] - item['x'], p[2] - item['y']) <= reach]
        if contacts(near, [item]):
            touched.append(f"{item['label'] or 'object'}#{index}")
    return touched

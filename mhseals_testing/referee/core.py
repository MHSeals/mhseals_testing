"""What a challenge judge works with and the one place a verdict is decided.

A challenge module (``challenges/<kind>.py``) is pure: it reads a ``Window`` of
the track and events against the course elements of its challenge and returns
checks, split times and details. ``judge`` adds the checks every challenge
shares (contacts, a reported count, the time limit) and derives the owner's
record: completion, time, splits, failures and contacts with their basis. No
points are scored.

Judging is monotone in ``window.t``: a check that is true stays true, so a
result can be frozen once its window closes. While the window is open
(``final=False``) a check that is not yet true is ``None`` (undecided), not
``False``; only a contact or an overrun fails early.

The course model (``Element``, ``Course``) is shared with the course builder (course.py);
``contacts_near`` gives the same answer as ``geometry.contacts`` but skips far-away objects.
"""

import math
from dataclasses import dataclass, field

from mhseals_testing.referee import geometry
from mhseals_testing.referee.geometry import HULL


@dataclass(frozen=True)
class Element:
    """A course object in the course frame; bays carry yaw, width, depth, banner, occupied in ``data``."""
    id: str  # 'nav_channel/3', stable within a course
    challenge: str  # truth ``task`` tag or the operator's assignment
    label: str  # detection class, '' for structures (dock bays)
    x: float
    y: float
    radius: float = 0.2
    data: dict = field(default_factory=dict)
    sigma: float = 0.0  # m; 0 for truth, tracker covariance for confirmed


@dataclass(frozen=True)
class Course:
    """``source`` is truth, confirmed or proposed; ``day`` holds the day's parameters (docking banner)."""
    name: str
    elements: tuple
    source: str = 'truth'  # 'truth' | 'confirmed' | 'proposed'
    day: dict = field(default_factory=dict)  # e.g. {'docking': {'banner': 'blue_square'}}
    frame: str = 'world'  # 'world' (sim) | 'map' (boat)
    datum: dict | None = None  # {'lat', 'lon', 'yaw'} for 'map' courses; None for sim

    def by_id(self, element_id):
        return next(e for e in self.elements if e.id == element_id)

    def of(self, challenge):
        """The elements assigned to one challenge, in course order."""
        return tuple(e for e in self.elements if e.challenge == challenge)


@dataclass(frozen=True)
class Window:
    """Everything one judge may look at, already cut to its span: the track is (t, x, y, yaw)."""
    t0: float
    t: float
    track: tuple
    events: tuple = ()
    track_source: str = 'truth'


@dataclass
class Check:
    name: str
    ok: bool | None  # None: not decided yet
    detail: str = ''


@dataclass
class Result:
    name: str
    verdict: str            # PENDING | PASS | FAIL | N/A
    completion: str         # done | failed | incomplete
    time_s: float           # the window's duration
    splits: dict            # seconds from the window start to each element, where meaningful
    failures: list          # one line per failed check: missed gates, wrong order, timeout, ...
    contacts: list          # labels the hull touched
    basis: str              # truth | confirmed | estimated: what contacts were judged against
    checks: list
    details: dict
    summary: str


def achieved(name, ok, final, detail=''):
    """A check that must come true: an unmet one is undecided until the window is final."""
    return Check(name, True if ok else (False if final else None), detail)


def kept(name, ok, final, detail=''):
    """A check that must stay true: a breach fails at once, otherwise it holds only when final."""
    return Check(name, True if ok and final else (None if ok else False), detail)


def basis(course, window):
    if course.source == 'truth' and window.track_source == 'truth':
        return 'truth'
    return 'confirmed' if course.source == 'confirmed' else 'estimated'


def as_items(elements):
    """Elements as the plain dicts geometry reads."""
    return [dict(x=e.x, y=e.y, radius=e.radius, label=e.label, **e.data) for e in elements]


def touched(track, items):
    """Labels of the buoys the hull footprint touched. The one place contacts are decided."""
    return geometry.contacts(track, [i for i in items if i.get('label')])  # a dock may be touched


def counted(window, items, spec, final):
    """The reported count against the course count (``count: {label, event, name, key}``)."""
    truth = len(geometry.points(items, spec['label']))
    reported = geometry.reported(window.events, spec['event'])
    detail = dict(true=truth, reported=reported)
    return achieved(spec['name'], reported == truth, final), {spec['key']: detail}


def judge(spec, course, window, final=True):
    """Judge one challenge (a ``pack.Spec``) over its window against the course."""
    elements = [e for e in course.elements if e.challenge == spec.truth_task]
    duration = round(window.t - window.t0, 2)
    if not elements:
        return Result(spec.name, 'N/A', 'incomplete', duration, {}, [], [], basis(course, window), [],
                      dict(reason='course has no elements for this challenge'), 'N/A')
    items = as_items(elements)
    checks, splits, details = spec.module.judge(items, window, spec.params, final, course)
    if 'count' in spec.params:
        check, extra = counted(window, items, spec.params['count'], final)
        checks, details = checks + [check], {**details, **extra}
    if spec.time_limit is not None:
        checks.append(kept('within the time limit', duration <= spec.time_limit, final))
    hit = touched(window.track, items)
    checks.append(kept('no contact', not hit, final, ', '.join(hit)))
    details['contacts'] = hit
    failures = [f'{c.name}: {c.detail}' if c.detail else c.name for c in checks if c.ok is False]
    verdict = 'FAIL' if failures else 'PASS' if all(c.ok for c in checks) else 'PENDING'
    completion = {'FAIL': 'failed', 'PASS': 'done', 'PENDING': 'incomplete'}[verdict]
    splits = {k: round(v - window.t0, 2) for k, v in splits.items()}
    summary = f'{completion} · {duration:.1f} s · {len(hit)} contacts'
    return Result(spec.name, verdict, completion, duration, splits, failures, hit,
                  basis(course, window), checks, details, summary)


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
        if geometry.contacts(near, [item]):
            touched.append(f"{item['label'] or 'object'}#{index}")
    return touched

"""Build, save and load a Course; a saved course is refused when the map datum has moved."""

import math

from mhseals_testing import data
from mhseals_testing.analysis.geo import enu
from mhseals_testing.referee.core import Course, Element

SCHEMA = 'mhseals.course/1'
TRUTH_SCHEMA = 'crane-roboboat-course-truth-v1'
DATUM_MAX_M = 1.0
DATUM_MAX_DEG = 2.0
BAY_CHALLENGE = 'docking'


class DatumMismatch(ValueError):
    """The saved course was made against a different navsat datum than the current one."""


def from_truth(truth):
    """Course from crane's course-truth JSON: elements keyed ``<task>/<n>``, bays ``docking/bay<i>``."""
    if truth.get('schema') != TRUTH_SCHEMA:
        raise ValueError(f"not a crane course truth: schema {truth.get('schema')!r}")
    count, elements = {}, []
    for raw in truth.get('elements', ()):
        task = raw['task']
        count[task] = count.get(task, 0) + 1
        elements.append(Element(f'{task}/{count[task]}', task, raw.get('label', ''),
                                raw['x'], raw['y'], raw.get('radius', 0.2)))
    for bay in truth.get('bays', ()):
        keep = {k: bay[k] for k in ('dock', 'yaw', 'width', 'depth', 'banner', 'occupied') if k in bay}
        elements.append(Element(f"{BAY_CHALLENGE}/bay{bay['index']}", BAY_CHALLENGE, '',
                                bay['x'], bay['y'], 0.0, keep))
    return Course(truth.get('name', ''), tuple(elements), 'truth', frame=truth.get('frame', 'world'))


def check_datum(course, datum, max_m=DATUM_MAX_M, max_deg=DATUM_MAX_DEG):
    """Raise DatumMismatch unless ``datum`` is within ``max_m`` and ``max_deg`` of the course's own."""
    have = course.datum
    if have is None:
        raise DatumMismatch(f'course {course.name!r} has no datum to compare with')
    east, north = enu(datum['lat'], datum['lon'], (have['lat'], have['lon']))
    metres = math.hypot(east, north)
    degrees = abs(math.degrees(math.remainder(datum.get('yaw', 0.0) - have.get('yaw', 0.0), 2 * math.pi)))
    if metres > max_m or degrees > max_deg:
        raise DatumMismatch(f'datum moved {metres:.1f} m and {degrees:.1f} deg since course {course.name!r}')


def save(path, course, **extra):
    """Atomically write ``course`` as course.json; ``extra`` adds fields such as ``from`` or ``confirmed_t_ns``."""
    record = dict(name=course.name, frame=course.frame, source=course.source, datum=course.datum,
                  day=course.day, **extra,
                  elements=[{k: v for k, v in vars(e).items() if v not in ({}, None)} for e in course.elements])
    data.write_json(path, record, SCHEMA)


def load(path, datum=None):
    """Course from a course.json; with ``datum`` given, refuses a course made against another datum."""
    record = data.read_json(path)
    if not isinstance(record, dict) or record.get('schema') != SCHEMA:
        raise ValueError(f'{path} is not a {SCHEMA} file')
    elements = tuple(Element(**{k: e[k] for k in Element.__dataclass_fields__ if k in e})
                     for e in record.get('elements', ()))
    course = Course(record.get('name', ''), elements, record.get('source', 'confirmed'),
                    record.get('day') or {}, record.get('frame', 'map'), record.get('datum'))
    if datum is not None:
        check_datum(course, datum)
    return course

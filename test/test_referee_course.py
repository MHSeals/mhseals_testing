"""Course model: 2025 truth fixture, save/load, datum check and the contact prefilter."""
import json
import random
from pathlib import Path

import pytest

from mhseals_testing.referee import course as course_mod
from mhseals_testing.referee.core import Course, Element, contacts_near
from mhseals_testing.referee.geometry import contacts

TRUTH = json.loads((Path(__file__).parent / 'data' / 'course_truth_2025.json').read_text())


def test_2025_truth_gives_elements_and_bays():
    course = course_mod.from_truth(TRUTH)
    assert (course.source, course.frame, course.datum) == ('truth', 'world', None)
    assert len(course.elements) == 27 and len(course.of('docking')) == 4
    assert len(course.of('follow_path')) == 12 and len(course.of('speed_challenge')) == 5
    assert len({e.id for e in course.elements}) == 27
    bay = course.by_id('docking/bay1')
    assert bay.id == 'docking/bay1' and bay.data['banner'] == 'red_circle' and bay.data['occupied'] is False
    assert course.of('nav_channel')[0].label == 'red_pole_buoy'


def test_other_schema_is_refused():
    with pytest.raises(ValueError):
        course_mod.from_truth({'schema': 'x'})


def test_save_load_round_trip(tmp_path):
    course = course_mod.from_truth(TRUTH)
    course_mod.save(tmp_path / 'course.json', course, **{'from': 'truth'})
    assert course_mod.load(tmp_path / 'course.json') == course


DATUM = {'lat': 42.0, 'lon': -71.0, 'yaw': 0.1}


def saved(tmp_path):
    course = Course('day', (Element('a/1', 'a', 'red', 1.0, 2.0, sigma=0.3),), 'confirmed', frame='map', datum=DATUM)
    course_mod.save(tmp_path / 'c.json', course)
    return tmp_path / 'c.json'


def test_same_datum_is_accepted(tmp_path):
    assert course_mod.load(saved(tmp_path), dict(DATUM, lat=42.000004)).elements[0].sigma == 0.3


@pytest.mark.parametrize('moved', [dict(lat=42.00002), dict(lon=-70.9999), dict(yaw=0.1 + 0.05)])
def test_datum_mismatch_is_refused(tmp_path, moved):
    with pytest.raises(course_mod.DatumMismatch):
        course_mod.load(saved(tmp_path), dict(DATUM, **moved))


def test_contacts_near_matches_contacts():
    rng = random.Random(4)
    elements = [Element(f'a/{i}', 'a', 'buoy', rng.uniform(-20, 20), rng.uniform(-20, 20), rng.choice((0.2, 0.5)))
                for i in range(60)]
    items = [dict(x=e.x, y=e.y, radius=e.radius, label=e.label) for e in elements]
    for _ in range(20):
        x, y = rng.uniform(-20, 20), rng.uniform(-20, 20)
        track = []
        for i in range(80):
            x, y = x + rng.uniform(-0.6, 0.6), y + rng.uniform(-0.6, 0.6)
            track.append((i * 0.2, x, y, rng.uniform(-3, 3)))
        assert contacts_near(track, elements) == contacts(track, items)

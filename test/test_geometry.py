"""The referee's pure geometry: crossings, clearance, circling and holding in a bay."""
import math

import pytest

from mhseals_testing.analysis import geometry


def walk(points, dt=1.0):
    return [(i * dt, x, y, 0.0) for i, (x, y) in enumerate(points)]


def test_crossing_sign_follows_the_side_crossed_from():
    segment = ((0, 1), (0, -1))
    forward = geometry.crossings(walk([(-1, 0), (1, 0)]), segment)
    back = geometry.crossings(walk([(1, 0), (-1, 0)]), segment)
    assert [d for _, d in forward] == [-d for _, d in back] and len(forward) == len(back) == 1


def test_a_sample_on_the_line_counts_once():
    segment = ((0, 1), (0, -1))
    assert len(geometry.crossings(walk([(-1, 0), (0, 0), (1, 0)]), segment)) == 1


def test_missing_the_segment_is_no_crossing():
    assert geometry.crossings(walk([(-1, 5), (1, 5)]), ((0, 1), (0, -1))) == []


def test_hull_clearance_is_negative_in_contact():
    length = geometry.HULL[0]
    assert geometry.hull_clearance((0, 0, 0), (length / 2 + 1, 0), 0.2) == pytest.approx(0.8)
    assert geometry.hull_clearance((0, 0, 0), (length / 2, 0), 0.2) < 0
    assert geometry.hull_clearance((0, 0, math.pi / 2), (0, length / 2 + 1), 0.2) == pytest.approx(0.8)


def test_contacts_name_each_touched_object_once():
    items = [dict(x=0.5, y=0, label='red_buoy', template=''), dict(x=9, y=9, label='far', template='')]
    track = [(0, 0, 0, 0), (1, 0.1, 0, 0)]
    assert geometry.contacts(track, items) == ['red_buoy#0']


def test_contacts_name_elements_without_label_or_template():
    items = [dict(x=0.5, y=0), dict(x=0.5, y=0, label='', template='gate'), dict(x=0.5, y=0, label='')]
    track = [(0, 0, 0, 0)]
    assert geometry.contacts(track, items) == ['object#0', 'gate#1', 'object#2']


def test_contacts_name_elements_without_label_or_template():
    items = [dict(x=0.5, y=0), dict(x=0.5, y=0, label='', template='gate'), dict(x=0.5, y=0, label='')]
    track = [(0, 0, 0, 0)]
    assert geometry.contacts(track, items) == ['object#0', 'gate#1', 'object#2']


def test_circled_sweeps_a_lap_and_ignores_far_samples():
    lap = [(i, 3 * math.cos(a), 3 * math.sin(a), 0) for i, a in
           enumerate(math.radians(d) for d in range(0, 361, 10))]
    assert geometry.circled(lap, (0, 0)) > 355
    assert geometry.circled(lap, (100, 100)) == 0


def test_held_in_bay_needs_facing_in():
    bay = dict(x=0, y=0, yaw=0.0, width=2, depth=2)
    inward = [(t, 0, 0, math.pi) for t in range(8)]
    outward = [(t, 0, 0, 0.0) for t in range(8)]
    assert geometry.held_in_bay(inward, bay, 5) == 7
    assert geometry.held_in_bay(outward, bay, 5) == 0


def test_circled_skips_non_finite_samples_instead_of_going_nan():
    lap = walk([(5 * math.cos(a / 10), 5 * math.sin(a / 10)) for a in range(70)])
    lap[30] = (30.0, math.nan, math.nan, 0.0)
    assert math.isfinite(geometry.circled(lap, (0, 0)))

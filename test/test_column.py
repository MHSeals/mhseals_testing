import pytest

from mhseals_testing.analysis.column import refine, worst

TOL = (0.05, 0.015)


def plant(column, y0=(0.2, -0.1), jacobian=((0.7, 0.0), (-0.2, 1.2))):
    return tuple(y + sum(j * c for j, c in zip(row, column)) for y, row in zip(y0, jacobian))


def test_three_holds_fix_an_affine_plant_and_the_fourth_verifies():
    found = refine(plant, (0.0, 0.0), TOL)
    assert found['converged'] and found['holds'] == 4
    assert worst(plant(found['column']), TOL) < 1e-6


def test_known_survey_point_is_not_measured_again():
    calls = []
    found = refine(lambda c: calls.append(c) or plant(c), (0.0, 0.0), TOL, points=[((0.0, 0.0), plant((0, 0)))])
    assert (0.0, 0.0) not in calls and found['converged'] and len(calls) == 3


def test_headroom_limits_the_column_and_unconverged_keeps_the_best_measured():
    found = refine(plant, (0.0, 0.0), TOL, limit=0.1, max_holds=4)
    assert not found['converged'] and sum(abs(c) for c in found['column']) <= 0.1 + 1e-9


def test_a_dead_response_stops_instead_of_dividing_by_zero():
    found = refine(lambda c: (0.2, 0.2), (0.0, 0.0), TOL)
    assert not found['converged'] and found['column'] == (0.0, 0.0)


def test_a_passing_survey_point_is_kept_without_probing():
    calls = []
    found = refine(lambda c: calls.append(c) or plant(c), (0.1, 0.0), TOL, points=[((0.1, 0.0), (0.01, 0.0))])
    assert not calls and found['converged'] and found['column'] == (0.1, 0.0)


def test_probes_respect_the_headroom_and_no_room_means_only_the_start():
    calls = []
    refine(lambda c: calls.append(c) or plant(c), (0.0, 0.0), TOL, limit=0.05, max_holds=2)
    assert all(sum(abs(v) for v in c) <= 0.05 + 1e-9 for c in calls)
    calls.clear()
    refine(lambda c: calls.append(c) or plant(c), (0.0, 0.0), TOL, limit=-0.1, max_holds=2)
    assert calls == [(0.0, 0.0)]  # only the start itself

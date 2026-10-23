"""The map view maths: ladder, fit, heading-up rotation and scale bar."""
import math

import pytest

from mhseals_testing.ui.minimap import view


def test_z_zooms_in_to_a_smaller_half_width_and_Z_out_to_a_larger():
    assert view.zoom_step(20, 1) == 10
    assert view.zoom_step(20, -1) == 40
    assert view.zoom_step(view.FIT, 1, fit_half=28) == 20
    assert view.zoom_step(view.FIT, -1, fit_half=28) == 40


def test_zoom_stops_at_both_ends_and_accepts_off_ladder_widths():
    assert view.zoom_step(5, 1) == 5
    assert view.zoom_step(80, -1) == 80
    assert view.zoom_step(7, 1) == 5
    assert view.zoom_step(7, -1) == 10


def test_fit_holds_every_point_with_the_margin():
    points = [(0, 0), (40, 10), (10, -6)]
    (cx, cy), half = view.fit(points, 100, 100)
    assert (cx, cy) == (20, 2)
    assert half >= 20 * 1.2 - 1e-9
    assert all(abs(x - cx) <= half / 1.1 and abs(y - cy) <= half / 1.1 for x, y in points)


def test_fit_uses_the_tall_axis_when_the_canvas_is_wide_and_short():
    _, half = view.fit([(0, 0), (4, 40)], 200, 100)
    assert half == pytest.approx(20 * 2 * 1.2)


def test_fit_of_one_point_keeps_a_minimum_scale():
    assert view.fit([(3, 4)], 100, 100) == ((3, 4), view.MIN_HALF)


@pytest.mark.parametrize('yaw', [0, 90, 180, 270, 37])
def test_heading_up_puts_ahead_straight_up(yaw):
    ahead = (5 * math.cos(math.radians(yaw)), 5 * math.sin(math.radians(yaw)))
    x, y = view.rotate(ahead, (0, 0), yaw)
    assert (x, y) == pytest.approx((0, 5), abs=1e-9)


def test_north_up_only_recentres():
    assert view.rotate((7, 3), (2, 1)) == (5, 2)


def test_scale_bar_snaps_to_one_two_five_within_the_cell_range():
    length, cells = view.scale_bar(20, 80)  # 0.5 m per cell
    assert length == 5 and cells == 10
    length, cells = view.scale_bar(5, 40)  # 0.25 m per cell
    assert length in (1, 2) and 4 <= cells <= 10


def test_fit_with_nothing_usable_is_the_origin_at_the_minimum_scale():
    assert view.fit([], 80, 40) == ((0.0, 0.0), view.MIN_HALF)
    assert view.fit([(math.nan, 1)], 80, 40) == ((0.0, 0.0), view.MIN_HALF)


def test_fit_ignores_non_finite_points_wherever_they_sit():
    clean = view.fit([(0, 0), (10, 10)], 80, 40)
    assert view.fit([(math.nan, 0), (0, 0), (10, 10)], 80, 40) == clean
    assert view.fit([(0, 0), (math.inf, 3), (10, 10), (0, -math.inf)], 80, 40) == clean


def test_fit_and_scale_bar_survive_a_zero_size_canvas():
    assert view.fit([(0, 0), (10, 10)], 80, 0) == ((0.0, 0.0), view.MIN_HALF)
    assert view.fit([(0, 0), (10, 10)], 0, 40) == ((0.0, 0.0), view.MIN_HALF)
    assert view.scale_bar(20, 0) == (0.01, 1)


def test_scale_bar_of_a_non_finite_half_width_is_the_smallest_bar():
    assert view.scale_bar(math.nan, 80) == (0.01, 1)
    assert view.scale_bar(math.inf, 80) == (0.01, 1)


def test_scale_bar_stays_in_the_cell_range_for_a_huge_view():
    length, cells = view.scale_bar(1e5, 80)
    assert length == 20000
    assert view.SCALE_CELLS[0] <= cells <= view.SCALE_CELLS[1]

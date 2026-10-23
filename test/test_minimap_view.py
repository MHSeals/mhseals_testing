"""The map view maths: ladder, fit, heading-up rotation and scale bar."""
import math

import pytest

from mhseals_testing.ui.minimap import view


def test_zoom_ladder_stops_at_both_ends():
    assert view.zoom_step(view.FIT, 1) == 5
    assert view.zoom_step(5, -1) == view.FIT
    assert view.zoom_step(view.FIT, -1) == view.FIT
    assert view.zoom_step(80, 1) == 80


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

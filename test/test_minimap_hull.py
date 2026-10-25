import math

from mhseals_testing.ui.minimap import hull


def _extent(dots, axis):
    values = [d[axis] for d in dots]
    return max(values) - min(values) + 1


def test_minimum_size_icon_points_its_bow_up():
    dots = hull.mask(90)
    assert 6 <= _extent(dots, 0) <= 8 and 11 <= _extent(dots, 1) <= 13   # beam across, length up
    bow = [d for d in dots if d[2] == 'bow']
    assert 2 <= len(bow) <= 3 and max(d[1] for d in bow) == min(d[1] for d in dots) + (len(bow) > 2)


def test_true_scale_above_the_minimum_and_cached_per_three_degrees():
    small, big = hull.mask(0), hull.mask(0, dots_per_m=24)
    assert len(big) > 4 * len(small)
    assert _extent(big, 0) == round(hull.LENGTH * 24) or abs(_extent(big, 0) - hull.LENGTH * 24) <= 2
    assert hull.mask(31) is hull.mask(30) and hull.mask(29) is hull.mask(30)


def test_rotation_mirrors_and_bad_yaw_falls_back_to_up():
    right, left = hull.mask(0), hull.mask(180)
    assert abs(len(right) - len(left)) <= 8 and abs(_extent(right, 0) - _extent(left, 0)) <= 1
    assert max(d[0] for d in right if d[2] == 'bow') > 0 > min(d[0] for d in left if d[2] == 'bow')
    assert hull.mask(float('nan')) == hull.mask(90)


def test_points_place_bow_ahead_and_sterns_behind():
    bow, port, starboard = hull.points(90)
    assert bow[1] < 0 < port[1] and bow[1] < 0 < starboard[1]
    assert port[0] < 0 < starboard[0]
    assert math.isclose(math.hypot(*bow), hull.MIN_DOTS[0] / 2)


def test_rose_size_is_small():
    assert _extent(hull.mask(0, minimum=(9, 6)), 0) <= 10

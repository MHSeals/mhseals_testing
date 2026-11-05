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
    assert len(big) > 2 * len(small)
    filled = hull.mask(0, look='classic'), hull.mask(0, dots_per_m=24, look='classic')
    assert len(filled[1]) > 4 * len(filled[0])
    assert _extent(big, 0) == round(hull.LENGTH * 24) or abs(_extent(big, 0) - hull.LENGTH * 24) <= 2
    assert hull.mask(31) is hull.mask(30) and hull.mask(29) is hull.mask(30)


def test_rotation_mirrors_and_bad_yaw_falls_back_to_up():
    right, left = hull.mask(0), hull.mask(180)
    assert abs(len(right) - len(left)) <= 8 and abs(_extent(right, 0) - _extent(left, 0)) <= 1
    assert max(d[0] for d in right if d[2] == 'bow') > 0 > min(d[0] for d in left if d[2] == 'bow')
    assert hull.mask(float('nan')) == hull.mask(90)


def test_points_place_bow_ahead_and_sterns_behind():
    bow, ((port, starboard),) = hull.points(90)
    assert bow[1] < 0 < port[1] and bow[1] < 0 < starboard[1]
    assert port[0] < 0 < starboard[0]
    assert math.isclose(math.hypot(*bow), hull.MIN_DOTS[0] / 2)


def test_rose_size_is_small():
    assert _extent(hull.mask(0, minimum=(9, 6)), 0) <= 10


def test_smooth_hull_is_a_one_dot_outline_with_its_inside_cleared():
    edge = {(x, y) for x, y, _ in hull.mask(90, dots_per_m=24, look='smooth')}
    inside = set(hull.interior(90, dots_per_m=24, look='smooth'))
    filled = {(x, y) for x, y, _ in hull.mask(90, dots_per_m=24, look='classic')}
    assert inside and not inside & edge and len(edge) < len(filled)
    assert all(any(n in inside or n not in edge | inside for n in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)))
               for x, y in edge)                     # every outline dot touches the inside or the outside
    assert hull.interior(90, look='classic') == ()


def test_size_comes_from_minimap_yaml():
    from mhseals_testing.analysis.classes import load_section
    size = load_section('hull', hull.DEFAULTS)
    assert hull.MIN_DOTS == tuple(size['min_dots'])


CAT = {'shape': 'catamaran', 'length': 1.34, 'beam': 0.9, 'pontoon': 0.23, 'deck': 0.63}


def test_each_tier_draws_the_vessels_shape_or_falls_back(monkeypatch):
    monkeypatch.setattr(hull, 'GEOMETRY', CAT)
    monkeypatch.setitem(hull.SIZE, 'draws', {'image': ['monohull', 'catamaran'], 'dots': ['monohull']})
    assert hull.shape('image') == 'catamaran' and hull.shape('dots') == 'monohull'
    monkeypatch.setitem(hull.SIZE, 'draws', {'image': ['monohull']})
    assert hull.shape('image') == 'monohull'


def test_catamaran_sprite_has_two_hulls_a_deck_and_two_wakes(monkeypatch):
    from mhseals_testing.ui.minimap import hull_sprite
    monkeypatch.setattr(hull, 'GEOMETRY', CAT)
    _, _, hulls, decks = hull_sprite.silhouette(90.0, 60.0, 40.0, 'catamaran')
    row = hulls[hulls.shape[0] // 2] > 0.5                               # across the middle: hull, gap, hull
    assert decks.max() == 1.0 and (row[1:] != row[:-1]).sum() == 4
    one = hull_sprite.wake(90.0, 60.0, 40.0, 30, 2, 18.0, 'monohull')
    two = hull_sprite.wake(90.0, 60.0, 40.0, 30, 2, 18.0, 'catamaran')
    assert two[2].sum() > 1.5 * one[2].sum()

"""Dock banner chips (tiers, occupied bays) and boat marks."""
from mhseals_testing.ui.minimap import canvas
from mhseals_testing.ui.minimap.marks import Projection
from mhseals_testing.ui.minimap.structure import Banner, Bay, arrow, draw_boat, draw_dock


def make(half=10.0, cols=40, rows=5):
    c = canvas.Canvas(cols, rows)
    return c, Projection(c, (0.0, 0.0), half)


def bay(width, occupied=False):
    return Bay((-width / 2, 1), (width / 2, 1), [Banner(0, 1, 'red', 't')], occupied)


def banner_cells(c):
    return [(x.text, x.fg, x.chip) for row in c.cells() for x in row if x.fg and x.fg.startswith('dock_target')]


def test_colour_tier_banner_is_one_shape_chip_in_the_banner_colour():
    c, p = make()
    draw_dock(p, bay(8))
    assert banner_cells(c) == [('▲', 'dock_target:obj.red', True)]


def test_plain_tier_is_a_two_cell_chip_when_the_bay_is_three_cells_wide_else_shape_only():
    c, p = make()   # 2 cells per metre
    draw_dock(p, bay(2), 'plain')
    assert [t for t, _, _ in banner_cells(c)] == ['r', '▲']
    c, p = make()
    draw_dock(p, bay(1), 'plain')
    assert [t for t, _, _ in banner_cells(c)] == ['▲']


def test_ascii_banner_is_bracketed_only_from_four_and_a_half_cells():
    c, p = make()
    draw_dock(p, bay(2.5), 'ascii')
    assert [t for t, _, _ in banner_cells(c)] == list('[rt]')
    c, p = make()
    draw_dock(p, bay(2), 'ascii')
    assert [t for t, _, _ in banner_cells(c)] == list('rt')


def test_occupied_narrow_bay_shows_no_banner_and_no_bare_object_glyph():
    c, p = make()
    draw_dock(p, bay(1, occupied=True))
    assert banner_cells(c) == []
    assert not any(x.fg and x.fg.startswith('object') for row in c.cells() for x in row)
    c, p = make()
    draw_dock(p, bay(4, occupied=True))
    assert len(banner_cells(c)) == 1


def test_arrows_cover_eight_headings_in_both_sets():
    assert [arrow(a) for a in range(0, 360, 45)] == list('→↗↑↖←↙↓↘')
    assert [arrow(a, True) for a in (0, 90, 180, 270)] == ['>', '^', '<', 'v']


def _layers(c):
    return {x.fg for row in c.cells() for x in row if x.fg}


def test_boat_is_a_filled_hull_with_a_bow_and_never_a_chip():
    for half in (20, 4):
        c, p = make(half=half, rows=8)
        draw_boat(p, (0, 0), 0)
        cells = [x for row in c.cells() for x in row]
        assert {'boat', 'bow'} <= _layers(c) and not any(x.chip for x in cells)
    big, small = make(half=2, rows=8), make(half=20, rows=8)
    draw_boat(big[1], (0, 0), 0)
    draw_boat(small[1], (0, 0), 0)
    assert sum(x.fg == 'boat' for row in big[0].cells() for x in row) > \
        sum(x.fg == 'boat' for row in small[0].cells() for x in row)


def test_unreliable_yaw_is_warn_and_heading_up_points_up():
    c, p = make(half=20, rows=8)
    draw_boat(p, (0, 0), 0, reliable=False)
    assert _layers(c) == {'boat:warn'}
    c = canvas.Canvas(40, 8)
    p = Projection(c, (0.0, 0.0), 20, yaw=30)
    draw_boat(p, (0, 0), 30, ascii_=True)
    assert '^' in ''.join(c.text()) and '#' in ''.join(c.text())


def test_wake_and_crab_leader_need_speed():
    c, p = make(half=20, rows=8)
    draw_boat(p, (0, 0), 0, speed=0.05, cog=60)
    assert 'wake' not in _layers(c)
    c, p = make(half=20, rows=8)
    draw_boat(p, (0, 0), 0, speed=1.0, cog=0)
    assert 'wake' in _layers(c) and 'wake:muted' not in _layers(c)
    c, p = make(half=20, rows=8)
    draw_boat(p, (0, 0), 0, speed=1.0, cog=40, wake=False)
    assert 'wake:muted' in _layers(c) and 'wake' not in _layers(c)


def test_a_north_south_bay_mouth_has_a_width_in_cells():
    c, p = make()
    vertical = Bay((0, -2), (0, 2), [Banner(0, 0, 'red', 't')], True)   # 16 dots high = 4 cells
    assert draw_dock(p, vertical) is None and banner_cells(c) == [('▲', 'dock_target:obj.red', True)]

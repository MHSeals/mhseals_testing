"""Map layers: trail pattern densities, costmap pooling, hits, plans."""
import numpy as np
import pytest

from mhseals_testing.ui.minimap import canvas, layers, panel
from mhseals_testing.ui.minimap.state import MapState
from mhseals_testing.ui.minimap.layers import Crumb


def _frame(cols=100, rows=10, half=20.0):
    return layers.Frame(canvas.Canvas(cols, rows), (0.0, 0.0), half)


def _dots(c):
    return sum(bin(ord(ch) - 0x2800).count('1') for line in c.text() for ch in line if '⠀' <= ch <= '⣿')


def _band(fix, n=8, x0=-4.0, step=0.25):
    """Crumbs along +x, 0.25 m apart (under a dot at 0.4 m/dot)."""
    return [Crumb(x0 + i * step, 0.0, fix) for i in range(n * 4)]


@pytest.mark.parametrize('fix,period', [('fix', 1), ('float', 2), ('dgps', 3), ('3d', 4)])
def test_gps_trail_density_is_one_over_period_even_with_crumbs_closer_than_a_dot(fix, period):
    f = _frame()  # 200 dots over 40 m: 0.2 m per dot, crumbs 0.25 m apart
    layers.draw_trail(f, _band(fix), 'gps')
    solid = _frame()
    layers.draw_trail(solid, _band('fix'), 'gps')
    assert _dots(f.canvas) == pytest.approx(_dots(solid.canvas) / period, abs=1)


def test_density_holds_at_four_tenths_of_a_metre_per_dot():
    f = _frame(half=40.0)  # 0.4 m per dot
    layers.draw_trail(f, _band('float', n=20), 'gps')
    ref = _frame(half=40.0)
    layers.draw_trail(ref, _band('fix', n=20), 'gps')
    assert _dots(f.canvas) == pytest.approx(_dots(ref.canvas) / 2, abs=1)


def test_no_fix_is_a_gap_mark_without_dots():
    f = _frame()
    layers.draw_trail(f, _band('none'), 'gps')
    assert _dots(f.canvas) == 0 and canvas.GAP in ''.join(f.canvas.text())


def test_track_bands_and_dot_layers():
    f = _frame()
    pts = [Crumb(i * 0.2, 0.0, xte=x) for i, x in enumerate([0.1] * 20 + [0.4] * 20 + [1.0] * 20)]
    layers.draw_trail(f, pts, 'track')
    tops = {c.fg for row in f.canvas.cells() for c in row if c.fg}
    assert tops == {'trail.track.ok', 'trail.track.mid', 'trail.track.bad'}


def test_speed_over_vmax_is_dashed_only_when_colourless():
    pts = [Crumb(i * 0.2, 0.0, speed=2.0) for i in range(60)]
    coloured, plain = _frame(), _frame()
    layers.draw_trail(coloured, pts, 'speed', vmax=1.0)
    layers.draw_trail(plain, pts, 'speed', vmax=1.0, colourless=True)
    assert _dots(plain.canvas) == pytest.approx(_dots(coloured.canvas) / 2, abs=1)
    assert 'trail.speed.spd.5' in {c.fg for row in coloured.canvas.cells() for c in row}


@pytest.mark.parametrize('speed, vmax', [(float('nan'), 1.0), (1.0, float('nan')), (float('inf'), float('inf'))])
def test_speed_trail_survives_non_finite_speed_or_vmax(speed, vmax):
    pts = [Crumb(i * 0.2, 0.0, speed=speed) for i in range(20)]
    frame = _frame()
    layers.draw_trail(frame, pts, 'speed', vmax=vmax)
    assert _dots(frame.canvas) > 0


def test_speed_trail_survives_unknown_speed_from_map_state():
    s = MapState()
    for i in range(20):  # no gps_raw feed: every crumb has speed None
        s.ingest('pose_map', t=float(i), x=i * 0.3, y=0.0)
    crumbs = panel.crumbs(s)
    assert crumbs and all(c.speed is None for c in crumbs)
    frame = _frame()
    layers.draw_trail(frame, crumbs, 'speed', vmax=1.0)
    assert _dots(frame.canvas) > 0
    assert 'trail.speed.spd.0' in {c.fg for row in frame.canvas.cells() for c in row}


def test_older_epochs_are_dim_and_never_joined_to_the_new_one():
    f = _frame()
    pts = [Crumb(-5, 0, epoch=0), Crumb(-4, 0, epoch=0), Crumb(4, 0, epoch=1), Crumb(5, 0, epoch=1)]
    layers.draw_trail(f, pts, 'plain', epoch=1)
    assert 'trail.old' in {c.fg for c in f.canvas.cells()[5]}
    assert f.canvas.text()[5][50] == canvas.BLANK  # nothing between the epochs


def test_plan_every_second_dot_and_local_plan_hidden_when_zoomed_out():
    f = _frame()
    layers.draw_plan(f, [(-5, 0), (5, 0)])
    assert _dots(f.canvas) == pytest.approx(25, abs=1)  # 10 m at 0.2 m per dot, every 2nd
    far = _frame(half=20.0)
    layers.draw_plan(far, [(-5, 0), (5, 0)], local=True)
    assert _dots(far.canvas) == 0
    near = _frame(half=10.0)
    layers.draw_plan(near, [(-5, 0), (5, 0)], local=True)
    assert _dots(near.canvas) > 0


def _grid(value, shape=(100, 100)):
    g = np.zeros(shape, dtype=np.int8)
    g[50, 50] = value
    return g


def test_a_lethal_cell_survives_pooling_into_coarser_canvas_cells():
    f = _frame(cols=10, rows=5, half=40.0)  # 8 m per cell column: far coarser than 0.4 m
    layers.draw_costmap(f, _grid(100), 0.4, (-20.0, -20.0))
    tints = {c.bg for row in f.canvas.cells() for c in row if c.bg}
    assert tints == {'lethal_bg@75'}


def test_cell_keeps_the_highest_cost_when_several_pool_together():
    g = _grid(60)
    g[50, 51] = 100
    f = _frame(cols=4, rows=2, half=10.0)
    layers.draw_costmap(f, g, 0.4, (-20.0, -20.0))
    assert {c.bg for row in f.canvas.cells() for c in row if c.bg} == {'lethal_bg@100'}


def test_far_zoom_drops_costs_below_inscribed_and_near_zoom_ramps():
    far = _frame(half=20.0)
    layers.draw_costmap(far, _grid(98), 0.4, (-20.0, -20.0))
    assert not any(c.bg for row in far.canvas.cells() for c in row)
    inscribed = _frame(half=20.0)
    layers.draw_costmap(inscribed, _grid(99), 0.4, (-20.0, -20.0))
    assert {c.bg for row in inscribed.canvas.cells() for c in row if c.bg} == {'lethal_bg@45'}
    near = _frame(half=5.0)
    layers.draw_costmap(near, _grid(98), 0.4, (-20.0, -20.0))
    assert {c.bg for row in near.canvas.cells() for c in row if c.bg} == {'cost_bg@100'}
    low = _frame(half=5.0)
    layers.draw_costmap(low, _grid(49), 0.4, (-20.0, -20.0))
    assert not any(c.bg for row in low.canvas.cells() for c in row)


def test_hits_are_lethal_cells_within_range_and_near_ones_use_the_fail_layer():
    g = np.zeros((100, 100), dtype=np.int8)
    g[50, 50] = 100    # centre (0.2, 0.2): near
    g[50, 60] = 100    # about 4.2 m east: hit
    g[50, 99] = 100    # 19.4 m east: still inside 20 m
    g[40, 50] = 99     # inscribed is not a hit
    g[0, 0] = 100      # far corner, 28 m away
    f = _frame(half=40.0)
    layers.draw_hits(f, g, 0.4, (-20.0, -20.0), (0.0, 0.0))
    layers_seen = [c.fg for row in f.canvas.cells() for c in row if c.fg]
    assert sorted(layers_seen) == ['hits', 'hits', 'hits.near']


def test_gps_trail_from_map_state_maps_fix_type_to_bands():
    from mhseals_testing.ui.minimap import panel
    from mhseals_testing.ui.minimap.state import MapState

    def trail(fix):
        s = MapState()
        s.ingest('gps_raw', fix_type=fix, h_acc_m=0.02, vel_cms=65535)
        for i in range(30):
            s.ingest('pose_map', t=float(i), x=i * 0.5, y=0.0)
        f = _frame()
        layers.draw_trail(f, panel.crumbs(s), 'gps')
        return _dots(f.canvas)

    fixed = trail(6)
    assert fixed > 0
    assert trail(3) == pytest.approx(fixed / 4, abs=2) and trail(3) > 0
    assert trail(5) < fixed and trail(4) < fixed
    assert trail(1) == 0 and trail(0) == 0

"""The whole map frame: perception overlay goldens (perception-v2 7.1, tui-visual 2.3) and its budgets."""
import json
import math
import os
import statistics
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from mhseals_testing.analysis.classes import load_classes
from mhseals_testing.ui import theme
from mhseals_testing.ui.minimap import frame, marks, panel, tasks, view
from mhseals_testing.ui.minimap.cache import LayerCache
from mhseals_testing.ui.minimap.emit import emit
from mhseals_testing.ui.minimap.layerbox import LayerBox
from mhseals_testing.ui.minimap.marks import Obj, Projection
from mhseals_testing.ui.minimap.state import MapState

GOLDEN = Path(__file__).parent / 'golden' / 'minimap'
SCALE = float(os.environ.get('MHS_BENCH_SCALE', '1'))  # slow CI host budget multiplier
TABLE = load_classes()


def grid(size=160, resolution=0.1, origin=(-8.0, -8.0)):
    """A fake OccupancyGrid: a cost ramp ring and a lethal wall, the shape of a 25.6 KB perception costmap."""
    cells = []
    for row in range(size):
        for col in range(size):
            d = math.hypot(col - size / 2, row - size / 2)
            cells.append(100 if col == 120 else 99 if col in (119, 121) else max(0, 90 - int(abs(d - 40))))
    position = SimpleNamespace(x=origin[0], y=origin[1])
    return SimpleNamespace(data=cells, info=SimpleNamespace(width=size, height=size, resolution=resolution,
                                                           origin=SimpleNamespace(position=position)))


def scene(trail=400, spiral=False):
    """A boat heading 30 deg with a trail, plan, GPS fix, camera fov, costmap and one track of each kind.

    ``spiral`` lays ``trail`` crumbs on a spiral filling the view first (the long-session bench)."""
    s = MapState()
    s.ingest('gps_raw', fix_type=6, h_acc_m=0.4, vel_cms=80)
    for i in range(trail if spiral else 0):
        r = 2 + 15 * i / trail
        s.ingest('pose_map', t=-1e4 + i, x=r * math.cos(i * 0.4 / r), y=r * math.sin(i * 0.4 / r), yaw=0.0)
    for i in range(trail):
        a = i / max(trail - 1, 1) * 1.2
        s.ingest('pose_map', t=float(i) * 0.4, x=14 * math.sin(a) - 14 * math.sin(1.2),
                 y=-14 * math.cos(a) + 14 * math.cos(1.2), yaw=0.0)
    s.ingest('pose_map', t=trail * 0.4, x=0.0, y=0.0, yaw=math.radians(30))
    s.ingest('plan', points=[(0, 0), (6, 4), (12, 9), (16, 14)])
    s.ingest('tf_static', transforms={('base_link', 'front_camera_link'): (0.5, 0.0, 0.3, 0.0)})
    s.ingest('fov', hfov_deg=110.0, range_m=12.0, frame='front_camera_link')
    s.ingest('costmap', data=grid())
    s.ingest('objects', items=[
        Obj(6, 1, 'red_buoy', 0.9), Obj(4, 6, 'green_buoy', 0.9), Obj(11, 4, 'red_pole_buoy', 0.8),
        Obj(-6, 8, 'yellow_target_boat', 0.85, motion='moving', vx=1.5, vy=-0.8),
        Obj(14, -4, 'black_buoy', 0.6, motion='unknown'),
        Obj(-8, -6, 'red_light_buoy', 0.7, source='prior')])
    return s


def with_tasks(state):
    """The mission's task hypotheses and goal on top of a scene (kept out of the goldens' scene)."""
    message = json.loads((Path(__file__).parent / 'fixtures' / 'task_hypotheses.json').read_text())
    state.ingest('tasks', items=tasks.parse(message))
    state.ingest('mission_goal', x=12.0, y=-2.0, yaw=0.0, why='search the east field for the speed gate')
    return state


def in_view_tasks(state):
    """The review's worst case: a confirmed 6x6 m dock box and four hypotheses round the boat."""
    def square(cx, cy, side):
        h = side / 2
        return [[cx - h, cy - h], [cx + h, cy - h], [cx + h, cy + h], [cx - h, cy + h], [cx - h, cy - h]]
    found = [{'id': 'dock', 'type': 'dock', 'state': 'confirmed', 'confidence': 0.9, 'pose': [2, 1],
              'extent': square(2, 1, 6)}]
    found += [{'id': f'h{i}', 'type': 'speed_gate', 'state': 'hypothesis', 'confidence': 0.3 + 0.1 * i,
               'pose': [x, y], 'extent': square(x, y, 3)}
              for i, (x, y) in enumerate(((-3, 3), (3, -3), (-3, -3), (4, 4)))]
    state.ingest('tasks', items=tasks.parse({'schema': tasks.SCHEMA, 'hypotheses': found}))
    state.ingest('mission_goal', x=1.0, y=-1.0, yaw=0.0, why='dock in the confirmed bay')
    return state


def box(state, **flags):
    b = LayerBox(None, state)
    b.fov = True
    for name, value in flags.items():
        setattr(b, name, value)
    return b


def text(canvas):
    return '\n'.join(line.replace('⠀', ' ').rstrip() for line in canvas.text()) + '\n'


@pytest.mark.skipif(theme.ASCII, reason='the Unicode goldens')
@pytest.mark.parametrize('up,half', [('N', 20.0), ('H', 20.0), ('N', 8.0)])
def test_overlay_goldens_north_and_heading_up(request, up, half):
    s = scene()
    canvas = panel.draw(s, 116, 27, half, TABLE, False, box(s, up=up), full=True)
    golden = GOLDEN / f'overlays_{up}_{half:g}.txt'
    if os.environ.get('UPDATE_GOLDEN') or request.config.getoption('--update-goldens'):
        golden.write_text(text(canvas))
    assert text(canvas) == golden.read_text(), f'{golden.name} differs; UPDATE_GOLDEN=1 to regenerate'


def test_ascii_frame_stays_legible():
    s = scene()
    lines = emit(panel.draw(s, 116, 60, 10.0, TABLE, True, box(s), full=True), lambda layer: '', ascii=True)
    body = '\n'.join(lines)
    assert body.isascii() and '#' in body and '| 5 m' in body and 'N^' in body
    assert 'Y85' in body and 'L70.' in body            # moving track and prior labels
    assert 'r90' in body and '+-0.4 m' in body


def layers_at(canvas):
    return {c.fg for row in canvas.cells() for c in row if c.fg} | {c.bg for row in canvas.cells() for c in row if c.bg}


def test_each_track_kind_draws_its_own_way():
    s = scene()
    s.ingest('gps_raw', fix_type=6, h_acc_m=2.0, vel_cms=80)   # a circle wider than the hull
    used = layers_at(panel.draw(s, 116, 27, 20.0, TABLE, False, box(s, costmap=True), full=True))
    assert {'object:obj.red', 'object:obj.black@50', 'object:obj.red@50', 'track.pred', 'fov:fov', 'grid',
            'chrome', 'boat', 'bow', 'wake', 'lethal'} <= used
    assert any(layer.startswith('track.vel:obj.yellow') for layer in used)
    assert any(layer.startswith('acc:gps.fix') for layer in used)


def test_toggles_remove_their_layers_and_the_drive_column_has_no_rings_or_fov():
    s = scene()
    off = box(s, fov=False, rings=False, wake=False, objects='off', costmap=False)
    used = layers_at(panel.draw(s, 116, 27, 20.0, TABLE, False, off, full=True))
    assert not used & {'fov:fov', 'grid', 'wake', 'lethal', 'object:obj.red'}
    drive = layers_at(panel.draw(s, 26, 12, 20.0, TABLE, False, box(s, costmap=True), full=False))
    assert 'fov:fov' not in drive and 'grid' not in drive and 'boat' in drive


def test_fov_cone_follows_the_camera_extrinsic():
    s = scene()
    x, y, yaw = s.camera()
    assert math.isclose(x, 0.5 * math.cos(math.radians(30)), abs_tol=1e-9) and math.isclose(yaw, math.radians(30))
    s.ingest('tf_static', transforms={('base_link', 'cam_mount'): (0.0, 0.0, 0.0, math.pi),
                                      ('cam_mount', 'front_camera_link'): (1.0, 0.0, 0.0, 0.0),
                                      ('front_camera_link', 'front_camera_optical_frame'): (0, 0, 0, -1.57)})
    s.ingest('fov', hfov_deg=90.0, range_m=5.0, frame='front_camera_optical_frame')
    x, y, yaw = s.camera()
    assert math.isclose(yaw, math.radians(210)) and math.isclose(x, -math.cos(math.radians(30)), abs_tol=1e-9)
    s.fov = None
    assert s.camera() is None


def test_accuracy_circle_hides_under_a_cell_and_labels_to_scale():
    c = panel.Canvas(40, 10)
    proj = Projection(c, (0.0, 0.0), 20.0)
    frame.draw_accuracy(proj, (0.0, 0.0), (0.02, 'fix'))
    assert not layers_at(c)
    frame.draw_accuracy(proj, (0.0, 0.0), (4.0, 'float'))
    assert layers_at(c) == {'acc:gps.float@50'}
    assert [frame.accuracy_text(v) for v in (0.02, 0.004, 0.4, 3.0, 12.0)] == ['±2 cm', '±1 cm', '±0.4 m', '±3.0 m', '±12 m']
    s = MapState()
    assert s.accuracy() is None
    s.ingest('gps_cov', h_acc_m=1.2, status=0)
    assert s.accuracy() == (1.2, '3d')
    s.ingest('gps_raw', fix_type=5, h_acc_m=0.05, vel_cms=0)
    assert s.accuracy() == (0.05, 'float')


def test_motion_arrowhead_needs_three_cells_and_unknowns_fade():
    c = panel.Canvas(40, 10)
    proj = Projection(c, (0.0, 0.0), 20.0)
    slow = Obj(0, 0, 'red_buoy', 0.9, motion='moving', vx=0.2, vy=0.0)
    assert marks.draw_motion(proj, slow, 'obj.red', TABLE.tracks) is None
    fast = Obj(0, 0, 'red_buoy', 0.9, motion='moving', vx=0.0, vy=8.0)
    head = marks.draw_motion(proj, fast, 'obj.red', TABLE.tracks)
    assert head is not None and c.cells()[head[1]][head[0]].text == '↑'
    assert marks._fade(Obj(0, 0, 'r', 0.9, motion='unknown'), TABLE.tracks) == 50


TIERS = ('truecolor', '256', '16', 'mono', 'ascii')
ZOOMS = sorted({*view.PRESETS, 8})   # every preset, and the 8 m detail golden


@pytest.mark.parametrize('half', ZOOMS)
def test_full_frame_with_every_layer_stays_in_the_byte_and_time_budget(half):
    """tui-visual 2.3: 116x27 with every layer on emits <= 10 KiB and draws and emits (cached
    costmap and rings, median frame) in <= 3 ms x scale, at every zoom preset and colour tier,
    also with the whole 4000-crumb trail ring inside the view (the TUI runs over SSH), and with
    a confirmed dock box and four hypotheses round the boat over that trail."""
    over = []
    for s in (with_tasks(scene()), with_tasks(scene(trail=4000, spiral=True)),
              in_view_tasks(scene(trail=4000, spiral=True))):
        b, lc = box(s, costmap=True, hits=True), LayerCache()
        for tier in TIERS:
            plain = tier == 'ascii'
            colours = 'mono' if plain else tier
            frame = lambda: emit(panel.draw(s, 116, 27, float(half), TABLE, plain, b, full=True, tier=colours,
                                            cache=lc), lambda layer: panel.style(layer, colours), colours, ascii=plain)
            size = len('\n'.join(frame()).encode())
            times = []
            for _ in range(11):
                t = time.perf_counter()
                frame()
                times.append((time.perf_counter() - t) * 1000)
            ms = statistics.median(times)
            if size > 10 * 1024 or ms > 3.0 * SCALE:
                over.append(f'{len(s.crumbs)} crumbs {tier}: {size} B {ms:.2f} ms')
    assert not over, over

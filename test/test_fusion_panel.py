"""Fusion tab: the feed (subscriptions only while shown, projection, pairing) and the panel in every tier."""

import io
import json
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest
from rich.console import Console

from mhseals_testing.health.model import Fact
from mhseals_testing.ros import fusion_feed
from mhseals_testing.ros.fusion_feed import FusionFeed, StaticTF, detections
from mhseals_testing.ui import layout, pixels, theme
from mhseals_testing.ui.panels import fusion

K = [700.0, 0, 640.0, 0, 700.0, 360.0, 0, 0, 1]
SIZE = (1280, 720)
CAMERA = 'front_camera_optical_frame'


def header(frame, t=10.0):
    return NS(frame_id=frame, stamp=NS(sec=int(t), nanosec=int((t % 1) * 1e9)))


def tf(parent, child, xyz=(0, 0, 0), quat=(0, 0, 0, 1)):
    return NS(header=NS(frame_id=parent), child_frame_id=child,
              transform=NS(translation=NS(x=xyz[0], y=xyz[1], z=xyz[2]),
                           rotation=NS(x=quat[0], y=quat[1], z=quat[2], w=quat[3])))


# optical (x right, y down, z ahead) from base (x ahead, y left, z up): the usual -90/0/-90 rpy
OPTICAL_Q = (-0.5, 0.5, -0.5, 0.5)


def statics():
    return NS(transforms=[tf('base_link', 'lidar_link', (0.2, 0, 0.5)),
                          tf('base_link', 'front_camera_link', (0.3, 0, 0.4)),
                          tf('front_camera_link', CAMERA, quat=OPTICAL_Q)])


def cloud(points, frame='lidar_link', t=10.0):
    data = np.asarray(points, '<f4').tobytes()
    fields = [NS(name=a, offset=4 * i) for i, a in enumerate('xyz')]
    return NS(header=header(frame, t), fields=fields, point_step=12, data=data)


def ob(cls, box, pos, cov=(0.0,) * 9, oid=''):
    return NS(class_id=cls, has_box2d=box is not None, box2d=box or (0, 0, 0, 0),
              position=NS(x=pos[0], y=pos[1], z=pos[2]), position_covariance=cov, backend_id=oid)


def array(obs, frame=CAMERA, t=10.0):
    return NS(header=header(frame, t), observations=obs)


def feed(clock=None, camera=None):
    subs, gone = [], []

    def subscribe(kind, topic, callback):
        subs.append((kind, topic, callback))
        return len(subs)
    f = FusionFeed(subscribe, gone.append, camera, clock=clock or (lambda: 0.0))
    return f, subs, gone


def by_topic(subs):
    return {topic: callback for _, topic, callback in subs}


def test_subscriptions_exist_only_while_wanted_and_closing_forgets():
    f, subs, gone = feed()
    f.on_tf_static(statics())
    assert not subs and not f.is_open
    f.want(True)
    assert {t for _, t, _ in subs} == {*f.cfg['topics'].values(), '/tf_static'}
    by_topic(subs)['/perception/observations'](array([ob('red_buoy', (0.4, 0.4, 0.5, 0.6), (0, 0, 7))]))
    assert f.snapshot()['dets']
    f.want(True)
    assert len(subs) == 5                       # idempotent
    f.want(False)
    assert len(gone) == 5 and not f.is_open and not f.snapshot()['dets']
    assert f.tf.edges                            # latched /tf_static survives a closed tab
    assert not f.points_due()                    # closed: scans are dropped before decoding



def test_tf_static_published_before_the_feed_exists_still_reaches_it():
    """The session latched /tf_static long before the tab opened; the feed's own latched subscription
    gets it again on open (a TRANSIENT_LOCAL subscriber is handed the stored message), and the scan projects."""
    latched = statics()                          # published at startup, before any FusionFeed
    subs = []

    def subscribe(kind, topic, callback):
        subs.append((kind, topic, callback))
        if kind == 'tf_static':
            callback(latched)
        return len(subs)
    f = FusionFeed(subscribe, lambda handle: None, clock=lambda: 0.0)
    assert f.tf.between(CAMERA, 'lidar_link') is None
    f.want(True)
    assert ('tf_static', '/tf_static') in {(k, t) for k, t, _ in subs}
    cb = by_topic(subs)
    cb[f.cfg['topics']['camera_info']](NS(k=K, width=SIZE[0], height=SIZE[1]))
    cb[f.cfg['topics']['observations']](array([ob('red_buoy', (0.45, 0.4, 0.55, 0.6), (0, 0, 7))]))
    f.on_points(cloud([(10.1, 0, -0.1)]))
    assert f.snapshot()['points'][1][0] == pytest.approx((0.5, 0.5), abs=1e-4)

def test_camera_opens_only_when_asked():
    camera = NS(opened=0, closed=0, latest=lambda: None)
    camera.open = lambda: setattr(camera, 'opened', camera.opened + 1)
    camera.close = lambda: setattr(camera, 'closed', camera.closed + 1)
    f, _, _ = feed(camera=camera)
    f.want(True, camera=False)
    assert camera.opened == 0
    f.want(True, camera=True)
    f.want(True, camera=True)
    assert camera.opened == 1
    f.want(False, camera=True)                   # off closes the camera too
    assert camera.closed == 1


def ready_feed(clock):
    f, subs, _ = feed(clock=clock)
    f.on_tf_static(statics())
    f.want(True)
    cb = by_topic(subs)
    cb[f.cfg['topics']['camera_info']](NS(k=K, width=SIZE[0], height=SIZE[1]))
    cb[f.cfg['topics']['observations']](array([ob('red_buoy', (0.45, 0.4, 0.55, 0.6), (0, 0, 7))]))
    return f, cb


def test_scan_projects_through_static_tf_and_is_rate_limited():
    now = [0.0]
    f, _ = ready_feed(lambda: now[0])
    # 10 m straight ahead of the camera, in lidar coordinates (the lidar sits 0.1 m behind, 0.1 m above)
    assert f.points_due()
    f.on_points(cloud([(10.1, 0, -0.1), (-5, 0, 0), (float('nan'), 0, 0)]))
    scan = f.snapshot()['points']
    assert scan[1].shape == (1, 2)
    assert scan[1][0] == pytest.approx((0.5, 0.5), abs=1e-4)
    assert scan[2][0] == pytest.approx(10.0, abs=1e-3)
    assert not f.points_due()                    # 2 Hz
    now[0] = 0.5
    assert f.points_due()


def test_applied_correction_moves_points_as_lidar_fusion_does():
    now = [0.0]
    f, _ = ready_feed(lambda: now[0])
    f.applied = (0.0, 1.0, 0.0)                  # deg about optical y (yaw)
    f.on_points(cloud([(10.1, 0, -0.1)]))
    u = f.snapshot()['points'][1][0][0] * SIZE[0]
    assert abs(u - 640) == pytest.approx(700 * math.tan(math.radians(1.0)), abs=0.2)


def test_decimation_keeps_at_most_points_max():
    f, _ = ready_feed(lambda: 0.0)
    xyz = fusion_feed.cloud_xyz(cloud(np.random.default_rng(1).uniform(1, 9, (10000, 3))), 3000)
    assert 2500 <= len(xyz) <= 3000


def test_static_tf_composes_chains():
    t = StaticTF()
    t.add(statics())
    R, p = t.between(CAMERA, 'lidar_link')
    assert R @ np.array([10.1, 0, -0.1]) + p == pytest.approx([0, 0, 10.0])
    assert t.between(CAMERA, 'elsewhere') is None


def test_fused_boxes_pair_with_stereo_by_box_and_take_source_labels():
    box_a, box_b = (0.1, 0.4, 0.2, 0.6), (0.6, 0.4, 0.7, 0.6)
    raw = (10.0, CAMERA, [{'cls': 'red_buoy', 'box': box_a, 'pos': (0, 0, 7.4), 'cov': (0,) * 9, 'id': ''},
                          {'cls': 'green_buoy', 'box': box_b, 'pos': (0, 0, math.nan), 'cov': (0,) * 9, 'id': ''}])
    cov = (0, 0, 0, 0, 0, 0, 0, 0, 0.04)
    fused = (10.0, CAMERA, [{'cls': 'red_buoy', 'box': box_a, 'pos': (0, 0, 7.0), 'cov': cov, 'id': ''},
                            {'cls': 'green_buoy', 'box': box_b, 'pos': (0, 0, 12.0), 'cov': (0,) * 9, 'id': ''}])
    rows = detections(raw, fused, (10.0, ['lidar', 'lidar_ray']))
    assert [(r['source'], r['range'], r['stereo']) for r in rows] == [('lidar', 7.0, 7.4), ('lidar_ray', 12.0, None)]
    assert rows[0]['sigma'] == pytest.approx(0.2)
    assert fusion.range_label(rows[0]) == '7.0L/7.4S' and fusion.range_label(rows[1]) == '12.0R'
    other = detections(raw, fused, (10.1, ['stereo', 'lidar']))   # another frame's labels, as many boxes
    assert [r['source'] for r in other] == ['fused', 'fused']
    stereo_only = detections(raw, None, None)
    assert [r['source'] for r in stereo_only] == ['stereo', 'stereo']


def test_ranges_are_horizontal_as_lidar_fusion_grades_rho():
    box = (0.1, 0.4, 0.2, 0.6)
    cov = (1.0, 0, 0, 0, 9.0, 0, 0, 0, 1.0)                       # the vertical (optical y) variance is ignored
    raw = (10.0, CAMERA, [{'cls': 'red_buoy', 'box': box, 'pos': (3.0, 2.0, 4.0), 'cov': cov, 'id': 'a'}])
    body = (10.0, 'base_link', [{'cls': 'red_buoy', 'box': box, 'pos': (4.0, 3.0, -2.0), 'cov': (0,) * 9, 'id': 'a'}])
    row, = detections(raw, None, None)
    assert row['range'] == pytest.approx(5.0) and row['sigma'] == pytest.approx(1.0) and row['id'] == 'a'
    row, = detections(raw, body, (10.0, ['lidar']))                 # body frame: z is the vertical
    assert (row['range'], row['stereo']) == pytest.approx((5.0, 5.0))


def test_sources_become_pixel_residuals():
    now = [100.0]
    f, cb = ready_feed(lambda: now[0])
    cb[f.cfg['topics']['sources']](NS(data='{"t": 10, "sources": ["lidar"], "ex": [[0, 0, 7, 0, 0.002, -0.001, 0, 0]]}'))
    cb[f.cfg['topics']['sources']](NS(data='not json'))
    (age, du, dv), = f.snapshot()['residuals']
    assert (age, du, dv) == pytest.approx((0.0, 1.4, -0.7), abs=1e-5)
    now[0] += f.cfg['feed']['residual_s'] + 1
    cb[f.cfg['topics']['sources']](NS(data='{"t": 11, "sources": []}'))
    assert f.snapshot()['residuals'] == []



def test_off_centre_residuals_are_true_pixel_offsets():
    """As lidar_fusion forms them: r = atan((uc - cx) / fx) - atan(x). An 8 px offset at x = 0.8 stays 8 px."""
    now = [100.0]
    f, cb = ready_feed(lambda: now[0])
    x, y, fx, fy = 0.8, -0.3, K[0], K[4]
    r_az = math.atan(x + 8.0 / fx) - math.atan(x)
    r_el = math.atan(y - 6.0 / fy) - math.atan(y)
    cb[f.cfg['topics']['sources']](NS(data=f'{{"t": 10, "sources": ["lidar"], "ex": [[{x}, {y}, 7, 0, {r_az}, {r_el}, 0, 0]]}}'))
    (_, du, dv), = f.snapshot()['residuals']
    assert (du, dv) == pytest.approx((8.0, -6.0), abs=1e-6)
    assert r_az * fx < 5.0                      # the old r_az * fx would have read under du_pass_px

def model(du=1.0, sources=('lidar', 'lidar_ray', 'stereo'), fused=True, n=12, dv=-0.5, ls=None):
    dets = []
    for (cls, box, rng, stereo), source in zip(
            (('red_buoy', (0.42, 0.45, 0.47, 0.6), 7.0, 7.3), ('green_buoy', (0.62, 0.48, 0.65, 0.56), 12.0, None),
             ('yellow_buoy', (0.2, 0.5, 0.22, 0.54), 22.0, 22.0)), sources):
        stereo = stereo if ls is None or stereo is None else rng / ls      # ls: every lidar/stereo ratio
        x = (box[0] + box[2] - 1) / 2 * rng * SIZE[0] / K[0]
        z = math.sqrt(rng * rng - x * x)
        dets.append({'cls': cls, 'box': box, 'source': source, 'frame': CAMERA, 'range': rng, 'stereo': stereo,
                     'sigma': 0.1 * rng / 7, 'pos': (x, 0.0, z),
                     'stereo_pos': None if stereo is None else (x * stereo / rng, 0.0, z * stereo / rng)})
    rng = np.random.default_rng(3)
    # water returns along a few rings near the boat, a shore line at 45 m, and a cluster on each lidar box
    # (the red one sits on its box, the green one 6 px right of it: a small yaw residual)
    water_v = np.repeat([0.98, 0.86, 0.78, 0.72], 40)
    water = np.column_stack([rng.uniform(0.05, 0.95, len(water_v)), water_v + rng.normal(0, 0.004, len(water_v))])
    shore = np.column_stack([np.linspace(0, 1, 160), 0.455 + rng.normal(0, 0.003, 160)])
    clusters = [np.column_stack([rng.uniform(0.425, 0.465, 30), rng.uniform(0.47, 0.59, 30)]),
                np.column_stack([rng.uniform(0.627, 0.652, 12) + 6 / SIZE[0], rng.uniform(0.49, 0.55, 12)])]
    uv = np.vstack([water, shore, *clusters]).astype(np.float32)
    ranges = np.concatenate([2.0 / (water_v - 0.5) / 1.2, np.full(160, 45.0), np.full(30, 6.8),
                             np.full(12, 11.9)]).astype(np.float32)
    xz = np.column_stack([(uv[:, 0] - 0.5) * ranges * SIZE[0] / K[0], ranges]).astype(np.float32)
    return {'version': 1, 'size': SIZE, 'K': K, 'dets': dets, 'fused': fused, 'frame': None,
            'points': (10.0, uv, ranges, xz),
            'residuals': [(-60 + 5 * i, du + 0.3 * (i % 3), dv) for i in range(n)],
            'grades': [(-60 + 5 * i, d['cls'], d['source'] in fusion_feed.LIDAR_SOURCES,
                        d['range'] / d['stereo'] if d['source'] in fusion_feed.LIDAR_SOURCES else None)
                       for i in range(n) for d in dets if d['stereo'] and d['stereo'] < 10 and d['source'] != 'held']}


def facts(applied='0.10 -0.30 0.02', state='ok'):
    return {'perc.lidar.fusion_state': Fact(state, 'lidar', {'applied_deg': applied, 'extrinsic_deg': '0.12 -0.33 0.01'}, 0),
            'perc.lidar.fusion_hz': Fact(9.8, 'lidar', {}, 0), 'perc.lidar.fusion_share': Fact(0.71, 'lidar', {}, 0)}


@pytest.mark.parametrize('du, sources, fused, n, level', [
    (1.0, ('lidar', 'lidar_ray', 'stereo'), True, 12, 'PASS'),
    (7.0, ('lidar', 'lidar_ray', 'stereo'), True, 12, 'WARN'),
    (12.0, ('lidar', 'lidar_ray', 'stereo'), True, 12, 'FAIL'),
    (1.0, ('stereo', 'lidar_ray', 'stereo'), True, 12, 'FAIL'),     # no lidar on the near box
    (1.0, ('lidar', 'lidar_ray', 'stereo'), True, 3, None),         # collecting
])
def test_live_check_follows_fusion_calib_rule(du, sources, fused, n, level):
    got, why = fusion.check(model(du, sources, fused, n), fusion_feed.load_config())
    assert got == level, why


@pytest.mark.parametrize('dv, level', [(0.5, 'PASS'), (-4.9, 'PASS'), (7.0, 'WARN'), (-7.0, 'WARN'),
                                       (15.0, 'FAIL'), (-9.5, 'FAIL')])
def test_live_check_grades_dv_like_du(dv, level):
    got, why = fusion.check(model(du=0.5, dv=dv), fusion_feed.load_config())
    assert got == level, why


@pytest.mark.parametrize('ls, level', [(1.02, 'PASS'), (0.97, 'PASS'), (1.07, 'WARN'), (0.92, 'WARN'),
                                       (1.18, 'FAIL'), (0.85, 'FAIL')])
def test_live_check_fails_past_range_fail_tol(ls, level):
    got, why = fusion.check(model(du=0.5, dv=0.5, ls=ls), fusion_feed.load_config())
    assert got == level, why
    assert f'L/S {ls:.2f}' in why


def frame_of(cb, f, boxes, t, skip=''):
    """One lidar_fusion frame: stereo array, fused array and its sources JSON. boxes: (id, stereo m, fused m, source)."""
    stereo = [ob('red_buoy', (0.1 * i, 0.4, 0.1 * i + 0.05, 0.6), (0, 0, s), oid=k) for i, (k, s, _, _) in enumerate(boxes)]
    fused = [ob('red_buoy', (0.1 * i, 0.4, 0.1 * i + 0.05, 0.6), (0, 0, r), oid=k) for i, (k, _, r, _) in enumerate(boxes)]
    cb[f.cfg['topics']['observations']](array(stereo, t=t))
    cb[f.cfg['topics']['fused']](array(fused, t=t))
    cb[f.cfg['topics']['sources']](NS(data=json.dumps({'t': t, 'sources': [b[3] for b in boxes], 'skip': skip,
                                                        'ex': [[0, 0, 7, 0, 0.0005, 0.0003, 0, 0]] * 2})))


def graded(boxes_per_frame):
    now = [100.0]
    f, cb = ready_feed(lambda: now[0])
    for i, (boxes, skip) in enumerate(boxes_per_frame):
        now[0] += 0.1
        frame_of(cb, f, boxes, 10.0 + 0.1 * i, skip)
    snap = f.snapshot()
    return snap['grades'], fusion.check({**snap, 'dets': snap['dets']}, f.cfg)


def test_held_and_skipped_frames_are_neither_hit_nor_miss():
    """Clean residuals with a held box at 6 m read near 0/1 FAIL before; skipped frames ('motion') are left out."""
    grades, (level, why) = graded([([('a', 6.0, 6.0, 'held')], '')] * 3 + [([('a', 6.0, 6.0, 'stereo')], 'motion')] * 3)
    assert grades == [] and level == 'PASS', why
    grades, (level, why) = graded([([('a', 6.0, 6.1, 'lidar')], '')] + [([('a', 6.0, 6.0, 'stereo')], 'no_scan')] * 4)
    assert len(grades) == 1 and level == 'PASS' and 'near 1/1' in why, why


def test_far_box_with_an_off_ratio_is_not_graded():
    """27 m lidar against 24 m stereo is normal stereo error at range (L/S 1.12 failed before)."""
    grades, (level, why) = graded([([('far', 24.0, 27.0, 'lidar'), ('near', 6.0, 6.1, 'lidar')], '')] * 3)
    assert {g[1] for g in grades} == {'near'} and level == 'PASS' and 'L/S 1.02' in why, why


def test_hits_pool_per_object_over_the_window():
    """An object that misses some frames but hits others passes (hit rate over the window); one never hit fails."""
    frames = [([('a', 6.0, 6.1, 'lidar')], ''), ([('a', 6.0, 6.0, 'stereo')], '')] * 3
    _, (level, why) = graded(frames)
    assert level == 'PASS' and 'near 1/1' in why, why
    _, (level, why) = graded([([('a', 6.0, 6.1, 'lidar'), ('b', 8.0, 8.0, 'stereo')], '')] * 3)
    assert level == 'FAIL' and 'near 1/2' in why, why
    _, (level, why) = graded([([('a', 6.0, 6.1, 'lidar')], '')] + [([('a', 6.0, 6.0, 'stereo')], '')] * 3)
    assert level == 'WARN' and 'near 1/1' in why, why        # hit rate 25% < hit_rate_min


def test_panel_check_keys_match_fusion_calib():
    c = fusion_feed.load_config()['check']
    assert {'du_pass_px', 'du_warn_px', 'dv_pass_px', 'dv_warn_px', 'range_tol', 'range_fail_tol'} <= set(c)
    assert (c['dv_pass_px'], c['dv_warn_px'], c['range_fail_tol']) == (c['du_pass_px'], c['du_warn_px'], 0.10)



def test_readout_keeps_the_estimate_and_the_ratio_when_narrow():
    cfg, status = fusion_feed.load_config(), fusion.fusion_status(facts())
    wide = [line.plain for line in fusion.readout(model(), cfg, status)]
    assert wide[0].endswith('L/S 0.96 · live') and 'du +1.3' in wide[0]
    assert wide[1] == 'corr  ox +0.10 oy -0.30 oz +0.02° · est +0.12 -0.33 +0.01'
    assert wide[2].endswith('px raw')                    # lidar_fusion's samples, before the applied correction
    narrow = [line.plain for line in fusion.readout(model(), cfg, status, 48)]
    assert narrow[0] == 'check PASS near 1/1 · L/S 0.96'
    assert narrow[1] == 'corr  +0.10 -0.30 +0.02° · est +0.12 -0.33 +0.01'
    tight = [line.plain for line in fusion.readout(model(), cfg, status, 40)]
    assert tight[1:3] == ['corr  +0.10 -0.30 +0.02°', '      est +0.12 -0.33 +0.01']
    assert all(len(line) <= 40 for line in tight[:3])

def test_readout_marks_a_kept_correction_stale():
    cfg = fusion_feed.load_config()
    line = fusion.readout({**model(), 'applied': (0.1, -0.3, 0.02)}, cfg, {})[1].plain
    assert line == 'corr  ox +0.10 oy -0.30 oz +0.02° stale'
    assert fusion.readout({**model(), 'applied': (0.0, 0.0, 0.0)}, cfg, {})[1].plain == 'corr  none applied'


def test_budgeted_link_slows_the_drawings(monkeypatch):
    cfg = fusion_feed.load_config()
    assert fusion.draw_rate(cfg) == cfg['draw_hz'] and fusion.draw_rate(cfg, 0.5) == cfg['draw_hz']
    monkeypatch.setattr(pixels, 'BUDGET', pixels.SSH_BUDGET)
    assert fusion.draw_rate(cfg) == min(cfg['draw_hz'], pixels.THROTTLE_HZ)
    assert fusion.draw_rate(cfg, 0.5) == fusion.draw_rate(cfg) / 2


def render(view, width, height=34):
    import test_dashboard
    from mhseals_testing.profiles import load_profile
    import test_inspect_popup
    profile = load_profile('boat')
    view = test_dashboard.view(profile, **view)
    snapshot = test_inspect_popup.scene()[0].snapshot(profile.required_tf)
    console = Console(width=width, height=height, file=io.StringIO(), force_terminal=True, color_system='truecolor')
    return [line for line in console.render_lines(layout.render(snapshot, view, width, height), pad=False)]


def tab_view(**extra):
    return {'tab': 'Fusion', 'focus': 'fusion', 'cursors': {}, 'modal': None, 'hints': [('q', 'Quit')],
            'notice': '', 'fusion': model(), 'fusion_facts': facts(), **extra}


@pytest.fixture
def text_tier():
    pixels.use('text')
    fusion._TEXT.clear()
    yield
    pixels.use('text')


@pytest.mark.parametrize('width', [80, 120, 160])
def test_tab_fits_every_width_in_text(width, text_tier, monkeypatch):
    monkeypatch.setattr(layout, 'header', lambda *a, **k: '')
    monkeypatch.setattr(layout, 'footer', lambda *a, **k: '')
    body = fusion.body(tab_view(), width, 29)
    console = Console(width=width, file=io.StringIO(), force_terminal=True, color_system='truecolor', height=29)
    lines = console.render_lines(body, console.options.update(height=29), pad=False)
    assert len(lines) == 29
    text = '\n'.join(''.join(s.text for s in line) for line in lines)
    assert 'Camera' in text and 'Around' in text and 'Alignment' in text
    assert '7.0L/7.3S' in text and '12.0R' in text and 'PASS' in text
    assert 'ox +0.10 oy -0.30 oz +0.02' in text or '+0.10 -0.30 +0.02° · est +0.12 -0.33 +0.01' in text
    assert any(0x2800 < ord(ch) <= 0x28FF for ch in text)          # the braille drawing


def test_text_tier_repaints_on_new_data_at_draw_hz_at_most(text_tier, monkeypatch):
    now = [100.0]
    monkeypatch.setattr(pixels, 'clock', lambda: now[0])
    cfg = fusion_feed.load_config()
    m = model()
    a = fusion.camera_rows(m, cfg, 40, 10)
    assert fusion.camera_rows(m, cfg, 40, 10) is a                    # same data: the held rows
    m['version'] = 2
    assert fusion.camera_rows(m, cfg, 40, 10) is a                    # new data within 1/draw_hz
    assert fusion.camera_rows(m, cfg, 41, 10) is not a                # a new size repaints at once
    now[0] += 1.0 / cfg['draw_hz']
    b = fusion.camera_rows(m, cfg, 41, 10)
    m['version'] = 3
    now[0] += 1.0 / cfg['draw_hz']
    assert fusion.camera_rows(m, cfg, 41, 10) is not b


def test_ascii_uses_layer_characters(text_tier, monkeypatch):
    monkeypatch.setattr(theme, 'ASCII', True)
    rows = fusion.camera_rows(model(), fusion_feed.load_config(), 60, 14)
    text = ''.join(r.plain for r in rows)
    assert '#' in text and '.' in text and all(ord(ch) < 128 for ch in text)


def test_image_tier_draws_tiles_with_label_holes():
    pixels.use('sixel', cell=(10, 20))
    try:
        rows = fusion.camera_rows(model(), fusion_feed.load_config(), 48, 12)
        text = ''.join(r.plain for r in rows)
        assert any(0xF0000 <= ord(ch) <= 0xFFFFD for ch in text)     # sixel marker cells
        assert '7.0L/7.3S' in text                                    # the label stays terminal text
    finally:
        pixels.use('text')


def test_no_node_and_tab_registration(text_tier):
    assert 'Fusion' in layout.TABS and layout.DEFAULT_FOCUS['Fusion'] == 'fusion'
    out = '\n'.join(''.join(s.text for s in line) for line in render(tab_view(fusion=None), 100))
    assert 'Fusion needs a ROS node' in out
    out = '\n'.join(''.join(s.text for s in line) for line in render(tab_view(fusion=None, fusion_review=True), 100))
    assert 'Fusion is live only' in out and 'ROS node' not in out


def test_stereo_only_says_fusion_is_off(text_tier):
    lines = fusion.readout(model(sources=('stereo',) * 3, fused=False), fusion_feed.load_config(), {})
    assert 'fusion off' in lines[-1].plain and 'none applied' in lines[1].plain


def test_app_feed_is_off_when_the_tab_is_not_shown(monkeypatch):
    from mhseals_testing.ui.app import App
    wants = []
    fake = NS(want=lambda on, camera=False: wants.append(on), snapshot=lambda: {'x': 1}, applied=None)
    app = NS(tab='Drive', review=None, fusion_feed=fake, facts={}, ros=None, profile=None)
    assert App.fusion_view(app) == {} and wants == [False]
    assert fake.applied is None                  # hidden: not even the status is read
    app.tab = 'Fusion'
    assert App.fusion_view(app)['fusion'] == {'x': 1} and wants == [False, True]
    assert fake.applied is None                  # no fusion_state yet: nothing to keep
    app.facts = facts()
    App.fusion_view(app)
    assert fake.applied == (0.10, -0.30, 0.02)
    app.facts = {}                               # the fact went stale: the last correction stays for the scan
    App.fusion_view(app)
    assert fake.applied == (0.10, -0.30, 0.02)
    app.review = object()                        # review: live only, and the feed closes
    assert App.fusion_view(app) == {'fusion': None, 'fusion_review': True} and wants[-1] is False
    app.fusion_feed = None
    assert App.fusion_view(app) == {'fusion': None, 'fusion_review': True}


MARKERS = {*range(0xF0000, 0xFFFFE), ord(pixels.PLACEHOLDER)}   # sixel markers and kitty placeholders (tiles)


def lines_of(view, width, height, tier):
    """Full-frame rows as (text, [segments]) in ``tier`` (pixels ``use``), header and footer included."""
    pixels.use(tier, cell=(10, 20)) if tier != 'text' else pixels.use('text')
    fusion._TEXT.clear()
    try:
        rows = render(view, width, height)
    finally:
        pixels.use('text')
    return [(''.join(s.text for s in row), row) for row in rows]


@pytest.mark.parametrize('tier', ['text', 'sixel', 'kitty'])
@pytest.mark.parametrize('width', [80, 120, 160])
def test_safety_rows_stay_text_over_the_tab(width, tier):
    """ARMED, STOPPED with the paused agent claim, and Space stay terminal text no tile reaches."""
    from mhseals_testing.ui import authority_chip
    import test_stop_ui
    view = tab_view(armed=True, arm_label='ARMED', agent=test_stop_ui.agent_banner(), hints=[('Space', 'Stop')],
                    authority=authority_chip.chip(test_stop_ui.held('keys', 'tui'), test_stop_ui.NOW))
    height = 30 if width == 80 else 40
    rows = lines_of(view, width, height, tier)
    assert len(rows) == height
    assert 'ARMED' in rows[0][0] and rows[1][0].lstrip().startswith(f"{theme.pick('⊘', '(/)')} STOPPED")
    assert rows[1][0].rstrip().endswith(layout.chrome.AGENT_STOP_KEYS)
    assert any('Space' in text for text, _ in rows[-3:])
    chrome = rows[:2] + rows[-3:]
    assert not any(ord(ch) in MARKERS for text, _ in chrome for ch in text)
    if tier != 'text':
        assert any(ord(ch) in MARKERS for text, _ in rows[2:-3] for ch in text)


@pytest.mark.parametrize('tier', ['sixel', 'kitty'])
def test_tiles_dim_under_a_popup_like_the_text(tier):
    view = tab_view(modal={'kind': 'message', 'title': 'Ready', 'message': 'Clear the area'})
    rows = lines_of(view, 120, 40, tier)
    tiles = [seg for _, row in rows for seg in row if any(ord(ch) in MARKERS for ch in seg.text)]
    assert tiles and all(seg.style is not None and seg.style.dim for seg in tiles)


SNIPPET = '''
import io, sys
sys.path.insert(0, "test")
from rich.console import Console
from mhseals_testing.ui import palette, pixels
from mhseals_testing.ui.panels import fusion
import test_fusion_panel as T
pixels.use("text")
c = Console(width=100, height=30, file=io.StringIO(), force_terminal=True,
            color_system={"mono": None, "16": "standard", "256": "256", "truecolor": "truecolor"}[palette.detect_tier()])
c.print(fusion.body(T.tab_view(), 100, 30), height=30)
sys.stdout.write(c.file.getvalue())
'''


@pytest.mark.parametrize('env, braille, colour', [
    ({'TERM': 'xterm'}, True, True),                       # 16 colours
    ({'TERM': 'xterm-256color', 'NO_COLOR': '1'}, True, False),
    ({'TERM': 'xterm-256color', 'MHS_ASCII': '1'}, False, True),
])
def test_tab_reads_in_16_colours_no_color_and_ascii(env, braille, colour):
    import os
    import subprocess
    import sys
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    base = {k: v for k, v in os.environ.items() if k not in ('NO_COLOR', 'MHS_ASCII', 'CX_ASCII', 'COLORTERM', 'TERM')}
    base['PYTHONPATH'] = os.pathsep.join([str(root), base.get('PYTHONPATH', '')])
    out = subprocess.run([sys.executable, '-c', SNIPPET], env={**base, **env}, cwd=root, capture_output=True,
                         text=True, timeout=60, check=True).stdout
    assert '7.0L' in out and '12.0R' in out and 'PASS' in out
    assert any(0x2800 < ord(ch) <= 0x28FF for ch in out) == braille
    assert ('\x1b[3' in out or '\x1b[9' in out or '\x1b[38' in out) == colour
    if not braille:
        assert all(ord(ch) < 128 for ch in out.replace('\x1b', ''))

"""Fusion tab: the feed (subscriptions only while shown, projection, pairing) and the panel in every tier."""

import io
import json
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest
from rich.console import Console

from mhseals_testing.health.model import Fact
from mhseals_testing.fusion import ros_feed as fusion_feed
from mhseals_testing.fusion.ros_feed import FusionFeed, StaticTF, detections
from mhseals_testing.ui import layout, theme
from mhseals_testing.ui import imaging as pixels
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
    topics = f.cfg['topics']
    assert {t for _, t, _ in subs} == {*(v for k, v in topics.items() if k != 'camera_info'), *topics['camera_info'],
                                       '/tf_static'}
    by_topic(subs)['/perception/observations'](array([ob('red_buoy', (0.4, 0.4, 0.5, 0.6), (0, 0, 7))]))
    assert f.snapshot()['dets']
    f.want(True)
    assert len(subs) == 6                       # idempotent (two camera_info topics)
    f.want(False)
    assert len(gone) == 6 and not f.is_open and not f.snapshot()['dets']
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
    cb[f.cfg['topics']['camera_info'][0]](NS(k=K, width=SIZE[0], height=SIZE[1]))
    cb[f.cfg['topics']['observations']](array([ob('red_buoy', (0.45, 0.4, 0.55, 0.6), (0, 0, 7))]))
    f.on_points(cloud([(10.1, 0, -0.1)]))
    assert f.snapshot()['points'][1][0] == pytest.approx((0.5, 0.5), abs=1e-4)

def test_the_first_listed_camera_info_wins_once_it_has_published():
    f, subs, _ = feed()
    f.want(True)
    cb = by_topic(subs)
    zed, relay = f.cfg['topics']['camera_info']
    other_k = [350.0, 0, 320.0, 0, 350.0, 180.0, 0, 0, 1]
    cb[relay](NS(k=other_k, width=640, height=360))               # alone, the lower priority one is used
    assert f.size == (640, 360)
    cb[zed](NS(k=K, width=SIZE[0], height=SIZE[1]))
    assert f.K == K and f.size == SIZE
    cb[relay](NS(k=other_k, width=640, height=360))               # and no longer once the ZED's has arrived
    assert f.K == K and f.size == SIZE
    cb[zed](NS(k=[v * 1.0 for v in K], width=SIZE[0], height=SIZE[1]))
    assert f.size == SIZE
    f.want(False)
    f.want(True)                                                   # a reopened feed starts over
    cb = by_topic(subs)
    cb[relay](NS(k=other_k, width=640, height=360))
    assert f.size == (640, 360)


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
    cb[f.cfg['topics']['camera_info'][0]](NS(k=K, width=SIZE[0], height=SIZE[1]))
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


def test_point_range_is_horizontal_as_the_box_labels():
    """A point 10 m ahead and 3 m up is coloured at 10 m (lidar_fusion's rho), not its 10.44 m ray."""
    f, _ = ready_feed(lambda: 0.0)
    f.on_points(cloud([(10.1, 0, 2.9)]))
    assert f.snapshot()['points'][2][0] == pytest.approx(10.0, abs=1e-3)


def test_points_tap_is_installed_only_while_open():
    f, _, _ = feed()
    taps = {}
    f.tap = lambda on: taps.update(points=1) if on else taps.pop('points', None)
    f.want(True)
    assert 'points' in taps
    f.want(False)
    assert taps == {}


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
    now[0] += f.cfg['feed']['residual_window'] + 1
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
    """One lidar_fusion frame: stereo array, fused array and its sources JSON.

    boxes: (id, stereo m, fused m, source[, class[, normalised box]]); by default red buoys spaced apart."""
    def spec(i, b):
        return (b[4] if len(b) > 4 else 'red_buoy'), (b[5] if len(b) > 5 else (0.1 * i + 0.1, 0.4, 0.1 * i + 0.15, 0.6))
    stereo = [ob(*spec(i, b), (0, 0, b[1]), oid=b[0]) for i, b in enumerate(boxes)]
    fused = [ob(*spec(i, b), (0, 0, b[2]), oid=b[0]) for i, b in enumerate(boxes)]
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
    assert grades == [] and level == 'WARN' and 'near 0' in why, why         # nothing graded: not a PASS
    grades, (level, why) = graded([([('a', 6.0, 6.1, 'lidar')], '')] + [([('a', 6.0, 6.0, 'stereo')], 'no_scan')] * 4)
    assert len(grades) == 1 and level == 'PASS' and 'near 1/1' in why, why


def test_far_box_with_an_off_ratio_is_not_graded():
    """27 m lidar against 24 m stereo is normal stereo error at range (L/S 1.12 failed before)."""
    grades, (level, why) = graded([([('far', 24.0, 27.0, 'lidar'), ('near', 6.0, 6.1, 'lidar')], '')] * 3)
    assert {g[1] for g in grades} == {'near'} and level == 'PASS' and 'L/S 1.02' in why, why


def test_near_box_lidar_fusion_would_not_sample_is_not_graded():
    """A near dock or boat box that stays stereo read near 1/2 FAIL with clean residuals; lidar_fusion samples
    only sample_classes (buoys), so fusion_calib never grades it. Nor a buoy box clipped at the image edge."""
    for other in (('dock', 5.0, 5.0, 'stereo', 'dock'), ('boat', 7.0, 7.0, 'stereo', 'target_boat'),
                  ('clip', 6.0, 6.0, 'stereo', 'red_buoy', (0.0, 0.4, 0.05, 0.6))):
        grades, (level, why) = graded([([('a', 6.0, 6.06, 'lidar'), other], '')] * 3)
        assert {g[1] for g in grades} == {'a'} and level == 'PASS' and 'near 1/1' in why, (other, why)


def test_overlapped_boxes_are_not_graded():
    """A buoy behind another (boxes within isolation_deg in u) gives lidar_fusion no sample; neither is graded."""
    grades, (level, why) = graded([([('a', 6.0, 6.06, 'lidar', 'red_buoy', (0.40, 0.4, 0.45, 0.6)),
                                     ('b', 8.0, 8.0, 'stereo', 'green_buoy', (0.44, 0.42, 0.48, 0.58)),
                                     ('c', 7.0, 7.07, 'lidar', 'green_buoy', (0.70, 0.4, 0.74, 0.6))], '')] * 3)
    assert {g[1] for g in grades} == {'c'} and level == 'PASS' and 'near 1/1' in why, why
    # just clear of each other: fx tan(2 deg) apart in u, both graded
    gap = K[0] * math.tan(math.radians(2.0)) / SIZE[0] + 0.002
    grades, _ = graded([([('a', 6.0, 6.06, 'lidar', 'red_buoy', (0.40, 0.4, 0.45, 0.6)),
                          ('b', 8.0, 8.08, 'lidar', 'green_buoy', (0.45 + gap, 0.4, 0.50 + gap, 0.6))], '')] * 3)
    assert {g[1] for g in grades} == {'a', 'b'}


def test_sample_rule_is_lidar_fusions():
    rule = fusion_feed.sample_rule()
    assert rule['sample_classes'] and rule['border_px'] >= 0 and rule['isolation_deg'] > 0


def test_hits_pool_per_object_over_the_window():
    """An object that misses some frames but hits others passes (hit rate over the window); one never hit fails."""
    frames = [([('a', 6.0, 6.1, 'lidar')], ''), ([('a', 6.0, 6.0, 'stereo')], '')] * 3
    _, (level, why) = graded(frames)
    assert level == 'PASS' and 'near 1/1' in why, why
    _, (level, why) = graded([([('a', 6.0, 6.1, 'lidar'), ('b', 8.0, 8.0, 'stereo')], '')] * 3)
    assert level == 'FAIL' and 'near 1/2' in why, why
    _, (level, why) = graded([([('a', 6.0, 6.1, 'lidar')], '')] + [([('a', 6.0, 6.0, 'stereo')], '')] * 3)
    assert level == 'WARN' and 'near 1/1' in why, why        # hit rate 25% < hit_rate_min


def test_fused_array_pairs_with_the_stereo_array_of_its_stamp():
    """lidar_fusion publishes frame t after the sweep and TF wait, when stereo t + 1/30 (boxes jittered) has arrived.
    Paired with the newest stereo array, the near box lost its stereo range: no grade, PASS while L/S was 1.14."""
    now = [100.0]
    f, cb = ready_feed(lambda: now[0])
    box, moved = (0.4, 0.4, 0.45, 0.6), (0.401, 0.4, 0.451, 0.6)
    for t in (10.0, 10.1, 10.2):
        now[0] += 0.1
        cb[f.cfg['topics']['observations']](array([ob('red_buoy', box, (0, 0, 7.0), oid='a')], t=t))
        cb[f.cfg['topics']['observations']](array([ob('red_buoy', moved, (0, 0, 7.05), oid='a')], t=t + 1 / 30))
        cb[f.cfg['topics']['fused']](array([ob('red_buoy', box, (0, 0, 8.0), oid='a')], t=t))
        cb[f.cfg['topics']['sources']](NS(data=json.dumps({'t': t, 'sources': ['lidar'], 'skip': '',
                                                            'ex': [[0, 0, 7, 0, 0.0005, 0.0003, 0, 0]] * 2})))
    snap = f.snapshot()
    det, = snap['dets']
    assert det['stereo'] == pytest.approx(7.0) and det['range'] == pytest.approx(8.0)
    assert [g[1:] for g in snap['grades']] == [('a', True, pytest.approx(8 / 7))] * 3
    level, why = fusion.check(snap, f.cfg)
    assert level == 'FAIL' and 'L/S 1.14' in why, why
    cb[f.cfg['topics']['fused']](array([ob('red_buoy', box, (0, 0, 8.0), oid='a')], t=9.0))   # older than RAW_S
    assert f.snapshot()['dets'][0]['stereo'] is None


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
    monkeypatch.setattr(fusion, '_HZ', {})
    assert fusion.draw_rate(cfg, 'a') == cfg['draw_rate'] and fusion.draw_rate(cfg, 'b', 0.5) == cfg['draw_rate']
    monkeypatch.setattr(pixels, 'BUDGET', pixels.SSH_BUDGET)
    assert fusion.draw_rate(cfg, 'a') == min(cfg['draw_rate'], pixels.THROTTLE_HZ) * fusion.START
    assert fusion.draw_rate(cfg, 'b', 0.5) == fusion.draw_rate(cfg, 'a') / 2


def test_drawing_rate_ramps_up_and_drops_at_once_over_the_budget(monkeypatch):
    """A drawing starts at START of the top rate and rises by at most STEP per update; the link going over its budget
    resizes the rate at once instead of after RATE_S."""
    cfg = fusion_feed.load_config()
    now = [100.0]
    top = min(cfg['draw_rate'], pixels.THROTTLE_HZ)
    monkeypatch.setattr(fusion, '_HZ', {})
    monkeypatch.setattr(pixels, 'clock', lambda: now[0])
    monkeypatch.setattr(pixels, 'BUDGET', pixels.SSH_BUDGET)
    monkeypatch.setattr(pixels, 'STATS', {'upload': 0, 'sixel': 0, 'tiles': 0})
    first = fusion.draw_rate(cfg, 'cam')
    assert first == top * fusion.START
    now[0] += fusion.RATE_S
    pixels.STATS['sixel'] += 1                                         # a nearly silent link: the rate steps up
    assert fusion.draw_rate(cfg, 'cam') == pytest.approx(first * fusion.STEP)
    held = fusion._HZ['cam'][0]
    now[0] += 0.5
    pixels.STATS['sixel'] += 10 * pixels.SSH_BUDGET                    # a burst over the budget: no 5 s wait
    monkeypatch.setattr(pixels, 'rate', lambda now=None: 2 * pixels.SSH_BUDGET)
    assert fusion.draw_rate(cfg, 'cam') < held / 4


def test_drawing_rate_settles_under_the_budget(monkeypatch):
    """rate / BUDGET (or its square) settles above the budget: a drawing that sends a fixed number of bytes a repaint
    settles at LINK_SHARE of the budget, and the rate comes back once the scan thins."""
    cfg = fusion_feed.load_config()
    now = [100.0]
    monkeypatch.setattr(fusion, '_HZ', {})
    monkeypatch.setattr(pixels, 'clock', lambda: now[0])
    monkeypatch.setattr(pixels, 'BUDGET', pixels.SSH_BUDGET)
    monkeypatch.setattr(pixels, 'STATS', {'upload': 0, 'sixel': 0, 'tiles': 0})
    per_paint = 3 * pixels.SSH_BUDGET / pixels.THROTTLE_HZ             # 3 x the budget at THROTTLE_HZ

    def run(seconds):
        for _ in range(int(seconds * 10)):
            now[0] += 0.1
            hz = fusion.draw_rate(cfg, 'cam')
            pixels.STATS['sixel'] += per_paint * hz * 0.1                  # the bytes that rate sends
        return hz
    run(4 * fusion.RATE_S)
    t0, b0 = now[0], pixels.STATS['sixel']
    run(4 * fusion.RATE_S)
    assert (pixels.STATS['sixel'] - b0) / (now[0] - t0) == pytest.approx(fusion.LINK_SHARE * pixels.SSH_BUDGET, rel=0.02)
    per_paint /= 10
    assert run(4 * fusion.RATE_S) == min(cfg['draw_rate'], pixels.THROTTLE_HZ)


def test_16_colour_ramp_gives_each_band_its_own_hue(monkeypatch):
    """spd.5/spd.4 are both magenta in 16 colours (beside a red buoy's box); the panel's own ramp is not."""
    monkeypatch.setattr(fusion.palette, 'detect_tier', lambda env=None: '16')
    layers = fusion.camera_layers(model()['dets'])
    styles = fusion.Inks(pixels.Raster(4, 2, cell=fusion.TEXT_CELL), layers).styles
    ramp = [styles[f'ramp{i}'] for i in range(len(fusion.RAMP))]
    assert len(set(ramp)) == len(ramp)
    assert theme.PALETTE.style('obj.red', tier='16') not in ramp


def test_16_colour_legend_matches_the_points(monkeypatch, text_tier):
    """The legend took the shared spd tokens (magenta, magenta, yellow, yellow, blue) while the points took RAMP16."""
    monkeypatch.setattr(fusion.palette, 'detect_tier', lambda env=None: '16')
    layers = fusion.camera_layers(model()['dets'])
    styles = fusion.Inks(pixels.Raster(4, 2, cell=fusion.TEXT_CELL), layers).styles
    points = [str(styles[f'ramp{len(fusion.RAMP) - 1 - i}']) for i in range(len(fusion.RAMP) - 1)]
    sub = fusion.camera_subtitle(model(), fusion_feed.load_config())
    legend = [str(span.style) for span in sub.spans if sub.plain[span.start:span.end] == theme.pick('⣿', '#')]
    assert legend == points and len(set(points)) == len(points)


def test_scan_far_from_the_boxes_or_while_turning_is_grey(text_tier):
    """No motion compensation here: a lagged scan sits off its boxes like an extrinsic error, so it is drawn grey
    and the subtitle says how far it is from the boxes."""
    cfg = fusion_feed.load_config()
    fresh = {**model(), 'scan_dt': 0.1, 'motion': False}
    assert not fusion.scan_stale(fresh, cfg)
    assert fusion.scan_stale({**fresh, 'scan_dt': -0.4}, cfg) and fusion.scan_stale({**fresh, 'motion': True}, cfg)

    def look(m):
        fusion._TEXT.clear()
        rows = fusion.camera_rows(m, cfg, 60, 14)
        return {str(span.style) for r in rows for span in r.spans}
    grey, ramp = str(theme.PALETTE.style(fusion.STALE)), str(theme.PALETTE.style('spd.5'))
    assert ramp in look(fresh)
    stale = look({**fresh, 'motion': True})
    assert grey in stale and ramp not in stale

    def subtitle(m):
        return fusion.camera_subtitle(m, cfg).plain
    assert subtitle(fresh).endswith('scan +0.10s')
    assert subtitle({**fresh, 'scan_dt': -0.4}).startswith('STALE scan -0.40s')
    assert subtitle({**fresh, 'motion': True}).startswith('TURNING scan +0.10s')
    stale_ink = fusion.ink_style(fusion.STALE)
    assert stale_ink not in {fusion.ink_style(t) for t in fusion.RAMP}


def test_feed_reports_scan_dt_and_motion():
    now = [0.0]
    f, cb = ready_feed(lambda: now[0])
    f.on_points(cloud([(10.1, 0, -0.1)], t=9.8))
    assert f.snapshot()['scan_dt'] == pytest.approx(-0.2) and not f.snapshot()['motion']
    cb[f.cfg['topics']['sources']](NS(data='{"t": 10, "sources": [], "skip": "motion"}'))
    assert f.snapshot()['motion']


def test_points_per_paint_are_capped_over_a_link_budget(monkeypatch):
    cfg = fusion_feed.load_config()
    assert fusion.link_stride(3000, cfg) == 1
    monkeypatch.setattr(pixels, 'BUDGET', pixels.SSH_BUDGET)
    assert -(-3000 // fusion.link_stride(3000, cfg)) <= cfg['feed']['points_link']
    drawn = []
    raster = pixels.Raster(40, 10, cell=fusion.TEXT_CELL)
    monkeypatch.setattr(raster, 'squares', lambda xs, ys, size, ink: drawn.append((ink, len(xs))))
    m = model()
    n = 3000
    m['points'] = (10.0, np.random.default_rng(1).uniform(0, 1, (n, 2)).astype(np.float32),
                   np.full(n, 7.0, np.float32), np.zeros((n, 2), np.float32))
    inks = fusion.Inks(raster, fusion.camera_layers(m['dets']))
    fusion.paint_camera(m, cfg)(raster, inks)
    points = {inks[name] for name in inks.order if name.startswith(('ramp', 'point'))}
    assert 0 < sum(n for ink, n in drawn if ink in points) <= cfg['feed']['points_link']


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


def test_text_tier_repaints_on_new_data_at_draw_rate_at_most(text_tier, monkeypatch):
    now = [100.0]
    monkeypatch.setattr(pixels, 'clock', lambda: now[0])
    cfg = fusion_feed.load_config()
    m = model()
    a = fusion.camera_rows(m, cfg, 40, 10)
    assert fusion.camera_rows(m, cfg, 40, 10) is a                    # same data: the held rows
    m['version'] = 2
    assert fusion.camera_rows(m, cfg, 40, 10) is a                    # new data within 1/draw_rate
    assert fusion.camera_rows(m, cfg, 41, 10) is not a                # a new size repaints at once
    now[0] += 1.0 / cfg['draw_rate']
    b = fusion.camera_rows(m, cfg, 41, 10)
    m['version'] = 3
    now[0] += 1.0 / cfg['draw_rate']
    assert fusion.camera_rows(m, cfg, 41, 10) is not b


def test_ascii_uses_layer_characters(text_tier, monkeypatch):
    monkeypatch.setattr(theme, 'ASCII', True)
    rows = fusion.camera_rows(model(), fusion_feed.load_config(), 60, 14)
    text = ''.join(r.plain for r in rows)
    assert '#' in text and '.' in text and all(ord(ch) < 128 for ch in text)


def test_ascii_boxes_draw_over_points(text_tier, monkeypatch):
    """At 80 columns a box is a few '#'; points on its corners hid it, so ASCII draws boxes last."""
    monkeypatch.setattr(theme, 'ASCII', True)
    m = model()
    box = m['dets'][0]['box']
    corners = np.array([(u, v) for u in (box[0], box[2] - 0.001) for v in (box[1], box[3] - 0.001)], np.float32)
    m['points'] = (10.0, corners, np.full(4, 7.0, np.float32), np.zeros((4, 2), np.float32))
    rows = fusion.camera_rows(m, fusion_feed.load_config(), 80, 20)
    for u, v in corners:
        assert rows[int(v * 20)].plain[int(u * 80)] == '#'


def test_ascii_around_labels_keep_a_gap(text_tier, monkeypatch):
    monkeypatch.setattr(theme, 'ASCII', True)
    rows = [r.plain for r in fusion.around_rows(model(), fusion_feed.load_config(), 40, 16)]
    line = next(r for r in rows if '7.0' in r)
    i = line.index('7.0')
    assert line[i - 1] == ' ' and line[i - 2] != ' ', line


def test_image_tier_draws_tiles_with_label_holes():
    pixels.use('sixel', cell=(10, 20))
    try:
        rows = fusion.camera_rows(model(), fusion_feed.load_config(), 48, 12)
        text = ''.join(r.plain for r in rows)
        assert any(0xF0000 <= ord(ch) <= 0xFFFFD for ch in text)     # sixel marker cells
        assert '7.0L/7.3S' in text                                    # the label stays terminal text
    finally:
        pixels.use('text')


def test_image_labels_stay_with_their_tile_until_it_repaints(monkeypatch):
    """Between repaints the old tile's holes were filled with the new frame's labels: '22.0S' read ' 22.0'."""
    now = [100.0]
    monkeypatch.setattr(pixels, 'clock', lambda: now[0])
    cfg = fusion_feed.load_config()
    pixels.use('sixel', cell=(10, 20))
    try:
        m = model()
        first = ''.join(r.plain for r in fusion.camera_rows(m, cfg, 48, 12))
        moved = model()
        moved['version'] = 2
        moved['dets'][0] = {**moved['dets'][0], 'box': (0.12, 0.1, 0.17, 0.25), 'range': 4.0}
        held = ''.join(r.plain for r in fusion.camera_rows(moved, cfg, 48, 12))   # within 1/draw_rate: the old tile
        assert held == first and '7.0L/7.3S' in held and '4.0L' not in held
        now[0] += 1.0 / cfg['draw_rate']
        fresh = ''.join(r.plain for r in fusion.camera_rows(moved, cfg, 48, 12))
        assert '4.0L' in fresh and '7.0L' not in fresh
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
    app = NS(tab='Drive', review=None, fusion_feed=fake, facts={}, ros=None, profile=None, fusion_image=None)
    app.fusion_on = lambda: App.fusion_on(app)
    assert App.fusion_view(app) == {} and wants == [False]
    assert fake.applied is None                  # hidden: not even the status is read
    app.tab = 'Fusion'
    assert App.fusion_view(app)['fusion'] == {'x': 1, 'image': True} and wants == [False, True]
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
from mhseals_testing.ui import palette
from mhseals_testing.ui import imaging as pixels
from mhseals_testing.ui.panels import fusion
import test_fusion_panel as T
pixels.use("text")
c = Console(width=100, height=30, file=io.StringIO(), force_terminal=True,
            color_system={"mono": None, "16": "standard", "256": "256", "truecolor": "truecolor"}[palette.detect_tier()])
c.print(fusion.body(T.tab_view(), 100, 30), height=30)
sys.stdout.write(c.file.getvalue())
'''


STALE_SNIPPET = '''
import io, sys
sys.path.insert(0, "test")
from rich.console import Console
from mhseals_testing.ui import palette, theme
from mhseals_testing.ui import imaging as pixels
from mhseals_testing.ui.panels import fusion
import test_fusion_panel as T
pixels.use("text")
w, h = int(sys.argv[1]), int(sys.argv[2])
m = {**T.model(), "scan_dt": 0.12, "motion": True}
c = Console(width=w, height=h, file=io.StringIO(), force_terminal=True,
            color_system={"mono": None, "16": "standard", "256": "256", "truecolor": "truecolor"}[palette.detect_tier()])
c.print(fusion.body({**T.tab_view(), "fusion": m}, w, h), height=h)
styles = {fusion.ink_style(t) for t in fusion.RAMP}
sys.stdout.write(f"{palette.detect_tier()}|{fusion.ink_style(fusion.STALE) in styles}|" + c.file.getvalue())
'''


@pytest.mark.parametrize('env, tier', [({'TERM': 'xterm'}, '16'), ({'TERM': 'xterm-256color', 'NO_COLOR': '1'}, 'mono'),
                                       ({'TERM': 'xterm-256color', 'MHS_ASCII': '1'}, None)])
@pytest.mark.parametrize('size', [(80, 24), (80, 30), (100, 30)])
def test_turning_scan_is_said_at_80_columns(env, tier, size):
    """The word came last in the title row and was cut at 80 and 100 columns; in 16 colours the grey scan was the
    >50 m band's bright_black, and NO_COLOR had no cue. It now leads the title, in an ink no band uses."""
    import os
    import subprocess
    import sys
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    base = {k: v for k, v in os.environ.items() if k not in ('NO_COLOR', 'MHS_ASCII', 'CX_ASCII', 'COLORTERM', 'TERM')}
    base['PYTHONPATH'] = os.pathsep.join([str(root), base.get('PYTHONPATH', '')])
    out = subprocess.run([sys.executable, '-c', STALE_SNIPPET, *map(str, size)], env={**base, **env}, cwd=root,
                         capture_output=True, text=True, timeout=60, check=True).stdout
    got_tier, shared, screen = out.split('|', 2)
    assert tier is None or got_tier == tier
    assert shared == 'False' or got_tier == 'mono'                    # mono: no hue, the word carries it
    title = next(line for line in screen.splitlines() if 'Camera' in line)
    assert 'TURNING' in title, title


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


# --- the camera image (i) ----------------------------------------------------------------------------

def jpeg(size=(128, 72), level=200):
    from PIL import Image
    out = io.BytesIO()
    Image.new('L', size, level).save(out, 'JPEG')
    return out.getvalue()


def camera_with(frame):
    camera = NS(frame=frame, opened=0)
    camera.open, camera.close = (lambda: setattr(camera, 'opened', 1)), (lambda: setattr(camera, 'opened', 0))
    camera.stamped = lambda: camera.frame
    return camera


def test_frame_waits_for_the_boxes_of_its_own_stamp():
    """The fusion publishes a frame's boxes a sweep after the camera: the frame is held until they come, then drawn
    with them (not the newest boxes), so the image, boxes and scan are one capture."""
    now = [0.0]
    camera = camera_with(None)
    f, subs, _ = feed(clock=lambda: now[0], camera=camera)
    f.on_tf_static(statics())
    f.want(True, camera=True)
    cb = by_topic(subs)
    cb[f.cfg['topics']['camera_info'][0]](NS(k=K, width=SIZE[0], height=SIZE[1]))
    frame_of(cb, f, [('a', 7.0, 7.1, 'lidar')], 9.9)
    camera.frame = (jpeg(), 0.0, 10.0)
    snap = f.snapshot()
    assert snap['frame'] is None                     # its boxes (stamp 10.0) have not come yet
    frame_of(cb, f, [('a', 6.0, 6.1, 'lidar', 'green_buoy')], 10.0)
    frame_of(cb, f, [('a', 5.0, 5.1, 'lidar')], 10.1)  # newer boxes: the live view's, not the frame's
    snap = f.snapshot()
    assert snap['frame'].shape == (72, 128) and snap['frame_aspect']
    assert [d['cls'] for d in snap['at_frame']['dets']] == ['green_buoy']
    assert snap['at_frame']['dets'][0]['source'] == 'lidar' and snap['at_frame']['frame_dt'] == 0.0
    assert snap['dets'][0]['range'] == pytest.approx(5.1)


def test_frame_without_boxes_of_its_stamp_takes_the_nearest_after_a_wait():
    now = [0.0]
    camera = camera_with((jpeg(), 0.0, 10.0))
    f, subs, _ = feed(clock=lambda: now[0], camera=camera)
    f.want(True, camera=True)
    cb = by_topic(subs)
    cb[f.cfg['topics']['camera_info'][0]](NS(k=K, width=SIZE[0], height=SIZE[1]))
    cb[f.cfg['topics']['observations']](array([ob('red_buoy', (0.45, 0.4, 0.55, 0.6), (0, 0, 7))], t=10.2))
    assert f.snapshot()['frame'] is None
    now[0] = fusion_feed.RAW_S
    snap = f.snapshot()
    assert snap['frame'] is not None and snap['at_frame']['frame_dt'] == pytest.approx(0.2)


def test_frame_of_another_aspect_is_not_drawn():
    camera = camera_with((jpeg(size=(128, 128)), 0.0, None))
    f, subs, _ = feed(camera=camera)
    f.want(True, camera=True)
    by_topic(subs)[f.cfg['topics']['camera_info'][0]](NS(k=K, width=SIZE[0], height=SIZE[1]))
    snap = f.snapshot()
    assert snap['frame'] is not None and snap['frame_aspect'] is False
    pixels.use('sixel', cell=(10, 20))
    try:
        assert not fusion.frame_shown(snap)
        assert fusion.image_note({**snap, 'image': True}, f.cfg)[0].startswith('frame aspect')
    finally:
        pixels.use('text')


def framed(**extra):
    m = model()
    frame = np.full((72, 128), 200, np.uint8)
    frame[:, :64] = 40
    at = {'dets': [dict(m['dets'][1], box=(0.1, 0.1, 0.2, 0.3))], 'points': None, 'scan_dt': None, 'frame_dt': 0.0}
    return {**m, 'frame': frame, 'frame_aspect': True, 'at_frame': at, 'image': True, **extra}


def camera_raster(m, cols=48, rows=12):
    r = pixels.Raster(cols, rows)
    fusion.paint_camera(fusion.camera_model(m), fusion_feed.load_config())(r, fusion.Inks(r, fusion.camera_layers(m['dets'])))
    return r


def test_image_draws_the_frame_under_its_own_boxes_and_turns_off():
    pixels.use('sixel', cell=(10, 20))
    try:
        m = framed()
        r = camera_raster(m)
        left, right = r.px[5, 5], r.px[5, r.w - 5]
        assert left and right and left != right                        # two grey levels of the frame
        assert fusion.image_note(m, fusion_feed.load_config()) is None
        text = ''.join(row.plain for row in fusion.camera_rows(m, fusion_feed.load_config(), 48, 12))
        assert '12.0R' in text and '7.0L' not in text                   # the frame's boxes, not the live ones
        off = {**m, 'frame': None, 'at_frame': None, 'image': False}      # the feed drops the frame when off
        assert not camera_raster(off).px[5, 5]
        assert fusion.image_note(off, fusion_feed.load_config())[0] == 'image off'
    finally:
        pixels.use('text')


def test_text_tier_says_the_image_needs_graphics(text_tier):
    cfg = fusion_feed.load_config()
    assert fusion.image_note(framed(), cfg)[0] == 'image needs graphics'
    assert 'image needs graphics' in fusion.camera_subtitle(framed(), cfg).plain
    assert not fusion.frame_shown(framed())
    assert fusion.image_note(framed(image=False), cfg)[0] == 'image off'


def test_frame_is_drawn_in_blocks_over_a_link_budget(monkeypatch):
    pixels.use('sixel', cell=(10, 20))
    try:
        m = framed()
        rng = np.random.default_rng(1)
        m['frame'] = rng.integers(0, 256, (72, 128)).astype(np.uint8)
        m['at_frame'] = dict(m['at_frame'], dets=[])
        monkeypatch.setattr(pixels, 'BUDGET', 25000)
        monkeypatch.setattr(fusion, '_STARTED', {'fusion.camera': pixels.GENERATION})
        monkeypatch.setattr(fusion, '_JOINED', {'fusion.camera': (fusion.link_block(), fusion.JOIN_STEPS)})
        monkeypatch.setattr(fusion, '_HZ', {})
        px = camera_raster(m).px
        b = fusion.link_block()
        assert b == fusion.LINK_BLOCKS[0] > 4                       # a link starts at the coarsest blocks
        blocks = px[:px.shape[0] // b * b, :px.shape[1] // b * b].reshape(px.shape[0] // b, b, px.shape[1] // b, b)
        assert (blocks == blocks[:, :1, :, :1]).all()
        monkeypatch.setattr(pixels, 'BUDGET', None)
        px = camera_raster(m).px
        blocks = px[:px.shape[0] // b * b, :px.shape[1] // b * b].reshape(px.shape[0] // b, b, px.shape[1] // b, b)
        assert not (blocks == blocks[:, :1, :, :1]).all()
    finally:
        pixels.use('text')


def test_frame_waits_for_the_tiles_first_repaint_and_comes_in_by_rows(monkeypatch):
    pixels.use('sixel', cell=(10, 20))
    try:
        monkeypatch.setattr(pixels, 'BUDGET', 25000)
        monkeypatch.setattr(fusion, '_STARTED', {})
        monkeypatch.setattr(fusion, '_JOINED', {})
        m = framed()
        m['frame'] = np.full((72, 128), 255, np.uint8)
        assert not fusion.frame_shown(m)                         # the first repaint over a link: no frame yet
        assert not camera_raster(m).px.any()                     # and an empty tile
        assert fusion.frame_shown(m)
        rows = []
        for _ in range(fusion.JOIN_STEPS):
            px = camera_raster(m).px
            rows.append(int(px.any(axis=1).sum()))
        assert rows == sorted(rows) and 0 < rows[0] < rows[-1] and rows[-1] == px.shape[0]    # top rows first
        assert camera_raster(m).px.any(axis=1).all()
    finally:
        pixels.use('text')


def test_image_off_draws_the_points_at_once_over_a_link(monkeypatch):
    pixels.use('sixel', cell=(10, 20))
    try:
        monkeypatch.setattr(pixels, 'BUDGET', 25000)
        monkeypatch.setattr(fusion, '_STARTED', {})
        monkeypatch.setattr(fusion, '_JOINED', {})
        m = model()
        m['points'] = (0.0, np.array([[0.5, 0.5]], np.float32), np.array([5.0], np.float32), np.array([[0, 5]], np.float32))
        m['image'] = False
        assert camera_raster(m).px.any()                         # points on the first repaint, no image to wait for
        assert fusion.image_note(m, fusion_feed.load_config())[0] == 'image off'
    finally:
        pixels.use('text')


def test_title_says_loading_until_the_frame_is_complete(monkeypatch):
    pixels.use('sixel', cell=(10, 20))
    try:
        cfg = fusion_feed.load_config()
        monkeypatch.setattr(pixels, 'BUDGET', 25000)
        monkeypatch.setattr(fusion, '_STARTED', {})
        monkeypatch.setattr(fusion, '_JOINED', {})
        m = framed()
        notes = []
        for _ in range(fusion.JOIN_STEPS + 2):
            notes.append((fusion.image_note(m, cfg) or ('',))[0])
            camera_raster(m)
        assert notes[:fusion.JOIN_STEPS + 1] == ['image loading'] * (fusion.JOIN_STEPS + 1)
        assert notes[-1] == ''
    finally:
        pixels.use('text')


def test_frame_blocks_follow_the_size_of_a_repaint(monkeypatch):
    cfg = fusion_feed.load_config()
    now = [100.0]
    monkeypatch.setattr(fusion, '_HZ', {})
    monkeypatch.setattr(pixels, 'clock', lambda: now[0])
    monkeypatch.setattr(pixels, 'BUDGET', pixels.SSH_BUDGET)
    monkeypatch.setattr(pixels, 'STATS', {'upload': 0, 'sixel': 0, 'tiles': 0})
    monkeypatch.setattr(pixels, '_sizes', {})
    assert fusion.link_block() == fusion.LINK_BLOCKS[0]
    fusion.draw_rate(cfg, 'fusion.camera')
    for size, level in ((0.1 * pixels.SSH_BUDGET, 1), (0.1 * pixels.SSH_BUDGET, 2),          # fits four times: finer
                        (0.9 * pixels.SSH_BUDGET, 1), (0.9 * pixels.SSH_BUDGET, 0), (0.9 * pixels.SSH_BUDGET, 0)):  # too big
        now[0] += fusion.RATE_S
        pixels.STATS['sixel'] += pixels.SSH_BUDGET
        pixels._sizes['fusion.camera'] = size
        fusion.draw_rate(cfg, 'fusion.camera')
        assert fusion._HZ['fusion.camera'][3] == level
    monkeypatch.setattr(pixels, 'BUDGET', None)
    assert fusion.link_block() == 1


def test_i_toggles_the_image_and_the_feed_follows(monkeypatch):
    from mhseals_testing.ui.app import App
    camera = []
    fake = NS(want=lambda on, camera_=False, **kw: camera.append(kw.get('camera')), snapshot=lambda: {}, applied=None)
    app = NS(tab='Fusion', review=None, fusion_feed=fake, facts={}, ros=None, profile=None, fusion_image=None)
    app.fusion_on = lambda: App.fusion_on(app)
    monkeypatch.setattr(pixels, 'active', lambda: True)
    monkeypatch.setattr(pixels, 'images', lambda name: True)
    monkeypatch.setattr(pixels, 'BUDGET', None)
    assert App.fusion_view(app)['fusion']['image'] is True and camera[-1] is True     # on by default locally
    monkeypatch.setattr(pixels, 'BUDGET', 25000)
    assert App.fusion_view(app)['fusion']['image'] is False and camera[-1] is False   # off by default over SSH
    app.fusion_image = True                                                            # turned on: kept
    assert App.fusion_view(app)['fusion']['image'] is True and camera[-1] is True

"""fusion_calib on synthetic buoy scans: fit recovery, observability, verdicts, range ratio."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

lf = pytest.importorskip('mhseals_nav.lidar_fusion')
from mhseals_testing.perception import fusion_calib as fc  # noqa: E402

K = (640.0, 640.0, 640.0, 360.0)
IMAGE = (1280, 720)
DEG = math.pi / 180
R_BUOY, Y_TOP, Y_BOT = 0.2, 0.4, 1.0            # optical Y down: 1 m mast, 0.6 m buoy
RING_DEG = 2.0
AZ = np.radians(np.arange(-45, 45, 0.2))


@pytest.fixture(scope='module')
def cfg():
    return fc.load_config()


def place(rho, bearing_deg):
    return rho * math.sin(bearing_deg * DEG), rho * math.cos(bearing_deg * DEG)


def scan(buoys, phase):
    """Optical-axes hits on vertical cylinders (X, Z), rings every RING_DEG from phase."""
    rings = np.radians(np.arange(-15, 15, RING_DEG) + phase)
    a, e = (m.ravel() for m in np.meshgrid(AZ, rings))
    dh = np.stack([np.sin(a), np.cos(a)], 1)
    best = np.full(len(a), np.inf)
    for X, Z in buoys:
        c = np.array([X, Z])
        b = dh @ c
        disc = b ** 2 - (c @ c - R_BUOY ** 2)
        t = np.where(disc >= 0, b - np.sqrt(np.maximum(disc, 0)), np.inf)
        y = -np.tan(e) * t
        best = np.where((t > 0) & (y >= Y_TOP) & (y <= Y_BOT) & (t < best), t, best)
    ok = np.isfinite(best)
    t = best[ok]
    return np.stack([np.sin(a[ok]) * t, -np.tan(e[ok]) * t, np.cos(a[ok]) * t], 1)


def observation(X, Z, rng, bid='', rho_scale=1.0, box_px=1.0):
    fx, fy, cx, cy = K
    W, H = IMAGE
    az, rho = math.atan2(X, Z), math.hypot(X, Z)
    half, near = math.asin(R_BUOY / rho), rho - R_BUOY
    edges = np.array([cx + fx * math.tan(az - half), cy + fy * Y_TOP / near,
                      cx + fx * math.tan(az + half), cy + fy * Y_BOT / near])
    edges += rng.normal(0, box_px, 4)
    p = np.array([X, (Y_TOP + Y_BOT) / 2, Z]) * rho_scale * (1 + rng.normal(0, 0.005))
    return NS(class_id='red_buoy', backend_id=bid, position=NS(x=p[0], y=p[1], z=p[2]),
              position_covariance=[0.0] * 9, has_box2d=True,
              box2d=list(edges / [W, H, W, H]))


def frames(buoys, w_deg=(0, 0, 0), c=(0, 0, 0), seconds=30.0, hz=5.0, seed=0, **obs):
    """Frames where the lidar sits at R(w) P + c in the camera's optical frame."""
    rng = np.random.default_rng(seed)
    R = lf.small_rotation(np.radians(w_deg))
    out = []
    for i in range(int(seconds * hz)):
        P = scan(buoys, rng.uniform(0, RING_DEG))
        P += rng.normal(0, 0.02, P.shape) * P / np.linalg.norm(P, axis=1)[:, None]
        obs_i = [observation(X, Z, rng, **obs) for X, Z in buoys]
        out.append(fc.Frame(i / hz, obs_i, P @ R.T + np.asarray(c), K, IMAGE))
    return out


SPREAD = [place(*rb) for rb in ((4, -25), (6, 12), (8, -6), (10, 28), (12, -16), (15, 4))]
NEAR = [place(*rb) for rb in ((5, -8), (7, 6), (9, -2))]


def fit(cfg, fr):
    rows, est, sample_t = fc.measure(fr, cfg)
    return rows, fc.calibrate(est, sample_t, cfg), est


def test_injected_rotation_is_recovered_within_two_se_and_the_se_is_honest(cfg):
    truth = {'pitch_deg': -0.5, 'yaw_deg': 1.0, 'roll_deg': 0.0}
    rows, params, est = fit(cfg, frames(SPREAD, (-0.5, 1.0, 0.0)))
    assert len({r['key'] for r in rows}) == 6 and all(r['hit'] for r in rows)
    for name, value in truth.items():
        p = params[name]
        assert p['constrained'], name
        assert abs(p['value'] - value) < 2 * p['se'], (name, p)
    # The bootstrap sees scatter across objects and time only. These buoys share no bias
    # beyond the box-vs-ring geometry common to all (the cluster's mean row sits above the
    # silhouette's centre, ~0.2 deg pitch at 4-6 m), so its interval is far narrower than the
    # GLS SE with its per-object bias term, and need not cover the truth.
    for p in params.values():
        assert p['ci95'][0] <= p['value'] + 1e-3 and p['value'] - 1e-3 <= p['ci95'][1] or \
            not p['constrained']
    # Honest: the scatter over independent recordings is no larger than the reported SE.
    yaw, se = [], []
    for seed in range(1, 9):
        _, p, _ = fit(cfg, frames(SPREAD, (-0.5, 1.0, 0.0), seconds=10, seed=seed))
        yaw.append(p['yaw_deg']['value'])
        se.append(p['yaw_deg']['se'])
    assert np.std(yaw, ddof=1) < 1.5 * np.mean(se)
    assert abs(np.mean(yaw) - 1.0) < 2 * np.mean(se)
    rec = fc.record(rows, fc.summarize(rows, 10.0), est, params, 'synthetic')
    assert rec['n_objects'] == 6 and rec['n_samples'] == len(est.samples)
    roll, pitch, yaw_d = rec['suggested_camera_delta']['rpy_deg']   # camera body axes
    assert (roll, pitch, yaw_d) == pytest.approx(
        (params['roll_deg']['value'], -params['pitch_deg']['value'],
         -params['yaw_deg']['value']), abs=1e-3)


def test_one_centred_buoy_at_one_range_leaves_roll_unconstrained(cfg):
    rows, params, est = fit(cfg, frames([place(8, 0)]))
    assert not params['roll_deg']['constrained']
    assert params['roll_deg']['se'] > 0.5 * params['roll_deg']['prior_se']
    assert fc.camera_delta(params)['rpy_deg'][0] is None
    assert len(est.samples) > 30
    for name in ('cx_m', 'cy_m', 'cz_m'):         # one range cannot separate translation
        assert not params[name]['constrained'], name
    assert fc.camera_delta(params)['xyz_m'] == [None, None, None]
    objects = fc.summarize(rows, cfg['check']['near_m'])
    lines = fc.fit_lines(params, objects)
    assert any('aliased' in x and 'yaw' in x for x in lines)
    assert any('GLS SE is the bound' in x for x in lines)
    assert fc.ALIAS_CAVEAT in fc.record(rows, objects, est, params, 'b')['caveats']


def test_spread_ranges_drop_the_aliasing_note_but_keep_the_bootstrap_caveat(cfg):
    rows, params, _ = fit(cfg, frames(SPREAD, seconds=6))
    lines = fc.fit_lines(params, fc.summarize(rows, cfg['check']['near_m']))
    assert not any('aliased' in x for x in lines)
    assert any('GLS SE is the bound' in x for x in lines)


@pytest.mark.parametrize('yaw, level', [(0.0, fc.PASS), (0.7, fc.WARN), (2.0, fc.FAIL)])
def test_check_verdicts_follow_the_injected_yaw(cfg, yaw, level):
    base_rows, base, _ = fit(cfg, frames(NEAR, seed=5))
    baseline = {'params': base}
    rows, params, _ = fit(cfg, frames(NEAR, (0.0, yaw, 0.0), seed=6))
    objects = fc.summarize(rows, cfg['check']['near_m'])
    got, reasons, levels = fc.verdict(objects, params, cfg, baseline)
    assert got == level, (reasons, [round(o['du_med'], 1) for o in objects])
    if level == fc.WARN:
        assert any('vs baseline' in r for r in reasons)
    assert len(fc.table(objects, levels).splitlines()) == 1 + len(NEAR)


def test_zero_hits_on_a_near_box_fails(cfg):
    fr = [f._replace(P=np.zeros((0, 3))) for f in frames(NEAR, seconds=2)]
    rows, params, _ = fit(cfg, fr)
    level, reasons, _ = fc.verdict(fc.summarize(rows, 10.0), params, cfg)
    assert params is None and level == fc.FAIL and 'no lidar hit' in reasons[0]


def test_range_ratio_uses_the_near_face_offset_and_tape(cfg):
    X, Z = place(6, 5)
    rows, _, _ = fit(cfg, frames([(X, Z)], seconds=4, bid='b1'))
    o, = fc.summarize(rows, 10.0)
    face = np.median([r['rho_face'] / r['rho_stereo'] for r in rows])
    assert face < 0.985                       # the near face alone reads short
    assert o['ratio'] == pytest.approx(1.0, abs=0.01)
    assert np.median([r['rho_lidar'] - r['rho_face'] for r in rows]) == \
        pytest.approx(0.67 * R_BUOY)          # red_buoy: near_face_k buoy 0.67
    assert fc.verdict([o], None, cfg, truth={'b1': 6.0})[0] == fc.PASS
    level, reasons, _ = fc.verdict([o], None, cfg, truth={'b1': 6.6})
    assert level == fc.WARN and 'tape' in reasons[0]          # 9% off: WARN
    level, reasons, _ = fc.verdict([o], None, cfg, truth={'b1': 7.0})
    assert level == fc.FAIL and 'tape' in reasons[0]          # 14% off: FAIL
    long_rows, _, _ = fit(cfg, frames([(X, Z)], seconds=4, bid='b1', rho_scale=1.08))
    level, reasons, _ = fc.verdict(fc.summarize(long_rows, 10.0), None, cfg)
    assert level == fc.WARN and 'lidar/stereo 0.9' in reasons[0]
    far_rows, _, _ = fit(cfg, frames([(X, Z)], seconds=4, bid='b1', rho_scale=1.15))
    level, reasons, _ = fc.verdict(fc.summarize(far_rows, 10.0), None, cfg)
    assert level == fc.FAIL and 'lidar/stereo 0.8' in reasons[0]


@pytest.mark.parametrize('pitch, level', [(0.0, fc.PASS), (0.6, fc.WARN), (1.5, fc.FAIL)])
def test_check_reads_the_vertical_offset_dv(cfg, pitch, level):
    rows, _, _ = fit(cfg, frames(NEAR, (pitch, 0.0, 0.0), seconds=4, seed=7))
    objects = fc.summarize(rows, cfg['check']['near_m'])
    got, reasons, _ = fc.verdict(objects, None, cfg)
    assert got == level, (reasons, [round(o['dv_med'], 1) for o in objects])
    assert level == fc.PASS or all(r.split(': ')[1].startswith('dv') for r in reasons)


def test_key_less_detections_group_by_class_and_bearing():
    keys = fc.Keys(3.0)
    ob = NS(backend_id='', class_id='red_buoy')
    assert keys(ob, 0.10) == keys(ob, 0.10 + 2 * DEG) != keys(ob, 0.10 + 5 * DEG)
    assert keys(NS(backend_id='7', class_id='red_buoy'), 0.1) == '7'


# ------------------------------------------------------------------ bag front end (fake reader)
def header(t, frame=''):
    return NS(stamp=NS(sec=int(t), nanosec=int(round((t % 1) * 1e9))), frame_id=frame)


def tf(parent, child, t=0.0, xyz=(0, 0, 0), q=(0, 0, 0, 1)):
    return NS(header=header(t, parent), child_frame_id=child,
              transform=NS(translation=NS(**dict(zip('xyz', xyz))),
                           rotation=NS(**dict(zip('xyzw', q)))))


def cloud(t, points):
    pts = np.asarray(points, np.float32)
    fields = [NS(name=n, offset=4 * i, datatype=7) for i, n in enumerate('xyz')]
    return NS(header=header(t, 'lidar_link'), fields=fields, width=len(pts), height=1,
              is_bigendian=False, point_step=12, data=pts.tobytes())


def fake_bag(with_info=True, urdf_yaw_deg=0.0, odom=False):
    """Lidar 0.3 m above the camera; the optical frame per REP 103 (q of body->optical).

    The static TF may yaw the camera by urdf_yaw_deg (base z) off its true pose; odom adds
    a still odom->base_link /tf at 50 Hz.
    """
    cam = 'front_camera_optical_frame'
    q_opt = (-0.5, 0.5, -0.5, 0.5)
    h = urdf_yaw_deg * DEG / 2
    q_tf = (-0.5 * math.cos(h) - 0.5 * math.sin(h), 0.5 * math.cos(h) - 0.5 * math.sin(h),
            -0.5 * math.cos(h) + 0.5 * math.sin(h), 0.5 * math.cos(h) + 0.5 * math.sin(h))
    msgs = [('/tf_static', NS(transforms=[tf('base_link', 'lidar_link', xyz=(0, 0, 1.3)),
                                          tf('base_link', cam, xyz=(0, 0, 1.0), q=q_tf)]))]
    if with_info:
        msgs.append(('/front/zed_node/rgb/color/rect/camera_info',
                     NS(k=[K[0], 0, K[2], 0, K[1], K[3], 0, 0, 1], width=IMAGE[0],
                        height=IMAGE[1])))
    if odom:
        msgs += [('/tf', NS(transforms=[tf('odom', 'base_link', t=99 + i * 0.02)]))
                 for i in range(300)]
    rng = np.random.default_rng(3)
    R_bo = lf.quat_matrix(*q_opt)
    for i in range(20):
        t = 100 + i * 0.2
        msgs.append(('/perception/observations', NS(
            header=header(t, cam), observations=[observation(X, Z, rng) for X, Z in NEAR])))
        P = scan(NEAR, rng.uniform(0, RING_DEG))
        msgs.append(('/points', cloud(t + 0.03, P @ R_bo.T + [0, 0, 1.0 - 1.3])))
    return msgs


@pytest.fixture
def bag_reader(monkeypatch):
    def use(msgs):
        def read(path, topics=None, **_):
            for topic, msg in msgs:
                if topics is None or topic in topics:
                    yield topic, msg, 0
        monkeypatch.setattr(fc.bagread, 'read', read)
    return use


def test_bag_front_end_projects_through_static_tf_and_checks(cfg, bag_reader, capsys, tmp_path):
    bag_reader(fake_bag())
    frames_, counts = fc.frames_from_bag('bag', cfg)
    assert counts == {'frames': 20, 'no_cloud': 0, 'compensated': 0, 'static_only': 20}
    rows, _, _ = fc.measure(frames_, cfg)
    assert len(rows) == 60 and all(r['hit'] for r in rows)
    assert np.median(np.abs([r['du'] for r in rows])) < 2.0
    assert fc.main(['check', 'bag']) == fc.PASS
    assert 'VERDICT PASS' in capsys.readouterr().out
    assert fc.main(['calibrate', 'bag', '--out', str(tmp_path)]) == 0
    assert (tmp_path / 'calibration.yaml').exists() and (tmp_path / 'detections.csv').exists()
    bag_reader(fake_bag(odom=True))
    still, counts = fc.frames_from_bag('bag', cfg)
    assert counts['compensated'] == 20
    assert np.allclose(still[0].P, frames_[0].P, atol=1e-5)


def test_bag_without_camera_info_errors_clearly(cfg, bag_reader, capsys):
    bag_reader(fake_bag(with_info=False))
    with pytest.raises(LookupError, match='no camera_info'):
        fc.frames_from_bag('bag', cfg)
    assert fc.main(['check', 'bag']) == 3
    assert 'no camera_info' in capsys.readouterr().err


def test_the_suggested_delta_undoes_a_camera_yaw_error_in_the_static_tf(cfg, bag_reader):
    bag_reader(fake_bag(urdf_yaw_deg=0.8))
    rows, est, sample_t = fc.measure(fc.frames_from_bag('bag', cfg)[0], cfg)
    delta = fc.camera_delta(fc.calibrate(est, sample_t, cfg))
    assert delta['rpy_deg'][2] == pytest.approx(-0.8, abs=0.1)

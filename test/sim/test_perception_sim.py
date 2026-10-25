"""Live metamorphic perception test (perception-v2 §8) on the crane rig, one session per mode.

Schedule ``metamorphic``: 0-60 s baseline, then bias 5 deg, lag 0.1 s, extrinsic 3 deg plus
stamp 0.1 s, lag 0.3 s. R1 and R2 are asserted on phases 1-4 and R4 on phase 5, from the
session's perception bag and course_truth.json. Tracks are in ``map``; a rigid fit of
/odom/global onto /crane/odom carries them into the truth (world) frame.
The analysis helpers are pure and are checked here without the rig.
"""
from fnmatch import fnmatch
import json
import math
import os
import uuid

import numpy as np
import pytest
from simrig import PERCEPTION, read_bag, run_perception_case
import yaml

CFG = yaml.safe_load(PERCEPTION.read_text())
MOVING, STATIC = 2, 1          # mhseals_nav/Track MOTION_*
OK, WARN = 0, 1                # diagnostic_msgs/DiagnosticStatus levels


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def fit_rigid(src, dst):
    """(R 2x2, t) with dst ~ R @ src + t in the least-squares sense (2D Kabsch)."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    a, b = src - src.mean(0), dst - dst.mean(0)
    u, _, vt = np.linalg.svd(a.T @ b)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    R = vt.T @ np.diag([1.0, d]) @ u.T
    return R, dst.mean(0) - R @ src.mean(0)


def nearest(times, t):
    i = int(np.clip(np.searchsorted(times, t), 1, len(times) - 1))
    return i if abs(times[i] - t) < abs(times[i - 1] - t) else i - 1


def phase_start(inject):
    """Sim time the case clock started, from the inject echo rows (t, values)."""
    return float(np.median([t - float(v['elapsed_s']) for t, v in inject
                            if v.get('case') == CFG['case']]))


def static_truth(truth):
    return [(i, e['label'], np.array([e['x'], e['y']])) for i, e in enumerate(truth['elements'])
            if any(fnmatch(e.get('label', ''), p) for p in CFG['static_classes'])]


def matched(label, xy, statics):
    """Index of the same-class static truth element within match_m, else None."""
    best = min(((np.hypot(*(xy - p)), i) for i, c, p in statics if c == label), default=None)
    return best[1] if best and best[0] <= CFG['match_m'] else None


def in_view(pose, xy):
    x, y, heading = pose
    d = np.hypot(xy[0] - x, xy[1] - y)
    bearing = (math.atan2(xy[1] - y, xy[0] - x) - heading + math.pi) % (2 * math.pi) - math.pi
    return d <= CFG['fov']['range_m'] and abs(bearing) <= math.radians(CFG['fov']['hfov_deg']) / 2


def analyse(tracks, inject, heading, truth_poses, statics):
    """R1 violations, R2 misses per phase and the heading rows per phase.

    ``tracks``: [(t, [(id, class_id, motion, world xy)])]; ``inject``: [(t, values)];
    ``heading``: [(t, level, values)]; ``truth_poses``: [(t, x, y, yaw)] world.
    """
    start, length = phase_start(inject), CFG['phase_s']

    def phase(t):
        return int((t - start) // length)

    r1 = [(round(t - start, 1), ident, m) for t, rows in tracks if 0 <= phase(t) < 4
          for ident, label, motion, xy in rows
          if motion == MOVING and (m := matched(label, xy, statics)) is not None]
    dt = float(np.median(np.diff([p[0] for p in truth_poses])))
    seen = {}
    for t, x, y, h in truth_poses:
        for i, _, p in statics:
            if 0 <= phase(t) < 4 and in_view((x, y, h), p):
                seen[phase(t), i] = seen.get((phase(t), i), 0.0) + dt
    held = {(phase(t), m) for t, rows in tracks if 0 <= phase(t) < 4
            and (t - start) % length >= length - CFG['settle_s']
            for _, label, motion, xy in rows
            if motion == STATIC and (m := matched(label, xy, statics)) is not None}
    r2 = sorted(k for k, s in seen.items() if s >= CFG['in_view_s'] and k not in held)
    by_phase = {}
    for t, level, values in heading:
        by_phase.setdefault(phase(t), []).append((t, level, values))
    return r1, r2, by_phase


def load_session(session):
    """Bag rows as ``analyse`` takes them, with tracks carried from map into world."""
    bag = read_bag(session / 'perception', CFG['record'])
    truth = json.loads((session / 'course_truth.json').read_text())
    world = [(stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y,
              yaw(m.pose.pose.orientation)) for m in bag['/crane/odom']]
    wt = np.array([w[0] for w in world])
    pairs = [((m.pose.pose.position.x, m.pose.pose.position.y),
              world[nearest(wt, stamp(m.header))][1:3]) for m in bag['/odom/global']]
    R, t = fit_rigid(*zip(*pairs))
    tracks = [(stamp(a.header), [(k.id, k.class_id, k.motion,
                                  R @ [k.pose.pose.position.x, k.pose.pose.position.y] + t)
                                 for k in a.tracks]) for a in bag['/perception/tracks']]
    inject, heading = [], []
    for a in bag['/perception/status']:
        for s in a.status:
            values = {v.key: v.value for v in s.values}
            if s.name == 'perception/sim/inject':
                inject.append((stamp(a.header), values))
            elif s.name == 'perception/sim/heading':
                heading.append((stamp(a.header), s.level[0] if isinstance(s.level, bytes)
                                else int(s.level), values))
    return analyse(tracks, inject, heading, world, static_truth(truth))


# ----------------------------------------------------------- pure checks (no rig needed)

def test_rigid_fit_recovers_a_map_to_world_transform():
    rng = np.random.default_rng(3)
    src = rng.uniform(-20, 20, (50, 2))
    c, s = math.cos(1.1), math.sin(1.1)
    R = np.array([[c, -s], [s, c]])
    Rf, tf = fit_rigid(src, src @ R.T + [4.0, -7.0])
    assert Rf == pytest.approx(R) and tf == pytest.approx([4.0, -7.0])


def test_analysis_flags_a_moving_buoy_and_an_unheld_one():
    statics = [(0, 'red_buoy', np.array([10.0, 0.0])), (1, 'green_buoy', np.array([10.0, 4.0]))]
    inject = [(100.0 + k, {'case': CFG['case'], 'elapsed_s': str(k)}) for k in range(300)]
    poses = [(100.0 + k / 10, 0.0, 0.0, 0.0) for k in range(3000)]
    tracks = [(100.0 + k / 10, [(1, 'red_buoy', MOVING if k == 700 else STATIC,
                                 np.array([10.2, 0.1]))]) for k in range(3000)]
    heading = [(350.0, WARN, {'lag_s': '0.28'})]
    r1, r2, by_phase = analyse(tracks, inject, heading, poses, statics)
    assert r1 == [(70.0, 1, 0)]
    assert r2 == [(p, 1) for p in range(4)]          # green in view, never held
    assert by_phase[4] == heading


def test_perception_command_sets_the_mode_and_case_after_the_node_appears():
    from simrig import perception_command
    lines = perception_command('zed', 'metamorphic', dict(CFG, seconds=300)).splitlines()
    wait = next(i for i, l in enumerate(lines) if '/sim_backend && break' in l)
    assert 'emulate zed' in lines[wait + 1] and 'case metamorphic' in lines[wait + 2]
    assert sum(r[0] for r in json.loads(lines[-2].split(' ', 3)[3].strip("'"))) >= 300


# ------------------------------------------------------------------- live (needs $SIMRIG)

@pytest.fixture(scope='module', params=['plain', 'zed'])
def perception(request, rig, tmp_path_factory):
    """Record a metamorphic session per mode; $SIMRIG_OVERLAY names a v2 mhseals_nav install."""
    domain = 200 + uuid.uuid4().int % 29
    out = tmp_path_factory.mktemp(f'perception_{request.param}') / 's'
    session = run_perception_case(rig, out, request.param, CFG['case'], CFG['seconds'],
                                  domain=domain, overlay=os.environ.get('SIMRIG_OVERLAY'))
    pytest.importorskip('rosbag2_py')
    return request.param, load_session(session)


def test_r1_no_static_buoy_is_ever_moving(perception):
    mode, (r1, _, _) = perception
    assert r1 == [], f'{mode}: (s into case, track, truth) MOVING: {r1[:10]}'


def test_r2_every_buoy_in_view_holds_a_static_track(perception):
    mode, (_, r2, _) = perception
    assert r2 == [], f'{mode}: (phase, truth) in view without a STATIC track: {r2}'


def test_r4_heading_is_ok_at_baseline_and_flags_the_lag(perception):
    mode, (_, _, by_phase) = perception
    end = CFG['phase_s'] - CFG['settle_s']
    assert by_phase.get(0) and all(level == OK for _, level, _ in by_phase[0])
    late = [(level, float(v['lag_s'])) for t, level, v in by_phase.get(4, [])
            if v.get('excited') == 'true' and t >= by_phase[4][0][0] + end]
    assert late, f'{mode}: no excited heading result late in phase 5'
    level, lag = late[-1]
    assert level >= WARN and abs(lag - CFG['r4']['lag_s']) <= CFG['r4']['tolerance_s'], late[-1]

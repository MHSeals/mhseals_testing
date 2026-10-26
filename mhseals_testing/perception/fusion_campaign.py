"""Lidar fusion campaign (mhseals_nav lidar-fusion-design.md §4): replay and metrics.

  replay SESSION NAME OUT   play a crane session under its own sim clock with injection NAME
                            (config/fusion_campaign.yaml) into sim_backend -> lidar_fusion and
                            two trackers (raw = before, fused = after); OUT/metrics.json
  report OUT...             one row per replay

Crane records no /clock and no EKF TF: /clock follows the header stamps and odom->base_link is
built from /crane/odom (truth), to which the TF injections apply. Run in a ROS container with
mhseals_nav (lidar-fusion) sourced and ROS_DOMAIN_ID set.
"""

import argparse
import gzip
import heapq
import json
import math
import os
from pathlib import Path
import random
import subprocess
import time

import numpy as np
import yaml

CONFIG = Path(__file__).resolve().parents[2] / 'config' / 'fusion_campaign.yaml'
# Crane's clock starts near 0 and the tracker looks up TF at t - 0.2 s (negative Time): every
# replayed stamp and the clock move by this much.
CLOCK_OFFSET = 100.0
TOPICS = ('/crane/odom', '/points', '/detections', '/imu/raw', '/front_camera/camera_info',
          '/tf_static')


def rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (f(a) for a in (roll, pitch, yaw) for f in (math.cos, math.sin))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat(R):
    w = math.sqrt(max(1e-12, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    return ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w),
            (R[1, 0] - R[0, 1]) / (4 * w), w)


def matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def cloud_edit(msg, inj, rng, T_bl, water_z_base, bow_x):
    """Lidar-side injections on one PointCloud2 (lidar_link): extrinsic, water, spray."""
    step = msg.point_step
    n = msg.width * msg.height
    rows = np.frombuffer(bytes(msg.data), np.uint8).reshape(n, step).copy()
    off = {f.name: f.offset for f in msg.fields}
    xyz = np.stack([rows[:, off[a]:off[a] + 4].copy().view('<f4')[:, 0] for a in 'xyz'], 1)
    if 'rpy_deg' in inj or 'xyz_m' in inj:
        R = rpy(*np.radians(inj.get('rpy_deg', [0, 0, 0])))
        xyz = xyz @ R.T + np.asarray(inj.get('xyz_m', [0, 0, 0]), float)
    extra = []
    R_bl, t_bl = T_bl
    if inj.get('water'):
        k = int(inj['water'])
        r, a = rng.uniform(2, 6, k), rng.uniform(-1.0, 1.0, k)
        pb = np.stack([r * np.cos(a), r * np.sin(a), np.full(k, water_z_base)], 1)
        extra.append((pb - t_bl) @ R_bl)
    if inj.get('spray'):
        k = int(rng.integers(*inj['spray']))
        d = rng.uniform(0.5, 4.0, k)
        pb = np.stack([bow_x + d, d * np.tan(rng.uniform(-0.5, 0.5, k)),
                       water_z_base + rng.uniform(0.0, 1.0, k)], 1)
        extra.append((pb - t_bl) @ R_bl)
    if extra:
        add = np.vstack(extra)
        rows = np.vstack([rows, np.tile(rows[:1], (len(add), 1))])
        if 'time' in off:
            rows[n:, off['time']:off['time'] + 4] = rng.uniform(-0.1, 0, len(add)).astype(
                '<f4').view(np.uint8).reshape(-1, 4)
        xyz = np.vstack([xyz, add])
    for i, a in enumerate('xyz'):
        rows[:, off[a]:off[a] + 4] = xyz[:, i].astype('<f4').view(np.uint8).reshape(-1, 4)
    msg.width, msg.height = len(rows), 1
    msg.row_step = step * len(rows)
    msg.data = rows.tobytes()
    return msg


def nodes(session, inj, cfg, share):
    """ros2 run commands: sim_backend, lidar_fusion and the raw and fused trackers."""
    cam = {**cfg['camera'], **inj.get('camera', {})}
    if inj.get('merge'):
        cam['merge_gap_px'] = cfg['merge_gap_px']
    sim = ['-p', 'use_sim_time:=true']
    backend = ['ros2', 'run', 'mhseals_nav', 'sim_backend', '--ros-args', '--params-file',
               f'{share}/sim_backend.yaml', *sim, '-p', f'emulate:={cam.pop("emulate")}',
               '-p', f'course_truth:={session}/course_truth.json']
    for k, v in cam.items():
        backend += ['-p', f'inject.{k}:={json.dumps(v)}']
    fusion = ['ros2', 'run', 'mhseals_nav', 'lidar_fusion', '--ros-args', '--params-file',
              f'{share}/lidar_fusion.yaml', *sim, '-p', 'points_transport:=direct',
              '-p', 'camera_axes:=body', '-p', 'camera_info_topic:=/front_camera/camera_info',
              '-p', 'attitude_min_hz:=15.0']
    trackers = []
    fused = ['-p', 'observations_topic:=/perception/observations_fused',
             '-p', 'meas.depth_term:=false']
    for name, extra in (('raw', []), ('fused', fused)):
        trackers.append(['ros2', 'run', 'mhseals_nav', 'object_tracker', '--ros-args',
                         '--params-file', f'{share}/tracker.yaml', *sim,
                         '-p', 'course_map.load:=false', '-p', 'course_map.save_period_s:=1e6',
                         *extra, '-r', f'__node:=tracker_{name}',
                         '-r', f'/perception/tracks:=/campaign/tracks_{name}',
                         '-r', f'/perception/status:=/campaign/status_{name}'])
    return {'sim_backend': backend, 'lidar_fusion': fusion, 'tracker_raw': trackers[0],
            'tracker_fused': trackers[1]}


class Player:
    """Publish the session in header-stamp order, paced at `rate`, and record the outputs."""

    def __init__(self, session, inj, cfg, rate):
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage
        from rosidl_runtime_py.utilities import get_message
        from mhseals_nav.msg import ObservationArray, TrackArray
        from diagnostic_msgs.msg import DiagnosticArray
        from std_msgs.msg import String
        rclpy.init()
        self.rclpy, self.get_message = rclpy, get_message
        self.node = rclpy.create_node('fusion_campaign')
        self.session, self.inj, self.cfg, self.rate = Path(session), inj, cfg, rate
        self.lidar = inj.get('lidar', {})
        self.rng = np.random.default_rng(7)
        self.Clock, self.TF = Clock, TFMessage
        best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pubs = {'/clock': self.node.create_publisher(Clock, '/clock', 10),
                     '/tf': self.node.create_publisher(TFMessage, '/tf', 100),
                     '/tf_static': self.node.create_publisher(TFMessage, '/tf_static', latched)}
        self.types = {}
        self.best = QoSProfile(depth=10)          # reliable: serves both kinds of reader
        self.out = {k: [] for k in ('before', 'fused', 'sources', 'tracks_raw', 'tracks_fused',
                                    'status', 'truth', 'map_base')}
        sub = self.node.create_subscription
        sub(ObservationArray, '/perception/observations', lambda m: self.out['before'].append(
            self.obs_rows(m)), best)
        sub(ObservationArray, '/perception/observations_fused', lambda m: self.out['fused'].append(
            self.obs_rows(m)), best)
        sub(String, '/perception/fusion_sources', lambda m: self.out['sources'].append(
            json.loads(m.data)), 50)
        for name in ('raw', 'fused'):
            sub(TrackArray, f'/campaign/tracks_{name}',
                lambda m, n=name: self.out[f'tracks_{n}'].append(self.track_rows(m)), 10)
        sub(DiagnosticArray, '/perception/status', self.on_status, 10)

    @staticmethod
    def obs_rows(m):
        return {'t': stamp(m.header), 'obs': [
            {'id': o.backend_id, 'cls': o.class_id, 'p': [o.position.x, o.position.y,
                                                          o.position.z],
             'cov': [float(c) for c in o.position_covariance],
             'box': [float(b) for b in o.box2d] if o.has_box2d else None}
            for o in m.observations]}

    def track_rows(self, m):
        return {'t': self.now, 'tracks': [
            {'id': t.id, 'cls': t.class_id, 'motion': t.motion,
             'xy': [t.pose.pose.position.x, t.pose.pose.position.y],
             'P': [t.pose.covariance[0], t.pose.covariance[1], t.pose.covariance[7]]}
            for t in m.tracks]}

    def on_status(self, m):
        for s in m.status:
            if s.name.endswith('/fusion'):
                level = s.level
                level = int.from_bytes(level, 'little') if isinstance(level, bytes) else int(level)
                self.out['status'].append({'t': self.now, 'level': level, 'message': s.message,
                                           **{v.key: v.value for v in s.values}})

    def publisher(self, topic, typ):
        if topic not in self.pubs:
            self.pubs[topic] = self.node.create_publisher(self.get_message(typ), topic, self.best)
        return self.pubs[topic]

    def messages(self):
        """(header time, topic, type, message), header-stamp ordered within a 0.5 s window."""
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(self.session / 'bag'), storage_id=''),
                    rosbag2_py.ConverterOptions('', ''))
        types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
        heap, seq = [], 0
        while reader.has_next():
            topic, data, _ = reader.read_next()
            msg = deserialize_message(data, self.get_message(types[topic]))
            if topic == '/tf_static':
                t = stamp(msg.transforms[0].header) + CLOCK_OFFSET
            else:
                t = stamp(msg.header) + CLOCK_OFFSET
                msg.header.stamp.sec += int(CLOCK_OFFSET)
            seq += 1
            heapq.heappush(heap, (t, seq, topic, types[topic], msg))
            while heap and heap[-1][0] - heap[0][0] > 0.5:
                yield (lambda h: (h[0], *h[2:]))(heapq.heappop(heap))
        while heap:
            yield (lambda h: (h[0], *h[2:]))(heapq.heappop(heap))

    def play(self):
        from geometry_msgs.msg import TransformStamped
        lid, late = self.lidar, []
        lag, hz = lid.get('tf_lag_s', 0.0), lid.get('attitude_hz', 0.0)
        held, last_imu = None, -math.inf
        T_bl = None
        statics = {}
        for child, xyz in self.cfg['static_tf'].items():
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = 'base_link', child
            tf.transform.translation.x, tf.transform.translation.y, \
                tf.transform.translation.z = map(float, xyz)
            tf.transform.rotation.w = 1.0
            statics[child] = tf
        self.pubs['/tf_static'].publish(self.TF(transforms=list(statics.values())))
        T_bl = self.chain(statics, 'lidar_link')
        wall0, t0 = time.monotonic(), None
        self.now = 0.0
        for t, topic, typ, msg in self.messages():
            if t0 is None:
                t0 = t
            while time.monotonic() - wall0 < (t - t0) / self.rate:
                self.rclpy.spin_once(self.node, timeout_sec=0.002)
            while late and late[0][0] <= t:
                self.publisher('/points', late[0][2]).publish(heapq.heappop(late)[3])
            self.now = t
            clock = self.Clock()
            clock.clock.sec, clock.clock.nanosec = int(t), int((t % 1) * 1e9)
            self.pubs['/clock'].publish(clock)
            if topic == '/tf_static':
                for tf in msg.transforms:
                    statics[tf.child_frame_id] = tf
                self.pubs['/tf_static'].publish(self.TF(transforms=list(statics.values())))
                T_bl = self.chain(statics, 'lidar_link')
            elif topic == '/crane/odom':
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                R = matrix(q)
                self.out['truth'].append((t, [p.x, p.y, p.z], R.tolist()))
                self.publisher(topic, typ).publish(msg)       # sim_backend's truth ids
                if hz and (held is None or t - held[0] >= 1.0 / hz):
                    held = (t, math.atan2(R[1, 0], R[0, 0]))
                if held is not None:
                    yaw = math.atan2(R[1, 0], R[0, 0])
                    R = rpy(0, 0, held[1] - yaw) @ R        # zero-order-held yaw
                tf = TransformStamped()
                ts = t + lag
                tf.header.stamp.sec, tf.header.stamp.nanosec = int(ts), int((ts % 1) * 1e9)
                tf.header.frame_id, tf.child_frame_id = 'odom', 'base_link'
                tf.transform.translation.x, tf.transform.translation.y = p.x, p.y
                tf.transform.translation.z = p.z
                (tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
                 tf.transform.rotation.w) = quat(R)
                m2 = TransformStamped()
                m2.header.stamp = tf.header.stamp
                m2.header.frame_id, m2.child_frame_id = 'map', 'odom'
                m2.transform.rotation.w = 1.0
                self.pubs['/tf'].publish(self.TF(transforms=[tf, m2]))
            elif topic == '/imu/raw':
                if hz and t - last_imu < 1.0 / hz:
                    continue
                last_imu = t
                self.publisher(topic, typ).publish(msg)
            elif topic == '/points':
                if T_bl is None:
                    continue
                if lid.get('drop_p') and random.random() < lid['drop_p']:
                    continue
                msg = cloud_edit(msg, lid, self.rng, T_bl, self.cfg['water_z_base_m'],
                                 self.cfg['bow_x_m'])
                if lid.get('late_s'):
                    heapq.heappush(late, (t + lid['late_s'], id(msg), typ, msg))
                    continue
                self.publisher(topic, typ).publish(msg)
            else:
                self.publisher(topic, typ).publish(msg)
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        self.statics = {k: self.chain(statics, k) for k in ('lidar_link', 'front_camera_link')}

    @staticmethod
    def chain(statics, child):
        """(R, t) base_link <- child through the static tree."""
        R, t = np.eye(3), np.zeros(3)
        while child in statics and child != 'base_link':
            tf = statics[child].transform
            Rc = matrix(tf.rotation)
            tc = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
            R, t = Rc @ R, Rc @ t + tc
            child = statics[child].header.frame_id
        return R, t


# ------------------------------------------------------------------ metrics
def truth_pose(truth, t):
    ts = [r[0] for r in truth]
    i = min(max(int(np.searchsorted(ts, t)), 0), len(truth) - 1)
    return np.array(truth[i][2]), np.array(truth[i][1])


def horiz(p):
    return math.hypot(p[0], p[1])


def extrinsic_fit(samples):
    """Batch WLS of the node's angle model over one run's extrinsic samples.

    Samples are (x, y, Z, yaw rate, r_az, r_el, s_az, s_el). Returns the
    angles and the median |r_az - model| and |r_el - model|, in deg.
    """
    from mhseals_nav.lidar_fusion import model_rows
    if len(samples) < 10:
        return None
    S = np.array(samples, float)
    A, r, w = [], [], []
    for x, y, Z, yr, r_az, r_el, s_az, s_el in S:
        A += model_rows(x, y, Z, yr)
        r += [r_az, r_el]
        w += [s_az ** -2, s_el ** -2]
    A, r, w = np.array(A), np.array(r), np.array(w)
    prior = np.array([0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.1]) ** -2
    N = A.T @ (A * w[:, None]) + np.diag(prior)
    theta = np.linalg.solve(N, A.T @ (w * r))
    res = np.degrees(np.abs(r - A @ theta))
    return {'n': len(S), 'deg': np.degrees(theta[:3]).round(3).tolist(),
            'c_m': theta[3:6].round(3).tolist(),
            'tau_ms': round(1e3 * theta[6], 1),
            'med_az_res_deg': float(np.median(res[0::2])),
            'med_el_res_deg': float(np.median(res[1::2])),
            'med_abs_yaw_rate': float(np.median(np.abs(S[:, 3])))}


def metrics(out, statics, elements, cfg):
    """Range error per bin and source, lidar share, association, ids, MOVING, NEES, alarms."""
    bins = cfg['bins_m']
    R_bc, t_bc = statics['front_camera_link']
    xy = np.array([[e['x'], e['y']] for e in elements])
    t0 = out['truth'][0][0] if out['truth'] else 0.0

    def cam_frame(t, i):
        """Truth element i in the camera frame at t."""
        R, p = truth_pose(out['truth'], t)
        e = np.array([*xy[i], p[2]])
        return R_bc.T @ (R.T @ (e - p) - t_bc)

    def world(t, pc):
        R, p = truth_pose(out['truth'], t)
        return R @ (R_bc @ np.asarray(pc) + t_bc) + p

    def tid(o):
        return int(o['id']) if o['id'].isdigit() and int(o['id']) < len(elements) else None

    def bin_of(r):
        return next((f'{a}-{b}' for a, b in zip(bins, bins[1:]) if a <= r < b), None)
    err, nees_obs = {}, {}
    share = {}
    assoc = {'lidar': 0, 'errors': 0}
    srcs = {round(s['t'], 4): s['sources'] for s in out['sources']}
    skips = {}
    for s in out['sources']:
        if s.get('skip'):
            skips[s['skip']] = skips.get(s['skip'], 0) + 1
    for kind in ('before', 'fused'):
        for frame in out[kind]:
            labels = srcs.get(round(frame['t'], 4)) if kind == 'fused' else None
            for k, o in enumerate(frame['obs']):
                src = labels[k] if labels and k < len(labels) else \
                    ('stereo' if kind == 'before' else 'unknown')
                i = tid(o)
                if i is None and src == 'split':
                    w = world(frame['t'], o['p'])[:2]
                    d = np.hypot(*(xy - w).T)
                    i = int(np.argmin(d)) if d.min() < cfg['truth_match_m'] else None
                    if i is None:
                        assoc['lidar'] += 1
                        assoc['errors'] += 1
                        continue
                if i is None or o['box'] is None:
                    continue
                rt = horiz(cam_frame(frame['t'], i))
                b = bin_of(rt)
                if b is None:
                    continue
                e = horiz(o['p']) - rt
                err.setdefault((kind, b), []).append(e)
                if kind == 'fused':
                    err.setdefault((src, b), []).append(e)
                    u = np.array(o['p'][:2]) / max(horiz(o['p']), 1e-6)
                    C = np.array(o['cov']).reshape(3, 3)[:2, :2]
                    s2 = float(u @ C @ u)
                    if s2 > 0:
                        nees_obs.setdefault(src, []).append(e * e / s2)
                    lidar = src in ('lidar', 'split')
                    share.setdefault(b, []).append(lidar)
                    if lidar:
                        assoc['lidar'] += 1
                        assoc['errors'] += abs(e) > cfg['assoc_error_m']
    ranges = {f'{k}|{b}': {'n': len(v), 'rms': float(np.sqrt(np.mean(np.square(v)))),
                           'bias': float(np.mean(v))} for (k, b), v in sorted(err.items())}
    tracks = {}
    for name in ('raw', 'fused'):
        ids, moving, total, nees = {}, 0.0, 0.0, []
        rows = out[f'tracks_{name}']
        for a, b in zip(rows, rows[1:] + [None]):
            dt = min((b['t'] - a['t']) if b else 0.1, 0.5)
            if a['t'] - t0 < cfg['warmup_s']:
                continue
            R, p = truth_pose(out['truth'], a['t'])
            for tr in a['tracks']:
                w = np.array(tr['xy'])          # map == odom == world here (TF from truth)
                d = np.hypot(*(xy - w).T)
                i = int(np.argmin(d))
                if d[i] > cfg['truth_match_m'] or elements[i]['label'] != tr['cls']:
                    continue
                ids.setdefault(i, set()).add(tr['id'])
                if tr['cls'].endswith('_buoy'):
                    total += dt
                    moving += dt * (tr['motion'] == 2)
                u = xy[i] - p[:2]
                u /= max(np.linalg.norm(u), 1e-6)
                P = np.array([[tr['P'][0], tr['P'][1]], [tr['P'][1], tr['P'][2]]])
                pr = float(u @ P @ u)
                if pr > 0:
                    nees.append(float(((w - xy[i]) @ u) ** 2 / pr))
        tracks[name] = {'id_switches': sum(len(v) - 1 for v in ids.values()),
                        'buoys_tracked': len(ids),
                        'moving_frac': moving / total if total else math.nan,
                        'nees_r_mean': float(np.mean(nees)) if nees else math.nan}
    status = out['status']
    first = {lvl: next((s['t'] - t0 for s in status if s['level'] >= lvl), None)
             for lvl in (1, 2)}
    proc = [float(s['proc_ms_p95']) for s in status if s.get('proc_ms_p95') not in (None, 'nan')]
    cpu = [float(s['cpu_pct']) for s in status if s.get('cpu_pct')]
    last = status[-1] if status else {}

    def pct(k):
        v = [s[k] for s in out['sources'] if k in s]
        return [float(np.percentile(v, q)) for q in (50, 95)] if v else None
    samples = [x for s in out['sources'] for x in s.get('ex', [])]
    return {'ranges': ranges,
            'nees_obs': {k: float(np.mean(v)) for k, v in nees_obs.items()},
            'latency_s': {'capture_to_publish_p50_p95': pct('lat'),
                          'wait_p50_p95': pct('wait')},
            'extrinsic_fit': extrinsic_fit(samples),
            'lidar_share': {b: float(np.mean(v)) for b, v in sorted(share.items())},
            'assoc': {**assoc, 'rate': assoc['errors'] / assoc['lidar'] if assoc['lidar'] else 0},
            'tracks': tracks, 'skips': skips,
            'warn_s': first[1], 'error_s': first[2],
            'alerts': sorted({a for s in status for a in s.get('alerts', '').split(',') if a}),
            'extrinsic_deg': last.get('extrinsic_deg'), 'applied_deg': last.get('applied_deg'),
            'extrinsic_n': last.get('extrinsic_n'), 'c_m': last.get('c_m'),
            'tau_ms': last.get('tau_ms'),
            'proc_ms_p95_max': max(proc) if proc else None,
            'cpu_pct_max': max(cpu) if cpu else None,
            'frames': {'before': len(out['before']), 'fused': len(out['fused'])}}


def replay(session, name, outdir, rate):
    from ament_index_python.packages import get_package_share_directory
    cfg = yaml.safe_load(CONFIG.read_text())
    inj = dict(cfg['injections'][name.split('+')[0]])
    if name.endswith('+merge'):
        inj['merge'] = True
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    share = Path(get_package_share_directory('mhseals_nav')) / 'config'
    procs = {}
    for key, cmd in nodes(session, inj, cfg, share).items():
        log = open(outdir / f'{key}.log', 'w')
        procs[key] = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
    player = None
    try:
        time.sleep(4.0)                 # node start-up before the first message
        player = Player(session, inj, cfg, rate)
        player.play()
    finally:
        for p in procs.values():        # each node runs in its own process group
            try:
                os.killpg(p.pid, 2)
            except ProcessLookupError:
                pass
        for p in procs.values():
            try:
                p.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, 9)
    statics = {k: [R.tolist(), t.tolist()] for k, (R, t) in player.statics.items()}
    with gzip.open(outdir / 'raw.json.gz', 'wt') as f:
        json.dump({'out': player.out, 'statics': statics, 'session': str(session),
                   'injection': name, 'rate': rate}, f)
    result = score(outdir)
    print(json.dumps({k: result[k] for k in ('lidar_share', 'assoc', 'tracks', 'warn_s',
                                             'error_s', 'frames')}))


def score(outdir):
    """metrics.json from raw.json.gz (re-run after a metrics change, without a replay)."""
    cfg = yaml.safe_load(CONFIG.read_text())
    with gzip.open(Path(outdir) / 'raw.json.gz', 'rt') as f:
        raw = json.load(f)
    session = Path(raw['session'])
    truth = session / 'course_truth.json'
    if not truth.exists():
        truth = Path(outdir) / 'course_truth.json'
    # sim_backend's truth ids index the labelled elements only (docks have no label).
    elements = [e for e in json.loads(truth.read_text())['elements'] if e.get('label')]
    statics = {k: (np.array(R), np.array(t)) for k, (R, t) in raw['statics'].items()}
    result = metrics(raw['out'], statics, elements, cfg)
    result.update(session=raw['session'], injection=raw['injection'], rate=raw['rate'])
    (Path(outdir) / 'metrics.json').write_text(json.dumps(result, indent=1))
    return result


def report(dirs):
    for d in dirs:
        m = json.loads((Path(d) / 'metrics.json').read_text())
        r = m['ranges']
        cells = []
        for b in ('2-5', '5-10', '10-15', '15-20'):
            before, after = r.get(f'before|{b}'), r.get(f'fused|{b}')
            if before and after:
                cells.append(f'{b}:{before["rms"]:.2f}->{after["rms"]:.2f}'
                             f'({m["lidar_share"].get(b, 0):.2f})')
        t = m['tracks']
        lat, fit = m.get('latency_s') or {}, m.get('extrinsic_fit') or {}
        print(f'{Path(m["session"]).name:8s} {m["injection"]:16s} {" ".join(cells)} '
              f'assoc {m["assoc"]["errors"]}/{m["assoc"]["lidar"]} '
              f'ids {t["raw"]["id_switches"]}->{t["fused"]["id_switches"]} '
              f'mov {t["raw"]["moving_frac"]:.3f}->{t["fused"]["moving_frac"]:.3f} '
              f'nees {t["raw"]["nees_r_mean"]:.2f}->{t["fused"]["nees_r_mean"]:.2f} '
              f'warn {m["warn_s"]} err {m["error_s"]} p95 {m["proc_ms_p95_max"]} '
              f'lat {lat.get("capture_to_publish_p50_p95")} '
              f'r_az {fit.get("med_az_res_deg")}')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('replay')
    r.add_argument('session')
    r.add_argument('name', help='injection in config/fusion_campaign.yaml; +merge for S5')
    r.add_argument('out')
    r.add_argument('--rate', type=float, default=1.0)
    for name in ('report', 'score'):
        sub.add_parser(name).add_argument('dirs', nargs='+')
    args = parser.parse_args()
    if args.cmd == 'replay':
        replay(args.session, args.name, args.out, args.rate)
    elif args.cmd == 'score':
        for d in args.dirs:
            score(d)
    else:
        report(args.dirs)


if __name__ == '__main__':
    main()

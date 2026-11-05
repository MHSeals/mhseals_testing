"""The real boat as a judge Source: a rosbag2/MCAP bag (or a session holding one) without ROS.

Channels (all on the bag's receive clock, seconds):
  pose.se2        /odom/global (map) else /odom/local, 'estimated', sigma from the pose covariance
  twist.body      the same odometry's twist (vx, vy, wz)
  sensor.gnss.enu /gps/fix in a local tangent plane at the first fix, sigma from its covariance,
                  plus the fix status per sample (2 = RTK/GBAS)
  actuator.cmd    /cmd_vel (surge, sway, yaw) as commanded (effort units on the boat profile)
Events: the session journal topic (/mhseals_testing/events: armed, disarmed, procedure phases) and
RTK route/phase changes (ros/rtk.STATUS_TOPIC).

Labels: the session manifest beside the bag (annotate/manifest.py), when there is one, adds the operator's water
conditions to the scenario, flags a window that overlaps excluded spans (bad data, interventions, link holds,
invalid tags), turns operator language commands into ``operator.said`` events and keeps the calibration
candidates and exclusions as the ``manifest`` attachment; ``plant_fit`` skips the excluded spans.

Clock: rosbag2 log time is the record clock; the median |log - header stamp| of the odometry is the
alignment residual (receive latency plus clock offset) the integrity check reports.

The ``real`` pack adds what only a real run can say: estimator vs GNSS consistency (a rigid 2-D fit
of the odometry track to the GNSS track; the residual is what localisation claims beyond GNSS), and
a first-order-plus-dead-time fit of commanded surge/yaw to measured body velocity, the numbers the
shared plant card (X4) and the fast tier's calibration (delay_s, speed_scale) need.
"""

import json
import math
from pathlib import Path

import numpy as np

from mhseals_testing.annotate import manifest as session_manifest
from mhseals_testing.events import TOPIC as EVENTS_TOPIC
from mhseals_testing.judge_pack.roboboat._core import core
from mhseals_testing.ros.rtk import STATUS_TOPIC as RTK_STATUS

core()

from judge.metric import Metric, MetricValue, Status, unscorable  # noqa: E402
from judge.pack import Pack  # noqa: E402
from judge.packs import generic_motion  # noqa: E402
from judge.record import (Channel, ChannelSpec, Clock, Event, Header, Robot, RunRecord, Scenario,  # noqa: E402
                          Validity, dedupe_time, se2_channel)
from judge.run import judge_runs  # noqa: E402
from judge.insight import Insight  # noqa: E402
from judge.source import FidelityProfile, Refused, Source  # noqa: E402

REAL_PROFILE = 'boat'                        # the plant whose bags this source reads


def _names(profile=REAL_PROFILE):
    """The bag's topic names from the profile: its topics: odometry and fix, and its command topic."""
    from mhseals_testing.profiles import load_profile
    plant = load_profile(profile)
    return {'global': plant.topics['odom_global'], 'local': plant.topics['odom_local'],
            'fix': plant.topics['gps_fix'], 'cmd': plant.command_topic}


NAMES = _names()
TOPICS = (NAMES['global'], NAMES['local'], NAMES['fix'], NAMES['cmd'], EVENTS_TOPIC, RTK_STATUS)
R_EARTH = 6378137.0


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read(path, topics=TOPICS):
    """{topic: [(log_t, stamp_t | None, msg)]} from every MCAP under ``path``."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    path = Path(path)
    files = sorted(path.rglob('*.mcap')) if path.is_dir() else [path]
    if not files:
        raise Refused(f'{path}: no MCAP files')
    out = {t: [] for t in topics}
    for f in files:
        with open(f, 'rb') as fh:
            try:
                it = make_reader(fh, decoder_factories=[DecoderFactory()]).iter_decoded_messages(topics=list(topics))
                for _, ch, m, msg in it:
                    h = getattr(msg, 'header', None)
                    st = h.stamp.sec + h.stamp.nanosec * 1e-9 if h is not None else None
                    out[ch.topic].append((m.log_time * 1e-9, st if st else None, msg))
            except Exception as e:     # a torn segment: keep what was read, flag it
                out.setdefault('_errors', []).append(f'{f.name}: {type(e).__name__}')
    for v in out.values():
        if v and isinstance(v[0], tuple):
            v.sort(key=lambda r: r[0])
    return out


def _odom(rows, name):
    t = np.array([r[0] for r in rows])
    p = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, _yaw(m.pose.pose.orientation)] for _, _, m in rows])
    cov = np.array([[m.pose.covariance[0], m.pose.covariance[7], m.pose.covariance[35]] for _, _, m in rows])
    tw = np.array([[m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z] for _, _, m in rows])
    t, p, cov, tw = dedupe_time(t, p, cov, tw)
    sigma = np.sqrt(np.clip(cov, 0, None))
    pose = se2_channel(t, p[:, 0], p[:, 1], p[:, 2], 'estimated', sigma, frame=name)
    twist = Channel.of(ChannelSpec('twist.body', ('vx', 'vy', 'wz'), ('m/s', 'm/s', 'rad/s'), 'body.flu', 'estimated'),
                       t, tw)
    lag = [abs(lt - st) for lt, st, _ in rows if st]
    return pose, twist, (float(np.median(lag)) if lag else None)


def _gnss(rows):
    good = [(t, m) for t, _, m in rows if m.status.status >= 0 and math.isfinite(m.latitude)]
    if len(good) < 2:
        return None, {}
    lat0, lon0 = math.radians(good[0][1].latitude), math.radians(good[0][1].longitude)
    t = np.array([g[0] for g in good])
    lat = np.radians([g[1].latitude for g in good])
    lon = np.radians([g[1].longitude for g in good])
    e = (lon - lon0) * math.cos(lat0) * R_EARTH
    n = (lat - lat0) * R_EARTH
    st = np.array([g[1].status.status for g in good], float)
    sig = np.sqrt(np.clip([[g[1].position_covariance[0], g[1].position_covariance[4]] for g in good], 0, None))
    t, e, n, st, sig = dedupe_time(t, e, n, st, sig)
    ch = Channel.of(ChannelSpec('sensor.gnss.enu', ('e', 'n', 'status'), ('m', 'm', ''), 'enu', 'estimated'), t,
                    np.column_stack([e, n, st]), np.column_stack([sig, np.zeros(len(t))]))
    return ch, {'rtk_fraction': float(np.mean(st >= 2)), 'sigma_median_m': float(np.median(sig[:, 0])),
                'origin': [math.degrees(lat0), math.degrees(lon0)]}


def _events(data):
    out = []
    for t, _, m in data.get(EVENTS_TOPIC, []):
        try:
            d = json.loads(m.data)
        except ValueError:
            continue
        out.append(Event(t, f'journal.{d.get("phase", "event")}', 'harness', d))
    last = None
    for t, _, m in data.get(RTK_STATUS, []):
        st = m.status[0] if m.status else None
        if st is None:
            continue
        kv = {v.key: v.value for v in st.values}
        key = (kv.get('phase'), kv.get('route'))
        if key != last:
            out.append(Event(t, 'rtk.state', 'rtk', {'why': f'{key[0]} via {key[1]}', **kv}))
            last = key
    return tuple(sorted(out, key=lambda e: e.t))


class BagSource(Source):
    name, tier, version = 'roboboat_real_bag', 'real', '1'
    profile = FidelityProfile('real', 'real', 'real', 'real', 3600.0, 'the boat')

    def load(self, path):
        path = Path(path)
        data = read(path)
        odom_global, fix, cmd = NAMES['global'], NAMES['fix'], NAMES['cmd']
        src = odom_global if data.get(odom_global) else NAMES['local']
        if len(data.get(src, [])) < 10:
            raise Refused(f'{path}: no odometry ({len(data.get(src, []))} samples)')
        pose, twist, lag = _odom(data[src], 'map' if src == odom_global else 'odom')
        channels = {'pose.se2': pose, 'twist.body': twist}
        gnss, gq = _gnss(data.get(fix, []))
        if gnss is not None:
            channels['sensor.gnss.enu'] = gnss
        if data.get(cmd):
            t = np.array([r[0] for r in data[cmd]])
            v = np.array([[m.linear.x, m.linear.y, m.angular.z] for _, _, m in data[cmd]])
            t, v = dedupe_time(t, v)
            channels['actuator.cmd'] = Channel.of(
                ChannelSpec('actuator.cmd', ('surge', 'sway', 'yaw'), ('effort', 'effort', 'effort'), 'body', 'commanded'), t, v)
        events = _events(data)
        armed = [e.t for e in events if e.kind == 'journal.armed']
        disarmed = [e.t for e in events if e.kind == 'journal.disarmed']
        t0 = armed[0] if armed else float(pose.t[0])
        t1 = next((d for d in disarmed if d > t0), float(pose.t[-1]))
        flags = {}
        if gq and gq['rtk_fraction'] < 0.5:
            flags['rtk_fraction'] = round(gq['rtk_fraction'], 3)
        if data.get('_errors'):
            flags['torn_segments'] = len(data['_errors'])
        sess = json_or_empty(path / 'session.json') or json_or_empty(path.parent.parent / 'session.json')
        condition = {'site': sess.get('site', ''), 'profile': sess.get('profile', 'boat')}
        labels = manifest_of(path)
        if labels is not None:
            condition.update(labels_condition(labels, t0))
            excluded = session_manifest.overlaps(labels, t0, t1)
            if excluded:
                flags['manifest_excluded'] = excluded
            events = tuple(sorted(events + tuple(
                Event(c['t'], 'operator.said', 'operator', {'why': c['text'], **c}) for c in labels['commands']),
                key=lambda e: e.t))
        header = Header(run_id=path.name, tier='real', source=self.name, domain='roboboat',
                        robot=Robot('blastoise', 'wam-v-mini'), task_id=str(sess.get('procedure', '')),
                        scenario=Scenario(family='field', condition=condition),
                        provenance={'bag': str(path), 'odometry': src, 'gnss': gq}, clock=Clock('wall', 'log_time', 0.0, lag),
                        window=(t0, t1, 'disarmed' if disarmed else 'bag_end'))
        # Without a surveyed course the truth is only GNSS: no gate geometry, no referee rows.
        caps = {'truth': None, 'contacts': None, 'claims': False}
        attachments = {'gnss_quality': gq}
        if labels is not None:
            attachments['manifest'] = {'path': labels['path'], 'calibration': labels['calibration'],
                                       'excluded': session_manifest.excluded(labels), 'commands': labels['commands']}
        return RunRecord(header, channels, events, caps, Validity(True, flags, frozenset()), {}, attachments)


def manifest_of(path):
    """The session manifest for a bag folder or a session holding one, or None (older sessions have none)."""
    for folder in (path, path.parent, path.parent.parent):
        try:
            return session_manifest.load(folder)
        except (OSError, ValueError):
            continue
    return None


def labels_condition(doc, t):
    """The operator's wind, waves and current scales in force at ``t`` (0 unset), as scenario conditions."""
    before = [c for c in doc['conditions'] if c['t'] <= t] or doc['conditions'][:1]
    return {k: v for k, v in (before[-1] if before else {}).items() if k != 't'}


def json_or_empty(p):
    try:
        return json.loads(Path(p).read_text())
    except (OSError, ValueError):
        return {}


# ------------------------------------------------------------------ real-only metrics

def rigid_fit(a, b):
    """Least-squares 2-D rotation+translation taking a onto b: (R, t, residuals)."""
    ca, cb = a.mean(0), b.mean(0)
    H = (a - ca).T @ (b - cb)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = cb - R @ ca
    res = np.linalg.norm((a @ R.T + t) - b, axis=1)
    return R, t, res


class GnssConsistency(Metric):
    """RMS distance between the odometry track (rigidly fitted) and the GNSS track. Large means
    the estimator's map frame drifts against GNSS (or GNSS is poor: see its sigma)."""
    name, unit, direction = 'real.odom_gnss_rms', 'm', 'lower'
    requires = frozenset({'pose.se2', 'sensor.gnss.enu'})

    def compute(self, rec, task):
        g = rec.channel('sensor.gnss.enu').window(*rec.header.window[:2])
        if len(g) < 20:
            return unscorable('fewer than 20 GNSS fixes in the window')
        p = rec.channel('pose.se2').at(g.t)[:, :2]
        if np.ptp(p, axis=0).max() < 2.0:
            return unscorable('the boat moved under 2 m: no geometry to fit')
        _, _, res = rigid_fit(p, g.v[:, :2])
        sig = float(np.median(g.sigma_of('e')))
        st = Status.PROVISIONAL if float(np.sqrt(np.mean(res ** 2))) < 2 * sig else Status.OK
        return MetricValue(float(np.sqrt(np.mean(res ** 2))), st, sig,
                           why=f'GNSS sigma median {sig:.2f} m', extra={'p95': float(np.percentile(res, 95))})


def fopdt(t, u, y, dt=0.1, max_delay=2.0, fresh=None):
    """First-order plus dead time fit of y to u on a uniform grid: {gain, tau_s, delay_s, r2, n}
    or None. y[k+1] = a y[k] + b u[k-d] by least squares over d; tau = -dt/ln(a), gain = b/(1-a);
    r2 is on the increments y[k+1] - y[k] (what the command must explain).
    ``fresh`` masks samples whose command is stale (no command message recently): those rows
    are left out instead of holding an old command for minutes."""
    if len(t) < 50:
        return None
    fresh = np.ones(len(t), bool) if fresh is None else fresh
    best = None
    for d in range(0, int(max_delay / dt) + 1):
        k = np.arange(d, len(y) - 1)
        k = k[fresh[k - d] & fresh[k]]
        if len(k) < 30:
            continue
        Y = y[k + 1]
        X = np.column_stack([y[k], u[k - d]])
        coef, res, *_ = np.linalg.lstsq(X, Y, rcond=None)
        r = Y - X @ coef
        sse = float(r @ r)
        if best is None or sse < best[0]:
            # r2 on the increments: y[k+1] from y[k] alone is always 'explained' on a slow signal
            best = (sse, d, coef, float(np.var(Y - y[k]) * len(Y)), len(k))
    if best is None:
        return None
    sse, d, (a, b), sst, n = best
    if not (0 < a < 1) or abs(b) < 1e-9:
        return {'delay_s': d * dt, 'r2': 1 - sse / sst if sst else None, 'n': n,
                'note': f'velocity not explained by the command (a={a:.3f}): another authority (RC) drove, '
                        'or the command is not what actuated'}
    return {'gain': float(b / (1 - a)), 'tau_s': float(-dt / math.log(a)), 'delay_s': d * dt,
            'r2': float(1 - sse / sst) if sst else None, 'n': n}


def plant_fit(rec, axis='surge', dt=0.1):
    if not (rec.has('actuator.cmd') and rec.has('twist.body')):
        return None
    c, v = rec.channel('actuator.cmd'), rec.channel('twist.body')
    t0, t1 = rec.header.window[:2]
    t = np.arange(max(t0, c.t[0], v.t[0]), min(t1, c.t[-1], v.t[-1]), dt)
    if len(t) < 50:
        return None
    ci, vi = {'surge': (0, 0), 'yaw': (2, 2)}[axis]
    # zero-order hold of the command (the driver keeps the last command)
    idx = np.clip(np.searchsorted(c.t, t, side='right') - 1, 0, len(c.t) - 1)
    u = c.v[idx, ci]
    fresh = (t - c.t[idx]) <= 0.5
    for a, b, _ in (rec.attachments.get('manifest') or {}).get('excluded', ()):
        fresh &= ~((t >= a) & (t <= b))           # labelled bad data, interventions, link holds
    y = np.interp(t, v.t, v.v[:, vi])
    if np.ptp(u[fresh]) < 1e-3 if fresh.any() else True:
        return None
    out = fopdt(t, u, y, dt, fresh=fresh)
    if out:
        out['excitation'] = float(np.std(u))
    return out


class PlantFit(Metric):
    unit, direction = 's', 'none'
    requires = frozenset({'actuator.cmd', 'twist.body'})

    def __init__(self, axis, field):
        self.axis, self.field = axis, field
        self.name = f'plant.{axis}.{field}'
        self.unit = {'delay_s': 's', 'tau_s': 's', 'gain': '(m/s)/effort' if axis == 'surge' else '(rad/s)/effort'}[field]

    def compute(self, rec, task):
        fit = plant_fit(rec, self.axis)
        if not fit or self.field not in fit:
            return unscorable(f'{self.axis}: no usable excitation' if not fit else fit.get('note', 'no fit'))
        st = Status.OK if (fit.get('r2') or 0) > 0.6 else Status.PROVISIONAL
        return MetricValue(fit[self.field], st, why=f"r2 {fit.get('r2'):.2f}, n {fit['n']}", extra=fit)


REAL = Pack('real', 'roboboat', {m.name: m for m in (
    GnssConsistency(), PlantFit('surge', 'delay_s'), PlantFit('surge', 'tau_s'), PlantFit('surge', 'gain'),
    PlantFit('yaw', 'delay_s'), PlantFit('yaw', 'tau_s'), PlantFit('yaw', 'gain'))})


def _outcome(rec, values):
    return 'session'      # free driving: no task outcome without a surveyed course


REAL_RUN = Pack('real_outcome', 'roboboat', {}, outcome=_outcome)


def fast_calibration(path=None):
    """The fast tier's current calibration epoch (delay_s per axis, speed_scale), for comparison."""
    import yaml
    cands = [path] if path else list(Path(__file__).resolve().parents[2].glob('config/fastsim_calibration/*.yaml'))
    for p in sorted(c for c in cands if c):
        try:
            doc = yaml.safe_load(Path(p).read_text())
            return {'file': Path(p).name, **(doc.get('plant') or {})}
        except (OSError, ValueError, AttributeError):
            continue
    return None


def judge_bag(path):
    rec = BagSource().load(path)
    rep = judge_runs([rec], None, [REAL, generic_motion.PACK, REAL_RUN], success=('session',))
    vals = rep['runs'][0]['values']
    ins = []
    gq = dict(rec.attachments.get('gnss_quality') or {})
    if rec.has('sensor.gnss.enu'):             # quality inside the judged window, not the whole bag
        g = rec.channel('sensor.gnss.enu').window(*rec.header.window[:2])
        if len(g):
            gq.update(rtk_fraction=float(np.mean(g.col('status') >= 2)),
                      sigma_median_m=float(np.median(g.sigma_of('e'))), n_window=len(g))
    if gq:
        ins.append(Insight('real.gnss', 'warn' if gq.get('rtk_fraction', 0) < 0.5 else 'info',
                           f"GNSS RTK-fixed {gq['rtk_fraction']:.0%} of in-window fixes, median sigma "
                           f"{gq['sigma_median_m']:.2f} m: "
                           + ('the track can only referee gates wider than about 2 sigma; sub-metre rows need RTK or a survey'
                              if gq.get('rtk_fraction', 0) < 0.5 else 'RTK truth available'),
                           scope={'run': rec.run_id}, data=gq))
    cal = fast_calibration()
    fits = {ax: plant_fit(rec, ax) for ax in ('surge', 'yaw')}
    for ax, fit in fits.items():
        if fit and 'tau_s' in fit:
            k = 0 if ax == 'surge' else 2
            fast_d = (cal or {}).get('delay_s', [None] * 3)[k] if cal else None
            ins.append(Insight('real.plant_fit', 'info',
                               f"{ax}: dead time {fit['delay_s']:.1f} s, tau {fit['tau_s']:.2f} s, gain {fit['gain']:.2f} "
                               f"(r2 {fit['r2']:.2f}, n {fit['n']})"
                               + (f"; fast tier ({cal['file']}) assumes {fast_d:g} s delay" if fast_d is not None else '')
                               + ('; NOT identifiable from this session (increment r2 < 0.3): use designed steps'
                                  if (fit.get('r2') or 0) < 0.3 else ''),
                               scope={'run': rec.run_id}, data={**fit, 'fast_calibration': cal}, basis='statistical'))
    od = vals.get('real.odom_gnss_rms') or {}
    if od.get('value') is not None:
        ins.append(Insight('real.localisation', 'warn' if od['value'] > 2 * (od.get('sigma') or 0) else 'info',
                           f"odometry vs GNSS after a rigid fit: RMS {od['value']:.2f} m (p95 {od['extra']['p95']:.2f} m) "
                           f"against GNSS sigma {od.get('sigma') or 0:.2f} m",
                           scope={'run': rec.run_id}, data=od))
    rep['insights'] = [i.as_dict() for i in ins] + rep['insights']
    rep['record'] = {'channels': {n: [len(c), c.spec.observability] for n, c in rec.channels.items()},
                     'window': list(rec.header.window), 'clock_residual_s': rec.header.clock.residual_s,
                     'events': [[round(e.t - rec.header.window[0], 1), e.kind, e.payload.get('why') or e.payload.get('phase')]
                                for e in rec.events][:40]}
    return rep

"""Lidar-camera calibration and pre-run check from a recording (perception-calibration.md 2b, 2c).

  analyze BAG [--out DIR]       per-detection CSV and per-object summary CSV
  calibrate BAG [--out DIR]     also the rotation/translation fit with GLS SE and a block
                                bootstrap; prints and writes OUT/calibration.yaml
  check BAG [--baseline REC] [--truth-range ID=M ...]
                                one table and a verdict; exit 0 PASS, 1 WARN, 2 FAIL (3 error)

Association and the fit are mhseals_nav lidar_fusion's pure functions (extrinsic_samples,
ExtrinsicEstimate.fit), so the offline and online numbers are the same math. Points are
taken in the camera optical frame through the bag's static TF, motion-compensated with
odom->base TF when the bag has it. Nothing is ever applied. Run with mhseals_nav importable
(sourced, or PYTHONPATH=../mhseals_nav).
"""

import argparse
import csv
from collections import namedtuple
import math
from pathlib import Path
import sys

import numpy as np
import yaml

from mhseals_testing.harness import bagread
from mhseals_testing.health.config import config_path
from mhseals_testing.perception.project import rotation
from mhseals_testing.strict_yaml import load_strict

CONFIG = config_path('config', 'fusion_calib.yaml')
DEG = math.pi / 180
# ExtrinsicEstimate's theta: w_x, w_y, w_z (optical axes, rad), c (m), image time (s).
PARAMS = ('pitch_deg', 'yaw_deg', 'roll_deg', 'cx_m', 'cy_m', 'cz_m', 'tau_s')
SCALE = np.array([1 / DEG] * 3 + [1.0] * 4)
PASS, WARN, FAIL = 0, 1, 2
LEVELS = ('PASS', 'WARN', 'FAIL')

# One camera frame: observations (header frame, axes A into optical), the lidar cloud already
# in optical axes at t, intrinsics (fx, fy, cx, cy), image (W, H) and the yaw rate (rad/s).
Frame = namedtuple('Frame', 't obs P K image A yaw_rate', defaults=(None, 0.0))


def nav():
    from mhseals_nav import lidar_fusion
    return lidar_fusion


def nav_params(name):
    """An mhseals_nav config: the installed share, else the source tree next to lidar_fusion."""
    lf = nav()
    try:
        return lf.load_yaml(name)
    except (ImportError, LookupError, OSError):
        path = Path(lf.__file__).resolve().parents[1] / 'config' / name
        return yaml.safe_load(path.read_text())['/**']['ros__parameters']   # mhseals_nav's ROS params


def load_config(path=CONFIG):
    """fusion_calib.yaml plus the merged lidar_fusion params ('nav') and tracker ('tracker')."""
    cfg = load_strict(Path(path).read_text(), path)
    extra = set(cfg) - {'lidar_fusion', 'extrinsic', 'bearing_gate_deg', 'bootstrap',
                        'constrained_se_ratio', 'check', 'dupes'}
    base = nav_params('lidar_fusion.yaml')
    extra |= {f'lidar_fusion.{k}' for k in set(cfg['lidar_fusion']) - set(base)}
    extra |= {f'extrinsic.{k}' for k in set(cfg['extrinsic']) - set(base['extrinsic'])}
    if extra:
        raise ValueError(f'{path}: unknown key {sorted(extra)[0]}')
    params = {**base, **cfg['lidar_fusion']}
    params['extrinsic'] = {**base['extrinsic'], **cfg['extrinsic']}
    return {**cfg, 'nav': params, 'tracker': nav_params('tracker.yaml')}


# ------------------------------------------------------------------ core
class Keys:
    """Object key: backend_id, else the class plus the nearest bearing it was first seen at."""

    def __init__(self, gate_deg):
        self.gate, self.seen = gate_deg * DEG, {}

    def __call__(self, ob, bearing):
        if ob.backend_id:
            return ob.backend_id
        known = self.seen.setdefault(ob.class_id, [])
        near = min(known, key=lambda b: abs(b - bearing), default=None)
        if near is None or abs(near - bearing) > self.gate:
            known.append(bearing)
            near = bearing
        return f'{ob.class_id}@{near / DEG:+.1f}'


def measure(frames, cfg):
    """Per-detection rows over time-ordered frames, the filled estimate and each sample's (t, key).

    A detection is a box lidar_fusion would sample (isolated, near, unclipped, sample class);
    its hit is the single raw-projection cluster. du, dv: cluster centre minus box centre (px).
    rho_lidar is the cluster's near-face median plus near_face_k R, against the stereo range.
    """
    lf = nav()
    est = lf.ExtrinsicEstimate({**cfg['nav']['extrinsic'], 'window_n': None,
                                'min_n': math.inf})     # keep every sample, fit once at the end
    keys, rows, sample_t = Keys(cfg['bearing_gate_deg']), [], []
    for f in frames:
        A = np.eye(3) if f.A is None else f.A
        ctx = lf.Context(cfg['nav'], cfg['tracker'], f.K, f.image, A)
        fx, fy, cx, cy = f.K
        for b, c in lf.extrinsic_samples(f.obs, np.asarray(f.P, float).reshape(-1, 3), ctx):
            uc, vc = (b.u0 + b.u1) / 2, (b.v0 + b.v1) / 2
            key = keys(b.ob, math.atan((uc - cx) / fx))
            row = {'t': f.t, 'key': key, 'class': b.ob.class_id, 'u': uc, 'v': vc,
                   'rho_stereo': b.rho, 'hit': c is not None, 'n': 0, 'du': math.nan,
                   'dv': math.nan, 'rho_face': math.nan, 'rho_lidar': math.nan,
                   'ratio': math.nan, 'sample': False}
            if c is not None:
                row.update(n=c.n, du=c.uc - uc, dv=c.vc - vc, rho_face=c.rho,
                           rho_lidar=c.rho + b.near, ratio=(c.rho + b.near) / b.rho)
                if est.add(f.t, key, b, c, ctx, f.yaw_rate):
                    row['sample'] = True
                    sample_t.append((f.t, key))
            rows.append(row)
    return rows, est, sample_t


def summarize(rows, near_m):
    """Per object: counts, hit rate, median and MAD of du/dv, median ranges and ratio."""
    out = []
    for key in dict.fromkeys(r['key'] for r in rows):
        g = [r for r in rows if r['key'] == key]
        hits = [r for r in g if r['hit']]

        def med(name, rs=hits):
            return float(np.median([r[name] for r in rs])) if rs else math.nan

        def mad(name):
            m = med(name)
            return float(np.median([abs(r[name] - m) for r in hits])) if hits else math.nan
        rho_s = med('rho_stereo', g)
        out.append({'key': key, 'class': g[0]['class'], 'n': len(g), 'hits': len(hits),
                    'samples': sum(r['sample'] for r in g), 'hit_rate': len(hits) / len(g),
                    'du_med': med('du'), 'du_mad': mad('du'), 'dv_med': med('dv'),
                    'dv_mad': mad('dv'), 'rho_stereo': rho_s, 'rho_lidar': med('rho_lidar'),
                    'ratio': med('ratio'), 'near': rho_s < near_m})
    return out


def bootstrap(est, sample_t, reps, block_s, seed, min_units):
    """Theta of `reps` fits over resampled (object, time block) units; None under min_units."""
    t0 = min((t for t, _ in sample_t), default=0.0)
    units = {}
    for s, (t, key) in zip(est.samples, sample_t):
        units.setdefault((key, math.floor((t - t0) / block_s)), []).append(s)
    units = list(units.values())
    if len(units) < min_units:
        return None
    rng, fit = np.random.default_rng(seed), nav().ExtrinsicEstimate(est.cfg)
    out = []
    for _ in range(reps):
        fit.samples = [s for i in rng.integers(len(units), size=len(units)) for s in units[i]]
        out.append(fit.fit()[0])
    return np.array(out)


def calibrate(est, sample_t, cfg):
    """Fit summary per parameter (display units): value, SE, bootstrap 95%, prior SE, constrained.

    None without samples. Constrained means the posterior SE is under
    constrained_se_ratio x the prior SE; any other number is the prior's, not a measurement.
    """
    if not est.samples:
        return None
    theta, se = est.fit()
    b = cfg['bootstrap']
    boot = bootstrap(est, sample_t, b['reps'], b['block'], b['seed'], b['min_units'])
    lo, hi = (None, None) if boot is None else np.percentile(boot, [2.5, 97.5], 0)
    prior = np.array(est.cfg['prior_sigma'], float)
    params = {}
    for i, name in enumerate(PARAMS):
        k = SCALE[i]
        params[name] = {'value': round(float(theta[i] * k), 4),
                        'se': round(float(se[i] * k), 4),
                        'ci95': None if boot is None else [round(float(lo[i] * k), 4),
                                                           round(float(hi[i] * k), 4)],
                        'prior_se': round(float(prior[i] * k), 4),
                        'constrained': bool(se[i] < cfg['constrained_se_ratio'] * prior[i])}
    return params


def camera_delta(params):
    """Camera-vs-lidar correction in camera body axes (x fwd, y left, z up), small angles.

    The fit says the lidar points sit at R(w) P + c in the URDF optical frame, so the camera
    turns by w about its own optical axes and moves by c: body (roll, pitch, yaw) = (w_z,
    -w_x, -w_y), body xyz = (c_z, -c_x, -c_y). Unconstrained components are None.
    """
    def get(name, sign):
        p = params[name]
        return round(sign * p['value'], 4) if p['constrained'] else None
    return {'rpy_deg': [get('roll_deg', 1), get('pitch_deg', -1), get('yaw_deg', -1)],
            'xyz_m': [get('cz_m', 1), get('cx_m', -1), get('cy_m', -1)]}


def record(rows, objects, est, params, source):
    ts = [r['t'] for r in rows]
    return {'source_bag': str(source),
            'time_span_s': [min(ts), max(ts)] if ts else None,
            'n_detections': len(rows), 'n_samples': len(est.samples),
            'n_objects': len({k for *_, k in est.samples}),
            'params': params,
            'suggested_camera_delta': camera_delta(params) if params else None,
            'caveats': caveats(params, objects),
            'objects': [{k: v for k, v in o.items()} for o in objects]}


def verdict(objects, params, cfg, baseline=None, truth=None):
    """(level, reasons, per-object levels) against cfg['check']; truth maps key -> metres."""
    c, reasons, levels = cfg['check'], [], {}
    truth = truth or {}
    for o in objects:
        lv, why = PASS, []

        def worse(level, text):
            nonlocal lv
            lv = max(lv, level)
            why.append(text)
        if o['hits'] == 0:
            if o['near']:
                worse(FAIL, f'no lidar hit at {o["rho_stereo"]:.1f} m')
        else:
            for d in ('du', 'dv'):          # dv: vertical (pitch) offset, same limits
                off = abs(o[d + '_med'])
                if off > c[d + '_warn_px']:
                    worse(FAIL, f'{d} {o[d + "_med"]:+.1f} px')
                elif off > c[d + '_pass_px']:
                    worse(WARN, f'{d} {o[d + "_med"]:+.1f} px')
            if o['near'] and o['hit_rate'] < c['hit_rate_min']:
                worse(WARN, f'hit rate {o["hit_rate"]:.0%}')

            def ranged(err, text):          # WARN past range_tol, FAIL past range_fail_tol
                if abs(err) > c['range_fail_tol']:
                    worse(FAIL, text)
                elif abs(err) > c['range_tol']:
                    worse(WARN, text)
            ranged(o['ratio'] - 1, f'lidar/stereo {o["ratio"]:.3f}')
            if o['key'] in truth:
                ranged(o['rho_lidar'] / truth[o['key']] - 1,
                       f'lidar {o["rho_lidar"]:.2f} m vs tape {truth[o["key"]]:.2f} m')
        levels[o['key']] = lv
        reasons += [(lv, f'{o["key"]}: {w}') for w in why]
    if not objects:
        reasons.append((FAIL, 'no qualifying buoy detection'))
    if baseline and params:
        for name in PARAMS[:3]:
            p, q = params[name], baseline['params'][name]
            if not (p['constrained'] and q['constrained']):
                continue
            d = p['value'] - q['value']
            lv = PASS if abs(d) <= c['domega_pass_deg'] else \
                WARN if abs(d) <= c['domega_warn_deg'] else FAIL
            if lv:
                reasons.append((lv, f'{name[:-4]} {d:+.2f} deg vs baseline'))
    level = max([lv for lv, _ in reasons] + [PASS])
    return level, [r for lv, r in reasons if lv == level], levels


def table(objects, levels=None, truth=None):
    truth = truth or {}
    head = f'{"object":<22}{"n":>5}{"hit":>6}{"rho_st":>8}{"li/st":>7}' \
           f'{"du med±mad":>14}{"dv med±mad":>14}{"tape":>7}  verdict'
    lines = [head]
    for o in objects:
        tape = f'{truth[o["key"]]:.2f}' if o['key'] in truth else '-'
        lines.append(f'{o["key"][:21]:<22}{o["n"]:>5}{o["hit_rate"]:>6.0%}'
                     f'{o["rho_stereo"]:>8.2f}{o["ratio"]:>7.3f}'
                     f'{o["du_med"]:>+8.1f}±{o["du_mad"]:<5.1f}{o["dv_med"]:>+8.1f}±'
                     f'{o["dv_mad"]:<5.1f}{tape:>7}  '
                     f'{LEVELS[levels[o["key"]]] if levels else "-"}')
    return '\n'.join(lines)


BOOT_CAVEAT = ('bootstrap 95% CIs miss the bias all buoys share and are optimistic: the GLS SE '
               'is the bound')
ALIAS_CAVEAT = ('one range only: yaw and sideways offset (cx), and pitch and height (cy), are '
                'aliased; translations are the prior (tape), rotations assume it')


def caveats(params, objects):
    """Report notes: always the bootstrap caveat; the aliasing one when the buoys ranged
    span under 1.25x (du = fx (yaw + cx / Z) separates only across ranges)."""
    out = [BOOT_CAVEAT]
    rs = [o['rho_stereo'] for o in objects if o['hits'] and math.isfinite(o['rho_stereo'])]
    if params and (len(rs) < 2 or max(rs) < 1.25 * min(rs)):
        out.append(ALIAS_CAVEAT)
    return out


def fit_lines(params, objects=()):
    if not params:
        return ['fit: no samples']
    out = []
    for name in PARAMS[:6]:
        p = params[name]
        mark = '' if p['constrained'] else '  (not constrained: prior)'
        ci = '95% [{:+.3f}, {:+.3f}]'.format(*p['ci95']) if p['ci95'] else 'no bootstrap'
        out.append(f'{name:<10}{p["value"]:+8.3f} ± {p["se"]:.3f}  {ci}{mark}')
    return out + ['note: ' + c for c in caveats(params, objects)]


# ------------------------------------------------------------------ bag front end
def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def chain(statics, frame, base):
    """(R, t) base<-frame through the static tree."""
    R, t, at = np.eye(3), np.zeros(3), frame
    while at != base:
        if at not in statics:
            raise LookupError(f'no static TF {base} <- {frame} (stops at {at})')
        parent, Rp, tp = statics[at]
        R, t, at = Rp @ R, Rp @ t + tp, parent
    return R, t


def transform(tf):
    r, p = tf.transform.rotation, tf.transform.translation
    return rotation(r.x, r.y, r.z, r.w), np.array([p.x, p.y, p.z])


class Odom:
    """odom<-base poses from /tf, interpolated per axis (yaw unwrapped)."""

    def __init__(self, samples):
        lf = nav()
        samples = sorted(samples, key=lambda s: s[0])
        self.t = np.array([s[0] for s in samples])
        self.p = np.array([s[2] for s in samples]).reshape(-1, 3)
        self.rpy = np.unwrap(np.array([lf.matrix_rpy(s[1]) for s in samples]).reshape(-1, 3),
                             axis=0)

    def covers(self, lo, hi, max_gap):
        return len(self.t) > 1 and nav().tf_gap(self.t, lo, hi) <= max_gap

    def __call__(self, t):
        q = [np.interp(t, self.t, self.rpy[:, j]) for j in range(3)]
        return nav().rpy_matrix(*q), np.array([np.interp(t, self.t, self.p[:, j])
                                               for j in range(3)])


def frames_from_bag(bag, cfg):
    """(frames, counts) from a rosbag2 dir (one session or segment, bagread layout)."""
    lf, p = nav(), cfg['nav']
    obs_topic, info_topic = p['observations_topic'], p['camera_info_topic']
    statics, odom, msgs, K, image = {}, [], [], None, None
    for topic, msg, _ in bagread.read(bag, [obs_topic, info_topic, '/tf', '/tf_static']):
        if topic == '/tf_static':
            statics.update({tf.child_frame_id: (tf.header.frame_id, *transform(tf))
                            for tf in msg.transforms})
        elif topic == '/tf':
            odom += [(stamp(tf.header), *transform(tf)) for tf in msg.transforms
                     if tf.header.frame_id == p['odom_frame']
                     and tf.child_frame_id == p['base_frame']]
        elif topic == info_topic:
            if K is None and msg.k[0] > 0:
                K, image = (msg.k[0], msg.k[4], msg.k[2], msg.k[5]), (msg.width, msg.height)
        else:
            msgs.append(msg)
    if K is None:
        raise LookupError(f'{bag}: no camera_info on {info_topic} (set camera_info_topic '
                          'under lidar_fusion in fusion_calib.yaml)')
    if not msgs:
        raise LookupError(f'{bag}: no observations on {obs_topic}')
    msgs.sort(key=lambda m: stamp(m.header))
    frame = p['camera_frame'] or msgs[0].header.frame_id
    A = lf.axes_matrix(frame, p['camera_axes'])
    R_bc, t_bo = chain(statics, frame, p['base_frame'])
    R_bo = R_bc @ A.T                                  # base <- optical
    half = math.atan(image[0] / 2 / K[0]) + 5 * DEG
    scans = []
    for _, msg, _ in bagread.read(bag, [p['points_topic']]):
        R, t = chain(statics, msg.header.frame_id, p['base_frame'])
        xyz, tau = lf.cloud_arrays(msg, p['r_min_m'])
        pb = xyz @ R.T + t
        keep = lf.crop_mask(pb, t_bo, R_bo[:, 2], half)
        if keep.any():
            t_s = stamp(msg.header)
            scans.append({'t_s': t_s, 't_med': t_s + float(np.median(tau[keep])),
                          'p': pb[keep], 'tau': tau[keep]})
    odom = Odom(odom) if odom else None
    counts = {'frames': len(msgs), 'no_cloud': 0, 'compensated': 0, 'static_only': 0}
    frames = []
    for msg in msgs:
        t_c = stamp(msg.header) + p['image_time_offset_s']
        scan = lf.choose_scan(scans, t_c, p['scan_max_dt_s'])
        if scan is None:
            counts['no_cloud'] += 1
            continue
        pb, yaw_rate = scan['p'], 0.0
        t_i = scan['t_s'] + scan['tau']
        knot_t = np.sort(np.append(np.linspace(t_i.min(), t_i.max(), 5), t_c))
        if odom is not None and odom.covers(knot_t[0], knot_t[-1], p['tf_max_gap_s']):
            poses = [odom(t) for t in knot_t]
            pb = lf.compensate(pb, t_i, knot_t, lf.delta_knots(poses, odom(t_c)))
            yaws = np.unwrap([lf.matrix_rpy(R)[2] for R, _ in poses])
            span = knot_t[-1] - knot_t[0]
            yaw_rate = float((yaws[-1] - yaws[0]) / span) if span > 1e-3 else 0.0
            counts['compensated'] += 1
        else:
            counts['static_only'] += 1
        near = np.hypot(pb[:, 0], pb[:, 1]) < p['water_range_m']
        pb = pb[~(near & (pb[:, 2] < p['water_z_m'] + 0.05))]
        P = (pb - t_bo) @ R_bo
        frames.append(Frame(t_c, msg.observations, P[P[:, 2] > 0.1], K, image, A, yaw_rate))
    return frames, counts


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else ['key'])
        w.writeheader()
        w.writerows(rows)


def truth_arg(text):
    key, _, metres = text.rpartition('=')
    if not key:
        raise argparse.ArgumentTypeError(f'--truth-range wants ID=M, got {text!r}')
    return key, float(metres)


def run(args):
    cfg = load_config(args.config)
    frames, counts = frames_from_bag(args.bag, cfg)
    rows, est, sample_t = measure(frames, cfg)
    objects = summarize(rows, cfg['check']['near'])
    print(f'{args.bag}: {counts["frames"]} frames, {counts["no_cloud"]} without a cloud, '
          f'{counts["compensated"]} motion-compensated, {counts["static_only"]} static TF only; '
          f'{len(rows)} detections, {len(est.samples)} samples')
    out = Path(args.out or f'fusion_calib_{Path(args.bag).resolve().name}')
    if args.cmd in ('analyze', 'calibrate'):
        write_csv(out / 'detections.csv', rows)
        write_csv(out / 'objects.csv', objects)
        print(f'wrote {out}/detections.csv, {out}/objects.csv')
    params = calibrate(est, sample_t, cfg) if args.cmd != 'analyze' else None
    if args.cmd == 'calibrate':
        rec = record(rows, objects, est, params, args.bag)
        print('\n'.join(fit_lines(params, objects)))
        print('suggested camera delta (not applied):', rec['suggested_camera_delta'])
        (out / 'calibration.yaml').write_text(yaml.safe_dump(rec, sort_keys=False))
        print(f'wrote {out}/calibration.yaml')
    if args.cmd != 'check':
        if args.cmd == 'analyze':
            print(table(objects))
        return 0
    truth = dict(args.truth_range)
    unknown = set(truth) - {o['key'] for o in objects}
    if unknown:
        raise LookupError(f'--truth-range: no object {sorted(unknown)[0]} '
                          f'(objects: {", ".join(o["key"] for o in objects) or "none"})')
    baseline = yaml.safe_load(Path(args.baseline).read_text()) if args.baseline else None   # a result file calib wrote
    level, reasons, levels = verdict(objects, params, cfg, baseline, truth)
    print(table(objects, levels, truth))
    print('\n'.join(fit_lines(params, objects)))
    print(f'VERDICT {LEVELS[level]}' + (f': {"; ".join(reasons)}' if reasons else ''))
    return level


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    for name in ('analyze', 'calibrate', 'check'):
        s = sub.add_parser(name)
        s.add_argument('bag')
        s.add_argument('--config', default=CONFIG)
        s.add_argument('--out', help='output dir (default fusion_calib_<bag name>)')
        if name == 'check':
            s.add_argument('--baseline', help='a calibrate record to compare the rotation with')
            s.add_argument('--truth-range', type=truth_arg, action='append', default=[],
                           metavar='ID=M', help='taped distance of one object (repeatable)')
    args = parser.parse_args(argv)
    try:
        return run(args)
    except (LookupError, ValueError, OSError) as error:
        print(f'fusion_calib: {error}', file=sys.stderr)
        return 3


if __name__ == '__main__':
    sys.exit(main())

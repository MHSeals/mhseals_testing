"""Duplicate-track check: track ids while the boat spins in place near known buoys.

  python3 -m mhseals_testing.perception.track_dupes BAG [--buoys N] [--config YAML]

Reads the tracker's TrackArray and odom->base /tf from a rosbag2 dir. Per class it counts the
distinct live ids, the most confirmed live ids at once, and same-class pairs closer than
dupes.radius_m (one buoy tracked twice). Verdict, exit 0 PASS, 1 WARN, 2 FAIL (3 error):
  FAIL  a duplicate pair lasting dupes.persist_s or more, or more confirmed live ids at once
        than --buoys;
  WARN  a shorter duplicate pair, ids re-born beyond --buoys over the run (churn), or a yaw
        sweep under dupes.min_sweep_deg (insufficient evidence);
  PASS  otherwise.
Thresholds live in config/fusion_calib.yaml (dupes).
"""
import argparse
from collections import defaultdict
import math
import sys

import numpy as np

from mhseals_testing.harness import bagread
from mhseals_testing.perception import fusion_calib as fc

PASS, WARN, FAIL = fc.PASS, fc.WARN, fc.FAIL


def analyze(frames, yaw, cfg, buoys=None):
    """(level, reasons, per-class rows) from frames [(t, [(id, class, x, y, hits)])] of live
    tracks and yaw samples [(t, yaw_rad)]."""
    c = cfg['dupes']
    ids, conc, pairs = defaultdict(set), defaultdict(int), {}
    for t, tracks in frames:
        n = defaultdict(int)
        for i, (a, ca, xa, ya, ha) in enumerate(tracks):
            ids[ca].add(a)
            n[ca] += ha >= c['min_hits']
            for b, cb, xb, yb, _ in tracks[i + 1:]:
                if ca == cb and math.hypot(xa - xb, ya - yb) < c['radius_m']:
                    k = (ca, min(a, b), max(a, b))
                    pairs[k] = (pairs.get(k, (t, t))[0], t)
        for k, v in n.items():
            conc[k] = max(conc[k], v)
    rows, reasons = [], []
    for cls in sorted(ids):
        dur = [t1 - t0 for (k, *_), (t0, t1) in pairs.items() if k == cls]
        rows.append({'class': cls, 'ids': len(ids[cls]), 'max_live': conc[cls],
                     'dup_pairs': len(dur), 'longest_dup_s': max(dur, default=0.0)})
    for (cls, a, b), (t0, t1) in sorted(pairs.items()):
        lv = FAIL if t1 - t0 >= c['persist_s'] else WARN
        reasons.append((lv, f'{cls} ids {a}/{b} within {c["radius_m"]} m for {t1 - t0:.1f} s'))
    if buoys is not None:
        live, total = sum(r['max_live'] for r in rows), sum(r['ids'] for r in rows)
        if live > buoys:
            reasons.append((FAIL, f'{live} confirmed live ids at once for {buoys} buoys'))
        elif total > buoys:
            reasons.append((WARN, f'{total} ids over the run for {buoys} buoys (churn)'))
    sweep = 0.0
    if len(yaw) > 1:
        y = np.unwrap([v for _, v in sorted(yaw)])
        sweep = math.degrees(float(y.max() - y.min()))
    if sweep < c['min_sweep_deg']:
        reasons.append((WARN, f'yaw sweep {sweep:.0f} deg < {c["min_sweep_deg"]:.0f}: '
                              'insufficient evidence'))
    if not frames:
        reasons.append((FAIL, 'no track messages'))
    level = max([lv for lv, _ in reasons] + [PASS])
    return level, [r for lv, r in reasons if lv == level], rows, sweep


def from_bag(bag, cfg, topic):
    p = cfg['nav']
    frames, yaw = [], []
    for t, msg, _ in bagread.read(bag, [topic, '/tf']):
        if t == '/tf':
            for tf in msg.transforms:
                if tf.header.frame_id == p['odom_frame'] and \
                        tf.child_frame_id == p['base_frame']:
                    q = tf.transform.rotation
                    yaw.append((fc.stamp(tf.header), math.atan2(
                        2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        else:
            frames.append((fc.stamp(msg.header), [
                (k.id, k.class_id, k.pose.pose.position.x, k.pose.pose.position.y, k.hits)
                for k in msg.tracks if k.source == 0]))       # SOURCE_LIVE only
    return frames, yaw


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--buoys', type=int, help='physical buoys in view during the spin')
    ap.add_argument('--config', default=fc.CONFIG)
    a = ap.parse_args(argv)
    try:
        cfg = fc.load_config(a.config)
        frames, yaw = from_bag(a.bag, cfg, cfg['dupes']['tracks_topic'])
        level, reasons, rows, sweep = analyze(frames, yaw, cfg, a.buoys)
    except Exception as e:      # noqa: BLE001 - one clear line, exit 3
        print(f'error: {e}', file=sys.stderr)
        return 3
    print(f'{a.bag}: {len(frames)} track messages, yaw sweep {sweep:.0f} deg')
    print(f'{"class":<22}{"ids":>5}{"max live":>10}{"dup pairs":>11}{"longest s":>11}')
    for r in rows:
        print(f'{r["class"][:21]:<22}{r["ids"]:>5}{r["max_live"]:>10}{r["dup_pairs"]:>11}'
              f'{r["longest_dup_s"]:>11.1f}')
    print(f'VERDICT {fc.LEVELS[level]}' + (f': {"; ".join(reasons)}' if reasons else ''))
    return level


if __name__ == '__main__':
    sys.exit(main())

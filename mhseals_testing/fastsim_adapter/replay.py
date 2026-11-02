"""Bag replay: a session's recorded inputs through the boat's tracker and mission logic offline,
with a candidate config, diffed against what the boat did.

    python3 -m mhseals_testing.fastsim_adapter.replay SESSION OUT [--tune JSON] [--config YAML]

No ROS: MCAP messages are decoded with mcap_ros2 and fed to mhseals_nav's pure cores.
  tracker   recorded observations -> TrackFilter (tracker.yaml + ``tune.tracker``), the boat's TF;
            diffed against the recorded tracks at each recorded tracks message
  mission   the tracks (replayed when the tracker layer ran, else recorded) and the recorded pose
            -> MissionHost (``tune.task``, ``tune.profile``, ``tune.params``); its task events and
            goals diffed against the recorded ones
The mission layer is open loop: the boat's own track carries on whatever the candidate decides,
so the diff is exact up to the first divergence and indicative after it. A layer whose inputs the
bag lacks is reported unavailable, never guessed. OUT/diff.json holds the result,
OUT/metrics.json its flat numbers (cluster metrics: ``cluster.ReplayBackend``).
"""

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

from mhseals_testing.fastsim_adapter.bag import Tf, messages
from mhseals_testing.fastsim_adapter.cluster import merge

CONFIG = Path(__file__).resolve().parents[2] / 'config' / 'fastsim_replay.yaml'


def match(replayed, recorded, radius):
    """Greedy nearest same-label pairs within ``radius``: (pairs [(a, b, d)], extra, missed)."""
    cand = sorted((math.hypot(a.x - b.x, a.y - b.y), i, j) for i, a in enumerate(replayed)
                  for j, b in enumerate(recorded) if a.label == b.label)
    used_a, used_b, pairs = set(), set(), []
    for d, i, j in cand:
        if d <= radius and i not in used_a and j not in used_b:
            used_a.add(i)
            used_b.add(j)
            pairs.append((replayed[i], recorded[j], d))
    return pairs, len(replayed) - len(used_a), len(recorded) - len(used_b)


class Recorder:
    """The Navigator contract, recording goals: nothing moves (the bag's pose is the boat's)."""

    def __init__(self, clock):
        self.clock, self.goals, self.report, self.goal = clock, [], None, None

    def send(self, route, precise=False, quiet=False, why='', reverse=None):
        from fastsim.plant import Goal
        if route and not quiet:
            x, y, yaw = route[-1]
            self.goals.append({'t': self.clock['now'], 'x': round(x, 2), 'y': round(y, 2), 'why': why})
            if self.report is not None:
                self.report('goal', x=round(x, 2), y=round(y, 2), yaw=round(yaw, 2), why=why)
        self.goal = Goal(route, precise, why, reverse)
        return self.goal

    def hold(self):
        if self.goal is not None:
            self.goal.cancel()

    def speed_limit(self, speed):
        pass

    def keep_still(self, quiet_s=0.3):
        pass

    def controller_vx_max(self):
        return 1.0

    def ready(self):
        return True

    def close(self):
        self.hold()


def tracker_cfg(overrides):
    import mhseals_nav
    path = Path(mhseals_nav.__file__).resolve().parents[1] / 'config' / 'tracker.yaml'
    return merge(next(iter(yaml.safe_load(path.read_text()).values()))['ros__parameters'], overrides or {})


def replay(stream, cfg, tune):
    """Run the layers over ``stream`` [(topic, t, msg)]; returns the diff (module docstring)."""
    from mhseals_nav.tasks import tracks as navtracks
    topics, frames, radius = cfg['topics'], cfg['frames'], float(cfg['diff']['match'])
    layers = set(tune['layers'])
    tf, events, recorded_tracks, observations = Tf(), [], [], []
    for topic, t, msg in stream:
        if topic in ('/tf', '/tf_static'):
            tf.add(msg, topic == '/tf_static')
        elif topic == topics['observations']:
            observations.append((t, msg))
        elif topic == topics['tracks']:
            recorded_tracks.append((t, navtracks.from_track_array(msg)))
        elif topic == topics['events']:
            events.append(json.loads(msg.data))
    out = {'layers': {}, 'counts': {'observations': len(observations), 'tracks': len(recorded_tracks),
                                    'events': len(events)}}

    def map_pose(t):
        R, p = tf.pose_at(t, frames['map'], frames['base'])
        return float(p[0]), float(p[1]), math.atan2(R[1, 0], R[0, 0])
    replayed = {}
    if 'tracker' in layers:
        if not observations:
            out['layers']['tracker'] = {'available': False, 'why': f"no {topics['observations']} recorded"}
        else:
            replayed = run_tracker(tf, frames, observations, recorded_tracks, tracker_cfg(tune.get('tracker')))
            out['layers']['tracker'] = diff_tracks(replayed, recorded_tracks, radius)
    if 'mission' in layers:
        if not recorded_tracks:
            out['layers']['mission'] = {'available': False, 'why': f"no {topics['tracks']} recorded"}
        else:
            feed = [(t, replayed.get(t, [])) for t, _ in recorded_tracks] if replayed else recorded_tracks
            out['layers']['mission'] = run_mission(feed, map_pose, events, tune, float(cfg['diff']['reached']))
    return out


def run_tracker(tf, frames, observations, recorded_tracks, cfg):
    """{recorded tracks time: replayed tracks in the map frame} (TrackFilter, the boat's TF)."""
    from mhseals_nav.tasks.tracks import Track
    from mhseals_nav.track_filter import MOTION_STATIC, SOURCE_PRIOR, TrackFilter
    filt, out, k = TrackFilter(cfg), {}, 0
    odom = cfg.get('tracking_frame', frames['odom'])

    def pose_at(t, frame):
        return tf.pose_at(t, odom, frame)
    for t_rec, _ in recorded_tracks:
        while k < len(observations) and observations[k][0] <= t_rec:
            filt.step(observations[k][1], observations[k][0], pose_at)
            k += 1
        filt.expire(t_rec)
        try:
            R, p = tf.pose_at(t_rec, frames['map'], odom)
        except LookupError:
            continue
        out[t_rec] = [Track(tr.class_id, *(R @ np.array([tr.x[0], tr.x[1], 0.0]) + p)[:2],
                            float(filt.confidence(tr, t_rec)), None, tr.motion == MOTION_STATIC,
                            tr.source == SOURCE_PRIOR, str(tr.id)) for tr in filt.tracks]
    return out


def diff_tracks(replayed, recorded_tracks, radius):
    n = extra = missed = 0
    errors = []
    for t, rec in recorded_tracks:
        if t not in replayed:
            continue
        pairs, e, m = match(replayed[t], rec, radius)
        n, extra, missed = n + len(rec), extra + e, missed + m
        errors += [d for _, _, d in pairs]
    return {'available': True, 'frames': len(replayed), 'recorded': n, 'extra': extra, 'missed': missed,
            'matched': round(1 - missed / n, 4) if n else None,
            'error': round(float(np.mean(errors)), 4) if errors else None}


def segments(events, tune, t0):
    """[(task, start)]: each task the boat started (``tune.task`` replaces their names), else one."""
    starts = [(e.get('task'), e.get('time', t0)) for e in events if e.get('event') == 'task_started']
    starts = starts or [(None, min((e.get('time', math.inf) for e in events), default=t0))]
    return [(tune.get('task') or task or 'mission', t) for task, t in starts]


def run_mission(feed, map_pose, events, tune, reached=1.0):
    """MissionHost on the recorded clock, one host per task the boat started (open loop: a goal
    succeeds when the boat's recorded pose comes within ``reached`` of its end)."""
    import py_trees
    from mhseals_nav.tasks.host import MissionHost
    from mhseals_testing.fastsim_adapter.adapter import mission_params
    clock, sink, ticks, outcome = {'now': feed[0][0]}, [], 0, None
    nav = Recorder(clock)
    parts = segments(events, tune, feed[0][0])
    for k, (task, start) in enumerate(parts):
        stop = parts[k + 1][1] if k + 1 < len(parts) else math.inf
        params = mission_params(task, None, tune.get('profile'), tune.get('params'))
        py_trees.blackboard.Blackboard.clear()
        host = MissionHost(task, params, nav, lambda p: sink.append({**p, 'received': clock['now']}),
                           lambda: clock['now'])
        for t, tracks in feed:
            if not start <= t < stop:
                continue
            clock['now'] = t
            try:
                pose = map_pose(t)
            except LookupError:
                continue
            ticks += 1
            goal = nav.goal
            if goal is not None and not goal.done and goal.route and \
                    math.hypot(goal.route[-1][0] - pose[0], goal.route[-1][1] - pose[1]) <= reached:
                goal.end('SUCCEEDED')  # the boat's own track reached it
            outcome = host.tick(pose, tracks, None)
            if outcome is not None:
                break
    start = parts[0][1]
    decided = lambda rows: [e for e in rows if e.get('event') != 'goal']  # noqa: E731
    rec_rows, rep_rows = decided(events), decided(sink)
    rec, rep = ([(e.get('task'), e.get('event')) for e in rows] for rows in (rec_rows, rep_rows))
    same = next((i for i, (a, b) in enumerate(zip(rec, rep)) if a != b), min(len(rec), len(rep)))
    diverged = same < max(len(rec), len(rep))
    at = rep_rows[same] if same < len(rep_rows) else rec_rows[same] if same < len(rec_rows) else {}
    first = at.get('received', at.get('time'))
    goals_rec = [e for e in events if e.get('event') == 'goal']
    offsets = [math.hypot(a['x'] - b['x'], a['y'] - b['y']) for a, b in zip(goals_rec, nav.goals)]
    return {'available': True, 'ticks': ticks, 'outcome': outcome, 'events_recorded': len(rec),
            'events_replayed': len(rep), 'events_same_prefix': same, 'identical': float(not diverged),
            'diverged_after': round(first - start, 3) if diverged and first is not None else None,
            'goals_recorded': len(goals_rec), 'goals_replayed': len(nav.goals),
            'goal_offset': round(float(np.mean(offsets)), 4) if offsets else None,
            'replayed': rep[:200], 'recorded': rec[:200]}


def numbers(diff):
    out = {}
    for layer, d in diff['layers'].items():
        out[f'{layer}.available'] = float(d.get('available', False))
        out.update({f'{layer}.{k}': float(v) for k, v in d.items()
                    if isinstance(v, (int, float)) and not isinstance(v, bool)})
    return out


def run(session, out, tune=None, config=None):
    cfg = yaml.safe_load(Path(config or CONFIG).read_text())
    t = merge(cfg['tune'], tune or {})
    topics = {*cfg['topics'].values(), '/tf', '/tf_static'}
    diff = replay(messages(session, topics), cfg, t)
    diff.update(session=str(session), tune=t)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'diff.json').write_text(json.dumps(diff, indent=1, default=str) + '\n')
    (out / 'metrics.json').write_text(json.dumps(numbers(diff), indent=1, sort_keys=True) + '\n')
    return diff


def main(argv=None):
    p = argparse.ArgumentParser(prog='fastsim_adapter.replay', description=__doc__.split('\n')[0])
    p.add_argument('session', type=Path, help='a session folder (its MCAP files) or one MCAP file')
    p.add_argument('out', type=Path)
    p.add_argument('--tune', type=json.loads, help='tune overrides (JSON)')
    p.add_argument('--config', type=Path, help='default: config/fastsim_replay.yaml')
    a = p.parse_args(argv)
    diff = run(a.session, a.out, a.tune, a.config)
    print(json.dumps({k: {x: y for x, y in v.items() if not isinstance(y, list)}
                      for k, v in diff['layers'].items()}))
    return 0


if __name__ == '__main__':
    sys.exit(main())

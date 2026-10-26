"""Score a finished session by the RoboBoat 2026 rules, headless: ``boat_trial analyze score``.

Reads files only (no ROS): ``course_truth.json``, ``track.jsonl`` (ground truth ``{t, x, y,
yaw}``, captured by the harness from its run start), authority changes from ``events.jsonl``
and optional ``states.jsonl`` (course state changes). The run window is the harness's: it
starts at ``session.json`` ``run_start_t`` (else the first track sample) and ends at the time
limit or the track end; the judge cuts it earlier at the exit, a hold, a takeover or out of
bounds. ``task_events.jsonl`` holds the boat's claims, never used to judge or to window. Writes
``<session>/analysis/score.<ts>.json`` (schema ``mhseals.score/1``, docs in the module
:mod:`mhseals_testing.analysis.score`). Exit 0 when scored, 2 when it cannot be.
"""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import sys

from mhseals_testing import data
from mhseals_testing.analysis import diagnostics, score

CLAIM_EVENTS = ('stage_completed', 'stage_failed', 'stage_skipped', 'gate_cleared', 'docked',
                'undocked', 'debris', 'speed_started', 'speed_finished', 'returned_home')
STAGE_TASK = {'nav_channel': 'entry_gates', 'return_home': 'entry_gates',
              'follow_path': 'debris_clearance', 'docking': 'marina'}   # 2025 stage names


class Refused(Exception):
    """The session cannot be scored; the message says why."""


def _claimed_window(task_events):
    """The boat's own task_started .. task_finished: a claim, reported beside the run only."""
    start = next((e['time'] for e in task_events if e.get('event') == 'task_started'), None)
    end = next((e['time'] for e in task_events if e.get('event') == 'task_finished'), None)
    return start, end


def run_window(track, meta):
    """(t0, t1, end reason) from the harness: run start, then the time limit or track end."""
    t0 = meta.get('run_start_t', track[0][0])
    limit = meta.get('time_limit_s') or score.PARAMS['time_limit_s']
    if t0 + limit < track[-1][0]:
        return t0, t0 + limit, 'time_limit'
    return t0, track[-1][0], 'track_end'


def _authority(events, t_start):
    """Owner changes in track time, aligned on the harness's mission_start = run start."""
    started = next((e for e in events if e.get('phase') == 'mission_start'), None)
    if started is None or t_start is None or 'ros_time' not in started:
        return [], 'authority events not aligned to track time (no mission_start)'
    offset = t_start - started['ros_time']
    out = []
    for e in events:
        if e.get('phase') == 'authority':
            out.append(dict(t=e['ros_time'] + offset, by=(e.get('to') or {}).get('owner'),
                            cause=e.get('cause')))
        elif e.get('phase') == 'disarmed':
            out.append(dict(t=e['ros_time'] + offset, by='external', cause='disarmed'))
    return out, None


def load(session):
    session = Path(session)
    if not session.is_dir():
        raise Refused(f'{session}: no such session')
    truth = data.read_json(session / 'course_truth.json')
    if truth is None:
        raise Refused(f'{session.name}: no course_truth.json')
    track = [(r['t'], r['x'], r['y'], r.get('yaw') or 0.0)
             for r in data.read_jsonl(session / 'track.jsonl') if {'t', 'x', 'y'} <= set(r)]
    if not track:
        raise Refused(f'{session.name}: no track.jsonl samples')
    task_events = list(data.read_jsonl(session / 'task_events.jsonl'))
    events = list(data.read_jsonl(session / 'events.jsonl'))
    changes = list(data.read_jsonl(session / 'states.jsonl'))
    return truth, track, task_events, events, changes


def statuses(session):
    """The boat's /task/status records (mhseals.mission_status/1), if the session kept them."""
    return list(data.read_jsonl(Path(session) / 'task_status.jsonl'))


def claims(task_events, report):
    """The boat's claims beside what ground truth credits, per task (claims never score)."""
    said = [dict(e, t=e.get('time')) for e in task_events
            if e.get('event') in CLAIM_EVENTS + ('decision', 'stage_started', 'spill_count',
                                                 'species_count')]
    out = {}
    for c in said:
        if c['event'] != 'stage_completed' or not c['stage']:
            continue
        task = STAGE_TASK.get(c['stage'], c['stage'])
        judged = report['tasks'].get(task)
        done = bool(judged) and any(r['status'] in ('earned', 'provisional')
                                    for r in judged['rows'])
        out[c['stage']] = dict(task=task, claimed=True, credited=done, false_claim=not done)
    for task, judged in report['tasks'].items():
        credited = any(r['status'] in ('earned', 'provisional') for r in judged['rows'])
        if credited and not any(v['task'] == task for v in out.values()):
            out[task] = dict(task=task, claimed=False, credited=True, missed_claim=True)
    return dict(events=said, by_stage=out)


def motion(track, t0, t1):
    """Path length, time, average/peak speed and time stopped (< 0.1 m/s) in the window."""
    part = [p for p in track if t0 <= p[0] <= t1]
    length = stopped = peak = 0.0
    j = 0
    for i, (a, b) in enumerate(zip(part, part[1:])):
        dt, d = b[0] - a[0], math.dist(a[1:3], b[1:3])
        length += d
        stopped += dt if dt > 0 and d / dt < 0.1 else 0.0
        j = max(j, i)
        while j < len(part) - 1 and part[j][0] - a[0] < 0.5:   # speed over >= 0.5 s
            j += 1
        if part[j][0] - a[0] >= 0.5:
            peak = max(peak, math.dist(a[1:3], part[j][1:3]) / (part[j][0] - a[0]))
    span = (part[-1][0] - part[0][0]) if len(part) > 1 else 0.0
    return dict(path_m=round(length, 1), time_s=round(span, 1),
                mean_speed=round(length / span, 3) if span else None,
                peak_speed=round(peak, 3), stopped_s=round(stopped, 1))


def analyze(session, now=None):
    truth, track, task_events, events, changes = load(session)
    meta = data.read_json(Path(session) / 'session.json', {}) or {}
    t0, t1, reason = run_window(track, meta)
    window = (t0, t1)
    authority, problem = _authority(events, t0)
    body = score.judge(truth, track, window=window, authority=authority, changes=changes,
                       end_reason=reason)
    if problem:
        body['run']['warnings'].append(problem)
    body['run']['window_source'] = ('session.json run_start_t' if 'run_start_t' in meta
                                    else 'harness track start') + ' + ' + \
        ('session.json time_limit_s' if meta.get('time_limit_s') else 'PARAMS time_limit_s')
    body['run']['claimed_window'] = list(_claimed_window(task_events))   # the boat's, unused
    gen = truth.get('generator') or {}
    source = meta.get('source') or ('kinematic' if meta.get('kind') == 'kinematic' else 'unity')
    body['run']['source'] = source
    physics = ' (no physics)' if source == 'kinematic' else ''
    body['penalties']['contact_basis'] = 'geometry' + physics
    body['run'].update(session=Path(session).name, course=dict(
        name=truth.get('name'), family=truth.get('family'), seed=gen.get('seed'),
        mode=gen.get('mode'), truth_sha256=gen.get('truth_sha256')))
    body['claims'] = claims(task_events, body)
    body['diagnostics'] = dict(motion=motion(track, window[0], body['run']['scored_until']))
    model = score.course(truth)
    run = score.with_heading([p for p in track
                              if window[0] <= p[0] <= body['run']['scored_until']])
    program = (model['states'].get('harbor_alert.horn') or {}).get('program')
    trigger = score.horn_sounded(changes, program)[0]
    if run:
        body['diagnostics'].update(diagnostics.diagnose(run, model, body, task_events,
                                                        statuses(session), trigger))
        body['penalties']['contacts'] = body['diagnostics']['contacts']['counts']
    record = dict(summary=score.summary(body),
                  scorer=dict(version=score.VERSION, rules=score.RULES,
                              handbook_sha256=(truth.get('handbook') or {}).get('sha256')),
                  **body)
    stamp = (now or datetime.now().astimezone()).strftime('%Y%m%dT%H%M%S')
    target = Path(session) / 'analysis' / f'score.{stamp}.json'
    data.write_json(target, record, score.SCHEMA)
    return target, record


def metrics(record):
    """The per-run numbers a batch aggregates."""
    out = dict(points=record['total']['points'], provisional=record['total']['provisional'],
               strikes=len(record['penalties']['strikes']),
               entered=float(record['run']['entry_t'] is not None),
               duration_s=record['run']['duration_s'])
    out.update({f'task.{k}': v for k, v in record['total']['by_task'].items()})
    diag = record.get('diagnostics', {})
    out.update({k: v for k, v in diag.get('motion', {}).items() if isinstance(v, (int, float))})
    touch = diag.get('contacts') or {}
    for kind, n in (touch.get('counts') or {}).items():
        out[f'contacts.{kind}'] = n
    end = record['run']['end']['reason']
    out['exited'] = float(record['run'].get('exit_t') is not None)
    ends = ('time_limit', 'intervention', 'out_of_bounds', 'track_end')
    out.update({f'end.{k}': float(k == end) for k in ends})
    if touch:
        out['contact_free'] = float(not touch.get('contacts'))
        out['safe_exit'] = float(out['exited'] and not touch.get('contacts')
                                 and not record['penalties']['out_of_bounds']
                                 and not record['penalties']['interventions'])
        out['near_misses'] = touch.get('near_miss_count')
        out['min_clearance_m'] = touch.get('min_clearance_m')
    for tid, row in (diag.get('timeline') or {}).items():
        out[f'completed.{tid}'] = float(row.get('outcome') == 'completed')
        if row.get('outcome') == 'completed':        # success-conditional: n differs per task
            out[f'time_s.{tid}'] = round(row['end_t'] - row['encounter_t'], 2)
        if row.get('encounter_t') is not None:      # every encounter; incomplete at the cutoff
            end_t = row['end_t'] if row.get('outcome') == 'completed' else \
                record['run']['scored_until']
            out[f'time_cut_s.{tid}'] = round(end_t - row['encounter_t'], 2)
    return out


BINARY = ('entered', 'exited', 'contact_free', 'safe_exit', 'end.', 'completed.')
LABELS = {
    'time_s.*': 'completed runs only (survivorship): read beside completed.* (completion rate)',
    'time_cut_s.*': 'every run that encountered the task; an incomplete run counts at its '
                    'cutoff, so its mean is a lower bound',
    'binary': 'rates (entered, exited, contact_free, safe_exit, end.*, completed.*): Wilson '
              'interval; differences: Newcombe hybrid score interval',
    'clear': 'interval excludes 0, unadjusted; clear_holm: Holm-adjusted p < 0.05 over every '
             'metric compared (one family)'}


def _betacf(a, b, x):
    """Continued fraction of the regularized incomplete beta (Numerical Recipes 6.4)."""
    c, d = 1.0, 1.0 - (a + b) * x / (a + 1)
    d = 1 / (d if abs(d) > 1e-300 else 1e-300)
    h = d
    for m in range(1, 300):
        for num in (m * (b - m) * x / ((a + 2 * m - 1) * (a + 2 * m)),
                    -(a + m) * (a + b + m) * x / ((a + 2 * m) * (a + 2 * m + 1))):
            d = 1 + num * d
            d = 1 / (d if abs(d) > 1e-300 else 1e-300)
            c = 1 + num / c
            c = c if abs(c) > 1e-300 else 1e-300
            h *= d * c
        if abs(d * c - 1) < 1e-12:
            break
    return h


def t_sf(t, df):
    """Two-sided p of Student's t with ``df`` (real) degrees of freedom."""
    x = df / (df + t * t)
    lb = math.lgamma(df / 2 + 0.5) - math.lgamma(df / 2) - math.lgamma(0.5)
    front = math.exp(lb + (df / 2) * math.log(x) + 0.5 * math.log1p(-x)) if 0 < x < 1 else 0.0
    if x <= 0:
        return 0.0
    if x < (df / 2 + 1) / (df / 2 + 1.5):
        return front * _betacf(df / 2, 0.5, x) / (df / 2)
    return 1 - front * _betacf(0.5, df / 2, 1 - x) / 0.5


def t_quantile(df, p=0.975):
    """t with P(T <= t) = p, by bisection on the exact distribution (df may be fractional)."""
    if df <= 0:
        return math.inf
    lo, hi = 0.0, 1e3
    for _ in range(200):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if t_sf(mid, df) / 2 > 1 - p else (lo, mid)
    return (lo + hi) / 2


def _norm_p(z):
    return math.erfc(abs(z) / math.sqrt(2))


def wilson(k, n, z=1.959964):
    """Wilson score interval for k successes in n."""
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return max(0.0, centre - half), min(1.0, centre + half)


def stats(values, binary=False):
    """n, mean, sd and a 95% interval: Wilson for a rate, else exact-t for the mean."""
    n = len(values)
    mean = sum(values) / n
    sd = math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1)) if n > 1 else 0.0
    out = dict(n=n, mean=round(mean, 3), sd=round(sd, 3))
    if binary:
        k = round(sum(values))
        lo, hi = wilson(k, n)
        return dict(out, k=k, ci95=[round(lo, 3), round(hi, 3)], interval='wilson')
    half = t_quantile(n - 1) * sd / math.sqrt(n) if n > 1 else None
    out['ci95'] = None if half is None else [round(mean - half, 3), round(mean + half, 3)]
    out['interval'] = 't'
    if n > 1 and sd == 0:
        out['note'] = 'sd=0: no spread observed; the interval is not a precision claim'
    return out


def batch(records):
    """Aggregate per metric across runs; sources and course seeds are listed."""
    rows = [metrics(r) for r in records]
    keys = sorted({k for r in rows for k in r})
    out = dict(runs=len(records),
               sources=sorted({r['run'].get('source', 'unity') for r in records}),
               seeds=sorted({str(r['run'].get('course', {}).get('seed')) for r in records}),
               metrics={k: stats([r[k] for r in rows if r.get(k) is not None],
                                 binary=k.startswith(BINARY)) for k in keys
                        if any(r.get(k) is not None for r in rows)}, labels=LABELS)
    out['time_by_task'] = {k[len('completed.'):]: dict(
        completion=out['metrics'][k],
        time_s_completed_only=out['metrics'].get('time_s.' + k[len('completed.'):]),
        time_cut_s_incl_incomplete=out['metrics'].get('time_cut_s.' + k[len('completed.'):]))
        for k in out['metrics'] if k.startswith('completed.')}
    return out


def mixing(a, b):
    """Why two batches are not like for like (empty when they are)."""
    why = [f'batch {name} mixes sources {x["sources"]}' for name, x in (('a', a), ('b', b))
           if len(x.get('sources', [])) > 1]
    if set(a.get('sources', [])) != set(b.get('sources', [])):
        why.append(f'sources differ: {a.get("sources")} vs {b.get("sources")}')
    if set(a.get('seeds', [])) != set(b.get('seeds', [])):
        why.append(f'seed sets differ: {a.get("seeds")} vs {b.get("seeds")}')
    return why


def _holm(out):
    tests = sorted((v['p'], k) for k, v in out.items() if v.get('p') is not None)
    m, running = len(tests), 0.0
    for i, (p, k) in enumerate(tests):
        running = max(running, min(1.0, (m - i) * p))
        out[k]['p_holm'] = round(running, 4)
        out[k]['clear_holm'] = running < 0.05
    for v in out.values():
        v.setdefault('clear_holm', False)


def compare(a, b, force=False):
    """Batch b minus batch a per metric. Rates: Newcombe hybrid score interval and a pooled
    two-proportion z p; other metrics: Welch with the exact t quantile. Refuses batches that
    mix sources or differ in seed set unless ``force`` (the reasons are then kept in
    ``_forced``)."""
    why = mixing(a, b)
    if why and not force:
        raise Refused('not like for like: ' + '; '.join(why) + ' (--force to compare anyway)')
    out = {}
    for k in sorted(set(a['metrics']) & set(b['metrics'])):
        x, y = a['metrics'][k], b['metrics'][k]
        diff = y['mean'] - x['mean']
        ci = p = None
        if x.get('interval') == 'wilson' and y.get('interval') == 'wilson':
            l1, u1 = wilson(x['k'], x['n'])
            l2, u2 = wilson(y['k'], y['n'])
            p1, p2 = x['k'] / x['n'], y['k'] / y['n']
            diff = p2 - p1
            ci = [round(diff - math.hypot(p2 - l2, u1 - p1), 3),
                  round(diff + math.hypot(u2 - p2, p1 - l1), 3)]
            pool = (x['k'] + y['k']) / (x['n'] + y['n'])
            se = math.sqrt(pool * (1 - pool) * (1 / x['n'] + 1 / y['n']))
            p = _norm_p(diff / se) if se > 0 else None
        else:
            va, vb = x['sd'] ** 2 / x['n'], y['sd'] ** 2 / y['n']
            var = va + vb
            if var > 0 and x['n'] > 1 and y['n'] > 1:
                df = var ** 2 / (va ** 2 / (x['n'] - 1) + vb ** 2 / (y['n'] - 1))
                half = t_quantile(df) * math.sqrt(var)
                ci = [round(diff - half, 3), round(diff + half, 3)]
                p = t_sf(abs(diff) / math.sqrt(var), df)
        out[k] = dict(diff=round(diff, 3), ci95=ci, p=None if p is None else round(p, 4),
                      clear=bool(ci) and (ci[0] > 0 or ci[1] < 0))
        if ci is None:
            out[k]['note'] = 'no interval: sd=0 in both batches or n < 2'
    _holm(out)
    if why:
        out['_forced'] = why
    return out


def _records(paths):
    sessions = [p for root in paths for p in sorted(Path(root).glob('**/track.jsonl'))]
    return [analyze(p.parent)[1] for p in sessions]


def main(argv=None, out=None):
    parser = argparse.ArgumentParser(prog='boat_trial analyze score')
    parser.add_argument('session', help='a session, or with --batch a directory of sessions')
    parser.add_argument('--json', action='store_true', help='print the record that was written')
    parser.add_argument('--batch', action='store_true', help='score every session under it')
    parser.add_argument('--compare', metavar='DIR', help='with --batch: a second batch (B - A)')
    parser.add_argument('--force', action='store_true',
                        help='compare batches that mix sources or differ in seed set')
    args = parser.parse_args(argv)
    if args.batch:
        try:
            first = batch(_records([args.session]))
            result = dict(schema='mhseals.score_batch/1', a=first)
            if args.compare:
                second = batch(_records([args.compare]))
                result.update(b=second, b_minus_a=compare(first, second, args.force))
                if args.force and mixing(first, second):
                    print('FORCED comparison, not like for like: ' +
                          '; '.join(mixing(first, second)), file=sys.stderr)
                    result['forced'] = mixing(first, second)
        except (Refused, ZeroDivisionError) as error:
            print(f'refused: {error or "no sessions"}', file=sys.stderr)
            return 2
        print(json.dumps(result, indent=2), file=out or sys.stdout)
        return 0
    try:
        target, _ = analyze(args.session)
    except Refused as error:
        print(f'refused: {error}', file=sys.stderr)
        return 2
    record = json.loads(target.read_text())
    print(json.dumps(record, indent=2) if args.json else record['summary'], file=out or sys.stdout)
    return 0

"""Score a finished session by the RoboBoat 2026 rules, headless: ``boat_trial analyze score``.

Reads files only (no ROS): ``course_truth.json``, ``track.jsonl`` (ground truth ``{t, x, y,
yaw}``), the run window from ``task_events.jsonl`` (``task_started`` .. ``task_finished``;
the boat's other events are kept as its claims, never used to judge), authority changes
from ``events.jsonl`` and optional ``states.jsonl`` (course state changes). Writes
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


def _window(task_events):
    start = next((e['time'] for e in task_events if e.get('event') == 'task_started'), None)
    end = next((e['time'] for e in task_events if e.get('event') == 'task_finished'), None)
    return start, end


def _authority(events, task_events):
    """Owner changes in track time, aligned on mission_start (ros_time) = task_started."""
    started = next((e for e in events if e.get('phase') == 'mission_start'), None)
    t_start = _window(task_events)[0]
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
    start, end = _window(task_events)
    window = (start if start is not None else track[0][0],
              end if end is not None else track[-1][0])
    authority, problem = _authority(events, task_events)
    body = score.judge(truth, track, window=window, authority=authority, changes=changes)
    if problem:
        body['run']['warnings'].append(problem)
    if start is None:
        body['run']['warnings'].append('no task_started event: the whole track is the run')
    gen = truth.get('generator') or {}
    meta = data.read_json(Path(session) / 'session.json', {}) or {}
    source = meta.get('source') or ('kinematic' if meta.get('kind') == 'kinematic' else 'unity')
    body['run']['source'] = source
    body['penalties']['contact_basis'] = 'geometry' + (' (no physics)' if source == 'kinematic'
                                                        else '')
    body['run'].update(session=Path(session).name, course=dict(
        name=truth.get('name'), family=truth.get('family'), seed=gen.get('seed'),
        mode=gen.get('mode'), truth_sha256=gen.get('truth_sha256')))
    body['claims'] = claims(task_events, body)
    body['diagnostics'] = dict(motion=motion(track, window[0], body['run']['scored_until']))
    model = score.course(truth)
    run = score.with_heading([p for p in track
                              if window[0] <= p[0] <= body['run']['scored_until']])
    trigger = ((model['states'].get('harbor_alert.horn') or {}).get('program') or {}) \
        .get('trigger_s')
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
    out.update({f'end.{k}': float(k == end) for k in ('finished', 'intervention',
                                                     'out_of_bounds', 'track_end')})
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
    return out


T95 = {1: 12.71, 2: 4.30, 3: 3.18, 4: 2.78, 5: 2.57, 6: 2.45, 7: 2.36, 8: 2.31, 9: 2.26,
       10: 2.23, 15: 2.13, 20: 2.09, 30: 2.04, 60: 2.00}


def _t95(df):
    return T95[min((k for k in T95 if k >= df), default=60)] if df >= 1 else math.inf


def stats(values):
    """n, mean, sd and the 95% t interval of the mean."""
    n = len(values)
    mean = sum(values) / n
    sd = math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1)) if n > 1 else 0.0
    half = _t95(n - 1) * sd / math.sqrt(n) if n > 1 else None
    return dict(n=n, mean=round(mean, 3), sd=round(sd, 3),
                ci95=None if half is None else [round(mean - half, 3), round(mean + half, 3)])


def batch(records):
    """Aggregate per metric across runs; sources and course seeds are listed."""
    rows = [metrics(r) for r in records]
    keys = sorted({k for r in rows for k in r})
    return dict(runs=len(records),
                sources=sorted({r['run'].get('source', 'unity') for r in records}),
                seeds=sorted({str(r['run'].get('course', {}).get('seed')) for r in records}),
                metrics={k: stats([r[k] for r in rows if r.get(k) is not None]) for k in keys
                         if any(r.get(k) is not None for r in rows)})


def compare(a, b):
    """Batch b minus batch a per metric: difference of means with a Welch 95% interval."""
    out = {}
    for k in sorted(set(a['metrics']) & set(b['metrics'])):
        x, y = a['metrics'][k], b['metrics'][k]
        diff = y['mean'] - x['mean']
        var = x['sd'] ** 2 / x['n'] + y['sd'] ** 2 / y['n']
        if var > 0 and x['n'] > 1 and y['n'] > 1:
            df = var ** 2 / ((x['sd'] ** 2 / x['n']) ** 2 / (x['n'] - 1) +
                             (y['sd'] ** 2 / y['n']) ** 2 / (y['n'] - 1))
            half = _t95(int(df)) * math.sqrt(var)
            ci = [round(diff - half, 3), round(diff + half, 3)]
        else:
            ci = None
        out[k] = dict(diff=round(diff, 3), ci95=ci,
                      clear=bool(ci) and (ci[0] > 0 or ci[1] < 0))
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
    args = parser.parse_args(argv)
    if args.batch:
        try:
            first = batch(_records([args.session]))
            result = dict(schema='mhseals.score_batch/1', a=first)
            if args.compare:
                second = batch(_records([args.compare]))
                result.update(b=second, b_minus_a=compare(first, second))
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

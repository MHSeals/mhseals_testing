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
from mhseals_testing.analysis import score

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


def claims(task_events, report):
    """The boat's claims beside what ground truth credits, per task (claims never score)."""
    said = [dict(t=e.get('time'), event=e['event'], stage=e.get('stage'))
            for e in task_events if e.get('event') in CLAIM_EVENTS]
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
    body['run'].update(session=Path(session).name, course=dict(
        name=truth.get('name'), family=truth.get('family'), seed=gen.get('seed'),
        mode=gen.get('mode'), truth_sha256=gen.get('truth_sha256')))
    body['claims'] = claims(task_events, body)
    body['diagnostics'] = dict(motion=motion(track, window[0], body['run']['scored_until']))
    record = dict(summary=score.summary(body),
                  scorer=dict(version=score.VERSION, rules=score.RULES,
                              handbook_sha256=(truth.get('handbook') or {}).get('sha256')),
                  **body)
    stamp = (now or datetime.now().astimezone()).strftime('%Y%m%dT%H%M%S')
    target = Path(session) / 'analysis' / f'score.{stamp}.json'
    data.write_json(target, record, score.SCHEMA)
    return target, record


def main(argv=None, out=None):
    parser = argparse.ArgumentParser(prog='boat_trial analyze score')
    parser.add_argument('session')
    parser.add_argument('--json', action='store_true', help='print the record that was written')
    args = parser.parse_args(argv)
    try:
        target, _ = analyze(args.session)
    except Refused as error:
        print(f'refused: {error}', file=sys.stderr)
        return 2
    record = json.loads(target.read_text())
    print(json.dumps(record, indent=2) if args.json else record['summary'], file=out or sys.stdout)
    return 0

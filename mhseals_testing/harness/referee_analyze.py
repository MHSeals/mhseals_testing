"""Re-judge a finished session with the referee, headless: ``boat_trial analyze referee``.

Reads files only (no ROS). A session directory holds ``course_truth.json`` (the crane
course), ``track.jsonl`` (ground-truth rows ``{t, x, y, yaw}``), the task's events
(``task_events.jsonl``, else ``events.jsonl``) and optionally ``session.json`` naming the
``task``, else the events do. A TUI mission rehearsal writes track.jsonl and task_events.jsonl
for its latest run. The verdict is
written to ``<session>/analysis/referee.<ts>.json``; ``--json`` prints that same record.
``boat_trial analyze referee SESSION [--task T] [--json]`` calls :func:`main` (``cli.ANALYSES``).
"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys

import yaml

from mhseals_testing import data
from mhseals_testing.analysis import referee
from mhseals_testing.profiles import PROFILE_DIR

MISSION_DIR = PROFILE_DIR.parent / 'missions'
SCHEMA = 'mhseals.referee/1'


class Refused(Exception):
    """The session cannot be judged; the message says why."""


def load(session):
    """(truth, track, events, task) from a session directory."""
    session = Path(session)
    if not session.is_dir():
        raise Refused(f'{session}: no such session')
    truth = data.read_json(session / 'course_truth.json')
    if truth is None:
        raise Refused(f'{session.name}: no course_truth.json to judge against')
    track = [(r['t'], r['x'], r['y'], r.get('yaw') or 0.0)
             for r in data.read_jsonl(session / 'track.jsonl')
             if 't' in r and 'x' in r and 'y' in r]
    if not track:
        raise Refused(f'{session.name}: no track.jsonl samples')
    kept = session / 'task_events.jsonl'           # the TUI's last mission run (procedures/mission.py)
    events = [dict(e, time=e.get('received', e.get('time', 0.0)))
              for e in data.read_jsonl(kept if kept.exists() else session / 'events.jsonl') if 'event' in e]
    task = ((data.read_json(session / 'session.json', {}) or {}).get('task')
            or next((e['task'] for e in events if e.get('task')), None))
    return truth, track, events, task


def summary(judged):
    """One line: ``<task>: <verdict> · n/m PASS · k contact(s)``."""
    stages = judged.get('stages') or {judged['task']: judged}
    passed = sum(r['verdict'] == 'PASS' for r in stages.values())
    touched = sum(len(r.get('contacts') or []) for r in stages.values())
    return (f"{judged['task']}: {judged['verdict']} · {passed}/{len(stages)} PASS"
            f" · {touched} contact{'' if touched == 1 else 's'}")


def analyze(session, task=None, now=None):
    """Judge ``session`` and write ``analysis/referee.<ts>.json``; returns (path, record)."""
    truth, track, events, named = load(session)
    task = task or named or ('mission' if any(e['event'].startswith('stage_') for e in events)
                             else None)
    rubric_file = MISSION_DIR / f'{task}.yaml'
    if not task or not rubric_file.is_file():
        known = ', '.join(sorted(p.stem for p in MISSION_DIR.glob('*.yaml')))
        raise Refused(f'unknown task {task!r}; one of {known}')
    rubric = yaml.safe_load(rubric_file.read_text())
    judged = referee.judge_task(task, truth, track, events, rubric)
    record = dict(summary=summary(judged), basis='truth', track_source='truth',
                  window=[track[0][0], track[-1][0]], verdict=judged['verdict'], result=judged)
    stamp = (now or datetime.now().astimezone()).strftime('%Y%m%dT%H%M%S')
    target = Path(session) / 'analysis' / f'referee.{stamp}.json'
    data.write_json(target, record, SCHEMA)
    return target, record


def main(argv=None, out=None):
    """CLI body: exit 0 on PASS/SKIPPED/N/A, 1 on FAIL, 2 when the session cannot be judged."""
    parser = argparse.ArgumentParser(prog='boat_trial analyze referee')
    parser.add_argument('session')
    parser.add_argument('--task', help='task to judge (default: session.json task, else mission)')
    parser.add_argument('--json', action='store_true', help='print the record that was written')
    args = parser.parse_args(argv)
    try:
        target, _ = analyze(args.session, args.task)
    except Refused as error:
        print(f'refused: {error}', file=sys.stderr)
        return 2
    record = json.loads(target.read_text())
    print(json.dumps(record, indent=2) if args.json else record['summary'], file=out or sys.stdout)
    return 1 if record['verdict'] == 'FAIL' else 0

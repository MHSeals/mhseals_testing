"""Judge a simulated run against the course it was run on (docs/math.tex, "The referee").

The task tree reports what it believes it did (/task/events); the referee checks
what the boat actually did, from the simulator's ground truth: the course
crane_ml placed (its truth JSON, each element tagged with its task) and the
boat's true track (/crane/odom), both in the simulator's world frame.

A track is a list of (time, x, y, yaw). The hull is a rectangle of the boat's
measured length and beam about the track point.
"""

import math

from mhseals_testing.referee.geometry import (  # noqa: F401  (re-exported for callers)
    HULL, circled, contacts, crossings, elements, gates, held_in_bay, hull_clearance, points)

GATE_WIDTH = {'nav_channel': 8.0, 'return_home': 4.0}  # widest pair that is one gate
SMALL_GATE = 4.0


def reported(events, name):
    """The count the tree reported as ``name``, or None."""
    return next((e.get('count') for e in reversed(events) if e.get('event') == name), None)


def judge(task, truth, track, events=(), banner=None, dock_time=5.0):
    """What the boat really did on ``task``; ``verdict`` is PASS only if every check holds."""
    items = elements(truth, task)
    if task in ('pair_test', 'circle_test') and not items:
        return dict(task=task, verdict='N/A',
                    reason='course truth has no elements tagged for this test')
    checks, details = {}, {}
    if task == 'pair_test':
        segments = gates(items, 'red_pole_buoy', 'green_pole_buoy', 8.0)
        passes = [crossings(track, segment) for segment in segments]
        checks['out and back through the same pair'] = any(
            len(run) >= 2 and run[0][1] != run[-1][1] for run in passes)
    if task == 'circle_test':
        marks = points(items, 'yellow_buoy')
        swept = max((circled(track, mark[:2]) for mark in marks), default=0.0)
        details['around_mark_deg'] = round(swept, 1)
        checks['circled a yellow buoy'] = swept >= 345
        checks['returned to start'] = bool(track) and math.dist(
            track[0][1:3], track[-1][1:3]) <= 0.75
    if task in ('nav_channel', 'follow_path', 'return_home'):
        red, green = {'nav_channel': ('red_pole_buoy', 'green_pole_buoy'),
                      'follow_path': ('red_buoy', 'green_buoy'),
                      'return_home': ('black_buoy', 'black_buoy')}[task]
        segments = gates(items, red, green, GATE_WIDTH.get(task, SMALL_GATE))
        crossed = [bool(crossings(track, segment)) for segment in segments]
        details['gates'] = f'{sum(crossed)}/{len(segments)}'
        checks['every gate crossed'] = bool(segments) and all(crossed)
    if task == 'follow_path':
        truth_count = len(points(items, 'yellow_buoy'))
        details['species'] = dict(true=truth_count, reported=reported(events, 'species_count'))
        checks['species count'] = details['species']['reported'] == truth_count
    if task == 'speed_challenge':
        [segment] = gates(items, 'red_buoy', 'green_buoy', SMALL_GATE)[:1] or [None]
        passes = crossings(track, segment) if segment else []
        mark = points(items, 'blue_buoy')
        details['around_mark_deg'] = round(circled(track, mark[0][:2]), 1) if mark else 0.0
        checks['out and back through the gate'] = (
            len(passes) >= 2 and passes[0][1] != passes[-1][1])
        checks['circled the mark'] = details['around_mark_deg'] >= 300
        if len(passes) >= 2:
            details['gate_to_gate_s'] = round(passes[-1][0] - passes[0][0], 2)
        truth_count = len(points(items, 'black_buoy'))
        details['spills'] = dict(true=truth_count, reported=reported(events, 'spill_count'))
        checks['spill count'] = details['spills']['reported'] == truth_count
    if task == 'docking':
        bays = truth.get('bays', [])
        right = [b for b in bays if b['banner'] == banner and not b['occupied']]
        held = max((held_in_bay(track, b, dock_time) for b in right), default=0.0)
        wrong = max((held_in_bay(track, b, dock_time) for b in bays if b not in right),
                    default=0.0)
        details['held_s'] = round(held, 1)
        checks['docked in an empty bay of the banner'] = held >= dock_time
        checks['never in a wrong bay'] = wrong < 1.0
    touched = contacts(track, [e for e in items if e.get('label')])  # the dock may be touched
    details['contacts'] = touched
    checks['no contact'] = not touched
    passed = all(checks.values())
    return dict(task=task, verdict='PASS' if passed else 'FAIL', checks=checks, **details)


def judge_mission(truth, track, events, tasks, banner=None, dock_time=5.0,
                  required_stages=None):
    """Each stage judged over the part of the track between its stage events."""
    results = {}
    required = set(tasks if required_stages is None else required_stages)
    for name in tasks:
        started = next((e['time'] for e in events
                        if e.get('event') == 'stage_started' and e.get('stage') == name), None)
        terminal = next((e for e in events if e.get('event') in (
            'stage_completed', 'stage_skipped') and e.get('stage') == name), None)
        ended = terminal['time'] if terminal else None
        if name not in required and (started is None or (
                terminal and terminal['event'] == 'stage_skipped')):
            part = [p for p in track if started is not None and
                    started <= p[0] <= (ended if ended is not None else math.inf)]
            touched = contacts(part, [e for e in elements(truth, name) if e.get('label')])
            results[name] = dict(task=name, verdict='FAIL' if touched else 'SKIPPED',
                                 checks={'no contact': not touched}, contacts=touched)
            continue
        if started is None:
            results[name] = dict(task=name, verdict='FAIL', checks={'attempted': False})
            continue
        part = [p for p in track if started <= p[0] <= (ended if ended else math.inf)]
        stage_events = [e for e in events if started <= e.get('time', 0) <= (ended or math.inf)]
        results[name] = judge(name, truth, part, stage_events, banner, dock_time)
    passed = all(result['verdict'] in ('PASS', 'SKIPPED') for result in results.values())
    return dict(task='mission', verdict='PASS' if passed else 'FAIL', stages=results)

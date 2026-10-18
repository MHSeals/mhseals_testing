"""Judge a simulated run against the course it was run on (docs/math.tex, "The referee").

The task tree reports what it believes it did (/task/events); the referee checks
what the boat actually did, from the simulator's ground truth: the course
crane_ml placed (its truth JSON, each element tagged with its task) and the
boat's true track (/crane/odom), both in the simulator's world frame.

A track is a list of (time, x, y, yaw). The hull is a rectangle of the boat's
measured length and beam about the track point.
"""

import math

HULL = (1.063, 0.895)  # m, length and beam
GATE_WIDTH = {'nav_channel': 8.0, 'return_home': 4.0}  # widest pair that is one gate
SMALL_GATE = 4.0


def elements(truth, task):
    return [e for e in truth['elements'] if e.get('task') == task]


def points(items, *labels):
    return [(e['x'], e['y'], e.get('radius', 0.2)) for e in items if e['label'] in labels]


def gates(items, red, green, max_width):
    """Gate segments from the truth, nearest pairs first; one colour pairs with itself."""
    reds, greens = points(items, red), points(items, green)
    same = red == green
    pairs = sorted((math.dist(r[:2], g[:2]), i, j) for i, r in enumerate(reds)
                   for j, g in enumerate(greens)
                   if 0.5 < math.dist(r[:2], g[:2]) <= max_width and not (same and j <= i))
    used, segments = set(), []
    for _, i, j in pairs:
        a, b = ('r', i), (('r' if same else 'g'), j)
        if a in used or b in used:
            continue
        used |= {a, b}
        segments.append((reds[i][:2], greens[j][:2]))
    return segments


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def crossings(track, segment):
    """Times the boat's centre crossed the segment between the two buoys, with direction.

    Direction is +1 or -1 by which side it crossed from (the sign of the 2-D
    cross product), so out-and-back through one gate shows as two opposite signs.
    """
    a, b = segment
    found = []
    for (t0, x0, y0, _), (t1, x1, y1, _) in zip(track, track[1:]):
        p, q = (x0, y0), (x1, y1)
        d1, d2 = _cross(a, b, p), _cross(a, b, q)
        d3, d4 = _cross(p, q, a), _cross(p, q, b)
        # A sample exactly on the line counts once: for the step that reaches it.
        if d1 != 0 and d1 * d2 <= 0 and d3 * d4 <= 0:
            found.append((t1, 1 if d1 < 0 else -1))
    return found


def hull_clearance(pose, centre, radius, hull=HULL):
    """Distance from a round object's edge to the hull rectangle (negative is contact)."""
    x, y, yaw = pose
    dx, dy = centre[0] - x, centre[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    along, across = c * dx + s * dy, -s * dx + c * dy
    outside = math.hypot(max(abs(along) - hull[0] / 2, 0), max(abs(across) - hull[1] / 2, 0))
    return outside - radius


def contacts(track, items):
    """Labels of the course objects the hull touched, each once."""
    touched = []
    for index, item in enumerate(items):
        if any(hull_clearance(pose[1:], (item['x'], item['y']), item.get('radius', 0.2)) < 0
               for pose in track):
            touched.append(f"{item['label'] or item['template']}#{index}")
    return touched


def circled(track, centre, radius=8.0):
    """Degrees the boat went round ``centre`` while within ``radius`` of it."""
    swept, previous = 0.0, None
    for _, x, y, _ in track:
        if math.dist((x, y), centre) > radius:
            previous = None
            continue
        angle = math.atan2(y - centre[1], x - centre[0])
        if previous is not None:
            swept += math.remainder(angle - previous, 2 * math.pi)
        previous = angle
    return abs(math.degrees(swept))


def held_in_bay(track, bay, seconds):
    """Longest stay (s) with the boat's centre inside the bay, facing into it."""
    c, s = math.cos(bay['yaw']), math.sin(bay['yaw'])
    best, since = 0.0, None
    for t, x, y, yaw in track:
        dx, dy = x - bay['x'], y - bay['y']
        along, across = c * dx + s * dy, -s * dx + c * dy
        facing = math.cos(yaw - (bay['yaw'] + math.pi)) > math.cos(math.radians(30))
        inside = abs(along) <= bay['depth'] / 2 + 0.3 and abs(across) <= bay['width'] / 2
        if inside and facing:
            since = t if since is None else since
            best = max(best, t - since)
        else:
            since = None
    return best


def reported(events, name):
    """The count the tree reported as ``name``, or None."""
    return next((e.get('count') for e in reversed(events) if e.get('event') == name), None)


def judge(task, truth, track, events=(), banner=None, dock_time=5.0):
    """What the boat really did on ``task``; ``verdict`` is PASS only if every check holds."""
    items = elements(truth, task)
    checks, details = {}, {}
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

"""Run diagnostics beside the 2026 score: never points, only what happened and what it cost.

All from ground truth and observable behaviour (the course truth and the true track), plus
the boat's public claims (/task/events, /task/status) shown beside truth, never shaping it.

- ``contacts``: every hull contact (strike objects, dock parts, shore) with speed at contact,
  near-miss episodes and clearance exposure. Geometry only (track against truth discs).
- ``timeline``: per task, encounter (the truth footprint grown by one hull length), first
  execution candidate (task-specific geometry), completion or leaving it unfinished, and the
  split transit/approach/execution/recovery, with path length against a straight-line bound.
- ``hindsight``: at each task switch, the unfinished tasks that were nearer and whether they
  were plausibly visible (range and line of sight, all round: no camera layout assumed);
  the ideal visiting order of the tasks the boat engaged. Hindsight with full knowledge:
  it never folds into the score.
- ``claims``: each public claim checked against truth (latency, bay, position error).
"""

import itertools
import math

from mhseals_testing.analysis import score

DPARAMS = dict(
    near_miss_m=0.5,            # D: a near miss is hull clearance below this without contact
    near_miss_exit_m=0.6,       # D: a near-miss episode ends once clear by this much
    contact_clear_m=0.2,        # D: a contact ends (recovered) once clear by this much
    speed_window_s=0.5,         # D: speed = displacement over this centred window
    execute_near_m=3.0,         # D: within this clearance of a task object counts as acting on it
    derived_region_m=3.0,       # D: region round a task with no truth footprint
    sensor_range_m=30.0,        # D, owner to confirm: plausible detection range (all round)
    visibility_dt_s=0.5,        # D: visibility is sampled this often
    claim_window_s=(-0.5, 5.0),  # D: a claim matches a truth event this close (latency kept)
)
EVENT_TASKS = ('entry_gates', 'debris_clearance', 'speed_challenge', 'supply_drop', 'marina')
STAGE_TASK = {'nav_channel': 'entry_gates', 'return_home': 'entry_gates',
              'follow_path': 'debris_clearance', 'docking': 'marina'}   # 2025 stage names


# ---------------------------------------------------------------- geometry helpers

def _seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    n = ax * ax + ay * ay
    f = 0.0 if n == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / n))
    return math.dist(p, (a[0] + f * ax, a[1] + f * ay))


def _poly_dist(p, poly):
    """0 inside ``poly``, else the distance to its edge."""
    if len(poly) >= 3 and score.inside_polygon(p, poly):
        return 0.0
    return min(_seg_dist(p, a, b) for a, b in zip(poly, poly[1:] + poly[:1]))


def _corners(pose, hull):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * u - s * v, y + s * u + c * v)
            for u, v in ((hull[0] / 2, hull[1] / 2), (hull[0] / 2, -hull[1] / 2),
                         (-hull[0] / 2, -hull[1] / 2), (-hull[0] / 2, hull[1] / 2))]


def _crosses(p, q, a, b):
    def side(o, u, v):
        return (u[0] - o[0]) * (v[1] - o[1]) - (u[1] - o[1]) * (v[0] - o[0])
    return side(p, q, a) * side(p, q, b) < 0 and side(a, b, p) * side(a, b, q) < 0


def shore_clearance(pose, hull, line):
    """Hull rectangle to the shore polyline: negative (-0.001) when they cross."""
    box = _corners(pose, hull)
    edges = list(zip(box, box[1:] + box[:1]))
    best = math.inf
    for a, b in zip(line, line[1:]):
        if any(_crosses(a, b, u, v) for u, v in edges):
            return -0.001
        best = min(best, *(_seg_dist(c, a, b) for c in box),
                   *(_seg_dist(a, u, v) for u, v in edges), *(_seg_dist(b, u, v) for u, v in edges))
    return best


def path_length(track):
    return sum(math.dist(a[1:3], b[1:3]) for a, b in zip(track, track[1:]))


def speeds(track, window=DPARAMS['speed_window_s']):
    """Speed at each sample: displacement over a centred window (shorter at the ends)."""
    out, j0, j1 = [], 0, 0
    for t, *_ in track:
        while track[j0][0] < t - window / 2:
            j0 += 1
        while j1 < len(track) - 1 and track[j1 + 1][0] <= t + window / 2:
            j1 += 1
        dt = track[j1][0] - track[j0][0]
        out.append(math.dist(track[j0][1:3], track[j1][1:3]) / dt if dt > 0 else 0.0)
    return out


def _at(track, t):
    return min(track, key=lambda p: abs(p[0] - t)) if track else None


# ---------------------------------------------------------------- contacts

def contacts(run, model, p=DPARAMS):
    """Every contact episode (strike objects, dock parts, shore) and every near miss."""
    hull = model['hull']
    reach = math.hypot(*hull) / 2
    v = speeds(run)
    found, near, below_s, closest = [], [], 0.0, math.inf
    objects = [(e, 'dock' if e.get('role') in score.DOCK_PARTS else 'strike')
               for e in model['elements']]
    shore = model.get('shore')
    if shore and len(shore) >= 2:
        objects.append((dict(id='shore', role='shore'), 'shore'))
    for item, kind in objects:
        state, episode, gap = 'clear', None, (reach + (item.get('radius') or 0.2) +
                                             p['near_miss_exit_m']) ** 2
        for i, (t, x, y, yaw) in enumerate(run):
            if kind == 'shore':
                clear = shore_clearance((x, y, yaw), hull, shore)
            elif (x - item['x']) ** 2 + (y - item['y']) ** 2 > gap:
                clear = math.inf
            else:
                clear = score.hull_clearance((x, y, yaw), (item['x'], item['y']),
                                             item.get('radius') or 0.2, hull)
            closest = min(closest, clear)
            if i and clear < p['near_miss_m']:
                below_s += t - run[i - 1][0]
            if clear < 0 and state != 'contact':
                episode = dict(object=item['id'], role=item.get('role'), kind=kind, t=round(t, 3),
                               speed_mps=round(v[i], 3), min_clearance_m=round(clear, 3),
                               recovered_s=None)
                found.append(episode)
                if state == 'near':
                    near.pop()          # it became a contact: not a near miss
                state = 'contact'
            elif state == 'contact':
                episode['min_clearance_m'] = round(min(episode['min_clearance_m'], clear), 3)
                if clear > p['contact_clear_m']:
                    episode['recovered_s'] = round(t - episode['t'], 3)
                    state = 'clear'
            elif clear < p['near_miss_m'] and state == 'clear':
                episode = dict(object=item['id'], role=item.get('role'), kind=kind,
                               t=round(t, 3), speed_mps=round(v[i], 3),
                               min_clearance_m=round(clear, 3))
                near.append(episode)
                state = 'near'
            elif state == 'near':
                if clear < episode['min_clearance_m']:
                    episode.update(t=round(t, 3), speed_mps=round(v[i], 3),
                                   min_clearance_m=round(clear, 3))
                if clear > p['near_miss_exit_m']:
                    state = 'clear'
    found.sort(key=lambda c: c['t'])
    near.sort(key=lambda c: c['t'])
    count = {k: sum(c['kind'] == k for c in found) for k in ('strike', 'dock', 'shore')}
    return dict(contacts=found, near_misses=near, counts=count, near_miss_count=len(near),
                seconds_below_near_miss=round(below_s, 2),
                min_clearance_m=None if closest == math.inf else round(closest, 3),
                peak_contact_speed_mps=max((c['speed_mps'] for c in found), default=None))


# ---------------------------------------------------------------- task timeline

def regions(model):
    """Per task: (test(xy) -> distance to its region, anchor point, how it was derived)."""
    out = {}
    grow = model['hull'][0]
    for tid, task in model['tasks'].items():
        poly = [tuple(q) for q in task.get('footprint') or []]
        mine = [e for e in model['elements'] if e.get('task') == tid] + \
            [b for b in model['bays'] if b.get('task') == tid]
        if tid == 'harbor_alert':
            zones = [a for k, a in (task.get('anchors') or {}).items() if k.startswith('zone')]
            if not zones:
                continue
            out[tid] = (lambda xy, z=zones: max(0.0, min(math.dist(xy, (a['x'], a['y'])) -
                                                         a.get('r', 3.0) for a in z)),
                        (zones[0]['x'], zones[0]['y']), 'zone anchors')
        elif len(poly) >= 3:
            anchor = (task.get('anchors') or {}).get('approach') or {}
            centre = (anchor['x'], anchor['y']) if anchor else \
                (sum(q[0] for q in poly) / len(poly), sum(q[1] for q in poly) / len(poly))
            out[tid] = (lambda xy, g=poly: max(0.0, _poly_dist(xy, g) - grow), centre,
                        'truth footprint + one hull length')
        elif mine:
            pts = [(e['x'], e['y']) for e in mine]
            d = DPARAMS['derived_region_m']
            out[tid] = (lambda xy, q=pts, d=d: max(0.0, min(math.dist(xy, a) for a in q) - d),
                        (sum(a[0] for a in pts) / len(pts), sum(a[1] for a in pts) / len(pts)),
                        f'derived: within {d} m of the task objects')
    return out


def _executing(tid, model, run, gate_passes, trigger):
    """First execution candidate (time) and the failed ones, from geometry only."""
    gates = [g['id'] for g in model['gates'] if g['task'] == tid]
    passes = sorted((p for g in gates for p in gate_passes.get(g, [])), key=lambda p: p['t'])
    failed = [p['t'] for p in passes if not p['hull_inside']]
    starts = [p['t'] for p in passes]
    near = DPARAMS['execute_near_m']
    if tid == 'debris_clearance':
        sem = (model['tasks'][tid].get('semantics') or {})
        by_id = {e['id']: e for e in model['elements']}
        marks = [by_id[i] for i in sem.get('avoid', []) + sem.get('rescue', []) if i in by_id]
        starts += [next((t for t, x, y, _ in run if any(math.dist((x, y), (m['x'], m['y'])) <= near
                                                       for m in marks)), None)]
    elif tid == 'supply_drop':
        hulls = [e for e in model['elements'] if e.get('task') == tid]
        starts += [next((t for t, x, y, _ in run if any(math.dist((x, y), (h['x'], h['y'])) <= near
                                                       for h in hulls)), None)]
    elif tid == 'marina':
        for bay in model['bays']:
            entered = next((t for t, x, y, _ in run if score.docked([(t, x, y, 0)], bay, 0)[0]
                            is not None), None)
            starts.append(entered)
    elif tid == 'harbor_alert':
        starts.append(trigger)
    starts = [t for t in starts if t is not None]
    return (min(starts) if starts else None), failed


def timeline(run, model, report, gate_passes, contact_list, trigger=None):
    """Per task: when it was met, acted on, finished or left, and what each part cost."""
    t0 = run[0][0] if run else 0.0
    regs = regions(model)
    out = {}
    for tid, (dist, _, how) in regs.items():
        inside = [p for p in run if dist(p[1:3]) <= 0]
        rows = (report['tasks'].get(tid) or {}).get('rows', [])
        done = [r['t'] for r in rows if r['status'] in ('earned', 'provisional') and r['t'] is not None]
        execute, failed = _executing(tid, model, run, gate_passes, trigger)
        if tid == 'harbor_alert' and trigger is not None:
            met = trigger
        else:
            met = inside[0][0] if inside else None
        if execute is not None and (met is None or execute < met):
            met = execute
        if met is None:
            out[tid] = dict(encountered=False, region=how)
            continue
        if done:
            end, outcome = max(done), 'completed'
        elif inside and inside[-1][0] >= run[-1][0] - 1e-6:
            end, outcome = run[-1][0], 'run_ended'
        else:
            end, outcome = (inside[-1][0] if inside else met), 'left_unfinished'
        out[tid] = dict(encountered=True, region=how, encounter_t=round(met, 3),
                        execute_t=None if execute is None else round(execute, 3),
                        end_t=round(end, 3), outcome=outcome, failed_candidates=len(failed))
    # transit before each task: from the latest end of another task (or the run start)
    for tid, row in out.items():
        if not row['encountered']:
            continue
        before = [o['end_t'] for k, o in out.items() if k != tid and o.get('encountered')
                  and o['end_t'] <= row['encounter_t']]
        start = max(before, default=t0)
        ex = row['execute_t'] if row['execute_t'] is not None else row['end_t']
        seg = [p for p in run if start <= p[0] <= ex]
        work = [p for p in run if ex <= p[0] <= row['end_t']]
        rec = sum(c['recovered_s'] or (row['end_t'] - c['t']) for c in contact_list
                  if ex <= c['t'] <= row['end_t'])
        straight = math.dist(seg[0][1:3], seg[-1][1:3]) if seg else 0.0
        travelled = path_length(seg)
        row.update(
            split_s=dict(search_or_transit=round(row['encounter_t'] - start, 2),
                         approach=round(ex - row['encounter_t'], 2),
                         execution=round(row['end_t'] - ex, 2), recovery=round(rec, 2)),
            distance_m=dict(to_task=round(travelled, 1), straight_line_bound=round(straight, 1),
                            efficiency=round(straight / travelled, 3) if travelled > 0 else None,
                            in_task=round(path_length(work), 1)))
    return out


def claimed_phases(status, task_events, t1):
    """The boat's own view, per stage: start/end/outcome events and time in each phase."""
    stages = {}
    for e in task_events:
        stage, kind = e.get('stage'), e.get('event')
        if not stage or kind not in ('stage_started', 'stage_completed', 'stage_failed',
                                     'stage_skipped'):
            continue
        row = stages.setdefault(stage, dict(task=STAGE_TASK.get(stage, stage)))
        if kind == 'stage_started':
            row.setdefault('started_t', e.get('time'))
        else:
            row.update(ended_t=e.get('time'), outcome=kind.split('_')[1])
    status = sorted((s for s in status if s.get('time') is not None), key=lambda s: s['time'])
    for a, b in zip(status, status[1:] + [dict(time=t1)]):
        stage = a.get('stage') or a.get('current_stage')
        if not stage or not a.get('phase'):
            continue
        row = stages.setdefault(stage, dict(task=STAGE_TASK.get(stage, stage)))
        phases = row.setdefault('phase_s', {})
        phases[a['phase']] = round(phases.get(a['phase'], 0.0) + max(0.0, b['time'] - a['time']), 2)
    return stages


# ---------------------------------------------------------------- hindsight (never scored)

def visible(pose_xy, element, model, rng=DPARAMS['sensor_range_m']):
    """Plausibly visible: within range and no other object's disc on the line of sight."""
    target = (element['x'], element['y'])
    if math.dist(pose_xy, target) > rng:
        return False
    for other in model['elements']:
        if other is element or math.dist((other['x'], other['y']), target) < 0.5:
            continue
        r = other.get('radius') or 0.2
        if _seg_dist((other['x'], other['y']), pose_xy, target) < r:
            return False
    return True


def first_seen(run, model, dt=DPARAMS['visibility_dt_s']):
    """Per task: first time any of its objects was plausibly visible, and from where."""
    out, last = {}, -math.inf
    tasks = {t for t in model['tasks']}
    for t, x, y, _ in run:
        if t - last < dt:
            continue
        last = t
        for tid in tasks - set(out):
            if any(visible((x, y), e, model) for e in model['elements'] if e.get('task') == tid):
                out[tid] = dict(t=round(t, 3), x=round(x, 2), y=round(y, 2))
        if len(out) == len(tasks):
            break
    return out


def hindsight(run, model, line):
    """At each switch to a new task, the unfinished tasks that were nearer; the ideal order."""
    regs = regions(model)
    order = sorted((tid for tid, r in line.items() if r.get('encountered')
                    and tid in EVENT_TASKS and tid != 'entry_gates'),
                   key=lambda k: line[k]['encounter_t'])
    switches = []
    for i, tid in enumerate(order):
        decided = line[order[i - 1]]['end_t'] if i else (
            line.get('entry_gates', {}).get('end_t') or run[0][0])
        here = _at(run, decided)
        if here is None:
            continue
        chosen = regs[tid][0](here[1:3])
        others = []
        for alt in EVENT_TASKS:
            r = line.get(alt) or {}
            if alt in (tid, 'entry_gates') or alt not in regs:
                continue
            if r.get('outcome') == 'completed' and r['end_t'] <= decided:
                continue
            d = regs[alt][0](here[1:3])
            if d < chosen:
                seen = any(visible(here[1:3], e, model) for e in model['elements']
                           if e.get('task') == alt)
                others.append(dict(task=alt, distance_m=round(d, 1),
                                   closer_by_m=round(chosen - d, 1), plausibly_visible=seen))
        switches.append(dict(t=round(decided, 3), to=tid, distance_m=round(chosen, 1),
                             closer_unfinished=others))
    start = _at(run, line.get('entry_gates', {}).get('end_t') or run[0][0]) if run else None
    ideal = None
    if start and 1 <= len(order) <= 7:
        anchor = {k: regs[k][1] for k in order}

        def cost(seq):
            pts = [start[1:3]] + [anchor[k] for k in seq]
            return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
        best = min(itertools.permutations(order), key=cost)
        ideal = dict(actual_order=order, actual_m=round(cost(order), 1),
                     ideal_order=list(best), ideal_m=round(cost(best), 1),
                     basis='straight lines between task anchors from where entry ended')
    return dict(label='hindsight with full course knowledge; never scored',
                switches=switches, order=ideal)


# ---------------------------------------------------------------- claims against truth

def _bay_at(model, run, t):
    pose = _at(run, t)
    if pose is None:
        return None
    return next((b for b in model['bays']
                 if score.docked([(0, pose[1], pose[2], 0)], b, 0)[0] is not None), None)


def check_claims(task_events, model, run, report, gate_passes, p=DPARAMS):
    """Each public claim beside truth: matched event, latency, bay or position error."""
    lo, hi = p['claim_window_s']
    events = sorted((e for e in task_events if e.get('event') in (
        'gate_cleared', 'docked', 'debris', 'speed_started', 'speed_finished', 'returned_home')),
        key=lambda e: e.get('time') or 0)
    entry_passes = [q for g in model['gates'] for q in gate_passes.get(g['id'], [])
                    if g['task'] in ('entry_gates', 'debris_clearance') and q['hull_inside']]
    speed = [q for g in model['gates'] if g['task'] == 'speed_challenge'
             for q in gate_passes.get(g['id'], [])]
    out = []
    for e in events:
        t, kind = e.get('time'), e['event']
        row = dict(event=kind, t=t)
        if kind == 'gate_cleared':
            match = [q for q in entry_passes if lo <= t - q['t'] <= hi]
            row.update(matched=bool(match),
                       latency_s=round(t - match[-1]['t'], 2) if match else None)
        elif kind in ('speed_started', 'speed_finished'):
            want = 1 if kind == 'speed_started' else -1
            match = [q for q in speed if q['dir'] == want and lo <= t - q['t'] <= hi]
            row.update(matched=bool(match),
                       latency_s=round(t - match[-1]['t'], 2) if match else None)
        elif kind == 'returned_home':
            t_exit = report['run'].get('exit_t')
            row.update(matched=t_exit is not None and lo <= t - t_exit <= hi,
                       latency_s=None if t_exit is None else round(t - t_exit, 2))
        elif kind == 'docked':
            bay = _bay_at(model, run, t)
            colour = None
            if bay:
                ind = bay.get('indicator') or {}
                colour = score.state_at(model, ind.get('id'), t, ()) or ind.get('color')
            number = (bay or {}).get('sign', {}).get('number') if bay else None
            row.update(matched=bay is not None, actual_bay=bay and bay['id'],
                       bay_number_ok=None if bay is None else e.get('bay_number') == number,
                       bay_colour_ok=None if bay is None else e.get('bay_colour') == colour)
        elif kind == 'debris':
            pos = e.get('position') or {}
            xy = (pos.get('x'), pos.get('y')) if isinstance(pos, dict) else tuple(pos[:2])
            if None in xy:
                row.update(matched=False, reason='no position')
            else:
                label = (e.get('label') or '').lower()
                pool = [q for q in model['elements'] if q.get('task') == 'debris_clearance'
                        and q.get('role') in ('debris', 'beacon', 'marker', 'obstacle')]
                same = [q for q in pool if label and label.split('_')[0] in (q.get('label') or '')]
                near = min(same or pool, key=lambda q: math.dist(xy, (q['x'], q['y'])),
                           default=None)
                err = math.dist(xy, (near['x'], near['y'])) if near else None
                row.update(matched=err is not None and err <= 1.0, nearest=near and near['id'],
                           label_ok=bool(same), position_error_m=None if err is None
                           else round(err, 2), frame='map frame assumed = course frame')
        out.append(row)
    return out


def diagnose(run, model, report, task_events=(), status=(), trigger=None):
    """All diagnostics for the scored part of a run (``run`` already clipped to the cutoff)."""
    gate_passes = {g['id']: score.passes(run, g, model['hull']) for g in model['gates']}
    touch = contacts(run, model)
    line = timeline(run, model, report, gate_passes, touch['contacts'], trigger)
    return dict(version=score.VERSION, params=dict(DPARAMS), contacts=touch, timeline=line,
                claimed=claimed_phases(list(status), list(task_events), run[-1][0] if run else 0),
                claims_vs_truth=check_claims(list(task_events), model, run, report, gate_passes),
                first_seen=first_seen(run, model), hindsight=hindsight(run, model, line))

"""Score a sim run by the RoboBoat 2026 rules, from ground truth only.

The judge's view: the course crane_ml placed (its truth JSON), the boat's true
track (``(t, x, y, yaw)`` from /crane/odom, the truth's frame) and the run's
start, finish and authority events. The boat's own task events are never used
to decide what it did; the run window is the only thing taken from them.

Every rule cites its source: S (live scoring §4.2), L (live task rules §3.2),
H (handbook 2025-11-11, page). Rules the handbook leaves open are tagged D and
take their thresholds from ``PARAMS``, which the report records.
A report row needs a RoboCommand message the sim does not capture yet, so its
``report`` is None: a row whose behaviour happened is ``provisional``, a
report-only row is ``unscorable``.
"""

import copy
import math

from mhseals_testing.analysis.geometry import HULL, hull_clearance

SCHEMA = 'mhseals.score/1'
VERSION = 4                    # 4: rules by source capability (state/referee.md "v4")
RULES = 'H 2025-11-11 + L/S live to 2026-02-04'
STRIKE_POINTS, STRIKE_MAX = 25, 20                     # S: 25*S, max 20
PARAMS = dict(
    gate_hysteresis_m=0.3,      # D: a side counts once the centre is this far off the line
    entry_contact_pad_s=3.0,    # D: contact window around the entry passes (L 3.2.1)
    circle_radius_m=8.0,        # D, owner to confirm: a circle counts while within this of the
                                # buoy; L 3.2.2 sets no radius, and without a cap a course-wide
                                # loop "circles" every buoy (evidence: sweep_deg_uncapped)
    full_circle_deg=360.0,      # L 3.2.2 (Feb 4): "full 360", over any sub-interval of one stay
    red_encounter_m=20.0,       # D, owner to confirm: a red within this (the diagnostics'
                                # plausible detection range) was an avoidance opportunity;
                                # L 3.2.2 sets no distance, so no closer approach is rewarded
    dock_dwell_s=2.0,           # D: S says "enters"; 2 s rejects a pass-through and jitter
    dock_gap_s=0.5,             # D: brief exits from the slip that do not reset the dwell
    strike_rearm_m=0.2,         # D: a new strike once clear by this much
    start_offset_warn_m=10.0,   # D: warn when the run starts this far from the course start
    start_standoff_m=1.8288,    # H p.45, L 3.2.1: start >= 6 ft before the first gate
    dock_tiers='highest',       # D, owner to confirm: 'highest' or 'additive' (S 4.2 lists rows)
    harbor_tiers='highest',     # D, owner to confirm: 'highest' or 'additive' (S 4.2 lists rows)
    green_circles_needed=1,     # D, owner to confirm: greens to circle ('all' = every one)
    dock_bay_choice='first',    # D, owner to confirm: 'first' = the first slip held decides the
                                # tier (a wrong bay is not undone by visiting others); 'best' =
                                # best over all slips held. L 3.2.5 / S 4.2 do not say.
    harbor_zone_r_m=3.0,        # D, owner to confirm: zone radius round a yellow alert buoy when
                                # the truth anchor gives none (L 3.2.6 gives no radius)
    harbor_response_s=120.0,    # D, owner to confirm: a zone entry counts only this long after
                                # the recorded horn (L 3.2.6 "immediately", no number)
    object_radius_m=0.2,        # D: radius of a truth object that gives none (H p.48: A-0 buoys
                                # are 20.3 cm across, so this is generous)
    missing_sign_number=9,      # D: a slip without a sign number ranks after signs 1-3
    time_limit_s=None,          # owner sets the official duration; no default is guessed: a
                                # run without session.json time_limit_s has its earned rows
                                # provisional (v4; v3 assumed 300 s, the mission config 1800 s)
)
DOCK_PARTS = ('pier_spine', 'pier_tine', 'dock_sign', 'dock_indicator')  # docking touches these
INTERVENING = ('rc', 'keyboard', 'hold', 'external')   # authority owners that are not autonomy
LEGACY_TASKS = {'nav_channel': 'entry_gates', 'follow_path': 'debris_clearance',
                'speed_challenge': 'speed_challenge', 'docking': 'marina'}


# ---------------------------------------------------------------- course model

def _xy(item):
    return (item['x'], item['y'])


def _gate(gid, task, order, port, stbd, normal_yaw):
    n = (math.cos(normal_yaw), math.sin(normal_yaw))
    return dict(id=gid, task=task, order=order, port=port, stbd=stbd, n=n,
                c=((port['x'] + stbd['x']) / 2, (port['y'] + stbd['y']) / 2))


def course(truth):
    """The course in one shape for v2 truth (crane.course_truth/2) and the 2025 v1 truth."""
    vessel = truth.get('vessel') or {}
    hull = (vessel.get('length') or HULL[0], vessel.get('beam') or HULL[1])
    start = (truth.get('startX', 0.0), truth.get('startY', 0.0))
    if truth.get('schema_version', 1) >= 2:
        by_id = {e['id']: e for e in truth['elements']}
        gates = [_gate(g['id'], g['task'], g['order'], by_id[g['port']], by_id[g['starboard']],
                       g['normal_yaw']) for g in truth.get('gates', [])]
        return dict(legacy=False, hull=hull, start=start, elements=truth['elements'],
                    gates=gates, bays=truth.get('bays', []),
                    tasks={t['id']: t for t in truth.get('tasks', [])},
                    states={s['id']: s for s in truth.get('states', [])},
                    boundary=truth.get('boundary') or None, shore=truth.get('shore') or None)
    return _legacy(truth, hull, start)


def _legacy(truth, hull, start):
    """The 2025 course mapped onto the 2026 tasks it resembles (scored rows only)."""
    elements, gates = [], []
    for index, e in enumerate(truth['elements']):
        elements.append(dict(e, id=f"{e['task']}.{e.get('label') or 'object'}{index}",
                             role='pier_spine' if not e.get('label') else 'buoy'))
    pairs = {'nav_channel': ('red_pole_buoy', 'green_pole_buoy', 8.0),
             'follow_path': ('red_buoy', 'green_buoy', 4.0),
             'speed_challenge': ('red_buoy', 'green_buoy', 4.0)}
    for old, (red, green, widest) in pairs.items():
        reds = [e for e in elements if e['task'] == old and e['label'] == red]
        greens = [e for e in elements if e['task'] == old and e['label'] == green]
        found = sorted((math.dist(_xy(r), _xy(g)), i, j) for i, r in enumerate(reds)
                       for j, g in enumerate(greens) if math.dist(_xy(r), _xy(g)) <= widest)
        used, made = set(), []
        for _, i, j in found:
            if ('r', i) in used or ('g', j) in used:
                continue
            used |= {('r', i), ('g', j)}
            r, g = reds[i], greens[j]
            c = ((r['x'] + g['x']) / 2, (r['y'] + g['y']) / 2)
            yaw = math.atan2(r['y'] - g['y'], r['x'] - g['x']) - math.pi / 2  # red to port
            if old != 'speed_challenge' and (math.cos(yaw) * (c[0] - start[0]) +
                                             math.sin(yaw) * (c[1] - start[1])) < 0:
                yaw += math.pi
            made.append((math.dist(start, c), r, g, yaw))
        for order, (_, r, g, yaw) in enumerate(sorted(made, key=lambda m: m[0]), 1):
            task = LEGACY_TASKS[old]
            port, stbd = (r, g) if math.cos(yaw) * (r['y'] - g['y']) - \
                math.sin(yaw) * (r['x'] - g['x']) > 0 else (g, r)
            gates.append(_gate(f'{task}.gate{order}' if task != 'speed_challenge'
                               else 'speed_challenge.gate', task, order, port, stbd, yaw))
    for g in gates:                     # the 2025 speed gate faces the blue mark
        if g['task'] == 'speed_challenge':
            mark = next((e for e in elements if e['label'] == 'blue_buoy'), None)
            ahead = (mark['x'] - g['c'][0]) * g['n'][0] + (mark['y'] - g['c'][1]) * g['n'][1] \
                if mark else 0
            if ahead < 0:
                g['n'] = (-g['n'][0], -g['n'][1])
                g['port'], g['stbd'] = g['stbd'], g['port']
    mark = next((e for e in elements if e['label'] == 'blue_buoy'), None)
    bays = [dict(b, id=f"docking.bay{b.get('index', i)}", task='marina')
            for i, b in enumerate(truth.get('bays', []))]
    tasks = {'entry_gates': dict(id='entry_gates', semantics={}),
             'debris_clearance': dict(id='debris_clearance', semantics={}),
             'marina': dict(id='marina', semantics={})}
    if mark:
        mark['id'] = 'speed_challenge.mark'
        tasks['speed_challenge'] = dict(id='speed_challenge',
                                        semantics=dict(turn_mark=mark['id']))
    for e in elements:
        e['task'] = LEGACY_TASKS.get(e['task'], e['task'])
    return dict(legacy=True, hull=hull, start=start, elements=elements, gates=gates,
                bays=bays, tasks=tasks, states={}, boundary=None, shore=None)


def state_at(model, sid, t, changes=()):
    """An object's state at time ``t``: its program, overridden by recorded changes."""
    state = model['states'].get(sid)
    if state is None:
        return None
    program = state.get('program') or {}
    value = state.get('initial')
    if program.get('type') == 'toggle':
        values = program.get('values') or ['red', 'green']
        period, phase = program.get('period_s') or 0, program.get('phase') or 0
        if period:
            value = values[int(math.floor((t + phase) / period)) % len(values)]
    elif program.get('type') == 'scripted':
        for step in sorted(program.get('steps', []), key=lambda s: s['t']):
            if step['t'] <= t:
                value = step.get('color', step.get('value'))
    for change in sorted(changes, key=lambda c: c.get('t_sim', math.inf)):
        if change.get('id') == sid and change.get('t_sim', math.inf) <= t:
            value = change.get('value')
    return value


# ---------------------------------------------------------------- track geometry

def _hull_span(pose, axis, hull):
    """(min, max) of the hull rectangle projected on ``axis``."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    half_l, half_b = hull[0] / 2, hull[1] / 2
    centre = x * axis[0] + y * axis[1]
    reach = abs(c * axis[0] + s * axis[1]) * half_l + abs(-s * axis[0] + c * axis[1]) * half_b
    return centre - reach, centre + reach


def passes(track, gate, hull, h=PARAMS['gate_hysteresis_m']):
    """Each pass of the hull centre through the gate opening, with direction and fit.

    The centre must be ``h`` clear of the line on each side, so jitter at the line
    is one pass. ``dir`` is +1 along the gate normal (outbound), -1 against it.
    ``hull_inside`` says the whole hull fitted between the buoys' edges as the
    centre crossed (L 3.2.1 "the entire ASV must pass through").
    """
    (cx, cy), (nx, ny) = gate['c'], gate['n']
    tx, ty = -ny, nx
    lp = (gate['port']['x'] - cx) * tx + (gate['port']['y'] - cy) * ty
    ls = (gate['stbd']['x'] - cx) * tx + (gate['stbd']['y'] - cy) * ty
    r0 = PARAMS['object_radius_m']
    lo = min(lp, ls) + (gate['port' if lp < ls else 'stbd'].get('radius') or r0)
    hi = max(lp, ls) - (gate['stbd' if lp < ls else 'port'].get('radius') or r0)
    found, side, last_zero = [], 0, None
    previous = None
    for sample in track:
        t, x, y, yaw = sample
        s = (x - cx) * nx + (y - cy) * ny
        if previous is not None and (previous[0] < 0) != (s < 0):
            p, q = previous[1], sample
            f = previous[0] / (previous[0] - s) if previous[0] != s else 0.0
            last_zero = tuple(a + f * (b - a) for a, b in zip(p, q))
            last_zero = last_zero[:3] + (p[3] + f * math.remainder(q[3] - p[3], 2 * math.pi),)
        previous = (s, sample)
        now = 1 if s > h else -1 if s < -h else 0
        if now and side and now != side and last_zero is not None:
            zt, zx, zy, zyaw = last_zero
            lateral = (zx - cx) * tx + (zy - cy) * ty
            if min(lp, ls) <= lateral <= max(lp, ls):
                axis_lo, axis_hi = _hull_span((zx - cx, zy - cy, zyaw), (tx, ty), hull)
                found.append(dict(t=zt, dir=now, lateral=round(lateral, 3),
                                  hull_inside=lo <= axis_lo and axis_hi <= hi,
                                  margin_m=round(min(axis_lo - lo, hi - axis_hi), 3)))
        if now:
            side = now
    return found


def sequence(passes_by_gate, direction, after, need_inside=True):
    """Earliest in-order passes of every gate in ``direction`` after ``after``; [] if none."""
    chosen, t = [], after
    for gate_passes in passes_by_gate:
        nxt = next((p for p in gate_passes if p['t'] > t and p['dir'] == direction
                    and (p['hull_inside'] or not need_inside)), None)
        if nxt is None:
            return []
        chosen.append(nxt)
        t = nxt['t']
    return chosen


def sweep(track, centre, radius=math.inf, reach=None):
    """(deg, t): the largest net signed sweep (+CCW) round ``centre`` over any contiguous
    part of one stay within ``radius`` (lead-in and reversals do not hide a later full
    turn), and when such a sweep first reached ``reach`` degrees either way (or None)."""
    best, net, previous, reached = 0.0, 0.0, None, None
    low = high = 0.0
    for t, x, y, _ in track:
        if not math.isfinite(x + y) or math.dist((x, y), centre) > radius:
            previous, net, low, high = None, 0.0, 0.0, 0.0
            continue
        angle = math.atan2(y - centre[1], x - centre[0])
        if previous is not None:
            net += math.remainder(angle - previous, 2 * math.pi)
            for span in (net - low, net - high):
                if abs(span) > abs(best):
                    best = span
            low, high = min(low, net), max(high, net)
            if reached is None and reach is not None and abs(math.degrees(best)) >= reach - 1e-9:
                reached = t
        previous = angle
    return math.degrees(best), reached


def winding(track, mark):
    """Signed turns (+CCW) round ``mark`` of ``track`` closed by the straight line from its
    last point back to its first (the gate line for a sprint loop): an integer, no threshold."""
    pts = [(p[1] - mark[0], p[2] - mark[1]) for p in track]
    pts.append(pts[0])
    total = 0.0
    for a, b in zip(pts, pts[1:]):
        total += math.remainder(math.atan2(b[1], b[0]) - math.atan2(a[1], a[0]), 2 * math.pi)
    return round(total / (2 * math.pi))


def strikes(track, items, hull, rearm=PARAMS['strike_rearm_m']):
    """One strike per contact episode with each object: [{object, task, t, min_clearance_m}]."""
    reach = math.hypot(*hull) / 2
    found = []
    for item in items:
        cx, cy, r = item['x'], item['y'], item.get('radius') or PARAMS['object_radius_m']
        near = (reach + r + rearm) ** 2
        touching, worst = False, None
        for t, x, y, yaw in track:
            if (x - cx) ** 2 + (y - cy) ** 2 > near:
                clear = rearm + 1
            else:
                clear = hull_clearance((x, y, yaw), (cx, cy), r, hull)
            if clear < 0:
                if not touching:
                    touching, worst = True, dict(object=item['id'], task=item.get('task'),
                                                 t=round(t, 3), min_clearance_m=clear)
                    found.append(worst)
                worst['min_clearance_m'] = min(worst['min_clearance_m'], clear)
            elif clear > rearm:
                touching = False
    for strike in found:
        strike['min_clearance_m'] = round(strike['min_clearance_m'], 3)
    return sorted(found, key=lambda s: s['t'])


def min_clearance(track, item, hull):
    if not track:
        return None
    return round(min(hull_clearance((x, y, yaw), _xy(item),
                                    item.get('radius') or PARAMS['object_radius_m'], hull)
                     for _, x, y, yaw in track), 3)


def docked(track, bay, dwell=PARAMS['dock_dwell_s'], gap=PARAMS['dock_gap_s']):
    """First time the hull centre had been held in ``bay`` for ``dwell`` s, and the longest
    hold. Heading is free: bow in, stern in or sideways all dock (no design favoured)."""
    c, s = math.cos(bay['yaw']), math.sin(bay['yaw'])
    best, since, last_in, done = 0.0, None, None, None
    for t, x, y, yaw in track:
        dx, dy = x - bay['x'], y - bay['y']
        along, across = c * dx + s * dy, -s * dx + c * dy
        inside = abs(along) <= bay['depth'] / 2 and abs(across) <= bay['width'] / 2
        if inside:
            if since is None or t - last_in > gap:
                since = t
            last_in = t
            best = max(best, t - since)
            if done is None and t - since >= dwell:
                done = t
    return done, best


def with_heading(track):
    """The track, with yaw from the motion when it carries none (every yaw 0 or None)."""
    if any(p[3] for p in track):
        return track
    out, yaw = [], 0.0
    for i, (t, x, y, _) in enumerate(track):
        j = min(i + 1, len(track) - 1)
        k = max(j - 1, 0)
        if math.dist(track[j][1:3], track[k][1:3]) > 1e-6:
            yaw = math.atan2(track[j][2] - track[k][2], track[j][1] - track[k][1])
        out.append((t, x, y, yaw))
    return out


def inside_polygon(point, polygon):
    x, y = point
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


# ---------------------------------------------------------------- scoring

class _Rows:
    """Rows of one task; ``allowed(t)`` says whether a behaviour at ``t`` is in the scored run."""

    def __init__(self, allowed):
        self.allowed, self.rows = allowed, []

    def add(self, rid, cite, tag, desc, possible, behaviour, t=None, report=False,
            status=None, points=None, **evidence):
        """``report`` False: no report part; None: a report is needed but not captured."""
        row = dict(id=rid, cite=cite, tag=tag, desc=desc, possible=possible, points=0,
                   provisional=0, status=status, t=None if t is None else round(t, 3),
                   behaviour=behaviour, report=report if report is not False else None,
                   evidence=evidence)
        if status is None:
            if not behaviour:
                row['status'] = 'missed'
            elif not self.allowed(t):
                row['status'] = 'missed'
                evidence['outside_run'] = True
            elif report is None:
                row['status'], row['provisional'] = 'provisional', \
                    possible if points is None else points
            elif report is False or report is True:
                row['status'], row['points'] = 'earned', possible if points is None else points
            else:
                row['status'] = 'missed'
        self.rows.append(row)
        return row


def _report_only(rows, rid, cite, desc, possible, **evidence):
    rows.add(rid, cite, 'H', desc, possible, None, status='unscorable',
             reason='report: no RoboCommand capture in the sim', **evidence)


def _entry_exit(model, run, rows_for, gate_passes):
    gates = sorted((g for g in model['gates'] if g['task'] == 'entry_gates'),
                   key=lambda g: g['order'])
    rows = rows_for('entry_gates')
    if len(gates) < 2:
        rows.add('T1.entry', 'S; L 3.2.1', 'H', 'entry gates', 100, None, status='n/a')
        return rows, None, (None, [])
    seq, t_entry, touched, attempts = [], None, [], 0
    buoys = [g[side] for g in gates for side in ('port', 'stbd')]
    pad = PARAMS['entry_contact_pad_s']
    for first in gate_passes[gates[0]['id']]:     # every attempt; the first clean one enters
        if first['dir'] != 1 or not first['hull_inside']:
            continue
        cand = sequence([gate_passes[g['id']] for g in gates], 1, first['t'] - 1e-9)
        if not cand or cand[0] is not first:
            continue
        attempts += 1
        lo, hi = cand[0]['t'] - pad, cand[-1]['t'] + pad
        hit = [s['object'] for s in strikes([p for p in run if lo <= p[0] <= hi],
                                            buoys, model['hull'])]
        seq, touched = cand, hit
        if not hit:
            t_entry = cand[-1]['t']
            break
    rows.rows.clear()
    first = _Rows(lambda t: True)       # the entry row is the start of the scored run
    first.add('T1.entry', 'S; L 3.2.1; H pp.45-47', 'H', 'through both entry gates, whole hull, '
              'no contact', 100, t_entry is not None, t=seq[-1]['t'] if seq else None,
              passes=[dict(p, gate=g['id']) for g, p in zip(gates, seq)], touched=touched,
              attempts=attempts)
    rows.rows.extend(first.rows)
    t_exit, exits = None, []
    if t_entry is not None:
        for start in gate_passes[gates[-1]['id']]:
            if start['t'] <= t_entry or start['dir'] != -1:
                continue
            back = sequence([gate_passes[g['id']] for g in reversed(gates)], -1, start['t'] - 1e-9)
            if back and back[0] is start:     # the first exit ends the run
                exits = back
                break
        t_exit = exits[-1]['t'] if exits else None
    return rows, t_entry, (t_exit, exits)


def _debris(model, run, rows, gate_passes, t_entry):
    task = model['tasks'].get('debris_clearance') or {}
    sem = task.get('semantics') or {}
    gates = sorted((g for g in model['gates'] if g['task'] == 'debris_clearance'),
                   key=lambda g: g['order'])
    by_id = {e['id']: e for e in model['elements']}
    after = t_entry if t_entry is not None else -math.inf
    inward = sequence([gate_passes[g['id']] for g in gates], 1, after) if gates else []
    rows.add('T2.channel_in', 'S; L 3.2.2; H pp.47-49', 'H', 'through every channel gate in '
             'order, outbound', 100, bool(inward), t=inward[-1]['t'] if inward else None,
             gates=f'{len(inward)}/{len(gates)}', na=not gates) if gates else \
        rows.add('T2.channel_in', 'S; L 3.2.2', 'H', 'channel in', 100, None, status='n/a')
    outward = sequence([gate_passes[g['id']] for g in reversed(gates)], -1,
                       inward[-1]['t']) if inward else []
    if gates:
        rows.add('T2.channel_out', 'S; L 3.2.2; H pp.47-49', 'H', 'back through every channel '
                 'gate in reverse order', 100, bool(outward),
                 t=outward[-1]['t'] if outward else None)
    else:
        rows.add('T2.channel_out', 'S; L 3.2.2', 'H', 'channel out', 100, None, status='n/a')
    scored = [p for p in run if p[0] > after]
    reds = [by_id[i] for i in sem.get('avoid', []) if i in by_id]
    if reds:
        detail = {}
        for red in reds:
            deg, _ = sweep(scored, _xy(red), PARAMS['circle_radius_m'])
            near = min(scored, key=lambda p: math.dist(p[1:3], _xy(red)), default=None)
            dist = math.dist(near[1:3], _xy(red)) if near else math.inf
            detail[red['id']] = dict(sweep_deg=round(deg, 1), closest_m=round(dist, 2),
                                     min_clearance_m=min_clearance(scored, red, model['hull']),
                                     t=near[0] if near else None)
        ok = [d for d in detail.values() if d['closest_m'] <= PARAMS['red_encounter_m']
              and abs(d['sweep_deg']) < PARAMS['full_circle_deg']
              and (d['min_clearance_m'] or 0) > 0]
        rows.add('T2.red_avoid', 'S; L 3.2.2', 'H+D', 'approach the red indicator, avoid it, '
                 'no circling; report its position', 100, bool(ok) and len(ok) == len(detail),
                 t=max((d['t'] for d in ok), default=None), report=None, avoid=detail)
    else:
        rows.add('T2.red_avoid', 'S; L 3.2.2', 'H', 'red indicator', 100, None, status='n/a')
    greens = [by_id[i] for i in sem.get('rescue', []) if i in by_id]
    if greens:
        detail, times = {}, []
        for green in greens:
            deg, t = sweep(scored, _xy(green), PARAMS['circle_radius_m'],
                           PARAMS['full_circle_deg'])
            wide, _ = sweep(scored, _xy(green))     # evidence for the radius question only
            detail[green['id']] = dict(sweep_deg=round(deg, 1), t=t,
                                       sweep_deg_uncapped=round(wide, 1))
            times += [t] if t is not None else []
        need = len(greens) if PARAMS['green_circles_needed'] == 'all' else \
            min(int(PARAMS['green_circles_needed']), len(greens))
        rows.add('T2.green_circle', 'S; L 3.2.2 (Feb 4: full 360)', 'H+D', 'a full circle round '
                 'a green indicator; report its position', 200, len(times) >= need,
                 t=sorted(times)[need - 1] if len(times) >= need else None, report=None,
                 rescue=detail, needed=need)
    else:
        rows.add('T2.green_circle', 'S; L 3.2.2', 'H', 'green indicator', 200, None, status='n/a')
    if sem.get('report'):
        _report_only(rows, 'T2.report_all', 'S; L 3.2.2', 'report every debris position', 200)
    else:
        rows.add('T2.report_all', 'S; L 3.2.2', 'H', 'report all debris', 200, None, status='n/a')


def _speed(model, run, rows, gate_passes, t_entry, changes):
    task = model['tasks'].get('speed_challenge')
    gate = next((g for g in model['gates'] if g['task'] == 'speed_challenge'), None)
    by_id = {e['id']: e for e in model['elements']}
    sem = (task or {}).get('semantics') or {}
    mark = by_id.get(sem.get('turn_mark'))
    if not (task and gate and mark):
        for rid, pts in (('T3.loop', 200), ('T3.direction', 150), ('T3.time', 250),
                         ('T3.report', 100)):
            rows.add(rid, 'S; L 3.2.3', 'H', 'speed challenge', pts, None, status='n/a')
        return
    after = t_entry if t_entry is not None else -math.inf
    mine = gate_passes[gate['id']]
    loop = None
    for p_in in (p for p in mine if p['dir'] == 1 and p['t'] > after and p['hull_inside']):
        p_out = next((p for p in mine if p['dir'] == -1 and p['t'] > p_in['t']), None)
        if p_out is None:               # an attempt ends at its next inbound pass
            break
        turns = winding([s for s in run if p_in['t'] <= s[0] <= p_out['t']], _xy(mark))
        if turns and p_out['hull_inside']:
            loop = (p_in, p_out, 360.0 * turns)
            break
        if loop:
            break
    rows.add('T3.loop', 'S; L 3.2.3; H pp.49-51', 'H+D', 'in through the gate, round the '
             'yellow mark, out through the same gate', 200, bool(loop),
             t=loop[1]['t'] if loop else None,
             turns=round(loop[2] / 360) if loop else None)
    source = sem.get('direction_source')
    sid = (sem.get(source) or {}).get('id') if source else None
    if sid is None:
        rows.add('T3.direction', 'S; L 3.2.3', 'H', 'circle direction', 150, None, status='n/a',
                 reason='no direction source on this course')
    else:
        colour = state_at(model, sid, loop[0]['t'] if loop else after, changes) \
            or sem.get('source_color')
        expected = {'red': 'ccw', 'green': 'cw'}.get(colour)
        done = ('ccw' if loop[2] > 0 else 'cw') if loop else None
        rows.add('T3.direction', 'S; L 3.2.3 (red = from the right, CCW; green = CW)', 'H',
                 'round the mark in the direction the indicator commands', 150,
                 bool(loop) and done == expected, t=loop[1]['t'] if loop else None,
                 source=sid, source_color_at_entry=colour, expected=expected, direction=done)
    took = loop[1]['t'] - loop[0]['t'] if loop else None
    rows.add('T3.time', 'S (250 - T)', 'H+D', 'gate to gate time; points 250 - T s, '
             'floored at 0 (D)', 250, bool(loop), t=loop[1]['t'] if loop else None,
             points=round(max(0.0, 250 - took), 1) if loop else None,
             T_s=round(took, 2) if loop else None)
    _report_only(rows, 'T3.report', 'S; L 3.2.3', 'report indicator colour and time', 100)


def _supply(model, run, rows):
    task = model['tasks'].get('supply_drop')
    sem = (task or {}).get('semantics') or {}
    by_id = {e['id']: e for e in model['elements']}
    for kind, cite in (('water', 'S; L 3.2.4; H pp.51-54'), ('ball', 'S; L 3.2.4')):
        hulls = [by_id[i] for i in sem.get(kind, []) if i in by_id]
        possible = sum((75, 125, 200)[:len(hulls)])
        if not hulls:
            rows.add(f'T4.{kind}', cite, 'H', f'{kind} delivery', 0, None, status='n/a')
            continue
        approach = {h['id']: min_clearance(run, h, model['hull']) for h in hulls}
        rows.add(f'T4.{kind}', cite, 'H', f'{kind} delivered to each hull (75/125/200)',
                 possible, None, status='unscorable', reason='the sim has no delivery sensing',
                 min_clearance_m=approach)
    if task:
        _report_only(rows, 'T4.report', 'S; L 3.2.4', 'report hull location, colour and '
                     'delivery type', 100)
    else:
        rows.add('T4.report', 'S; L 3.2.4', 'H', 'delivery report', 100, None, status='n/a')


TIER_POINTS = {'unoccupied': 50, 'green': 200, 'lowest_green': 350}


def _marina(model, run, rows, changes, allowed):
    bays = model['bays']
    if not bays:
        rows.add('T5.dock', 'S; L 3.2.5', 'H', 'docking', 350, None, status='n/a')
        rows.add('T5.report', 'S; L 3.2.5', 'H', 'dock report', 100, None, status='n/a')
        return []
    warnings, held = [], []

    def busy(bay):
        return bool(bay.get('occupied') or bay.get('vessel'))

    def colour(bay, t):
        indicator = bay.get('indicator') or {}
        return state_at(model, indicator.get('id'), t, changes) or indicator.get('color')

    for bay in bays:
        t, longest = docked(run, bay)
        held.append(dict(bay=bay['id'], t=t, held_s=round(longest, 1), occupied=busy(bay)))
    best, tiers = None, set()
    order_held = sorted((hb for hb in zip(held, bays) if hb[0]['t'] is not None
                         and allowed(hb[0]['t'])), key=lambda hb: hb[0]['t'])
    if PARAMS['dock_bay_choice'] == 'first':
        order_held = order_held[:1]
    for h, bay in order_held:
        green = [b for b in bays if not busy(b) and colour(b, h['t']) == 'green']
        nosign = PARAMS['missing_sign_number']
        low = min(((b.get('sign') or {}).get('number', nosign) for b in green), default=None)
        number = (bay.get('sign') or {}).get('number', nosign)
        tier = ('occupied' if h['occupied'] else 'unoccupied' if colour(bay, h['t']) != 'green'
                else 'lowest_green' if number == low else 'green')     # ties all count
        h['tier'] = tier
        if tier != 'occupied':
            tiers.add(tier)
            if best is None or TIER_POINTS[tier] > TIER_POINTS[best[0]]:
                best = (tier, h)
    order = list(TIER_POINTS)
    if PARAMS['dock_tiers'] == 'additive' and best:     # every tier up to the best one reached
        points = sum(TIER_POINTS[k] for k in order[:order.index(best[0]) + 1])
    else:
        points = TIER_POINTS[best[0]] if best else None
    additive = PARAMS['dock_tiers'] == 'additive'
    possible = 50 if model['legacy'] else (sum(TIER_POINTS.values()) if additive else 350)
    rows.add('T5.dock', 'S; L 3.2.5; H pp.54-56', 'H+D', 'held in a slip; 50 unoccupied, 200 '
             'green, 350 lowest green (highest tier only, D)', possible, bool(best),
             t=best[1]['t'] if best else None, points=points,
             choice=PARAMS['dock_bay_choice'], slips_held=len([h for h in held if h['t']]),

             tier=best[0] if best else None, entered=best[1]['bay'] if best else None,
             bays=[h for h in held if h['held_s'] > 0])
    _report_only(rows, 'T5.report', 'S; L 3.2.5', 'report dock N/S and slip', 100)
    return warnings


def horn_sounded(changes, program=None, logged=None):
    """(t, value, basis) of the first horn sound. A recorded state log decides (basis
    'recorded'; a log with no horn record means it never sounded). Only with no state log at
    all does the program's planned trigger_s stand in (basis 'planned'), and then the zone row
    can be provisional at most, never earned. ``logged`` says whether a state log exists (an
    empty complete log is a log); None takes a non-empty ``changes`` as the log (v3)."""
    if changes if logged is None else logged:
        on = sorted((c['t_sim'], c.get('value')) for c in changes
                    if c.get('id') == 'harbor_alert.horn' and c.get('value') not in (None, 'off')
                    and isinstance(c.get('t_sim'), (int, float)))
        return (*on[0], 'recorded') if on else (None, None, 'recorded: never sounded')
    planned = (program or {}).get('trigger_s')
    return planned, None, None if planned is None else 'planned (no state log)'


def _harbor(model, run, rows, t_from, t_cut, changes=(), worked=lambda a, b: [], horn=None):
    """L 3.2.6: after the horn the ASV abandons its task and goes to the commanded zone.

    Scored: the first real entry (outside -> inside) into a zone after the recorded horn and
    within ``harbor_response_s``; being in a zone when it sounds is not an entry. Whether other
    tasks still earned credit before the entry is evidence only (``dropped_task``), never a gate:
    it must not punish a route that completes a task on the way."""
    task = model['tasks'].get('harbor_alert')
    program = ((model['states'].get('harbor_alert.horn') or {}).get('program')) or {}
    trigger, value, basis = horn_sounded(
        changes, program if horn in (None, 'planned') else None,
        None if horn is None else horn in ('recorded', 'partial'))
    why = dict(unknown='no horn record from this source (caps.horn unknown)',
               contradictory='the state log contradicts itself (caps.horn contradictory)',
               partial='the state log has gaps or is cut short, so silence is not known '
                       '(caps.horn partial)')
    if task and horn in why and (trigger is None or horn == 'contradictory'):
        rows.add('T6.zone', 'S; L 3.2.6', 'H', 'harbor alert', 300, None, status='unscorable',
                 reason=why[horn], horn=horn)
        _report_only(rows, 'T6.report', 'S; L 3.2.6', 'confirm the signal', 150)
        return
    if horn == 'partial':
        basis = 'recorded (partial log: an earlier horn may be missing)'
    if not task or trigger is None or not (t_from <= trigger <= t_cut):
        rows.add('T6.zone', 'S; L 3.2.6', 'H', 'harbor alert', 300, None, status='n/a',
                 reason='no horn sounded' if trigger is None else
                 'horn outside the scored run', horn=basis, horn_t=trigger,
                 planned_trigger_s=program.get('trigger_s'))
        rows.add('T6.report', 'S; L 3.2.6', 'H', 'alert report', 150, None, status='n/a')
        return
    pattern = value if value in ('one', 'two') else program.get('pattern')
    target = {'one': 'zone1', 'two': 'zone2'}.get(pattern)     # L 3.2.6; else unknown
    zones = {z: task['anchors'][z] for z in ('zone1', 'zone2') if z in task.get('anchors', {})}

    def zone_at(x, y):
        return next((z for z, a in zones.items() if math.dist((x, y), (a['x'], a['y']))
                     <= (a.get('r') or PARAMS['harbor_zone_r_m'])), None)

    before = [p for p in run if p[0] <= trigger]
    at_horn = zone_at(*before[-1][1:3]) if before else None
    deadline = trigger + PARAMS['harbor_response_s']
    reached = late = None
    prev = at_horn
    for t, x, y, _ in run:
        if t <= trigger:
            continue
        here = zone_at(x, y)
        if here and here != prev:                       # a real entry, not presence
            if t <= deadline and reached is None:
                reached = (here, t)
            elif t > deadline and late is None:
                late = dict(zone=here, t=round(t, 2))
        prev = here
    tier = 300 if reached and reached[0] == target else 150 if reached else 0
    if tier == 300 and PARAMS['harbor_tiers'] == 'additive':
        tier = 450
    between = worked(trigger, reached[1] if reached else min(deadline, t_cut))
    rows.add('T6.zone', 'S; L 3.2.6; H pp.56-58', 'H+D', 'after the recorded horn, enter a '
             'zone (150) or the commanded zone (300) within the response window (D)',
             450 if PARAMS['harbor_tiers'] == 'additive' else 300, bool(reached),
             t=reached[1] if reached else None, points=tier or None, horn_t=trigger, horn=basis,
             report=None if basis != 'recorded' else False,    # planned/partial: provisional
             planned_trigger_s=program.get('trigger_s'), pattern=pattern,
             in_zone_at_horn=at_horn, response_window_s=PARAMS['harbor_response_s'],
             target_zone=target, reached=reached[0] if reached else None,
             correct=bool(reached) and reached[0] == target, late_entry=late,
             response_s=round(reached[1] - trigger, 2) if reached else None,
             credited_after_horn=between, dropped_task=not between)    # evidence, not a gate
    _report_only(rows, 'T6.report', 'S; L 3.2.6', 'confirm the signal', 150)


def judge(truth, track, window=None, authority=(), changes=(), require_entry=True,
          end_reason=None, horn=None):
    """The scored run: a ``mhseals.score/1`` report body (without ``schema``/``summary``).

    ``window`` (t0, t1) is the run; ``authority`` is [{t, by, cause}] owner changes in track
    time; ``changes`` are recorded state changes [{t_sim, id, value}].
    ``require_entry`` False scores each task alone (fixtures that skip the entry gates).
    ``end_reason`` names why the window ends (the harness's ``time_limit`` or ``track_end``).
    ``horn`` is the source's horn capability (``unknown``: no planned stand-in, unscorable).
    """
    model = course(truth)
    hull = model['hull']
    t0, t1 = window or (track[0][0], track[-1][0]) if track else (0.0, 0.0)
    run = with_heading([(p[0], p[1], p[2], p[3] or 0.0) for p in track
                        if t0 <= p[0] <= t1 and all(map(math.isfinite, p[:3]))])
    warnings = []
    end = dict(reason=end_reason or ('track_end' if not window else 'finished'), t=t1)
    interventions = sorted((dict(a, kind='intervention') for a in authority
                            if t0 <= a['t'] <= t1 and a.get('by') in INTERVENING),
                           key=lambda a: a['t'])
    if interventions:
        end = dict(reason='intervention', t=interventions[0]['t'])
    out = []
    if model['boundary']:
        poly = [tuple(p) for p in model['boundary']]
        first = next((p for p in run if not inside_polygon(p[1:3], poly)), None)
        if first:
            out.append(dict(t=first[0], x=round(first[1], 2), y=round(first[2], 2)))
            if first[0] < end['t']:
                end = dict(reason='out_of_bounds', t=first[0])      # H p.66 §5.1.12
    run = [p for p in run if p[0] <= end['t']]
    gate_passes = {g['id']: passes(run, g, hull) for g in model['gates']}
    tasks = {}
    allowed = {}

    def rows_for(name):
        tasks[name] = _Rows(lambda t: allowed['f'](t))
        return tasks[name]

    entry_rows, t_entry, (t_exit, exits) = _entry_exit(model, run, rows_for, gate_passes)
    t_cut = t_exit if t_exit is not None else end['t']
    run = [p for p in run if p[0] <= t_cut]          # nothing after the cutoff is evidence
    gate_passes = {g: [p for p in v if p['t'] <= t_cut] for g, v in gate_passes.items()}
    if require_entry:
        allowed['f'] = lambda t: t is not None and t_entry is not None and t_entry < t <= t_cut
    else:
        allowed['f'] = lambda t: t is not None and t <= t_cut
    if t_exit is not None:
        entry_rows.add('T1.exit', 'S; L 3.2.1; H pp.45-47', 'H', 'back out through both '
                       'gates at the end of the run', 200, True, t=t_exit,
                       passes=[dict(p) for p in exits])
    else:
        entry_rows.add('T1.exit', 'S; L 3.2.1', 'H', 'exit through both gates', 200, False)
    _report_only(entry_rows, 'T1.entry_report', 'S; L 3.2.1; C', 'report entry time and '
                 'position', 100)
    _report_only(entry_rows, 'T1.exit_report', 'S; L 3.2.1; C', 'report exit time and '
                 'position', 100)
    _debris(model, run, rows_for('debris_clearance'), gate_passes,
            t_entry if require_entry else None)
    _speed(model, run, rows_for('speed_challenge'), gate_passes,
           t_entry if require_entry else None, changes)
    _supply(model, run, rows_for('supply_drop'))
    warnings += _marina(model, run, rows_for('marina'), changes, allowed['f'])

    def worked(a, b):
        """Task work completed in (a, b); avoiding the red indicator is passive, not work."""
        return [r['id'] for name, task in tasks.items() if name != 'harbor_alert'
                for r in task.rows if r['status'] in ('earned', 'provisional')
                and r['id'] != 'T2.red_avoid' and r['t'] is not None and a < r['t'] < b]

    _harbor(model, run, rows_for('harbor_alert'),
            (t_entry if require_entry and t_entry is not None else t0), t_cut, changes,
            worked, horn)
    scored = [p for p in run if p[0] <= t_cut]
    items = [e for e in model['elements'] if e.get('role') not in DOCK_PARTS]
    hits = strikes(scored, items, hull)
    penalty = STRIKE_POINTS * min(len(hits), STRIKE_MAX)
    start = None
    if run:
        start = dict(x=round(run[0][1], 2), y=round(run[0][2], 2),
                     offset_from_course_start_m=round(math.dist(run[0][1:3], model['start']), 2))
        gate1 = next((g for g in model['gates'] if g['task'] == 'entry_gates'
                      and g['order'] == 1), None)
        if gate1:
            behind = -((run[0][1] - gate1['c'][0]) * gate1['n'][0] +
                       (run[0][2] - gate1['c'][1]) * gate1['n'][1])
            start['before_gate1_m'] = round(behind, 2)
            start['ok'] = behind >= PARAMS['start_standoff_m']
        if start['offset_from_course_start_m'] > PARAMS['start_offset_warn_m']:
            warnings.append(f"run started {start['offset_from_course_start_m']} m from the "
                            'course start')
    if require_entry and t_entry is None:
        warnings.append('never entered through the entry gates: no other task scores (S 4.2)')
    body = dict(tasks={}, penalties=dict(strikes=hits, strike_points=-penalty,
                                         interventions=interventions, out_of_bounds=out))
    for name, rows in tasks.items():
        task = model['tasks'].get(name) or {}
        body['tasks'][name] = dict(type=task.get('type', name), cite=task.get('cite'),
                                   attempted=any(r['behaviour'] for r in rows.rows),
                                   rows=rows.rows)
    _totals(body)
    body['run'] = dict(window=[t0, t1], end=dict(end, t=round(end['t'], 3)),
                       scored_until=round(t_cut, 3),
                       entry_t=None if t_entry is None else round(t_entry, 3),
                       exit_t=None if t_exit is None else round(t_exit, 3),
                       duration_s=round(t_cut - t0, 2), start=start,
                       mode='run' if require_entry else 'task', legacy_course=model['legacy'],
                       hull=dict(length=hull[0], beam=hull[1]), warnings=warnings)
    body['params'] = dict(PARAMS)
    return body


def _totals(body):
    """Task and run totals from the rows and the strike penalty (recomputed after any
    capability adjustment)."""
    penalty = -body['penalties']['strike_points']
    total = dict(points=0, provisional=0, possible=0, possible_scorable=0,
                 penalty=-penalty, by_task={})
    for name, task in body['tasks'].items():
        rows = task['rows']
        task.update(points=sum(r['points'] for r in rows),
                    provisional=sum(r['provisional'] for r in rows),
                    possible=sum(r['possible'] for r in rows if r['status'] != 'n/a'))
        total['points'] += task['points']
        total['provisional'] += task['provisional']
        total['possible'] += task['possible']
        total['possible_scorable'] += sum(r['possible'] for r in rows
                                          if r['status'] not in ('n/a', 'unscorable'))
        total['by_task'][name] = task['points']
    total['points'] -= penalty
    body['total'] = total


# ---------------------------------------------------------------- capabilities

CONTACT_BASIS = {'geometric': 'geometry (no physics)', 'physics': 'geometry',
                 'judge': 'judge-reported (geometric contacts are diagnostic only)'}


def surveyed(truth, d):
    """``truth`` with every geometric margin moved by ``d`` m: objects grow by ``d``, slips
    shrink by ``2d`` and harbor zones by ``d`` (``d`` > 0 is strict, < 0 lenient)."""
    out = copy.deepcopy(truth)
    for e in out.get('elements', []):
        e['radius'] = max(0.01, (e.get('radius') or PARAMS['object_radius_m']) + d)
    for bay in out.get('bays', []):
        bay['width'], bay['depth'] = max(0.0, bay['width'] - 2 * d), max(0.0, bay['depth'] - 2 * d)
    for task in out.get('tasks', []):
        for anchor in (task.get('anchors') or {}).values():
            if isinstance(anchor, dict) and anchor.get('x') is not None:
                anchor['r'] = max(0.0, (anchor.get('r') or PARAMS['harbor_zone_r_m']) - d)
    return out


def _bracket(body, judged):
    """Surveyed truth: a row whose verdict flips within one survey sigma (strict vs lenient
    truth) was decided inside the margin, so it is provisional at the more generous points."""
    strict, lenient = ({r['id']: r for t in j['tasks'].values() for r in t['rows']}
                       for j in judged)
    sigma = body['run']['capabilities']['truth_sigma_m']
    for task in body['tasks'].values():
        for row in task['rows']:
            if row['status'] in ('unscorable', 'n/a'):
                continue
            lo, hi = strict.get(row['id']), lenient.get(row['id'])
            got = row['points'] + row['provisional']
            best = max(got, (hi or {}).get('points', 0) + (hi or {}).get('provisional', 0))
            if (lo or {}).get('points', 0) + (lo or {}).get('provisional', 0) == best:
                continue                                  # same verdict across the margin
            row['evidence'].update(margin_m=sigma, decided_within_truth_sigma=True,
                                   strict_status=(lo or {}).get('status'),
                                   lenient_status=(hi or {}).get('status'))
            row.update(status='provisional', points=0, provisional=best)


def _judged_contacts(body, reported):
    """Judge-reported contacts are the strikes; geometric strikes become evidence only."""
    pen = body['penalties']
    pen['geometric_strikes'] = pen['strikes']
    if reported is None:
        pen.update(strikes=[], strike_points=0, strikes_status='unscorable')
        return
    t0, t1 = body['run']['window'][0], body['run']['scored_until']
    hits = sorted((dict(object=c.get('object'), task=c.get('task'), t=c['t'], by=c.get('by'),
                        min_clearance_m=None) for c in reported if t0 <= c['t'] <= t1),
                  key=lambda c: c['t'])
    pen.update(strikes=hits, strike_points=-STRIKE_POINTS * min(len(hits), STRIKE_MAX),
               strikes_status='judge-reported')


STATE_ROWS = ('T3.direction', 'T5.dock')     # rows that read light/indicator state changes


def _provisional(body, why, ids=None):
    """Earned rows (all, or ``ids``) become provisional, with the reason as evidence."""
    for task in body['tasks'].values():
        for row in task['rows']:
            if row['status'] == 'earned' and (ids is None or row['id'] in ids):
                row['evidence'].setdefault('provisional_because', []).append(why)
                row.update(status='provisional', provisional=row['points'], points=0)


def judge_record(record):
    """Judge a run record (``run_record.load``) by its capabilities only, never its source."""
    caps = record['caps']
    t0, t1, reason = record['window']

    def run(truth):
        return judge(truth, record['track'], window=(t0, t1), authority=record['authority'],
                     changes=record['changes'], end_reason=reason, horn=caps['horn'])

    body = run(record['truth'])
    body['run']['capabilities'] = dict(caps)
    if caps['truth'] == 'surveyed':
        sigma = caps['truth_sigma_m']
        _bracket(body, (run(surveyed(record['truth'], sigma)),
                        run(surveyed(record['truth'], -sigma))))
    if caps['contacts'] == 'judge':
        _judged_contacts(body, record['contacts_reported'])
    if caps['state_log'] in ('partial', 'contradictory'):
        _provisional(body, f"state log {caps['state_log']}: {record['state_log_detail']}",
                     STATE_ROWS)
    body['run']['time_limit_s'] = record['time_limit_s']
    if record['time_limit_s'] is None:
        why = 'no time_limit_s for this run: the official duration is unset, so a row may ' \
              'fall after it'
        _provisional(body, why)
        body['run']['warnings'].append(why + ' (scored to the track end, rows provisional)')
    body['penalties']['contact_basis'] = CONTACT_BASIS[caps['contacts']]
    _totals(body)
    return body


def summary(report):
    """One line: points, provisional, scorable possible, task marks, strikes, end."""
    total, run = report['total'], report['run']
    marks = ' '.join(f"{name.split('_')[0]}:{t['points']}" for name, t in report['tasks'].items()
                     if t['possible'])
    strikes_n = len(report['penalties']['strikes'])
    return (f"{total['points']} pts (+{total['provisional']} provisional) of "
            f"{total['possible_scorable']} scorable · {marks} · {strikes_n} strike"
            f"{'' if strikes_n == 1 else 's'} · ended {run['end']['reason']}")

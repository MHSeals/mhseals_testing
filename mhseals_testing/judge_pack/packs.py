"""The RoboBoat packs: the referee wrapped unchanged, marine motion/safety, the dependency graph.

``roboboat``: ``score.judge_record`` runs once per record (cached); rows become metrics with the
referee's statuses mapped onto the judge's (earned/missed -> ok, provisional rows -> provisional,
nothing scorable -> unscorable). The legacy ``mhseals.score/1`` body stays reachable
(``referee(rec)``), byte-identical to the referee's own (tests/test_judge_pack.py).

``marine``: keep-off clearance as a three-valued Property (circle bound against every course
element, sigma from the survey and the track), speed and turn-rate profiles, and the mission's
decision diagnostics that expose perception gaps across tiers: the share of 'search' decisions and
the time to the first 'execute' decision.
"""

import math

import numpy as np

from mhseals_testing.judge_pack._core import core

core()

from judge.metric import Margin, Metric, MetricValue, Property, Status, unscorable  # noqa: E402
from judge.pack import DepGraph, HintRule, Node, Pack  # noqa: E402
from judge.packs import generic_motion  # noqa: E402
from judge.task import Objective, TaskSpec  # noqa: E402

from mhseals_testing.analysis import reliability, score  # noqa: E402

TASK_ROW = score.LEGACY_TASKS          # condition task -> referee task (nav_channel -> entry_gates)


def referee(rec):
    cache = rec.attachments.setdefault('cache', {}) if isinstance(rec.attachments, dict) else {}
    if 'score' not in cache:
        cache['score'] = score.judge_record(rec.attachments['roboboat.record'])
    return cache['score']


def _status(total):
    if not total.get('possible_scorable'):
        return Status.UNSCORABLE
    return Status.PROVISIONAL if total.get('provisional') else Status.OK


class Points(Metric):
    name, unit, direction = 'referee.points', 'pts', 'higher'
    requires = frozenset({'cap:truth'})

    def compute(self, rec, task):
        tot = referee(rec)['total']
        st = _status(tot)
        if st == Status.UNSCORABLE:
            return unscorable('no scorable row')
        return MetricValue(float(tot['points']), st, extra={'provisional': tot['provisional'],
                                                            'possible_scorable': tot['possible_scorable']})


class TaskPoints(Metric):
    """Points of the run's own task (the pair condition's task, mapped to the referee's)."""
    name, unit, direction = 'referee.task_points', 'pts', 'higher'
    requires = frozenset({'cap:truth'})

    def compute(self, rec, task):
        name = TASK_ROW.get(rec.header.task_id, rec.header.task_id)
        t = referee(rec)['tasks'].get(name)
        if not t:
            return unscorable(f'referee has no task {name!r}')
        rows = t['rows']
        status = Status.PROVISIONAL if t.get('provisional') else Status.OK
        return MetricValue(float(t['points']), status,
                           extra={'rows': {r['id']: r['status'] for r in rows}, 'possible': t.get('possible')})


class RowEarned(Metric):
    """1 when the referee row was earned, 0 when missed, unscorable otherwise."""
    unit, direction = '', 'higher'
    requires = frozenset({'cap:truth'})

    def __init__(self, row):
        self.row, self.name = row, f'referee.{row}'

    def compute(self, rec, task):
        for t in referee(rec)['tasks'].values():
            for r in t['rows']:
                if r['id'] == self.row:
                    if r['status'] in ('earned', 'missed'):
                        return MetricValue(float(r['status'] == 'earned'), extra={'t': r.get('t')})
                    if r['status'] == 'provisional':
                        return MetricValue(1.0, Status.PROVISIONAL)
                    return unscorable(f'{self.row}: {r["status"]}')
        return unscorable(f'no row {self.row}')


class Strikes(Metric):
    name, unit, direction = 'referee.strikes', 'n', 'lower'
    requires = frozenset({'cap:contacts'})

    def compute(self, rec, task):
        pen = referee(rec)['penalties']
        if pen.get('strikes_status') == 'unscorable':
            return unscorable('strikes unscorable (no judge sheet)')
        return MetricValue(float(len(pen['strikes'])))


def outcome(rec, values):
    """The referee's failure categories (reliability.run_flags) plus whether the task scored."""
    r = rec.attachments.get('roboboat.record') or {}
    flags = reliability.run_flags(referee(rec), r.get('authority_known', True), r.get('mission_stop'),
                                  r.get('mission_stop_why'))
    failed = flags['failed']
    if rec.header.task_id:             # a single-task run never exits the course: not a DNF
        failed = [c for c in failed if c != 'did_not_finish']
    if failed:
        return failed[0]
    tp = values.get('referee.task_points')
    if tp is not None and tp.value is not None:
        return 'success' if tp.value > 0 else 'no_task_points'
    return 'clean' if flags['clean'] else 'unknown'


# ------------------------------------------------------------------ marine

def _hull_radius(rec):
    v = rec.truth.get('vessel') or {}
    return 0.5 * math.hypot(v.get('length') or 1.063, v.get('beam') or 0.895)


class KeepOff(Property):
    """Distance from the hull's bounding circle to every course element's circle (>= 0 holds).
    Conservative (the circle covers the hull's corners), so a violation here is a near miss, not a
    scored contact; the referee's strike rows score contacts."""
    name, unit = 'marine.keep_off', 'm'
    requires = frozenset({'pose.se2'})

    def margin(self, rec, task):
        els = [e for e in rec.truth.get('elements', []) if 'x' in e and 'y' in e]
        if not els:
            return None
        ch = rec.channel('pose.se2')
        t0, t1 = rec.header.window[:2]
        ch = ch.window(t0, t1)
        xy = ch.v[:, :2]
        c = np.array([[e['x'], e['y']] for e in els])
        r = np.array([e.get('radius', 0.25) for e in els])
        d = np.linalg.norm(xy[:, None, :] - c[None], axis=2) - r[None] - _hull_radius(rec)
        g = d.min(axis=1)
        sig = (rec.caps.get('truth_sigma_m') or 0.0) + float(np.max(ch.sigma_of('x')))
        return Margin(ch.t, g, np.full(len(g), 2 * sig) if sig else None)


class SOG(Metric):
    unit, direction = 'm/s', 'none'
    requires = frozenset({'pose.se2'})

    def __init__(self, q):
        self.q, self.name = q, f'marine.sog_p{q}'

    def compute(self, rec, task):
        ch = rec.channel('pose.se2').window(*rec.header.window[:2])
        if len(ch) < 5:
            return unscorable('short track')
        tt = np.arange(ch.t[0], ch.t[-1], 0.2)
        p = np.column_stack([np.interp(tt, ch.t, ch.v[:, k]) for k in range(2)])
        v = np.linalg.norm(np.diff(p, axis=0), axis=1) / 0.2
        return MetricValue(float(np.percentile(v, self.q)))


class SearchShare(Metric):
    """Share of mission decisions that were 'search' (the boat had not seen what it needed)."""
    name, unit, direction = 'mission.search_share', '', 'lower'

    def compute(self, rec, task):
        d = rec.events_of('task.decision')
        if not d:
            return unscorable('no decision events (claims not kept)')
        return MetricValue(sum(1 for e in d if e.payload.get('why') == 'search') / len(d))


class FirstExecute(Metric):
    """Seconds from the run start to the first 'execute' decision (perception + world model
    latency to a usable target); unscorable if it never executed."""
    name, unit, direction = 'mission.time_to_execute', 's', 'lower'

    def compute(self, rec, task):
        d = [e for e in rec.events_of('task.decision') if e.payload.get('why') == 'execute']
        if not rec.events_of('task.decision'):
            return unscorable('no decision events')
        if not d:
            return MetricValue(None, Status.UNSCORABLE, why='never executed', extra={'censored': rec.duration})
        return MetricValue(max(0.0, d[0].t - rec.header.window[0]))


class GoalCount(Metric):
    name, unit, direction = 'mission.goals', 'n', 'lower'

    def compute(self, rec, task):
        if not rec.events_of('task.decision', 'task.goal'):
            return unscorable('no task events')
        return MetricValue(float(len(rec.events_of('task.goal'))))


class Interventions(Metric):
    name, unit, direction = 'mission.interventions', 'n', 'lower'

    def compute(self, rec, task):
        r = rec.attachments.get('roboboat.record') or {}
        if not r.get('authority_known', True):
            return unscorable('authority journal not aligned')
        return MetricValue(float(sum(1 for e in rec.events_of('authority')
                                     if e.actor in score.INTERVENING)))


# ------------------------------------------------------------------ dependency graph

def _perception(rec):
    out = []
    for e in rec.events_of('task.decision'):
        if e.payload.get('why') == 'search':
            out.append((e.t, f'search: {str(e.payload.get("reason", ""))[:60]}'))
            break
    return out


def _planner(rec):
    g = [e.t for e in rec.events_of('task.goal')]
    for k in range(len(g)):
        n = sum(1 for x in g[k:] if x - g[k] <= 10.0)
        if n >= 6:
            return [(g[k], f'replan storm: {n} goals in 10 s')]
    return []


def _authority(rec):
    return [(e.t, f'{e.actor} took over ({e.payload.get("why")})') for e in rec.events_of('authority')
            if e.actor in score.INTERVENING][:1]


GRAPH = DepGraph('roboboat-v1', (
    Node('perception', (), _perception, ('events',)),
    Node('world_model', ('perception',), None),
    Node('mission', ('world_model',), None, ('events',)),
    Node('planner', ('mission',), _planner, ('events',)),
    Node('controller', ('planner',), None),
    Node('authority', ('controller',), _authority, ('events',)),
    Node('plant', ('authority',), None, ('pose.se2',)),
))


def failure_node(rec, values):
    oc = outcome(rec, values)
    return {'contact': 'plant', 'did_not_finish': 'mission', 'no_task_points': 'mission',
            'mission_failure': 'mission', 'takeover': 'authority', 'hold': 'authority',
            'false_claim': 'mission'}.get(oc)


def _perception_hint(by, ctx):
    dec = [i for i in by.get('events.first_difference', []) if i.data.get('kind') == 'decision'
           and 'search' in f"{i.data.get('low')}{i.data.get('high')}"]
    if dec:
        return (f'{len(dec)} pairs part on search vs execute with the boats together: the fast tier\'s '
                'truth-cone perception sees what Unity\'s rendered pipeline does not (Unity /objects/map is '
                'empty) - add detection latency/dropout to the fast tier or fix the Unity perception path '
                'before trusting fast-tier timing')
    return None


def _validity_hint(by, ctx):
    if any('invalidWaterSearches' in i.claim for i in by.get('integrity.batch_invalid', [])):
        return 'Unity water queries invalid: rebuild the player from origin/main and check the HDRP water query in batchmode'
    return None


def roboboat_pack():
    rows = ('T1.entry',)
    ms = [Points(), TaskPoints(), Strikes()] + [RowEarned(r) for r in rows]
    return Pack('roboboat', 'roboboat', {m.name: m for m in ms}, outcome=outcome,
                decision_events=('task.decision', 'task.goal'), anchor_event='task.task_started',
                dependency=GRAPH, failure_node=failure_node,
                taxonomy={c: c.replace('_', ' ') for c in reliability.CATEGORIES},
                hints=(HintRule('perception-gap', _perception_hint), HintRule('unity-water', _validity_hint)))


def marine_pack():
    ms = [KeepOff(), SOG(50), SOG(95), SearchShare(), FirstExecute(), GoalCount(), Interventions()]
    return Pack('marine', 'roboboat', {m.name: m for m in ms})


ROBOBOAT, MARINE = roboboat_pack(), marine_pack()
PACKS = (ROBOBOAT, MARINE, generic_motion.PACK)

# Bands are screening values for the fast <-> Unity trust gate (G4), owner to confirm.
TASK = TaskSpec('roboboat-2026', '1', 'roboboat', (
    Objective('marine.keep_off', 'higher', 'safety', band=0.5, margin_cite='screening'),
    Objective('referee.task_points', 'higher', 'task', ni_margin=25, band=25, margin_cite='screening'),
    Objective('referee.T1.entry', 'higher', 'task', band=0.0),
    Objective('motion.duration', 'lower', 'efficiency', band=10.0, margin_cite='screening'),
    Objective('motion.path_length', 'lower', 'efficiency', band=3.0),
    Objective('marine.sog_p50', 'higher', 'diagnostic', band=0.1),
    Objective('mission.time_to_execute', 'lower', 'diagnostic', band=2.0),
    Objective('mission.search_share', 'lower', 'diagnostic', band=0.1),
    Objective('mission.goals', 'lower', 'diagnostic', band=3.0),
), params={'cmd_limit': 1.0})

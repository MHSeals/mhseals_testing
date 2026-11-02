"""Harness sessions (kinematic fast tier, Unity) -> judge RunRecords.

The referee's adapter (``analysis.run_record.load``) reads the session; this module maps its
record onto the judge's typed record without reinterpreting anything: the track becomes
``pose.se2``, task events/statuses/authority/mission stop become events, the adapter's caps stay
caps, and the legacy record rides along (``attachments['roboboat.record']``) so the roboboat pack
scores it with ``score.judge_record`` unchanged.

Validity: a session that reports a Unity benchmark validity block with invalid water searches,
or a crash, is invalid and excluded from G3/G4 (counted, never dropped).
"""

import json
from pathlib import Path
import re

import numpy as np

from mhseals_testing.judge_pack._core import core

core()

from judge.record import (Clock, Event, Header, Robot, RunRecord, Scenario, Validity,  # noqa: E402
                          dedupe_time, se2_channel, twist_from_pose)
from judge.source import FidelityProfile, Refused, Source  # noqa: E402

from mhseals_testing.analysis import run_record  # noqa: E402

PROFILES = {
    'kinematic': FidelityProfile('kinematic', 'truth_cone', 'fixed_step', 'none', 0.5,
                                 'fast tier: no drift/current/waves/hydrodynamics; truth labels in a cone; '
                                 'no timing or latency; straight-line routes; geometric contacts'),
    'unity': FidelityProfile('physics', 'rendered', 'realtime_soft', 'rendered', 300.0,
                             'Unity full stack: rendered sensors, mhseals_nav, physics water'),
    'real': FidelityProfile('real', 'real', 'real', 'real', 3600.0, 'the boat'),
}
TIER = {'kinematic': 'fast', 'unity': 'unity', 'real': 'real'}
_NUM = re.compile(r'-?\d+(\.\d+)?\s*°?')


def why_class(text):
    """A goal/decision reason without its numbers: 'look toward unexplored bearing -60°' ->
    'look toward unexplored bearing'."""
    return _NUM.sub('', str(text or '')).replace('  ', ' ').strip(' :')


def _world(frame, x, y, yaw=0.0):
    """Mission-frame (map, origin at the course start) -> course world frame."""
    if not frame:
        return x, y, yaw
    import math
    c, s = math.cos(frame.get('yaw', 0.0)), math.sin(frame.get('yaw', 0.0))
    return frame.get('x', 0.0) + c * x - s * y, frame.get('y', 0.0) + s * x + c * y, yaw + frame.get('yaw', 0.0)


def _events(rec):
    frame = (rec.get('truth') or {}).get('course_frame')
    out = []
    for e in rec.get('task_events') or []:
        t = e.get('time', e.get('received'))
        if t is None:
            continue
        kind = e.get('event', 'event')
        payload = {k: v for k, v in e.items() if k not in ('schema',)}
        if kind == 'decision':
            payload['why'] = e.get('kind')
            payload['reason_class'] = why_class(e.get('reason'))
        elif kind == 'goal':
            payload['why'] = why_class(e.get('why'))
            if 'x' in e and 'y' in e:
                payload['wx'], payload['wy'], payload['wyaw'] = _world(frame, e['x'], e['y'], e.get('yaw', 0.0))
        out.append(Event(float(t), f'task.{kind}', 'mission', payload))
    for a in rec.get('authority') or []:
        out.append(Event(float(a['t']), 'authority', str(a.get('by')), {'why': a.get('cause')}))
    stop = rec.get('mission_stop')
    if stop:
        out.append(Event(float(rec['window'][1]), 'mission.stop', str(stop.get('by')), dict(stop)))
    return tuple(sorted(out, key=lambda e: e.t))


def unity_validity(session):
    """{flag: value} from any benchmark validity block the session carries (Unity runs report
    ``invalidWaterSearches``; the Oct 27 player gave about 21 per run)."""
    flags = {}
    for name in ('benchmark.json', 'summary.json', 'analysis/report.json', 'unity_metrics.json'):
        p = Path(session) / name
        if not p.exists():
            continue
        try:
            text = p.read_text(errors='replace')
        except OSError:
            continue
        for m in re.finditer(r'"invalidWaterSearches"\s*:\s*(\d+)', text):
            flags['invalidWaterSearches'] = max(flags.get('invalidWaterSearches', 0), int(m.group(1)))
        if re.search(r'"valid"\s*:\s*false', text):
            flags.setdefault('valid_false', 1)
    return {k: v for k, v in flags.items() if v}


def to_record(rec, scenario=None, provenance=None, run_name=None):
    """A run_record dict -> RunRecord."""
    source = rec['source']
    track = rec['track']
    if len(track) < 2:
        raise Refused(f'{rec["session"]}: track has {len(track)} samples')
    a = np.asarray(track, float)
    t, x, y, yaw = dedupe_time(a[:, 0], a[:, 1], a[:, 2], a[:, 3])
    caps = dict(rec['caps'])
    obs = 'surveyed' if caps.get('truth') == 'surveyed' else 'exact' if source != 'real' else 'estimated'
    sigma = None
    if caps.get('truth_sigma_m'):
        sigma = np.array([caps['truth_sigma_m'], caps['truth_sigma_m'], 0.0])
    pose = se2_channel(t, x, y, yaw, obs, sigma)
    session = Path(rec['session'])
    flags = unity_validity(session) if source == 'unity' else {}
    if rec.get('mission_stop') is None:
        flags['no_mission_stop'] = 1
    hard = {k: v for k, v in flags.items() if k != 'no_mission_stop'}
    validity = Validity(not hard, flags, frozenset({'G3', 'G4'}) if hard else frozenset())
    truth = rec.get('truth') or {}
    ident = ((truth.get('generator') or {}).get('identity')) or {}
    meta = rec.get('meta') or {}
    course = (meta.get('course') or {}).get('identity') or {}
    sc = scenario or Scenario(course.get('pair_key') or ident.get('pair_key'),
                              course.get('truth_sha256') or ident.get('truth_sha256'),
                              course.get('seed', (truth.get('generator') or {}).get('seed')),
                              truth.get('family'), {})
    w = rec['window']
    header = Header(run_id=run_name or f'{session.name}/{rec.get("run_id") or "run"}', tier=TIER.get(source, source),
                    source=f'roboboat_{source}', domain='roboboat', robot=Robot('blastoise', 'wam-v-mini'),
                    task_id=str((sc.condition or {}).get('task', '')), scenario=sc,
                    provenance=provenance or {}, clock=Clock(meta.get('clock', {}).get('domain', 'sim')),
                    window=(float(w[0]), float(w[1]), str(w[2])))
    return RunRecord(header, {'pose.se2': pose, 'twist.body': twist_from_pose(pose)}, _events(rec), caps,
                     validity, {'course': truth.get('name'), 'elements': truth.get('elements', []),
                                'vessel': truth.get('vessel') or {}},
                     {'roboboat.record': rec, 'cache': {}})


class SessionSource(Source):
    """A harness session (or a run record file) of any tier the referee has an adapter for."""
    name = 'roboboat_session'

    def __init__(self, tier=None):
        self.tier = tier or ''

    def discover(self, root):
        root = Path(root)
        yield from ([root] if (root / 'session.json').exists() else sorted(p.parent for p in root.rglob('session.json')))

    def load(self, ref, scenario=None, provenance=None, run_name=None):
        try:
            rec = run_record.load(ref)
        except run_record.Refused as e:
            raise Refused(str(e))
        out = to_record(rec, scenario, provenance, run_name)
        return out

    @property
    def profile(self):
        return PROFILES.get(self.tier, FidelityProfile())


def provenance_of(index_side):
    """{'code': {repo: commit}} from a pair-index side ({git: {...}})."""
    git = (index_side or {}).get('git') or {}
    code = {}
    for k, v in git.items():
        name = {'mhseals_nav': 'nav', 'mhseals_testing': 'testing'}.get(k, k)
        code[name] = (v or {}).get('commit') or (v or {}).get('hash') if isinstance(v, dict) else v
    return {'code': code, 'git': git, 'config': (index_side or {}).get('config')}


def json_load(path, default=None):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return default

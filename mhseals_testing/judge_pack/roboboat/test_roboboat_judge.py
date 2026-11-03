"""RoboBoat on the crane judge: the referee path is unchanged, stats agree with reliability.py,
the sources and real-boat fits find planted facts. Needs the crane judge core (CRANE_ROOT)."""

import json
from pathlib import Path
import random

import numpy as np
import pytest

from mhseals_testing.judge_pack.roboboat._core import core

if not core():
    pytest.skip('crane judge core not importable (set CRANE_ROOT)', allow_module_level=True)

from judge import stats  # noqa: E402
from judge.metric import Status  # noqa: E402
from judge.run import judge_tiers  # noqa: E402

from mhseals_testing.analysis import reliability, run_record, score  # noqa: E402
from mhseals_testing.judge_pack.roboboat import nlc, packs, real, sources  # noqa: E402

DATA = Path(__file__).resolve().parents[3] / "test" / "data" / "score2026"
CASES = sorted(p for p in DATA.iterdir() if p.is_dir() and (p / 'truth').exists()) if DATA.exists() else []


def _session(root, case, source='kinematic', extra=None, events=()):
    truth = json.loads((DATA / f"truth-{(case / 'truth').read_text().strip()}.json").read_text())
    d = root / f'{case.name}-{source}'
    d.mkdir(parents=True)
    (d / 'course_truth.json').write_text(json.dumps(truth))
    (d / 'track.jsonl').write_text((case / 'track.jsonl').read_text())
    (d / 'task_events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in events))
    meta = dict(kind='kinematic', time_limit_s=300.0) if source == 'kinematic' else dict(source='unity', time_limit_s=300.0)
    (d / 'session.json').write_text(json.dumps(meta))
    for name, text in (extra or {}).items():
        (d / name).write_text(text)
    return d


@pytest.mark.skipif(not CASES, reason='referee fixtures not found')
@pytest.mark.parametrize('case', CASES[:8], ids=lambda p: p.name)
def test_referee_body_is_unchanged_through_the_judge(tmp_path, case):
    d = _session(tmp_path, case)
    legacy = score.judge_record(run_record.load(d))
    rec = sources.SessionSource().load(d)
    assert json.dumps(packs.referee(rec), sort_keys=True, default=str) == json.dumps(legacy, sort_keys=True, default=str)
    v = packs.ROBOBOAT.metrics['referee.points'](rec, packs.TASK)
    assert v.value == legacy['total']['points'] or v.status == Status.UNSCORABLE


@pytest.mark.parametrize('seed', range(20))
def test_stats_match_reliability(seed):
    rng = random.Random(seed)
    n1, n2 = rng.randint(1, 40), rng.randint(1, 40)
    k1, k2 = rng.randint(0, n1), rng.randint(0, n2)
    assert stats.wilson(k1, n1) == reliability.wilson(k1, n1)
    assert stats.newcombe(k1, n1, k2, n2) == reliability.newcombe(k1, n1, k2, n2)
    cells = [rng.randint(0, 10) for _ in range(4)]
    if sum(cells):
        assert stats.newcombe_paired(*cells) == reliability.newcombe_paired(*cells)
    a = rng.uniform(0.01, 0.2)
    assert stats.z_upper(a) == reliability._z(a)


@pytest.mark.skipif(not CASES, reason='referee fixtures not found')
def test_unity_validity_excludes_from_g4_and_is_counted(tmp_path):
    case = CASES[0]
    fast = sources.SessionSource().load(_session(tmp_path / 'f', case))
    bad = json.dumps({'workers': [{'valid': False, 'invalidWaterSearches': 21}]})
    unity = sources.SessionSource().load(_session(tmp_path / 'u', case, 'unity', {'summary.json': bad}))
    assert not unity.validity.valid and unity.validity.flags['invalidWaterSearches'] == 21
    rep = judge_tiers([('p', fast, unity)], packs.TASK, packs.PACKS)
    assert rep['g4']['n_usable'] == 0 and rep['g4']['excluded'] == {'unity:invalidWaterSearches,no_mission_stop,valid_false': 1}
    assert any(i['kind'] == 'integrity.batch_invalid' for i in rep['insights'])


def test_goal_events_are_mapped_to_the_course_frame():
    x, y, yaw = sources._world({'x': -2.0, 'y': 1.0, 'yaw': np.pi / 2}, 1.0, 0.0, 0.0)
    assert abs(x + 2.0) < 1e-9 and abs(y - 2.0) < 1e-9 and abs(yaw - np.pi / 2) < 1e-9
    assert sources.why_class('look toward unexplored bearing -60°') == 'look toward unexplored bearing'


def test_plant_fit_recovers_a_planted_first_order_dead_time():
    rng = np.random.default_rng(0)
    dt, n, tau, gain, delay = 0.1, 3000, 1.5, 0.8, 0.6
    u = np.repeat(rng.choice([-1.0, 0.0, 0.5, 1.0], n // 40), 40)
    a = np.exp(-dt / tau)
    y = np.zeros(n)
    d = int(delay / dt)
    for k in range(n - 1):
        y[k + 1] = a * y[k] + (1 - a) * gain * (u[k - d] if k >= d else 0.0) + rng.normal(0, 0.002)
    fit = real.fopdt(np.arange(n) * dt, u, y, dt)
    assert abs(fit['delay_s'] - delay) < 0.11 and abs(fit['tau_s'] - tau) / tau < 0.1
    assert abs(fit['gain'] - gain) / gain < 0.05 and fit['r2'] > 0.8


def test_session_manifest_labels_reach_the_record(tmp_path):
    from mhseals_testing.annotate import manifest
    S = 1_000_000_000
    session, bag = tmp_path / 's1', tmp_path / 's1' / 'bag'
    bag.mkdir(parents=True)
    manifest.write(session, [
        {'t_ns': 100 * S, 'phase': 'conditions_set', 'wind': 2, 'waves': 1, 'current': 0},
        {'t_ns': 110 * S, 'phase': 'bad_data', 'on': True}, {'t_ns': 120 * S, 'phase': 'bad_data', 'on': False},
        {'t_ns': 130 * S, 'phase': 'said', 'id': 1, 'text': 'hold position here', 'pose': {'x': 1.0}}])
    doc = real.manifest_of(bag)
    assert doc is not None and real.labels_condition(doc, 105.0) == {'wind': 2, 'waves': 1, 'current': 0}
    assert manifest.overlaps(doc, 100.0, 200.0) == ['bad'] and doc['commands'][0]['text'] == 'hold position here'
    assert real.manifest_of(tmp_path / 'none') is None


def test_rigid_fit_finds_a_rotated_track():
    rng = np.random.default_rng(1)
    a = rng.uniform(-20, 20, (200, 2))
    th = 0.7
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    b = a @ R.T + [5.0, -3.0] + rng.normal(0, 0.1, a.shape)
    _, _, res = real.rigid_fit(a, b)
    assert np.sqrt(np.mean(res ** 2)) < 0.2


def test_nlc_pilot_records(tmp_path):
    t = np.arange(0, 10.0, 0.5)
    pose = np.zeros((2, len(t), 3))
    pose[:, :, 0] = t
    np.savez(tmp_path / 'episodes.npz', **{'S1/E6/pose': pose, 'S1/E6/t': t, 'S1/E6/arrived': np.array([8.0, 9.0]),
                                           'S1/TIDE-K/pose': pose, 'S1/TIDE-K/t': t,
                                           'S1/TIDE-K/arrived': np.array([6.0, 7.0])})
    summ = {'level': 'D1', 'tier': 'U*', 'S1': {
        'E6': {'min_margin': [0.5, 0.4], 'violation_s': [0, 0], 'ttg': [8.0, 9.0], 'peak_excursion': [0, 0]},
        'TIDE-K': {'min_margin': [0.3, -0.1], 'violation_s': [0, 1.0], 'ttg': [6.0, 7.0], 'peak_excursion': [0, 0.1]}}}
    (tmp_path / 'summary.json').write_text(json.dumps(summ))
    rep = nlc.judge_outputs(tmp_path)
    c = {x['metric']: x for x in rep['comparisons']}
    assert c['nlc.time_to_goal']['estimate'] == -2.0
    assert c['nlc.violated']['estimate'] == 0.5

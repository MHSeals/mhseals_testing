"""Golden cases for the 2026 scorer (mhseals_testing.analysis.score)."""

import json
import math
from pathlib import Path

import pytest

from mhseals_testing.analysis import score

DATA = Path(__file__).parent / 'data' / 'score2026'
CASES = sorted(p.name for p in DATA.iterdir() if p.is_dir())


def load(case):
    folder = DATA / case
    truth = json.loads((DATA / f"truth-{(folder / 'truth').read_text().strip()}.json").read_text())
    track = [(r['t'], r['x'], r['y'], r.get('yaw', 0.0))
             for r in map(json.loads, (folder / 'track.jsonl').read_text().splitlines())]
    return truth, track, json.loads((folder / 'expected.json').read_text())


def rows(report):
    return {r['id']: r for task in report['tasks'].values() for r in task['rows']}


@pytest.mark.parametrize('case', CASES)
def test_course_fixture_verdicts(case):
    """Crane's per-tier fixtures (task-level: they skip the entry gates)."""
    truth, track, expected = load(case)
    report = score.judge(truth, track, require_entry=False)
    got = rows(report)
    want = expected.get('tasks', {})
    if 'marina' in want:
        ev = got['T5.dock']['evidence']
        if want['marina']['tier'] == 'occupied':
            assert got['T5.dock']['status'] == 'missed'
            assert any(b['bay'] == want['marina']['entered'] and b['occupied'] for b in ev['bays'])
        else:
            marina = want['marina']
            assert (ev['entered'], ev['tier']) == (marina['entered'], marina['tier'])
    if 'speed_challenge' in want:
        ev = got['T3.direction']['evidence']
        for key in ('source', 'source_color_at_entry', 'expected', 'direction'):
            assert ev[key] == want['speed_challenge'][key]
        assert (got['T3.direction']['status'] == 'earned') == want['speed_challenge']['correct']
    if 'harbor_alert' in want:
        ev = got['T6.zone']['evidence']
        assert (ev['target_zone'], ev['reached'], ev['correct']) == tuple(
            want['harbor_alert'][k] for k in ('target_zone', 'reached', 'correct'))
    if 'entry_gates' in want:
        assert got['T1.entry']['status'] == 'earned' and got['T1.exit']['status'] == 'earned'
    debris = want.get('debris_clearance', {})
    for bid, val in debris.get('rescue', {}).items():
        deg = got['T2.green_circle']['evidence']['rescue'][bid]['sweep_deg']
        assert deg == pytest.approx(val['approx'], abs=val['tol'])
    for bid, val in debris.get('avoid', {}).items():
        deg = got['T2.red_avoid']['evidence']['avoid'][bid]['sweep_deg']
        assert deg == pytest.approx(val['sweep_deg']['approx'], abs=val['sweep_deg']['tol'])
        assert (got['T2.red_avoid']['status'] == 'provisional') == (abs(deg) < 350)
    if 'contacts' in expected and case.startswith(('contact', 'dock_wrong')):
        assert [s['object'] for s in report['penalties']['strikes']] == expected['contacts']


# ---- synthetic course: two entry gates along +x, a speed gate and mark to port

def gate_truth():
    def buoy(i, x, y):
        return dict(id=i, task=i.split('.')[0], role='gate', label='b', x=x, y=y, radius=0.2)
    els = [buoy('entry_gates.g1.p', 5, 1.2), buoy('entry_gates.g1.s', 5, -1.2),
           buoy('entry_gates.g2.p', 15, 1.2), buoy('entry_gates.g2.s', 15, -1.2),
           buoy('speed_challenge.gate.p', 19.0, 20.0), buoy('speed_challenge.gate.s', 21.0, 20.0),
           dict(id='speed_challenge.mark', task='speed_challenge', role='mark', label='yellow',
                x=20.0, y=40.0, radius=0.2)]
    gates = [dict(id='entry_gates.gate1', task='entry_gates', order=1, port='entry_gates.g1.p',
                  starboard='entry_gates.g1.s', normal_yaw=0.0),
             dict(id='entry_gates.gate2', task='entry_gates', order=2, port='entry_gates.g2.p',
                  starboard='entry_gates.g2.s', normal_yaw=0.0),
             dict(id='speed_challenge.gate', task='speed_challenge', order=1,
                  port='speed_challenge.gate.p', starboard='speed_challenge.gate.s',
                  normal_yaw=math.pi / 2)]
    tasks = [dict(id='entry_gates', type='entry_gates', semantics={}),
             dict(id='speed_challenge', type='speed_challenge',
                  semantics=dict(turn_mark='speed_challenge.mark'))]
    return dict(schema_version=2, startX=0, startY=0, elements=els, gates=gates, tasks=tasks,
                vessel=dict(length=1.0, beam=0.8))


def path(points, speed=1.0, t0=0.0, step=0.1):
    """A track along straight legs between ``points`` at ``speed``."""
    out, t = [], t0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        n = max(1, int(math.dist((x0, y0), (x1, y1)) / (speed * step)))
        yaw = math.atan2(y1 - y0, x1 - x0)
        for k in range(n):
            out.append((t, x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n, yaw))
            t += step
    out.append((t, *points[-1], out[-1][3]))
    return out


def loop(direction):
    """Entry, then the speed gate, round the mark ``direction`` (+1 CCW), back, exit."""
    side = 3 * direction
    return [(0, 0), (20, 0), (20, 10), (20, 30), (20 + side, 40), (20, 43), (20 - side, 40),
            (20, 30), (20, 10), (20, 0), (0, 0), (-2, 0)]


def test_entry_then_exit_scores_and_requires_order():
    report = score.judge(gate_truth(), path([(0, 0), (20, 0), (0, 0)]))
    got = rows(report)
    assert got['T1.entry']['status'] == 'earned' and got['T1.exit']['status'] == 'earned'
    backwards = rows(score.judge(gate_truth(), path([(20, 0), (0, 0)])))
    assert backwards['T1.entry']['status'] == 'missed'


def test_gate_line_jitter_is_one_pass():
    jitter = [(i * 0.1, 5 + 0.05 * (-1) ** i, 0.0, 0.0) for i in range(40)]
    track = path([(0, 0), (4.9, 0)]) + [(p[0] + 10, *p[1:]) for p in jitter]
    track += path([(5.2, 0), (8, 0)], t0=track[-1][0] + 0.1)
    gate = score.course(gate_truth())['gates'][0]
    assert [p['dir'] for p in score.passes(track, gate, (1.0, 0.8))] == [1]


def test_hull_must_fit_the_gate():
    wide = dict(gate_truth(), vessel=dict(length=1.0, beam=2.6))
    assert rows(score.judge(wide, path([(0, 0), (20, 0)])))['T1.entry']['status'] == 'missed'


@pytest.mark.parametrize('direction', [1, -1])
def test_speed_loop_scores_the_same_either_way_round(direction):
    """Symmetric: the loop row and time do not depend on the side (no direction source)."""
    got = rows(score.judge(gate_truth(), path(loop(direction))))
    assert got['T3.loop']['status'] == 'earned'
    assert math.copysign(1, got['T3.loop']['evidence']['sweep_deg']) == direction
    assert got['T3.time']['points'] == pytest.approx(250 - got['T3.time']['evidence']['T_s'])
    assert got['T3.direction']['status'] == 'n/a'


def test_nothing_scores_before_the_entry_gates():
    """S 4.2: the entry gates come first; a loop before them earns nothing."""
    pre = [(20, -5), (20, 10), (20, 30), (17, 40), (20, 43), (23, 40), (20, 30), (20, 10),
           (20, -5), (0, -5), (0, 0), (20, 0)]
    got = rows(score.judge(gate_truth(), path(pre)))
    assert got['T1.entry']['status'] == 'earned'
    assert got['T3.loop']['status'] == 'missed'


def test_strikes_are_contact_episodes_and_capped():
    truth = gate_truth()
    through = path([(5, -3), (5, 3)] * 3)                   # five legs over one buoy
    report = score.judge(truth, through, require_entry=False)
    assert [s['object'] for s in report['penalties']['strikes']].count('entry_gates.g1.p') == 5
    assert report['penalties']['strike_points'] == -25 * len(report['penalties']['strikes'])
    many = path([(5, -3), (5, 3)] * 30)
    capped = score.judge(truth, many, require_entry=False)
    assert capped['penalties']['strike_points'] == -500                # S: max 20 strikes


def test_intervention_ends_the_scored_run():
    track = path([(0, 0), (20, 0), (0, 0)])
    hold = [dict(t=track[len(track) // 2][0], by='hold', cause='test')]
    report = score.judge(gate_truth(), track, authority=hold)
    assert report['run']['end']['reason'] == 'intervention'
    assert rows(report)['T1.exit']['status'] == 'missed'
    agent = [dict(t=1.0, by='agent', cause='claim')]
    assert score.judge(gate_truth(), track, authority=agent)['run']['end']['reason'] != \
        'intervention'


def test_out_of_bounds_ends_the_run():
    truth = dict(gate_truth(), boundary=[[-5, -10], [30, -10], [30, 10], [-5, 10]])
    report = score.judge(truth, path([(0, 0), (20, 0), (20, 15), (0, 0)]))
    assert report['run']['end']['reason'] == 'out_of_bounds'
    assert rows(report)['T1.exit']['status'] == 'missed'


def test_report_is_json_and_summarised():
    report = score.judge(gate_truth(), path(loop(1)))
    json.dumps(report, allow_nan=False)
    assert 'pts' in score.summary(report)


def test_legacy_2025_truth_scores_entry_gates():
    truth = json.loads((Path(__file__).parent / 'data' / 'course_truth_2025.json').read_text())
    model = score.course(truth)
    assert model['legacy'] and any(g['task'] == 'entry_gates' for g in model['gates'])

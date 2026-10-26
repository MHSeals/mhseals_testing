"""Golden cases for the 2026 scorer (mhseals_testing.analysis.score)."""

import json
import math
from pathlib import Path

import pytest

from mhseals_testing.analysis import score

DATA = Path(__file__).parent / 'data' / 'score2026'
CASES = sorted(p.name for p in DATA.iterdir() if p.is_dir() and p.name != 'sources')


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
    horn = next((st.get('program') for st in truth.get('states', [])
                 if st.get('id') == 'harbor_alert.horn' and st.get('program')), None)
    log = [dict(t_sim=horn['trigger_s'], id='harbor_alert.horn', value=horn.get('pattern'))] \
        if horn and horn.get('trigger_s') is not None else []   # the sim's record of the horn
    report = score.judge(truth, track, require_entry=False, changes=log)
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
        assert (got['T2.red_avoid']['status'] == 'provisional') == (abs(deg) < 360)
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
    assert got['T3.loop']['evidence']['turns'] == direction
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


def test_batch_stats_and_compare():
    from mhseals_testing.harness import score_analyze as sa
    a = dict(metrics=dict(points=sa.stats([100, 120, 110, 90])), sources=['unity'], seeds=['1'])
    b = dict(a, metrics=dict(points=sa.stats([300, 320, 310, 290])))
    assert a['metrics']['points']['mean'] == 105
    lo, hi = a['metrics']['points']['ci95']
    assert lo < 105 < hi
    assert sa.compare(a, b)['points']['clear'] and sa.compare(a, b)['points']['diff'] == 200
    same = sa.compare(a, a)['points']
    assert same['diff'] == 0 and not same['clear']


# ---- v2 fixes (ideator round 1)

def test_a_clean_entry_after_a_touched_one_counts():
    truth = gate_truth()
    touched = path([(0, 0.95), (20, 0.95)])          # hull grazes the port buoys
    back = path([(20, 0.95), (0, 0.0)], t0=touched[-1][0] + 0.1)
    clean = path([(0, 0), (20, 0)], t0=back[-1][0] + 0.1)
    got = rows(score.judge(truth, touched + back + clean))
    assert got['T1.entry']['status'] == 'earned'
    assert got['T1.entry']['evidence']['attempts'] >= 1
    assert got['T1.entry']['t'] > back[-1][0]


def test_nothing_after_the_exit_changes_the_score():
    run = path(loop(1))
    base = score.judge(gate_truth(), run)
    more = run + path([(-2, 0), (20, 0), (20, 10)], t0=run[-1][0] + 0.1)
    after = score.judge(gate_truth(), more)
    assert after['total'] == base['total']
    assert rows(after)['T1.exit']['t'] == rows(base)['T1.exit']['t']


def test_sprint_near_loop_that_retraces_does_not_count():
    """Out to the mark's side and back the same way winds zero times: no loop."""
    pts = [(0, 0), (20, 0), (20, 10), (20, 30), (23, 40), (20, 30), (20, 10), (20, 0), (0, 0)]
    assert rows(score.judge(gate_truth(), path(pts)))['T3.loop']['status'] == 'missed'


def test_circle_after_a_reversal_still_counts():
    """A lead-in sweep the other way does not hide a later full turn."""
    centre = (0.0, 0.0)
    back = [(t * 0.1, 5 * math.cos(-t * 0.05), 5 * math.sin(-t * 0.05), 0.0) for t in range(40)]
    a0 = -40 * 0.05
    turn = [(4 + t * 0.1, 5 * math.cos(a0 + t * 0.05), 5 * math.sin(a0 + t * 0.05), 0.0)
            for t in range(int(2 * math.pi / 0.05) + 2)]
    deg, t = score.sweep(back + turn, centre, 8.0, 360.0)
    assert deg >= 360 and t is not None
    short = turn[:int(1.9 * math.pi / 0.05)]
    assert abs(score.sweep(short, centre, 8.0)[0]) < 360


def test_harbor_unknown_pattern_never_picks_a_zone():
    truth = gate_truth()
    truth['tasks'].append(dict(id='harbor_alert', type='harbor_alert', anchors=dict(
        zone1=dict(x=20, y=0, r=3), zone2=dict(x=40, y=0, r=3))))
    truth['states'] = [dict(id='harbor_alert.horn', program=dict(trigger_s=1.0, pattern='?'))]
    log = [dict(t_sim=1.0, id='harbor_alert.horn', value='on')]
    got = rows(score.judge(truth, path([(0, 0), (20, 0)]), require_entry=False, changes=log))
    assert got['T6.zone']['evidence']['target_zone'] is None
    assert got['T6.zone']['points'] == 150


def test_dock_tie_for_lowest_green_and_no_report_needed():
    bays = [dict(id=f'marina.{s}1', x=x, y=20.0, yaw=math.pi / 2, width=1.5, depth=2.4,
                 occupied=False, sign=dict(number=1), indicator=dict(id=f'i{s}', color='green'))
            for s, x in (('N', 0.0), ('S', 10.0))]
    for x in (0.0, 10.0):
        truth = dict(gate_truth(), bays=bays)
        track = path([(x, 15), (x, 20)]) + [(10 + k * 0.1, x, 20.0, 0.0) for k in range(40)]
        got = rows(score.judge(truth, track, require_entry=False))['T5.dock']
        assert got['status'] == 'earned' and got['points'] == 350


# ---- diagnostics (never scored)

def _diag(truth, track, **kw):
    from mhseals_testing.analysis import diagnostics
    report = score.judge(truth, track)
    model = score.course(truth)
    run = [p for p in track if p[0] <= report['run']['scored_until']]
    return report, diagnostics.diagnose(run, model, report, **kw)


def test_timeline_contacts_and_near_misses():
    truth = gate_truth()
    truth['elements'].append(dict(id='speed_challenge.rock', task='speed_challenge',
                                  role='obstacle',
                                  label='black', x=20.0, y=15.0, radius=0.2))
    truth['elements'].append(dict(id='speed_challenge.rock2', task='speed_challenge',
                                  role='obstacle', label='black', x=20.75, y=25.0, radius=0.2))
    report, diag = _diag(truth, path(loop(1)))
    line = diag['timeline']['speed_challenge']
    assert line['outcome'] == 'completed'
    assert line['encounter_t'] <= line['execute_t'] <= line['end_t']
    assert set(line['split_s']) == {'search_or_transit', 'approach', 'execution', 'recovery'}
    hits = diag['contacts']['contacts']
    assert [c['object'] for c in hits] == ['speed_challenge.rock'] * 2   # out and back
    assert all(c['speed_mps'] == pytest.approx(1.0, abs=0.05) for c in hits)
    assert hits[0]['kind'] == 'strike' and hits[0]['recovered_s'] > 0
    assert any(n['object'] == 'speed_challenge.rock2' for n in diag['contacts']['near_misses'])
    assert diag['hindsight']['label'].startswith('hindsight')
    assert len(report['penalties']['strikes']) == 2      # diagnostics agree with the strikes


def test_claims_are_checked_never_scored():
    run = path(loop(1))
    exit_t = rows(score.judge(gate_truth(), run))['T1.exit']['t']
    events = [dict(event='returned_home', time=exit_t + 1.0),
              dict(event='speed_finished', time=1.0),
              dict(event='stage_completed', stage='speed_challenge', time=2.0)]
    plain = score.judge(gate_truth(), run)
    _, diag = _diag(gate_truth(), run, task_events=events)
    got = {c['event']: c for c in diag['claims_vs_truth']}
    assert got['returned_home']['matched'] and got['returned_home']['latency_s'] == 1.0
    assert not got['speed_finished']['matched']
    assert diag['claimed']['speed_challenge']['outcome'] == 'completed'
    assert score.judge(gate_truth(), run)['total'] == plain['total']


@pytest.mark.parametrize('direction', [1, -1])
def test_diagnostics_symmetric_either_way_round(direction):
    _, diag = _diag(gate_truth(), path(loop(direction)))
    line = diag['timeline']['speed_challenge']
    assert line['outcome'] == 'completed'
    assert line['distance_m']['in_task'] == pytest.approx(
        _diag(gate_truth(), path(loop(-direction)))[1]['timeline']['speed_challenge']
        ['distance_m']['in_task'], abs=0.2)


def _harbor_truth():
    truth = gate_truth()
    truth['tasks'].append(dict(id='harbor_alert', type='harbor_alert', anchors=dict(
        zone1=dict(x=20, y=0, r=3), zone2=dict(x=40, y=0, r=3))))
    truth['states'] = [dict(id='harbor_alert.horn', initial='off',
                            program=dict(trigger_s=1.0, pattern='one'))]
    return truth


def test_harbor_zone_by_chance_with_the_horn_off_earns_nothing():
    """L 3.2.6: points need a response to a sounded horn; crossing a zone is not one."""
    log = [dict(t_sim=0.0, id='speed_challenge.light', value='red')]   # a log, no horn
    got = rows(score.judge(_harbor_truth(), path([(0, 0), (20, 0)]), require_entry=False,
                           changes=log))['T6.zone']
    assert got['status'] == 'n/a' and got['points'] == 0
    assert got['evidence']['horn'] == 'recorded: never sounded'


HORN = [dict(t_sim=1.0, id='harbor_alert.horn', value='one')]


def test_harbor_route_that_finishes_a_task_on_the_way_still_earns():
    """Dropping the task is evidence only: completing the entry on the way is not punished."""
    got = rows(score.judge(_harbor_truth(), path([(0, 0), (20, 0)]), require_entry=False,
                           changes=HORN))['T6.zone']
    assert got['status'] == 'earned' and got['points'] == 300
    assert got['evidence']['credited_after_horn'] == ['T1.entry']
    assert got['evidence']['dropped_task'] is False


def test_harbor_already_in_the_zone_is_not_an_entry():
    still = [(k * 0.1, 20.0, 0.0, 0.0) for k in range(300)]
    got = rows(score.judge(_harbor_truth(), still, require_entry=False, changes=HORN))['T6.zone']
    assert got['status'] == 'missed' and got['points'] == 0
    assert got['evidence']['in_zone_at_horn'] == 'zone1'
    again = path([(20, 0), (20, 6), (20, 0)])           # leave, then really enter
    got = rows(score.judge(_harbor_truth(), again, require_entry=False, changes=HORN))['T6.zone']
    assert got['status'] == 'earned' and got['points'] == 300


def test_harbor_pass_through_after_the_response_window_earns_nothing(monkeypatch):
    monkeypatch.setitem(score.PARAMS, 'harbor_response_s', 5.0)
    got = rows(score.judge(_harbor_truth(), path([(0, 0), (20, 0), (20, 6)]),
                           require_entry=False, changes=HORN))['T6.zone']
    assert got['status'] == 'missed' and got['points'] == 0
    assert got['evidence']['late_entry']['zone'] == 'zone1'


def test_harbor_planned_horn_without_a_state_log_is_provisional_only():
    got = rows(score.judge(_harbor_truth(), path([(0, 0), (20, 0)]),
                           require_entry=False))['T6.zone']
    assert got['status'] == 'provisional' and got['points'] == 0
    assert got['provisional'] == 300 and got['evidence']['horn'] == 'planned (no state log)'


def test_harbor_response_to_a_recorded_horn_earns():
    got = rows(score.judge(_harbor_truth(), path([(25, 5), (20, 0)]), require_entry=False,
                           changes=HORN))['T6.zone']
    assert got['status'] == 'earned' and got['points'] == 300


# ---- symmetry (bias guard): the same outcome by different paths or orders scores the same

def _bay(x, y, colour='red'):
    return dict(id='marina.N1', x=x, y=y, yaw=math.pi / 2, width=1.5, depth=2.4, occupied=False,
                sign=dict(number=1), indicator=dict(id='iN', color=colour))


def _then(track, points=None, pause=0.0):
    """Continue ``track`` along ``points``, or hold still for ``pause`` seconds."""
    t, x, y, yaw = track[-1]
    if pause:
        return track + [(t + 0.1 * (k + 1), x, y, yaw) for k in range(int(pause / 0.1))]
    return track + path([(x, y)] + points, t0=t + 0.1)[1:]


def test_same_tasks_in_a_different_order_score_the_same():
    truth = dict(gate_truth(), bays=[_bay(30, 5)])
    sprint = loop(1)[2:10]                 # (20, 10) .. round the mark .. back to (20, 0)
    a = _then(_then(path([(0, 0), (20, 0)] + sprint + [(30, 0), (30, 5)]), pause=3),
              [(30, 0), (20, 0), (0, 0), (-2, 0)])
    b = _then(_then(path([(0, 0), (20, 0), (30, 0), (30, 5)]), pause=3),
              [(30, 0), (20, 0)] + sprint + [(0, 0), (-2, 0)])
    ra, rb = score.judge(truth, a), score.judge(truth, b)
    assert rows(ra)['T5.dock']['status'] == rows(rb)['T5.dock']['status'] == 'earned'
    assert rows(ra)['T3.loop']['status'] == rows(rb)['T3.loop']['status'] == 'earned'
    assert ra['total']['points'] == pytest.approx(rb['total']['points'], abs=0.05)
    assert ra['total']['by_task'].keys() == rb['total']['by_task'].keys()
    for task in ra['total']['by_task']:
        assert ra['total']['by_task'][task] == pytest.approx(rb['total']['by_task'][task],
                                                             abs=0.05)


def _green_truth():
    truth = gate_truth()
    truth['elements'].append(dict(id='debris_clearance.green1', task='debris_clearance',
                                  role='indicator', label='green', x=0.0, y=30.0, radius=0.2))
    truth['tasks'].append(dict(id='debris_clearance', type='debris_clearance',
                               semantics=dict(rescue=['debris_clearance.green1'])))
    return truth


@pytest.mark.parametrize('direction', [1, -1])
def test_green_circle_scores_the_same_clockwise_or_anticlockwise(direction):
    ring = [(5 * math.cos(direction * k * 0.05), 30 + 5 * math.sin(direction * k * 0.05))
            for k in range(int(2.2 * math.pi / 0.05))]
    got = rows(score.judge(_green_truth(), path([(5, 20)] + ring), require_entry=False))
    row = got['T2.green_circle']
    assert row['status'] == 'provisional' and row['provisional'] == 200
    assert abs(row['evidence']['rescue']['debris_clearance.green1']['sweep_deg']) >= 360


@pytest.mark.parametrize('stern_in', [False, True])
def test_docking_bow_in_or_stern_in_scores_the_same(stern_in):
    truth = dict(gate_truth(), bays=[_bay(0, 20, 'green')])
    track = _then(path([(0, 15), (0, 20)]), pause=3)
    if stern_in:                                       # same positions, reversed heading
        track = [(t, x, y, yaw + math.pi) for t, x, y, yaw in track]
    report = score.judge(truth, track, require_entry=False)
    got = rows(report)['T5.dock']
    assert got['status'] == 'earned' and got['points'] == 350
    assert got['evidence']['entered'] == 'marina.N1'
    assert report['penalties']['strikes'] == []


# ---- v3 batch statistics (bias audit)

def test_t_quantile_is_exact_not_rounded_up():
    from mhseals_testing.harness import score_analyze as sa
    assert sa.t_quantile(11) == pytest.approx(2.201, abs=1e-3)   # the old table gave 2.13 (df 15)
    assert sa.t_quantile(9.6) > sa.t_quantile(10)


def test_rates_use_wilson_and_newcombe_and_survive_sd_zero():
    from mhseals_testing.harness import score_analyze as sa
    none = sa.stats([0.0] * 10, binary=True)
    assert none['interval'] == 'wilson' and none['ci95'][0] == 0 and none['ci95'][1] > 0.2
    a = dict(metrics=dict(entered=none), sources=['unity'], seeds=['1'])
    b = dict(metrics=dict(entered=sa.stats([1.0] * 10, binary=True)), sources=['unity'],
             seeds=['1'])
    got = sa.compare(a, b)['entered']
    assert got['diff'] == 1 and got['ci95'][0] > 0 and got['clear'] and got['clear_holm']
    flat = dict(metrics=dict(points=sa.stats([5, 5, 5])), sources=['unity'], seeds=['1'])
    assert 'sd=0' in flat['metrics']['points']['note']
    assert sa.compare(flat, flat)['points']['ci95'] is None


def test_compare_refuses_mixed_sources_or_seeds_unless_forced():
    from mhseals_testing.harness import score_analyze as sa
    m = dict(points=sa.stats([1, 2, 3]))
    a = dict(metrics=m, sources=['unity'], seeds=['1', '2'])
    with pytest.raises(sa.Refused):
        sa.compare(a, dict(metrics=m, sources=['kinematic'], seeds=['1', '2']))
    with pytest.raises(sa.Refused):
        sa.compare(a, dict(metrics=m, sources=['unity'], seeds=['1', '3']))
    forced = sa.compare(a, dict(metrics=m, sources=['unity'], seeds=['1', '3']), force=True)
    assert forced['_forced'] and 'seed sets differ' in forced['_forced'][0]


def test_dock_first_slip_held_decides_unless_best(monkeypatch):
    bays = [dict(id=f'marina.N{n}', x=x, y=20.0, yaw=math.pi / 2, width=1.5, depth=2.4,
                 occupied=False, sign=dict(number=n), indicator=dict(id=f'i{n}', color='green'))
            for n, x in ((1, 0.0), (2, 10.0))]
    truth = dict(gate_truth(), bays=bays)
    wrong_first = _then(path([(10, 15), (10, 20)]), pause=3)
    track = _then(_then(wrong_first, [(10, 15), (0, 15), (0, 20)]), pause=3)
    got = rows(score.judge(truth, track, require_entry=False))['T5.dock']
    assert (got['evidence']['tier'], got['points']) == ('green', 200)    # wrong slip first
    monkeypatch.setitem(score.PARAMS, 'dock_bay_choice', 'best')
    got = rows(score.judge(truth, track, require_entry=False))['T5.dock']
    assert (got['evidence']['tier'], got['points']) == ('lowest_green', 350)


def _session(folder, track, task_events=(), meta=None):
    folder.mkdir()
    (folder / 'course_truth.json').write_text(json.dumps(gate_truth()))
    (folder / 'track.jsonl').write_text(''.join(
        json.dumps(dict(t=t, x=x, y=y, yaw=yaw)) + '\n' for t, x, y, yaw in track))
    (folder / 'task_events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in task_events))
    if meta:
        (folder / 'session.json').write_text(json.dumps(meta))
    return folder


def test_the_harness_not_the_boat_sets_the_scored_window(tmp_path):
    """A boat that claims to start late or finish early does not move the window."""
    from mhseals_testing.harness import score_analyze as sa
    run = path([(0, 0), (20, 0), (0, 0), (-2, 0)])
    claims = [dict(event='task_started', time=30.0), dict(event='task_finished', time=31.0)]
    _, rec = sa.analyze(_session(tmp_path / 's1', run, claims, dict(time_limit_s=300.0)))
    assert rec['run']['window'][0] == run[0][0] and rec['run']['end']['reason'] == 'track_end'
    assert rec['run']['claimed_window'] == [30.0, 31.0]
    assert rows(rec)['T1.exit']['status'] == 'earned'
    _, rec = sa.analyze(_session(tmp_path / 's2', run, claims, dict(time_limit_s=10.0)))
    assert rec['run']['end']['reason'] == 'time_limit' and rec['run']['scored_until'] <= 10.0


def test_batch_reports_completion_and_time_including_cutoffs():
    from mhseals_testing.harness import score_analyze as sa

    def record(outcome, end_t):
        return dict(total=dict(points=0, provisional=0, by_task={}),
                    penalties=dict(strikes=[], out_of_bounds=[], interventions=[]),
                    run=dict(entry_t=0, duration_s=50, end=dict(reason='time_limit'),
                             scored_until=50.0),
                    diagnostics=dict(timeline=dict(marina=dict(
                        outcome=outcome, encounter_t=10.0, end_t=end_t))))
    done, cut = sa.metrics(record('completed', 20.0)), sa.metrics(record('cut_off_incomplete',
                                                                         None))
    assert done['time_s.marina'] == 10 and 'time_s.marina' not in cut
    assert cut['time_cut_s.marina'] == 40 and cut['completed.marina'] == 0
    side = sa.batch([record('completed', 20.0), record('cut_off_incomplete', None)])
    side = side['time_by_task']['marina']
    assert side['completion']['mean'] == 0.5 and side['completion']['interval'] == 'wilson'
    assert side['time_s_completed_only']['n'] == 1 and side['time_cut_s_incl_incomplete']['n'] == 2

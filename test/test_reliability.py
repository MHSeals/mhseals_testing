"""Reliability-first goldens (owner 10-26): timeslot check, per-run failure flags, batch failure
rate and finals view, and the non-inferiority verdict (decision support, never points)."""

import json
import shutil
from pathlib import Path

import pytest

from mhseals_testing.analysis import reliability as rel
from mhseals_testing.harness import score_analyze as sa

SOURCES = Path(__file__).parent / 'data' / 'score2026' / 'sources'
RUN = 'runs/004-mission.json'


def report(seed=1, points=500, duration=300.0, exit_t=True, strikes=0, by=None, false=False,
           outcome='complete', source='unity'):
    """A score report as analyze() writes it, reduced to what the batch and flags read."""
    rec = dict(total=dict(points=points, provisional=0, by_task={}),
               penalties=dict(strikes=[dict(object='b1', t=5.0)] * strikes, out_of_bounds=[],
                              interventions=[dict(t=9.0, by=by)] if by else []),
               run=dict(entry_t=0.0, exit_t=duration if exit_t else None, duration_s=duration,
                        end=dict(reason='time_limit', t=duration), scored_until=duration,
                        mode='run', source=source, course=dict(seed=seed)),
               claims=dict(by_stage=dict(docking=dict(task='marina', false_claim=True))
                           if false else {}))
    rec['reliability'] = rel.run_flags(rec, True, dict(outcome=outcome, by='procedure'))
    return rec


# ---- timeslot (owner, 2026-10-26: 900 s per run slot)

def _limit(tmp_path, limit):
    folder = tmp_path / 'run'
    shutil.copytree(SOURCES / 'unity_run', folder)
    run = json.loads((folder / RUN).read_text())
    run['result']['time_limit_s'] = limit
    (folder / RUN).write_text(json.dumps(run))
    return sa.analyze(folder)[1]


def test_a_run_budget_over_the_timeslot_is_flagged_and_still_its_own_window(tmp_path):
    over = _limit(tmp_path / 'a', 1800.0)
    slot = over['run']['timeslot']
    assert slot['timeslot_s'] == 900.0 and slot['over'] is True and 'owner, 2026-10-26' in \
        slot['cite']
    assert any('over the 900 s timeslot' in w for w in over['run']['warnings'])
    assert over['run']['time_limit_s'] == 1800.0          # the run's own limit is the window
    inside = _limit(tmp_path / 'b', 900.0)
    assert inside['run']['timeslot']['over'] is False
    assert inside['total'] == over['total']               # a flag, never a score change
    unset = _limit(tmp_path / 'c', None)
    assert unset['run']['timeslot']['over'] is None       # no silent default
    assert unset['run']['time_limit_s'] is None


# ---- per run

def test_clean_run_and_each_failure_category_with_its_cause():
    assert report()['reliability'] == dict(clean=True, failed=[], unknown=[],
                                           flags=report()['reliability']['flags'])
    cases = dict(did_not_finish=report(exit_t=False), hold=report(by='hold'),
                 takeover=report(by='rc'), contact=report(strikes=1),
                 false_claim=report(false=True), mission_failure=report(outcome='aborted'))
    for cat, rec in cases.items():
        r = rec['reliability']
        assert r['clean'] is False and cat in r['failed'], cat
        assert r['flags'][cat]['causes'], cat


def test_unobservable_categories_make_clean_unknown_but_a_failure_stays_a_failure():
    rec = report()
    rec['claims'] = dict(events=[], by_stage={}, unavailable='no task events or status kept')
    got = rel.run_flags(rec, authority_known=False, mission_stop=None)
    assert got['clean'] is None and set(got['unknown']) == {'hold', 'takeover', 'false_claim',
                                                            'mission_failure'}
    rec['penalties']['strikes'] = [dict(object='b1', t=5.0)]
    assert rel.run_flags(rec, authority_known=False)['clean'] is False


def test_harness_run_reads_mission_stop_outcome(tmp_path):
    rec = sa.analyze(shutil.copytree(SOURCES / 'unity_run', tmp_path / 'r'))[1]
    flags = rec['reliability']['flags']
    assert flags['mission_failure']['failed'] is False and flags['hold']['failed'] is False


# ---- batch and finals view

def test_batch_failure_rate_wilson_and_single_attempt_view():
    runs = [report(points=600)] * 7 + [report(points=200, strikes=1)] * 3
    got = sa.batch(runs)['reliability']
    assert got['failure_rate']['k'] == 3 and got['failure_rate']['n'] == 10
    lo, hi = got['failure_rate']['ci95']
    assert lo < 0.3 < hi and got['failure_rate']['interval'] == 'wilson'
    assert got['single_attempt']['p_clean']['rate'] == 0.7
    assert got['single_attempt']['p_points_at_least']['rate'] == 0.7      # X = 500
    assert sa.batch(runs, points_at_least=100)['reliability']['single_attempt'][
        'p_points_at_least']['rate'] == 1.0
    assert got['by_category']['contact']['k'] == 3 and got['by_category']['hold']['k'] == 0


# ---- reliability-first comparison

def _batch(n, failures, duration):
    """n runs over seeds 0..n-1; the first ``failures`` hit a buoy (same points throughout)."""
    return sa.batch([report(seed=i, duration=duration + (i % 5), strikes=int(i < failures))
                     for i in range(n)])


def _decide(a, b):
    return sa.decide(a, b, sa.compare(a, b))


def test_faster_but_flakier_candidate_is_rejected():
    got = _decide(_batch(100, 2, 300.0), _batch(100, 12, 240.0))
    assert got['verdict'] == 'not non-inferior' and got['gains'] is None
    assert got['line'].startswith('not non-inferior (more failures)')
    assert got['non_inferiority']['upper'] >= 0.05


def test_faster_with_equal_failures_is_accepted():
    got = _decide(_batch(100, 2, 300.0), _batch(100, 2, 240.0))
    assert got['non_inferiority']['non_inferior'] and got['verdict'] == 'better'
    assert got['line'].startswith('better (faster and non-inferior)')
    assert got['gains']['time']['diff'] == -60 and got['gains']['time']['clear']


def test_same_time_equal_failures_is_no_clear_difference():
    got = _decide(_batch(100, 2, 300.0), _batch(100, 2, 300.0))
    assert got['verdict'] == 'no clear difference'


def test_too_few_runs_cannot_show_non_inferiority():
    """Equal failures in 10 runs each: the margin cannot be excluded, so no 'better'."""
    got = _decide(_batch(10, 1, 300.0), _batch(10, 1, 240.0))
    assert got['verdict'] == 'not non-inferior' and not got['non_inferiority']['non_inferior']


def test_margin_is_configurable():
    a, b = _batch(100, 2, 300.0), _batch(100, 12, 240.0)
    got = sa.decide(a, b, sa.compare(a, b), margin=0.25)
    assert got['non_inferiority']['margin'] == 0.25 and got['verdict'] == 'better'


def test_newcombe_paired_identical_sides_is_centred_on_zero():
    diff, (lo, hi) = rel.newcombe_paired(3, 0, 0, 97)
    assert diff == 0 and lo <= 0 <= hi and hi < 0.05


@pytest.mark.parametrize('cand_fail, verdict', [(2, 'better'), (12, 'not non-inferior')])
def test_paired_decision(cand_fail, verdict):
    pairs = [(report(seed=i, duration=300.0 + i % 5, strikes=int(i < 2)),
              report(seed=i, duration=240.0 + i % 5, strikes=int(i < cand_fail)))
             for i in range(100)]
    per = [(sa.metrics(a), sa.metrics(b)) for a, b in pairs]
    out = {}
    for k in ('clean_time_s', 'points'):
        both = [(x[k], y[k]) for x, y in per if k in x and k in y]
        out[k] = dict(diff=sum(y - x for x, y in both) / len(both), clear=True)
    out['points']['clear'] = False
    got = sa.decide_pairs(pairs, out)
    assert got['verdict'] == verdict
    assert 'Newcombe paired score (method 10)' in got['non_inferiority']['test']

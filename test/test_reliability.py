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
    stop = outcome and dict(outcome=outcome, by='procedure')     # None: no mission_stop
    rec['reliability'] = rel.run_flags(rec, True, stop)
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


# ---- v7: known runs, unjudged runs, refusals

def test_a_failed_run_with_an_unknown_category_is_left_out_like_a_clean_one():
    runs = [report(seed=i, strikes=int(i < 2)) for i in range(8)] + \
        [report(seed=8, outcome=None), report(seed=9, strikes=1, outcome=None)]
    got = sa.batch(runs)['reliability']
    assert got['failure_rate']['k'] == 2 and got['failure_rate']['n'] == 8
    assert got['unknown_runs']['failed_with_unknown'] == 1 and got['unknown'] == 2
    assert got['failure_rate_bounds_with_unknown'] == [0.3, 0.4]      # best 3/10, worst 4/10
    assert 'clean_run' not in sa.metrics(runs[9])


def test_an_under_journaled_baseline_does_not_help_a_candidate():
    """Baseline: 80 journaled runs with 4 failures and 20 without mission_stop (10 of which hit
    a buoy). Counting the 10 failed unjournaled runs and dropping the 10 clean-looking ones
    would put the baseline at 14/90 and pass a 10/100 candidate; known runs only: 4/80."""
    base = [report(seed=i, strikes=int(i < 4)) for i in range(80)] + \
        [report(seed=80 + i, strikes=int(i < 10), outcome=None) for i in range(20)]
    cand = [report(seed=i, duration=240.0, strikes=int(i < 10)) for i in range(100)]
    a, b = sa.batch(base), sa.batch(cand)
    assert a['reliability']['failure_rate']['k'] == 4 and a['reliability']['failure_rate'][
        'n'] == 80
    got = sa.decide(a, b, sa.compare(a, b))
    assert got['verdict'] == 'not non-inferior'
    assert 'excluded (a category unknown): baseline 20/100' in got['line']
    lenient = sa.batch(base)['reliability']
    lenient['failure_rate'] = rel.rate(14, 90)                 # the v6 count, for contrast
    assert rel.unpaired_ni(lenient, b['reliability'])['non_inferior'] is True


def test_paired_pairs_with_an_unknown_side_are_excluded_and_counted():
    pairs = [(report(seed=i, strikes=int(i < 2)), report(seed=i, strikes=int(i < 2)))
             for i in range(100)]
    pairs[50] = (report(seed=50, strikes=1, outcome=None), pairs[50][1])
    got = sa.decide_pairs(pairs, {})
    assert got['non_inferiority']['n_pairs'] == 99
    assert 'excluded 1/100 pairs (a category unknown)' in got['line']


def _journal(folder, *events):
    with (folder / 'events.jsonl').open('a') as out:
        for e in events:
            out.write(json.dumps(dict(e, procedure='mission', node='boat_trial')) + '\n')


def _run(folder, run_id, **result):
    (folder / 'runs' / f'{run_id}.json').write_text(json.dumps(dict(
        schema='mhseals.run/1', run_id=run_id, procedure='mission', status=result.pop(
            'status', 'MEASURED'), result=dict(procedure='mission', task='mission', **result))))


def test_an_aborted_run_without_files_is_a_failure_and_a_refusal_is_listed(tmp_path):
    folder = shutil.copytree(SOURCES / 'unity_run', tmp_path / 'batch' / 's')
    _journal(folder, dict(phase='mission_start', run_id='005-mission', t_ns=1793001000000000000),
             dict(phase='mission_stop', run_id='005-mission', outcome='abort', by='operator',
                  cause='lost localization', t_ns=1793001010000000000))
    _run(folder, '005-mission', verdict='ABORTED')
    _run(folder, '006-mission', verdict='REFUSED', reason='sim not reset to the course start')
    scored, unjudged, _ = sa._records([tmp_path / 'batch'])
    assert [r['run']['run_id'] for r in scored] == ['004-mission']
    got = sa.batch(scored, unjudged=unjudged)['reliability']
    base = int(not scored[0]['reliability']['clean'])     # the fixture run, fully known
    assert got['runs'] == 2 and got['known'] == 2 and got['failures'] == base + 1
    assert got['unjudged']['failures'] == [dict(
        run_id='005-mission', cause="outcome 'abort' by operator (lost localization)")]
    assert got['by_category']['mission_failure'] == rel.rate(1, 2)
    refused = got['not_started']
    assert refused['n'] == 1 and refused['runs'][0]['run_id'] == '006-mission'
    assert 'sim not reset' in refused['runs'][0]['cause']
    assert 'not started (harness refusal)' in got['note']
    b = sa.batch(scored, unjudged=unjudged)
    line = sa.decide(b, b, sa.compare(b, b))['line']
    assert 'not started (harness refusal): baseline 1, candidate 1' in line


def test_a_started_run_record_without_stop_or_files_reads_its_verdict():
    started = rel.unjudged(dict(run_id='r', result=dict(verdict='CANCELED')), True)
    assert started['outcome'] == 'failure' and rel.known(started)
    assert 'CANCELED' in started['cause']
    unknown = rel.unjudged(dict(run_id='r', result=dict(verdict='PASS')), True)
    assert unknown['outcome'] == 'unknown' and not rel.known(unknown)
    refused = rel.unjudged(dict(run_id='r', result=dict(verdict='REFUSED')), False)
    assert refused['outcome'] == 'not_started'


# ---- v8: the writer's outcome fields (testing mission-run-referee-fields dc042f84)

WRITER = SOURCES / 'writer_dc042f84'     # the writer's test_app.py records, verbatim


def _writer(folder, name, run_id, track_rows=None):
    """The writer's own run record from its tests (test_app.py, dc042f84) as run ``run_id`` of
    the unity fixture: links and clocks moved onto that run; ``track_rows`` of 004's track
    are written as its partial track (None: the linked file is absent)."""
    run = json.loads((WRITER / f'{name}.json').read_text())
    run['run_id'] = run_id
    result = run['result']
    if result['files']:
        result['files'] = {'track': f'runs/{run_id}/track.jsonl'}
        result.update(run_start_t=1000.0, run_start_t_ns=1793001000000000000)
    if track_rows is not None:
        rows = (folder / 'runs' / '004-mission' / 'track.jsonl').read_text().splitlines()
        (folder / 'runs' / run_id).mkdir()
        (folder / 'runs' / run_id / 'track.jsonl').write_text('\n'.join(rows[:track_rows]) + '\n')
    (folder / 'runs' / f'{run_id}.json').write_text(json.dumps(run))
    return run


def test_writer_records_refusals_and_cancels_before_launch_are_not_started(tmp_path):
    folder = shutil.copytree(SOURCES / 'unity_run', tmp_path / 'batch' / 's')
    _writer(folder, 'refused', '005-mission')
    _writer(folder, 'cancelled', '006-mission')
    scored, unjudged, _ = sa._records([tmp_path / 'batch'])
    assert [r['run']['run_id'] for r in scored] == ['004-mission']
    got = sa.batch(scored, unjudged=unjudged)['reliability']
    assert got['runs'] == 1 and got['not_started']['n'] == 2
    assert [r['cause'] for r in got['not_started']['runs']] == [
        'not started (harness refusal): no time limit: x', 'not started (cancelled): cancelled']
    with pytest.raises(sa.Refused, match=r'not started \(refused\): no time limit: x'):
        sa.analyze(folder / 'runs' / '005-mission.json')


def test_writer_records_started_and_failed_are_failures_scoring_their_partial_track(tmp_path):
    folder = shutil.copytree(SOURCES / 'unity_run', tmp_path / 'batch' / 's')
    _writer(folder, 'aborted', '005-mission', track_rows=60)    # partial track, no journal
    _writer(folder, 'errored', '006-mission')                    # linked track never written
    scored, unjudged, _ = sa._records([tmp_path / 'batch'])
    assert [r['run']['run_id'] for r in scored] == ['004-mission', '005-mission']
    aborted = scored[1]['reliability']
    assert aborted['flags']['mission_failure']['failed'] is True
    assert "outcome 'aborted' by guard (guard:hold)" in aborted['flags']['mission_failure'][
        'causes'][0]
    assert aborted['clean'] is False
    assert [u['cause'] for u in unjudged] == ["outcome 'error' by harness (TypeError: bad stream)"]
    got = sa.batch(scored, unjudged=unjudged)['reliability']
    assert got['by_category']['mission_failure']['k'] == 2


def test_partial_links_do_not_fall_back_to_the_session_latest_run_files(tmp_path):
    folder = shutil.copytree(SOURCES / 'unity_run', tmp_path / 's')
    (folder / 'task_status.jsonl').write_text('{"t": 1001.0, "task": "x", "status": "done"}\n')
    _writer(folder, 'aborted', '005-mission', track_rows=60)
    from mhseals_testing.analysis import run_record
    rec = run_record.load(folder / 'runs' / '005-mission.json')
    assert rec['statuses'] == [] and rec['task_events'] == [] and rec['caps']['claims'] is False


def test_a_complete_writer_record_scores_like_the_v7_record(tmp_path):
    folder = shutil.copytree(SOURCES / 'unity_run', tmp_path / 's')
    before = sa.analyze(folder / 'runs' / '004-mission.json')[1]
    path = folder / 'runs' / '004-mission.json'
    run = json.loads(path.read_text())
    done = json.loads((WRITER / 'completed.json').read_text())['result']
    run['result'].update({k: done[k] for k in ('outcome', 'cause', 'started')})
    path.write_text(json.dumps(run))
    after = sa.analyze(path)[1]
    for key in ('tasks', 'penalties', 'total', 'claims', 'reliability'):
        assert after[key] == before[key], key


# ---- the real procedure title: run records are titled 'Mission rehearsal' (procedures registry)

def _titled(folder, name, run_id, track_rows=None, **result):
    """``_writer``'s record retitled as a real run ('Mission rehearsal'), ``result`` merged."""
    run = _writer(folder, name, run_id, track_rows)
    run['procedure'] = run['result']['procedure'] = 'Mission rehearsal'
    run['result'].update(result)
    (folder / 'runs' / f'{run_id}.json').write_text(json.dumps(run))
    return run


def test_real_titled_refused_cancelled_and_errored_before_link_runs_are_kept(tmp_path):
    folder = shutil.copytree(SOURCES / 'unity_run', tmp_path / 'batch' / 's')
    for run_id in ('005', '006', '007'):                   # started, partial track: scored
        _titled(folder, 'aborted', f'{run_id}-mission', track_rows=60)
    for run_id in ('008', '009'):                          # linked track never written
        _titled(folder, 'errored', f'{run_id}-mission')
    _titled(folder, 'errored', '010-mission', files={})     # errored before its track was linked
    _titled(folder, 'cancelled', '011-mission', started=True, run_start_t=4.0)  # after launch
    _titled(folder, 'refused', '012-mission')
    _titled(folder, 'cancelled', '013-mission')             # cancelled before launch
    other = dict(run_id='014-waypoints', procedure='Waypoint square',
                 result=dict(verdict='ABORTED', files={}))  # an older non-mission record
    (folder / 'runs' / '014-waypoints.json').write_text(json.dumps(other))
    scored, unjudged, _ = sa._records([tmp_path / 'batch'])
    assert len(scored) == 4
    got = sa.batch(scored, unjudged=unjudged)['reliability']
    assert got['runs'] == 8                                 # 6 before: 010 and 011 vanished
    assert [r['run_id'] for r in got['unjudged']['failures']] == [
        '008-mission', '009-mission', '010-mission', '011-mission']
    assert got['unjudged']['failures'][2]['cause'] == \
        "outcome 'error' by harness (TypeError: bad stream)"
    assert got['not_started']['n'] == 2
    assert [r['run_id'] for r in got['not_started']['runs']] == ['012-mission', '013-mission']


def _keyed(folder, name, run_id, key, title, track_rows=None, **result):
    """``_titled``'s record with the writer's ``procedure_key`` (top level and in ``result``)."""
    run = _titled(folder, name, run_id, track_rows, **result)
    run['procedure'] = run['result']['procedure'] = title
    run['procedure_key'] = run['result']['procedure_key'] = key
    (folder / 'runs' / f'{run_id}.json').write_text(json.dumps(run))
    return run


def test_a_mixed_session_counts_only_the_batch_procedure_and_lists_the_others(tmp_path):
    folder = shutil.copytree(SOURCES / 'unity_run', tmp_path / 'batch' / 's')
    _keyed(folder, 'aborted', '005-mission', 'mission', 'Mission rehearsal', track_rows=60)
    _keyed(folder, 'refused', '006-waypoint-square', 'square', 'Waypoint square')  # outcome
    scored, unjudged, excluded = sa._records([tmp_path / 'batch'])
    assert [r['run']['run_id'] for r in scored] == ['004-mission', '005-mission']  # 004: no key
    assert unjudged == [] and excluded == [
        dict(run_id='006-waypoint-square', procedure_key='square', session='s')]
    got = sa.batch(scored, unjudged=unjudged, excluded=excluded)
    assert got['runs'] == 2 and got['reliability']['runs'] == 2  # the square is no run
    assert (got['procedure'], got['excluded']) == (
        'mission', [dict(procedure_key='square', count=1)])
    scored, unjudged, excluded = sa._records([tmp_path / 'batch'], procedure='square')
    assert scored == [] and [u['run_id'] for u in unjudged] == ['006-waypoint-square']
    assert [e['procedure_key'] for e in excluded] == [None, 'mission']  # 004: v7 mission

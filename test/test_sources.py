"""Tier adapters (kinematic | unity | real): one run record, rules by capability only."""

import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from test_score import _harbor_truth, DATA, gate_truth, loop, path, rows  # noqa: E402

SOURCES = DATA / 'sources'
SNAPSHOT = SOURCES / 'v3_sim_reports.json'
HORN_AT = 16.0


def _write(folder, truth, track, meta=None, task_events=(), states=None, extra=None):
    folder.mkdir(parents=True)
    (folder / 'course_truth.json').write_text(json.dumps(truth))
    (folder / 'track.jsonl').write_text(''.join(
        json.dumps(dict(t=t, x=x, y=y, yaw=yaw)) + '\n' for t, x, y, yaw in track))
    (folder / 'task_events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in task_events))
    if states is not None:
        (folder / 'states.jsonl').write_text(''.join(json.dumps(s) + '\n' for s in states))
    if meta is not None:
        (folder / 'session.json').write_text(json.dumps(meta))
    for name, text in (extra or {}).items():
        (folder / name).write_text(text)
    return folder


def _synthetic():
    """(name, truth, track, states): entry/exit loop, harbor with and without a horn log,
    strikes after the entry."""
    harbor = path([(0, 0), (30, 0)])
    planned = _harbor_truth()
    planned['states'][0]['program']['trigger_s'] = HORN_AT
    hit = path([(0, 0), (17, 0), (15, -3), (15, 3), (15, -3), (17, 0)])
    return [('loop_ccw', gate_truth(), path(loop(+1)), None),
            ('harbor_recorded', _harbor_truth(), harbor,
             [dict(t_sim=HORN_AT, id='harbor_alert.horn', value='one')]),
            ('harbor_planned', planned, harbor, None),
            ('strikes_after_entry', gate_truth(), hit, None)]


def sim_sessions(root):
    """Every sim session the v3 snapshot covers: crane's fixtures and the synthetic runs, each
    as a unity and a kinematic session with v3's 300 s limit stated (v4 guesses none)."""
    out = {}
    cases = sorted(p for p in DATA.iterdir() if p.is_dir() and p.name != 'sources')
    for meta_name, meta in (('unity', dict(source='unity', time_limit_s=300.0)),
                            ('kinematic', dict(kind='kinematic', time_limit_s=300.0))):
        for case in cases:
            truth = json.loads((DATA / f"truth-{(case / 'truth').read_text().strip()}.json")
                               .read_text())
            track = [(r['t'], r['x'], r['y'], r.get('yaw', 0.0)) for r in
                     map(json.loads, (case / 'track.jsonl').read_text().splitlines())]
            claims = [json.loads(line) for line in (case / 'events.jsonl').read_text()
                      .splitlines()] if (case / 'events.jsonl').exists() else []
            name = f'{meta_name}-{case.name}'
            out[name] = _write(root / name, truth, track, meta, claims)
        for case, truth, track, states in _synthetic():
            name = f'{meta_name}-{case}'
            out[name] = _write(root / name, truth, track, meta, (), states)
    return out


NEW_KEYS = (('run', 'capabilities'), ('run', 'time_limit_s'), ('scorer', 'version'),
            ('diagnostics', 'version'), ('params', 'time_limit_s'), ('run', 'timeslot'),
            ('params', 'timeslot_s'))    # v6 adds the timeslot check and reliability


def digest(record):
    """sha256 of the report without what v4 adds (capabilities, time limit) or changes
    (versions, the PARAMS time limit default, now None)."""
    rec = json.loads(json.dumps(record))
    for a, b in NEW_KEYS:
        rec.get(a, {}).pop(b, None)
    rec.pop('reliability', None)
    return hashlib.sha256(json.dumps(rec, sort_keys=True).encode()).hexdigest()


def score_view(record):
    """The scored part, readable: totals, strikes, every row's status and points."""
    return dict(total=record['total'], strikes=len(record['penalties']['strikes']),
                basis=record['penalties'].get('contact_basis'),
                end=record['run']['end']['reason'], source=record['run'].get('source'),
                rows={r['id']: [r['status'], r['points'], r['provisional']]
                      for t in record['tasks'].values() for r in t['rows']})


def _analyze_all(root):
    from mhseals_testing.harness import score_analyze as sa
    return {name: sa.analyze(folder)[1] for name, folder in sim_sessions(root).items()}


SIGMA = 0.3
LIMIT = 1800.0      # the mission config's duration; stated per run, never defaulted
JUDGE_SHEET = [dict(t=12.0, object='entry_gates.g2.p', by='judge 1'),
               dict(t=999.0, object='entry_gates.g1.s', by='judge 1')]   # after the run: ignored


def real_truth():
    """A surveyed course in the crane truth format: no state program, survey sigma 0.3 m."""
    truth = _harbor_truth()
    del truth['states']
    truth['survey'] = dict(method='RTK', sigma_m=SIGMA)
    return truth


def real_track():
    """Clean entry on the centreline; exit 0.45 m off it, so the hull clears gate 1 by 0.15 m:
    inside one survey sigma."""
    return path([(0, 0), (20, 0), (20, 0.45), (-2, 0.45)], step=0.2)


def write_fixtures():
    """One committed run per source (``python3 test/test_sources.py --fixtures``)."""
    import shutil
    for name in ('kinematic', 'unity', 'real'):
        shutil.rmtree(SOURCES / name, ignore_errors=True)
    _write(SOURCES / 'kinematic', gate_truth(), path(loop(+1), step=0.2),
           dict(source='kinematic', kind='kinematic', seed=1, time_limit_s=LIMIT))
    _write(SOURCES / 'unity', _harbor_truth(), path([(0, 0), (30, 0)], step=0.2),
           dict(source='unity', seed=1, time_limit_s=LIMIT), (),
           [dict(t_sim=HORN_AT, id='harbor_alert.horn', value='one')])
    sheet = ''.join(json.dumps(c) + '\n' for c in JUDGE_SHEET)
    _write(SOURCES / 'real', real_truth(), real_track(), dict(source='real', time_limit_s=LIMIT),
           (), None,
           {'judge_contacts.jsonl': sheet})


# ---- proof: v4 changes nothing for sim runs (63 reports, byte-equal to v3 bar versions)

def test_sim_reports_are_unchanged_from_v3(tmp_path):
    snap = json.loads(SNAPSHOT.read_text())
    got = _analyze_all(tmp_path)
    assert set(got) == set(snap)
    changed = [n for n, r in got.items() if digest(r) != snap[n]['sha256']]
    assert not changed, {n: (score_view(got[n]), snap[n]['score']) for n in changed[:2]}


# ---- adapters and capabilities

def _copy(name, tmp_path, **files):
    import shutil
    folder = tmp_path / name
    shutil.copytree(SOURCES / name, folder)
    for fname, text in files.items():
        if text is None:
            (folder / fname).unlink()
        else:
            (folder / fname).write_text(text)
    return folder


@pytest.mark.parametrize('name, caps', [
    ('kinematic', dict(truth='exact', truth_sigma_m=None, contacts='geometric',
                       state_log='missing',
                       horn='planned', claims=True, reports=False)),
    ('unity', dict(truth='exact', truth_sigma_m=None, contacts='physics', state_log='complete',
                   horn='recorded', claims=True, reports=False)),
    ('real', dict(truth='surveyed', truth_sigma_m=SIGMA, contacts='judge', state_log='missing',
                  horn='unknown', claims=True, reports=False))])
def test_each_adapter_loads_one_record_and_declares_its_capabilities(name, caps, tmp_path):
    from mhseals_testing.analysis import run_record
    rec = run_record.load(_copy(name, tmp_path))
    assert rec['source'] == name and rec['caps'] == caps
    assert {'truth', 'track', 'window', 'authority', 'changes', 'task_events',
            'contacts_reported'} <= set(rec)


def test_real_run_rules_follow_its_capabilities(tmp_path):
    from mhseals_testing.harness import score_analyze as sa
    _, rec = sa.analyze(_copy('real', tmp_path))
    got = rows(rec)
    assert got['T1.entry']['status'] == 'earned'             # 0.6 m clear: robust to sigma
    assert got['T1.exit']['status'] == 'provisional'         # 0.15 m clear: inside sigma
    assert got['T1.exit']['evidence']['margin_m'] == SIGMA
    assert got['T6.zone']['status'] == 'unscorable'          # no horn log, silence unknown
    pen = rec['penalties']
    assert [s['object'] for s in pen['strikes']] == ['entry_gates.g2.p']   # judge sheet only
    assert pen['strike_points'] == -25 and pen['geometric_strikes'] == []
    assert pen['contact_basis'].startswith('judge-reported')
    assert rec['total']['points'] == 100 - 25 and rec['total']['provisional'] == 200


def test_real_run_without_a_judge_sheet_has_unscorable_strikes(tmp_path):
    from mhseals_testing.harness import score_analyze as sa
    _, rec = sa.analyze(_copy('real', tmp_path, **{'judge_contacts.jsonl': None}))
    assert rec['penalties']['strikes_status'] == 'unscorable'
    assert rec['penalties']['strikes'] == [] and rec['total']['penalty'] == 0


def test_real_run_without_a_survey_sigma_is_refused(tmp_path):
    from mhseals_testing.harness import score_analyze as sa
    truth = real_truth()
    del truth['survey']
    with pytest.raises(sa.Refused, match='sigma'):
        sa.analyze(_copy('real', tmp_path, **{'course_truth.json': json.dumps(truth)}))


def test_the_same_run_as_unity_differs_only_by_capability(tmp_path):
    """Same track and course, exact truth and geometric contacts: the exit earns, the strike
    sheet is ignored and the harbor row is n/a (the sim program says the horn never sounded)."""
    from mhseals_testing.harness import score_analyze as sa
    meta = json.dumps(dict(source='unity', time_limit_s=LIMIT))
    folder = _copy('real', tmp_path, **{'session.json': meta})
    _, rec = sa.analyze(folder)
    got = rows(rec)
    assert got['T1.exit']['status'] == 'earned' and got['T6.zone']['status'] == 'n/a'
    assert rec['penalties']['strikes'] == [] and rec['total']['points'] == 300


def test_rules_read_capabilities_not_the_source_name(tmp_path):
    """A record labelled unity with real capabilities scores exactly as the real run."""
    from mhseals_testing.analysis import run_record, score
    real = run_record.load(_copy('real', tmp_path))
    renamed = dict(real, source='unity')
    assert score.judge_record(real) == score.judge_record(renamed)


def test_no_scoring_code_names_a_source():
    """Bias/tier guard: the rules never branch on kinematic | unity | real."""
    import inspect
    import re
    from mhseals_testing.analysis import diagnostics, score
    from mhseals_testing.harness import score_analyze as sa
    texts = [inspect.getsource(score), inspect.getsource(diagnostics),
             inspect.getsource(sa.analyze), inspect.getsource(sa.metrics)]
    named = re.compile(r"""['"](kinematic|unity|real)['"]|\bsource\s*==""")
    assert not [m.group(0) for t in texts for m in named.finditer(t)]


# ---- state log: missing vs complete-empty vs partial vs contradictory (review finding 1)

HORN_REC = dict(t_sim=HORN_AT, id='harbor_alert.horn', value='one')
LOGS = {   # name: (states.jsonl text or None, session source, T6 status, horn basis or reason)
    'missing_sim': (None, 'unity', 'provisional', 'planned (no state log)'),
    'missing_real': (None, 'real', 'unscorable', 'caps.horn unknown'),
    'complete_empty': ('', 'unity', 'n/a', 'recorded: never sounded'),
    'complete_other_state': (json.dumps(dict(t_sim=1.0, id='speed_challenge.light',
                                             value='red')) + '\n', 'unity', 'n/a',
                             'recorded: never sounded'),
    'complete_horn': (json.dumps(HORN_REC) + '\n', 'unity', 'earned', 'recorded'),
    'partial_gap_no_horn': (json.dumps(dict(t_sim=5.0, id='log_gap')) + '\n', 'unity',
                            'unscorable', 'caps.horn partial'),
    'partial_torn_with_horn': (json.dumps(HORN_REC) + '\n{"t_sim": 20.0, "id": "har',
                               'unity', 'provisional', 'recorded (partial log'),
    'partial_ends_early': (json.dumps(dict(t_sim=10.0, id='log_end')) + '\n', 'unity',
                           'unscorable', 'caps.horn partial'),
    'contradictory': (json.dumps(HORN_REC) + '\n' + json.dumps(dict(HORN_REC, value='two'))
                      + '\n', 'unity', 'unscorable', 'caps.horn contradictory')}


@pytest.mark.parametrize('case', sorted(LOGS))
def test_state_log_status_decides_the_horn(case, tmp_path):
    from mhseals_testing.harness import score_analyze as sa
    text, source, status, basis = LOGS[case]
    truth = json.loads((SOURCES / 'unity' / 'course_truth.json').read_text())
    truth['states'][0]['program']['trigger_s'] = HORN_AT      # a planned horn inside the run
    if source == 'real':
        truth['survey'] = dict(sigma_m=SIGMA)
    folder = _copy('unity', tmp_path, **{
        'states.jsonl': text, 'course_truth.json': json.dumps(truth),
        'session.json': json.dumps(dict(source=source, time_limit_s=LIMIT))})
    _, rec = sa.analyze(folder)
    got = rows(rec)['T6.zone']
    assert got['status'] == status
    assert basis in str(got['evidence'].get('horn')) + str(got['evidence'].get('reason'))
    assert rec['run']['capabilities']['state_log'] == case.split('_')[0]


def test_partial_state_log_makes_state_reading_rows_provisional():
    from mhseals_testing.analysis import score
    body = dict(tasks=dict(speed_challenge=dict(rows=[dict(
        id='T3.direction', status='earned', points=150, provisional=0, evidence={})])))
    score._provisional(body, 'state log partial', score.STATE_ROWS)
    row = body['tasks']['speed_challenge']['rows'][0]
    assert (row['status'], row['points'], row['provisional']) == ('provisional', 0, 150)


# ---- time limit: none is guessed (review finding 2)

def test_no_time_limit_scores_to_the_track_end_with_rows_provisional(tmp_path):
    from mhseals_testing.harness import score_analyze as sa
    folder = _copy('kinematic', tmp_path, **{'session.json': json.dumps(dict(kind='kinematic'))})
    _, rec = sa.analyze(folder)
    assert rec['run']['time_limit_s'] is None and rec['run']['end']['reason'] == 'track_end'
    assert rec['total']['points'] == 0 and rec['total']['provisional'] == 700.8
    assert 'no time_limit_s' in ' '.join(rec['run']['warnings'])
    assert 'no time_limit_s' in rows(rec)['T1.entry']['evidence']['provisional_because'][0]


def test_a_stated_time_limit_cuts_the_run(tmp_path):
    from mhseals_testing.harness import score_analyze as sa
    folder = _copy('kinematic', tmp_path, **{'session.json': json.dumps(
        dict(kind='kinematic', time_limit_s=30.0))})
    _, rec = sa.analyze(folder)
    assert rec['run']['end']['reason'] == 'time_limit' and rec['run']['time_limit_s'] == 30.0
    assert rows(rec)['T1.exit']['status'] == 'missed'


# ---- pairing across tiers

def _pairs(tmp_path, rows_):
    pairs = tmp_path / 'pairs.jsonl'
    pairs.write_text(''.join(json.dumps(r) + '\n' for r in rows_))
    return pairs


def _twin(tmp_path, name, source, seed):
    import shutil
    folder = tmp_path / name
    shutil.copytree(SOURCES / 'real', folder)
    (folder / 'session.json').write_text(json.dumps(dict(source=source, time_limit_s=LIMIT)))
    if source == 'kinematic':
        (folder / 'course_truth.json').write_text(json.dumps(gate_truth()))
    return folder.name


@pytest.mark.parametrize('lower, upper, loop', [('kinematic', 'unity', 'loop 1'),
                                                ('unity', 'real', 'loop 2')])
def test_explicit_pairs_compare_both_loops(lower, upper, loop, tmp_path):
    from mhseals_testing.harness import score_analyze as sa
    pairs = _pairs(tmp_path, [dict(a_session=_twin(tmp_path, f'a{i}', lower, i),
                                   b_session=_twin(tmp_path, f'b{i}', upper, i), seed=i,
                                   order='fixed') for i in range(3)])
    out = sa.paired(pairs)
    assert out['schema'] == 'mhseals.score_pairs/1' and out['loop'].startswith(loop)
    assert out['n_pairs'] == 3 and out['sources'] == dict(a=lower, b=upper)
    assert out['b_minus_a']['exited']['test'] == 'exact McNemar'
    if upper == 'real':
        assert set(out['capability_differences']) >= {'truth', 'contacts', 'horn'}
        assert out['b_minus_a']['exited']['diff'] == 0.0     # exit 0 in real, provisional
        assert out['b_minus_a']['provisional']['diff'] == 200


def test_pairs_refuse_mixed_source_pairs_and_non_loops_unless_forced(tmp_path):
    from mhseals_testing.harness import score_analyze as sa
    mixed = _pairs(tmp_path, [dict(a=_twin(tmp_path, 'a0', 'kinematic', 0),
                                   b=_twin(tmp_path, 'b0', 'unity', 0)),
                              dict(a=_twin(tmp_path, 'a1', 'unity', 1),
                                   b=_twin(tmp_path, 'b1', 'real', 1))])
    with pytest.raises(sa.Refused, match='mix source pairs'):
        sa.paired(mixed)
    skip = _pairs(tmp_path, [dict(a='a0', b='b1')])
    with pytest.raises(sa.Refused, match='not a loop'):
        sa.paired(skip)
    assert sa.paired(skip, force=True)['forced']


def test_unpaired_batches_of_different_tiers_are_still_refused(tmp_path):
    from mhseals_testing.harness import score_analyze as sa
    for i in range(2):
        _twin(tmp_path / 'fast', f's{i}', 'kinematic', i)
        _twin(tmp_path / 'unity', f's{i}', 'unity', i)
    out = tmp_path / 'out.txt'
    with out.open('w') as fh:
        code = sa.main([str(tmp_path / 'fast'), '--batch', '--compare',
                        str(tmp_path / 'unity')], out=fh)
    assert code == 2


if __name__ == '__main__' and sys.argv[1:] == ['--snapshot']:
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        got = _analyze_all(Path(tmp))
        SNAPSHOT.write_text(json.dumps({n: dict(sha256=digest(r), score=score_view(r))
                                        for n, r in sorted(got.items())}, indent=1,
                                       sort_keys=True) + '\n')
    print(f'{len(got)} reports -> {SNAPSHOT}')

if __name__ == '__main__' and sys.argv[1:] == ['--fixtures']:
    write_fixtures()


# ---- harness runs: the run record and the session journal (writer mission-run-referee-fields)
# unity_run was built with the writer's own code at bb947204 (events.payload + journal.append as
# ros/node.py emits, HornJournal, runs.write_run): runs 003 and 004 in one journal, run 004
# judged (runs/004-mission.json links its files), horn 16 s after its start, sim time at 0.8x.

RUN = 'runs/004-mission.json'


def _journal(folder):
    return [json.loads(line) for line in (folder / 'events.jsonl').read_text().splitlines()]


def _harness(tmp_path, edit=None, record=None, text=None):
    folder = _copy('unity_run', tmp_path)
    if edit or text is not None:
        lines = [json.dumps(e) for e in edit(_journal(folder))] if edit else []
        (folder / 'events.jsonl').write_text(text if text is not None else
                                             '\n'.join(lines) + '\n')
    if record:
        run = json.loads((folder / RUN).read_text())
        record(run['result'])
        (folder / RUN).write_text(json.dumps(run))
    return folder


def _of(run_id, phase=None):
    return lambda e: e.get('run_id') == run_id and (phase is None or e['phase'] == phase)


def _horns(run_id, journal):
    """Run ``run_id``'s horn edges: those between its mission_start and mission_stop."""
    start = next(n for n, e in enumerate(journal) if _of(run_id, 'mission_start')(e))
    stop = next(n for n, e in enumerate(journal) if _of(run_id, 'mission_stop')(e))
    return [e for e in journal[start:stop] if e['phase'] == 'horn']


def test_a_harness_run_is_read_from_its_run_record_and_journal(tmp_path):
    from mhseals_testing.analysis import run_record
    from mhseals_testing.harness import score_analyze as sa
    folder = _harness(tmp_path)
    rec = run_record.load(folder)
    assert rec['run_id'] == '004-mission' and rec['time_limit_s'] == LIMIT
    assert rec['window'][0] == 1000.0 and 'run record' in rec['window_source']
    assert rec['caps']['horn'] == 'recorded' and rec['caps']['state_log'] == 'complete'
    # the journal's horn on the track clock (t_ns, not 0.8x sim time); crane's sim-clock
    # states.jsonl horn and run 003's horn are not this run's
    assert [(c['t_sim'], c['value']) for c in rec['changes']] == [(1016.0, 'one'),
                                                                  (1017.0, 'off')]
    assert rec['statuses'][0]['phase'] == 'harbor'
    target, report = sa.analyze(folder / RUN)
    assert report['run']['run_id'] == '004-mission' and '004-mission' in target.name
    zone = rows(report)['T6.zone']
    assert zone['status'] == 'provisional' and zone['provisional'] == 300
    assert 'mission_start' in zone['evidence']['provisional_because'][0]
    assert any('run_start_t_ns' in w for w in report['run']['warnings'])


def test_the_launch_time_on_the_journal_clock_makes_the_horn_exact(tmp_path):
    from mhseals_testing.harness import score_analyze as sa
    folder = _harness(tmp_path, record=lambda r: r.update(run_start_t_ns=1793000000 * 10**9))
    _, report = sa.analyze(folder)
    zone = rows(report)['T6.zone']
    assert zone['status'] == 'earned' and zone['points'] == 300
    assert not any('mission_start' in w for w in report['run']['warnings'])


_STOP = _of('004-mission', 'mission_stop')
JOURNALS = {   # name: (journal edit, caps.horn, T6 status, horn basis or reason)
    'complete_no_horn': (lambda j: [e for e in j if e['phase'] != 'horn'],
                         'recorded', 'n/a', 'recorded: never sounded'),
    'earlier_runs_horn_only': (lambda j: [e for e in j if e not in _horns('004-mission', j)],
                               'recorded', 'n/a', 'recorded: never sounded'),
    'partial_no_stop_horn': (lambda j: [e for e in j if not _STOP(e)],
                             'partial', 'provisional', 'recorded (partial log'),
    'partial_no_stop_no_horn': (lambda j: [e for e in j if not _STOP(e)
                                           and e not in _horns('004-mission', j)],
                                'partial', 'unscorable', 'caps.horn partial'),
    'contradictory': (lambda j: j[:-1] + [dict(_horns('004-mission', j)[0], on=False)] + j[-1:],
                      'contradictory', 'unscorable', 'caps.horn contradictory')}


@pytest.mark.parametrize('case', sorted(JOURNALS))
def test_the_journal_slice_decides_the_harness_runs_horn(case, tmp_path):
    from mhseals_testing.harness import score_analyze as sa
    edit, horn, status, basis = JOURNALS[case]
    folder = _harness(tmp_path, edit, record=lambda r: r.update(run_start_t_ns=1793 * 10**15))
    _, report = sa.analyze(folder)
    got = rows(report)['T6.zone']
    assert report['run']['capabilities']['horn'] == horn and got['status'] == status
    assert basis in str(got['evidence'].get('horn')) + str(got['evidence'].get('reason'))


def test_a_torn_line_inside_the_run_makes_the_journal_partial(tmp_path):
    from mhseals_testing.analysis import run_record
    folder = _copy('unity_run', tmp_path)
    lines = (folder / 'events.jsonl').read_text().splitlines()
    lines.insert(-1, lines[-1][:40])                    # torn before run 004's mission_stop
    (folder / 'events.jsonl').write_text('\n'.join(lines) + '\n')
    rec = run_record.load(folder)
    assert rec['caps']['horn'] == 'partial' and 'torn' in ' '.join(rec['state_log_detail'])


def test_without_a_journal_the_run_falls_back_to_the_state_log(tmp_path):
    from mhseals_testing.analysis import run_record
    folder = _harness(tmp_path, text='')
    (folder / 'events.jsonl').unlink()
    rec = run_record.load(folder)
    assert rec['caps']['horn'] == 'recorded' and rec['changes'][0]['value'] == 'on:600'
    assert 'no mission_start' in rec['authority_problem']


def test_journal_holds_cut_only_their_own_run(tmp_path):
    from mhseals_testing.analysis import run_record

    def takeovers(journal):
        start = next(e for e in journal if _of('004-mission', 'mission_start')(e))
        hold = dict(start, phase='authority', to=dict(owner='hold'), cause='operator',
                    t_ns=start['t_ns'] + 10 * 10**9, ros_time=start['ros_time'] + 8.0)
        hold.pop('run_id')
        after = dict(hold, t_ns=start['t_ns'] + 40 * 10**9, to=dict(owner='rc'))
        return journal[:-1] + [hold, journal[-1], after]
    rec = run_record.load(_harness(tmp_path, takeovers))
    assert [(round(a['t'], 6), a['by']) for a in rec['authority']] == [(1010.0, 'hold')]


def test_the_run_records_limit_decides_even_when_null(tmp_path):
    from mhseals_testing.analysis import run_record
    folder = _harness(tmp_path, record=lambda r: r.update(time_limit_s=None))
    (folder / 'session.json').write_text(json.dumps(dict(source='unity', time_limit_s=30.0)))
    assert run_record.load(folder)['time_limit_s'] is None
    folder = _harness(tmp_path / 'b', record=lambda r: r.pop('time_limit_s'))
    (folder / 'session.json').write_text(json.dumps(dict(source='unity', time_limit_s=30.0)))
    assert run_record.load(folder)['window'][2] == 'time_limit'


def test_a_batch_scores_each_judged_run_once(tmp_path):
    from mhseals_testing.analysis import run_record
    from mhseals_testing.harness import score_analyze as sa
    _harness(tmp_path / 'batch')
    _copy('unity', tmp_path / 'batch')
    got = sa._records([tmp_path / 'batch'])
    assert sorted(str(r['run'].get('run_id')) for r in got) == ['004-mission', 'None']
    folder = tmp_path / 'batch' / 'unity_run'
    run = json.loads((folder / RUN).read_text())
    del run['result']['files']
    (folder / 'runs' / '005-mission.json').write_text(json.dumps(run))
    with pytest.raises(run_record.Refused, match='not judged'):
        run_record.load(folder / 'runs' / '005-mission.json')

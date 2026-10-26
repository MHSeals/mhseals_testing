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
    as a unity session, a kinematic session and a legacy session without session.json."""
    out = {}
    cases = sorted(p for p in DATA.iterdir() if p.is_dir() and p.name != 'sources')
    for meta_name, meta in (('unity', dict(source='unity')), ('kinematic', dict(kind='kinematic')),
                            ('legacy', None)):
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


NEW_KEYS = (('run', 'capabilities'), ('scorer', 'version'), ('diagnostics', 'version'))


def digest(record):
    """sha256 of the report without the fields v4 adds (capabilities) or bumps (versions)."""
    rec = json.loads(json.dumps(record))
    for a, b in NEW_KEYS:
        rec.get(a, {}).pop(b, None)
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
           dict(source='kinematic', kind='kinematic', seed=1))
    _write(SOURCES / 'unity', _harbor_truth(), path([(0, 0), (30, 0)], step=0.2),
           dict(source='unity', seed=1), (),
           [dict(t_sim=HORN_AT, id='harbor_alert.horn', value='one')])
    sheet = ''.join(json.dumps(c) + '\n' for c in JUDGE_SHEET)
    _write(SOURCES / 'real', real_truth(), real_track(), dict(source='real'), (), None,
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
    ('kinematic', dict(truth='exact', truth_sigma_m=None, contacts='geometric', state_log=False,
                       horn='planned', claims=True, reports=False)),
    ('unity', dict(truth='exact', truth_sigma_m=None, contacts='physics', state_log=True,
                   horn='recorded', claims=True, reports=False)),
    ('real', dict(truth='surveyed', truth_sigma_m=SIGMA, contacts='judge', state_log=False,
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
    folder = _copy('real', tmp_path, **{'session.json': json.dumps(dict(source='unity'))})
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


# ---- pairing across tiers

def _pairs(tmp_path, rows_):
    pairs = tmp_path / 'pairs.jsonl'
    pairs.write_text(''.join(json.dumps(r) + '\n' for r in rows_))
    return pairs


def _twin(tmp_path, name, source, seed):
    import shutil
    folder = tmp_path / name
    shutil.copytree(SOURCES / 'real', folder)
    (folder / 'session.json').write_text(json.dumps(dict(source=source)))
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

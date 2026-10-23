"""The journal fold: fixtures B (characterize abort), C (override), D (config churn), E (long)."""

import json
import math
import pathlib
import random

from mhseals_testing.ui.timeline import Timeline

DATA = pathlib.Path(__file__).parent / 'data' / 'timeline'
H = lambda h, m, s=0: h * 3600 + m * 60 + s


def load(name, shuffle=False):
    lines = [json.loads(x) for x in (DATA / f'{name}.jsonl').read_text().splitlines()]
    if shuffle:
        random.Random(3).shuffle(lines)
    tl = Timeline()
    for line in lines:
        tl.add(line)
    return tl


def runs(tl):
    return [s for s in tl.spans() if s.kind in ('proc', 'manual')]


def test_fixture_b_abort_colours_only_the_cut_slice():
    tl = load('B')
    magcal, aborted, rerun = runs(tl)
    assert (magcal.key, magcal.end, magcal.level) == ('g', '', 0)
    assert (aborted.key, aborted.end, aborted.level, aborted.cause) == ('f', 'warn', 1, 'space')
    assert [s[2] for s in aborted.slices] == [0, 0, 0, 1]
    assert aborted.slices[-1][:2] == (H(14, 15), H(14, 16, 40))
    assert rerun.t1 is None and rerun.slices[0][2] == 0


def test_fixture_b_items_fold_hold_into_the_abort_row():
    tl = load('B')
    rows = tl.items('events')
    assert [r.kind for r in rows] == ['run', 'mark', 'health', 'abort', 'config', 'proc', 'config']
    assert next(r for r in rows if r.kind == 'abort').result == 'ABORTED+hold'
    assert not any(r.kind == 'auth' for r in rows)
    assert [r.kind for r in tl.items('markers')] == ['mark']
    assert tl.items('results')[0].result == 'TRUST 96'  # the running rerun


def test_fixture_b_health_span_and_onset_only():
    tl = load('B')
    health = [s for s in tl.spans() if s.kind == 'health']
    assert [(s.label, s.level, s.t0, s.t1) for s in health] == [
        ('compass.innovation', 1, H(14, 15), H(14, 16, 50))]
    assert [p.t for p in tl.points() if p.kind == 'health'] == [H(14, 15)]


def test_fixture_c_manual_bar_hold_and_guest_point():
    tl = load('C')
    manual = [s for s in runs(tl) if s.kind == 'manual']
    assert [(s.key, s.t1 is None) for s in manual] == [('m', False), ('m', True)]
    assert next(s for s in tl.spans() if s.kind == 'hold').t1 is None
    assert any(p.kind == 'auth' and p.label.startswith('guest') for p in tl.points())
    assert next(s for s in runs(tl) if s.key == 'f').end == 'warn'


def test_fixture_d_fault_stop_is_red_cap_only():
    tl = load('D')
    faults = [s for s in runs(tl) if s.end]
    assert len(faults) == 1 and faults[0].end == 'fail' and faults[0].slices == ()
    configs = [p for p in tl.points() if p.kind == 'config']
    assert sum(p.sev == 2 for p in configs) == 1 and len(configs) == 15


def test_fixture_e_long_session_spans_and_busy():
    tl = load('E')
    spans = tl.spans()
    assert [s.t0 for s in spans] == sorted(s.t0 for s in spans)
    assert sum(s.kind == 'armed' for s in spans) == 3
    assert [s.end for s in runs(tl) if s.end] == ['warn', 'fail', 'fail']
    assert any(b[1] == math.inf for b in tl.busy())
    assert len(tl.busy()) == sum(s.kind in ('armed', 'proc', 'manual', 'health') for s in spans) + len(tl.points())


def test_armed_and_recorder_off_needs_the_intersection():
    tl = load('E')
    # recorder off 15:01-15:47, armed again only at 15:48: the hazard is not "either".
    assert tl.unrecorded() == []
    tl = Timeline()
    for t, phase in [(H(1, 0), 'armed'), (H(1, 5), 'rec_stop'), (H(1, 9), 'rec_start')]:
        tl.add(dict(t_ns=int(t * 1e9), phase=phase))
    assert tl.unrecorded() == [(H(1, 5), H(1, 9))]


def test_out_of_order_lines_give_the_same_fold():
    for name in 'BCDE':
        a, b = load(name), load(name, shuffle=True)
        assert a.spans() == b.spans() and a.points() == b.points()
        assert a.version == b.version


def test_version_bumps_and_bad_lines_are_skipped():
    tl = Timeline()
    tl.add(dict(t_ns=1_000_000_000, phase='armed'))
    v = tl.version
    for bad in ({'phase': 'armed'}, {'t_ns': 'x'}, {'t_ns': float('nan')}):
        tl.add(bad)
    assert tl.version == v and len(tl.spans()) == 1
    tl.add(dict(t_ns=2_000_000_000, phase='disarmed', by='fault', reason='fcu'))
    assert tl.version == v + 1
    assert (tl.spans()[0].end, tl.spans()[0].t1) == ('fail', 2.0)


def test_malformed_fields_are_skipped_and_do_not_break_refolds():
    tl = Timeline()
    tl.add(dict(t_ns=1_000_000_000, phase='characterize_start', procedure='characterize'))
    for bad in (dict(t_ns=2e9, phase='result', procedure='characterize', window=[1]),
                dict(t_ns=2e9, phase='result', procedure='characterize', window=['a', 'b']),
                dict(t_ns=2e9, phase='result', procedure='characterize', window=[1, float('nan')]),
                dict(t_ns=3e9, phase='health', level='warn')):
        tl.add(bad)
    tl.add(dict(t_ns=1.5e9, phase='marker', note='late'))      # out of order: full refold
    tl.add(dict(t_ns=4e9, phase='result', procedure='characterize', window=[1e9, 2e9]))
    assert len(runs(tl)) == 1 and len(tl.points()) == 1
    assert runs(tl)[0].slices[0][:2] == (1.0, 2.0)


def test_unknown_view_is_refused():
    try:
        Timeline().items('nope')
    except ValueError:
        return
    raise AssertionError('expected ValueError')

"""Challenge judges: every assertion of test_referee.py has a twin here, plus the owner's record."""

import pytest

from mhseals_testing.referee import core, course, pack
from mhseals_testing.referee.core import Window
from referee_helpers import TRUTH, element, line

COURSE = course.from_truth(TRUTH)
SPECS = {n: pack.load_spec(pack.CHALLENGE_DIR / d / f'{n}.yaml')
         for d, n in (('2025', 'nav_channel'), ('commissioning', 'pair_test'))}


def with_params(name, **judge):
    spec = SPECS[name]
    return pack.Spec(**{**spec.__dict__, 'params': {**spec.params, **judge}})


def judge(name, track, events=(), truth_course=COURSE, final=True, banner=None, source='truth'):
    spec = SPECS[name]
    t0 = track[0][0] if track else 0.0
    window = Window(t0, track[-1][0] if track else 0.0, tuple(track), tuple(events), source)
    return core.judge(spec, truth_course, window, final)


def checks(result):
    return {c.name: c.ok for c in result.checks}


def test_channel_passes_only_when_both_gates_are_crossed():
    result = judge('nav_channel', line((0, 0), (16, 0)))
    assert result.verdict == 'PASS' and result.completion == 'done' and result.failures == []
    short = judge('nav_channel', line((0, 0), (10, 0)))
    assert short.verdict == 'FAIL' and short.details['gates'] == '1/2'
    assert short.completion == 'failed' and short.failures == ['every gate crossed: missed gate 2']


def test_touching_a_buoy_fails_even_through_the_gates():
    result = judge('nav_channel', line((0, 1.0), (16, 1.0)))
    assert checks(result)['every gate crossed'] and result.contacts
    assert result.verdict == 'FAIL' and result.basis == 'truth'


@pytest.mark.parametrize('swapped', [False, True])
def test_pair_commissioning_requires_both_crossings_of_the_same_pair(swapped):
    sign = -1 if swapped else 1
    truth = course.from_truth(dict(schema=course.TRUTH_SCHEMA, elements=[
        element('pair_test', 'red_pole_buoy', 7, sign * 1.5),
        element('pair_test', 'green_pole_buoy', 7, -sign * 1.5)]))
    out = line((0, 0), (10, 0))
    back = line((10, 0), (5.5, 0), t0=11)
    assert judge('pair_test', out + back, truth_course=truth).verdict == 'PASS'
    assert judge('pair_test', out, truth_course=truth).verdict == 'FAIL'


def test_open_window_leaves_undecided_checks_none_and_never_regresses():
    track = line((0, 0), (16, 0))
    early = judge('nav_channel', track[:30], final=False)  # past gate 1 only
    assert early.verdict == 'PENDING' and early.completion == 'incomplete'
    assert checks(early)['every gate crossed'] is None and early.splits.keys() == {'gate1'}
    done = judge('nav_channel', track, final=False)
    assert checks(done)['every gate crossed'] is True and done.verdict == 'PENDING'  # contacts can still come
    assert done.splits['gate1'] == early.splits['gate1']
    assert judge('nav_channel', track).verdict == 'PASS'
    scrape = judge('nav_channel', line((0, 1.0), (8, 1.0)), final=False)
    assert scrape.verdict == 'FAIL'  # a contact fails at once


def test_splits_are_seconds_from_the_window_start_and_time_is_its_duration():
    result = judge('nav_channel', line((0, 0), (16, 0), t0=100))
    assert result.time_s == 10.0 and 0 < result.splits['gate1'] < result.splits['gate2'] <= 10


def test_exceeding_the_time_limit_is_a_failure():
    slow = line((0, 0), (16, 0), steps=50, dt=5.0)
    result = judge('nav_channel', slow)
    assert result.verdict == 'FAIL' and 'within the time limit' in result.failures


def test_contacts_carry_the_basis_of_the_course_and_track():
    track = line((0, 1.0), (16, 1.0))
    assert judge('nav_channel', track, source='estimate').basis == 'estimated'
    confirmed = core.Course('day', COURSE.elements, 'confirmed')
    assert judge('nav_channel', track, truth_course=confirmed, source='estimate').basis == 'confirmed'


def test_sequence_order_flags_a_gate_taken_out_of_order():
    spec = with_params('nav_channel', order='sequence')
    nearest_first = Window(0, 10, tuple(line((11, 0), (15, 0)) + line((15, 0), (5, 0), t0=11)))
    assert core.judge(spec, COURSE, nearest_first).verdict == 'PASS'
    other_way = Window(0, 10, tuple(line((11, 0), (5, 0)) + line((5, 0), (16, 0), t0=11)))
    result = core.judge(spec, COURSE, other_way)
    assert result.verdict == 'FAIL' and 'gates in order' in result.failures


def test_gates_in_order_is_undecided_while_open_and_holds_once_final():
    spec = with_params('nav_channel', order='sequence')
    track = tuple(line((0, 0), (16, 0)))
    state = {c.name: c.ok for c in core.judge(spec, COURSE, Window(0, 10, track), final=False).checks}
    assert state['gates in order'] is None
    state = {c.name: c.ok for c in core.judge(spec, COURSE, Window(0, 10, track)).checks}
    assert state['gates in order'] is True

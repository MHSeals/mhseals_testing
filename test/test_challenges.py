"""Challenge judges: every assertion of test_referee.py has a twin here, plus the owner's record."""
import math

import pytest

from mhseals_testing.referee import core, course, pack
from mhseals_testing.referee.core import Window
from referee_helpers import TRUTH, element, line

COURSE = course.from_truth(TRUTH)
YEAR = pack.load('2025')
COMMISSIONING = pack.load('commissioning')


def judge(name, track, events=(), truth_course=COURSE, final=True, banner=None, source='truth'):
    spec = (YEAR.challenges.get(name) or COMMISSIONING.challenges[name])
    if banner:
        truth_course = core.Course(truth_course.name, truth_course.elements, truth_course.source,
                                   dict(docking=dict(banner=banner)))
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


def test_speed_challenge_out_around_and_back_with_the_right_count():
    out = line((0, 11), (16.5, 11))
    around = [(out[-1][0] + 0.2 * (k + 1), 19 - 2.5 * math.cos(a), 11 + 2.5 * math.sin(a), a)
              for k, a in enumerate(i * 2 * math.pi / 60 for i in range(61))]
    back = line((16.5, 11), (0, 11), t0=around[-1][0] + 0.2)
    track = out + around + back
    result = judge('speed_challenge', track, [dict(event='spill_count', count=2)])
    assert result.verdict == 'PASS', result
    assert result.details['around_mark_deg'] >= 300 and result.details['gate_to_gate_s'] > 0
    assert result.splits['gate_back'] - result.splits['gate_out'] == result.details['gate_to_gate_s']
    wrong = judge('speed_challenge', track, [dict(event='spill_count', count=3)])
    assert checks(wrong)['spill count'] is False
    assert wrong.details['spills'] == dict(true=2, reported=3)


def test_docking_needs_the_empty_bay_of_the_banner():
    def hold(x, seconds=8):
        return [(t * 0.5, x, -18.7, -math.pi / 2) for t in range(int(seconds / 0.5))]

    good = judge('docking', hold(16), banner='blue_square')
    assert good.verdict == 'PASS' and good.details['held_s'] >= 5
    occupied = judge('docking', hold(12), banner='blue_square')
    assert occupied.verdict == 'FAIL'
    assert not checks(occupied)['never in a wrong bay']


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


def test_yellow_circle_requires_a_full_lap_and_return_to_origin():
    truth = course.from_truth(dict(schema=course.TRUTH_SCHEMA, elements=[element('circle_test', 'yellow_buoy', 10, 0)]))
    approach = line((0, 0), (7, 0))
    lap = [(11 + k * 0.2, 10 - 3 * math.cos(a), 3 * math.sin(a), a)
           for k, a in enumerate(i * 2 * math.pi / 60 for i in range(61))]
    back = line((7, 0), (0, 0), t0=24)
    assert judge('circle_test', approach + lap + back, truth_course=truth).verdict == 'PASS'
    assert judge('circle_test', approach + back, truth_course=truth).verdict == 'FAIL'
    assert judge('circle_test', approach + lap, truth_course=truth).verdict == 'FAIL'


@pytest.mark.parametrize('task', ['pair_test', 'circle_test'])
def test_commissioning_needs_matching_course_truth(task):
    assert judge(task, [], ).verdict == 'N/A'


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
    spec = YEAR.challenges['nav_channel']
    spec = pack.Spec(**{**spec.__dict__, 'params': {**spec.params, 'order': 'sequence'}})
    nearest_first = Window(0, 10, tuple(line((11, 0), (15, 0)) + line((15, 0), (5, 0), t0=11)))
    assert core.judge(spec, COURSE, nearest_first).verdict == 'PASS'
    other_way = Window(0, 10, tuple(line((11, 0), (5, 0)) + line((5, 0), (16, 0), t0=11)))
    result = core.judge(spec, COURSE, other_way)
    assert result.verdict == 'FAIL' and 'gates in order' in result.failures


def test_docking_defaults_to_the_banner_in_the_pack():
    track = [(t * 0.5, 16, -18.7, -math.pi / 2) for t in range(16)]
    assert judge('docking', track).verdict == 'PASS'  # no day injected: from_truth has none
    assert judge('docking', track, banner='red_circle').verdict == 'FAIL'  # the day's banner wins


def test_gates_in_order_is_undecided_while_open_and_holds_once_final():
    spec = pack.Spec(**{**YEAR.challenges['nav_channel'].__dict__,
                        'params': {**YEAR.challenges['nav_channel'].params, 'order': 'sequence'}})
    track = tuple(line((0, 0), (16, 0)))
    state = {c.name: c.ok for c in core.judge(spec, COURSE, Window(0, 10, track), final=False).checks}
    assert state['gates in order'] is None
    state = {c.name: c.ok for c in core.judge(spec, COURSE, Window(0, 10, track)).checks}
    assert state['gates in order'] is True

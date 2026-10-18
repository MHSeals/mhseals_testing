"""The referee judges what the boat did against the course truth, not what it reported."""
import math

import pytest

from mhseals_testing.analysis import referee


def element(task, label, x, y, radius=0.2):
    return dict(task=task, label=label, template='', x=x, y=y, radius=radius)


TRUTH = dict(elements=[
    element('nav_channel', 'red_pole_buoy', 7, 1.5), element('nav_channel', 'green_pole_buoy', 7, -1.5),
    element('nav_channel', 'red_pole_buoy', 13, 1.5),
    element('nav_channel', 'green_pole_buoy', 13, -1.5),
    element('speed_challenge', 'red_buoy', 3, 12.25), element('speed_challenge', 'green_buoy', 3, 9.75),
    element('speed_challenge', 'black_buoy', 10, 12.6),
    element('speed_challenge', 'black_buoy', 14, 9.4),
    element('speed_challenge', 'blue_buoy', 19, 11),
], bays=[
    dict(dock=0, index=0, x=12, y=-19, yaw=math.pi / 2, width=1.8, depth=1.8,
         banner='blue_square', occupied=True),
    dict(dock=0, index=2, x=16, y=-19, yaw=math.pi / 2, width=1.8, depth=1.8,
         banner='blue_square', occupied=False),
])


def line(a, b, t0=0.0, steps=50, dt=0.2):
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    return [(t0 + i * dt, a[0] + (b[0] - a[0]) * i / steps, a[1] + (b[1] - a[1]) * i / steps, yaw)
            for i in range(steps + 1)]


def test_channel_passes_only_when_both_gates_are_crossed():
    through = line((0, 0), (16, 0))
    assert referee.judge('nav_channel', TRUTH, through)['verdict'] == 'PASS'
    short = referee.judge('nav_channel', TRUTH, line((0, 0), (10, 0)))
    assert short['verdict'] == 'FAIL' and short['gates'] == '1/2'


def test_touching_a_buoy_fails_even_through_the_gates():
    scrape = line((0, 1.0), (16, 1.0))  # the hull's port side reaches the red buoys
    result = referee.judge('nav_channel', TRUTH, scrape)
    assert result['checks']['every gate crossed'] and result['contacts']
    assert result['verdict'] == 'FAIL'


def test_speed_challenge_out_around_and_back_with_the_right_count():
    out = line((0, 11), (16.5, 11))
    around = [(out[-1][0] + 0.2 * (k + 1), 19 - 2.5 * math.cos(a), 11 + 2.5 * math.sin(a), a)
              for k, a in enumerate(i * 2 * math.pi / 60 for i in range(61))]
    back = line((16.5, 11), (0, 11), t0=around[-1][0] + 0.2)
    events = [dict(event='spill_count', count=2)]
    result = referee.judge('speed_challenge', TRUTH, out + around + back, events)
    assert result['verdict'] == 'PASS', result
    assert result['around_mark_deg'] >= 300 and result['gate_to_gate_s'] > 0
    wrong = referee.judge('speed_challenge', TRUTH, out + around + back,
                          [dict(event='spill_count', count=3)])
    assert wrong['checks']['spill count'] is False


def test_docking_needs_the_empty_bay_of_the_banner():
    def hold(x, seconds=8):
        return [(t * 0.5, x, -18.7, -math.pi / 2) for t in range(int(seconds / 0.5))]

    good = referee.judge('docking', TRUTH, hold(16), banner='blue_square')
    assert good['verdict'] == 'PASS' and good['held_s'] >= 5
    occupied = referee.judge('docking', TRUTH, hold(12), banner='blue_square')
    assert occupied['verdict'] == 'FAIL'
    assert not occupied['checks']['never in a wrong bay']


def test_mission_judges_each_stage_on_its_own_part_of_the_track():
    track = line((0, 0), (16, 0))
    events = [dict(event='stage_started', stage='nav_channel', time=0.0),
              dict(event='stage_completed', stage='nav_channel', time=10.0)]
    result = referee.judge_mission(TRUTH, track, events, ['nav_channel', 'docking'],
                                   banner='blue_square')
    assert result['stages']['nav_channel']['verdict'] == 'PASS'
    assert result['stages']['docking']['checks'] == {'attempted': False}
    assert result['verdict'] == 'FAIL'


def test_optional_skipped_stages_are_not_mission_failures():
    events = [dict(event='stage_started', stage='nav_channel', time=0.),
              dict(event='stage_completed', stage='nav_channel', time=10.),
              dict(event='stage_started', stage='docking', time=10.),
              dict(event='stage_skipped', stage='docking', time=12.)]
    result = referee.judge_mission(TRUTH, line((0, 0), (16, 0)), events,
                                   ['nav_channel', 'docking'], required_stages=['nav_channel'])
    assert result['verdict'] == 'PASS'
    assert result['stages']['docking']['verdict'] == 'SKIPPED'
    required = referee.judge_mission(TRUTH, [], events, ['docking'], required_stages=['docking'])
    assert required['verdict'] == 'FAIL'


@pytest.mark.parametrize('swapped', [False, True])
def test_pair_commissioning_requires_both_crossings_of_the_same_pair(swapped):
    sign = -1 if swapped else 1
    truth = dict(elements=[
        element('pair_test', 'red_pole_buoy', 7, sign * 1.5),
        element('pair_test', 'green_pole_buoy', 7, -sign * 1.5),
    ])
    out = line((0, 0), (10, 0))
    back = line((10, 0), (5.5, 0), t0=11)
    assert referee.judge('pair_test', truth, out + back)['verdict'] == 'PASS'
    assert referee.judge('pair_test', truth, out)['verdict'] == 'FAIL'


def test_yellow_circle_requires_a_full_lap_and_return_to_origin():
    truth = dict(elements=[element('circle_test', 'yellow_buoy', 10, 0)])
    approach = line((0, 0), (7, 0))
    lap = [(11 + k * 0.2, 10 - 3 * math.cos(a), 3 * math.sin(a), a)
           for k, a in enumerate(i * 2 * math.pi / 60 for i in range(61))]
    back = line((7, 0), (0, 0), t0=24)
    good = referee.judge('circle_test', truth, approach + lap + back)
    assert good['verdict'] == 'PASS'
    shortcut = referee.judge('circle_test', truth, approach + back)
    assert shortcut['verdict'] == 'FAIL'
    no_return = referee.judge('circle_test', truth, approach + lap)
    assert no_return['verdict'] == 'FAIL'


@pytest.mark.parametrize('task', ['pair_test', 'circle_test'])
def test_commissioning_referee_needs_matching_course_truth(task):
    assert referee.judge(task, TRUTH, [])['verdict'] == 'N/A'

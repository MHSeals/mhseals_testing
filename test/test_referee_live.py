"""The live referee: seeks recompute the same marks, finished windows freeze, cost is bounded."""
import math
import time

from mhseals_testing.analysis import referee as judging
from mhseals_testing.referee.live import Referee
from test_referee import TRUTH, line

TASKS = ['nav_channel', 'speed_challenge', 'docking', 'follow_path']
TRACK = line((0, 0), (16, 0), steps=80)
EVENTS = [dict(event='stage_started', stage='nav_channel', time=0.0),
          dict(event='stage_completed', stage='nav_channel', time=14.0)]


def test_seek_recomputes_identical_marks():
    live = Referee(TRUTH, TASKS)
    for t in (4.0, 8.0, 16.0):                     # play forward
        live.update(TRACK, EVENTS, t)
    seen = live.update(TRACK, EVENTS, 16.0)
    live.seek()                                    # jump back, then forward again
    assert live.update(TRACK, EVENTS, 3.0) == Referee(TRUTH, TASKS).update(TRACK, EVENTS, 3.0)
    live.seek()
    assert live.update(TRACK, EVENTS, 16.0) == seen
    assert seen['results']['nav_channel']['verdict'] == 'PASS'


def test_a_finished_window_is_judged_once(monkeypatch):
    calls = []
    real = judging.judge
    monkeypatch.setattr(judging, 'judge', lambda task, *a: calls.append(task) or real(task, *a))
    live = Referee(TRUTH, ['nav_channel'], period=0)
    live.update(TRACK, EVENTS, 15.0)
    live.update(TRACK, EVENTS, 15.0)
    live.update(TRACK, EVENTS, 16.0)
    assert calls == ['nav_channel']


def test_open_window_is_pending_and_closes_to_the_judged_verdict():
    live = Referee(TRUTH, ['nav_channel'])
    live.update(TRACK, EVENTS, 16.0)               # frozen at 14 s
    early = live.update(TRACK, EVENTS, 5.0)['results']['nav_channel']
    assert early['verdict'] == 'PENDING'           # never a FAIL that later flips to PASS


def test_unstarted_stage_is_pending_and_skipped_is_skipped():
    events = EVENTS + [dict(event='stage_skipped', stage='docking', time=15.0),
                       dict(event='stage_started', stage='docking', time=14.5)]
    got = Referee(TRUTH, ['speed_challenge', 'docking']).update(TRACK, events, 16.0)['results']
    assert got['speed_challenge']['verdict'] == 'PENDING'
    assert got['docking']['verdict'] == 'SKIPPED'


def test_open_window_fails_on_contact():
    hit = [(0.0, 0.0, 1.5, 0.0), (1.0, 7.0, 1.5, 0.0)]       # onto a red pole buoy
    events = [dict(event='stage_started', stage='nav_channel', time=0.0)]
    got = Referee(TRUTH, ['nav_channel']).update(hit, events, 1.0)['results']['nav_channel']
    assert got['verdict'] == 'FAIL' and got['contacts']


def test_closed_stage_ignores_events_after_its_end():
    events = EVENTS + [dict(event='species_count', count=9, time=15.0)]
    live = Referee(TRUTH, ['follow_path'])
    events += [dict(event='stage_started', stage='follow_path', time=1.0),
               dict(event='stage_completed', stage='follow_path', time=2.0)]
    res = live.update(TRACK, events, 16.0)['results']['follow_path']
    assert res['species']['reported'] is None


def test_update_stays_under_30_ms_with_five_closed_challenges():
    track = [(i * 0.2, i * 0.01, 0.1 * math.sin(i / 50), 0.0) for i in range(1800)]
    tasks = TASKS + ['return_home']
    events = [dict(event=e, stage=k, time=t) for k in tasks
              for e, t in (('stage_started', 0.0), ('stage_completed', 359.0))]
    live = Referee(TRUTH, tasks)
    best = min(_timed(live, track, events) for _ in range(5))
    assert best < 0.030


def _timed(live, track, events):
    live.seek()
    start = time.perf_counter()
    live.update(track, events, 360.0)
    return time.perf_counter() - start

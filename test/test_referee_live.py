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


def test_playhead_before_the_window_closes_sees_it_open():
    live = Referee(TRUTH, ['nav_channel'])
    live.update(TRACK, EVENTS, 16.0)               # frozen at 14 s
    early = live.update(TRACK, EVENTS, 5.0)['results']['nav_channel']
    assert early['verdict'] == 'FAIL' and early['gates'] == '0/2'


def test_update_stays_under_30_ms_on_a_six_minute_track():
    track = [(i * 0.2, i * 0.01, 0.1 * math.sin(i / 50), 0.0) for i in range(1800)]
    live = Referee(TRUTH, TASKS)
    start = time.perf_counter()
    live.update(track, [], 360.0)
    assert time.perf_counter() - start < 0.030

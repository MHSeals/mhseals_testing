"""The referee run live and in replay (referee.md section 4.3).

``Referee.update`` judges each challenge over its own window of the track, cut at
``t`` (the live edge or the replay playhead). Finished windows are judged once and
frozen; open ones are re-judged at most every ``period`` seconds of track time, and
only when the window grew. ``seek`` drops the memo, so a seek recomputes exactly what
a fresh referee would show at that ``t``. Nothing here reads a clock or ROS.
"""

import bisect
import math

from mhseals_testing.analysis import referee as judging


def _window(events, task, t):
    """(start, end|None) of ``task``'s stage as of ``t``; (None, None) with no stage events."""
    start = end = None
    for e in events:
        if e.get('stage') != task or e.get('time', math.inf) > t:
            continue
        if e.get('event') == 'stage_started' and start is None:
            start = e['time']
        elif e.get('event') in ('stage_completed', 'stage_skipped') and end is None:
            end = e['time']
    return start, end


class Referee:
    """Judge ``tasks`` against ``truth``; ``update(track, events, t)`` returns the view."""

    def __init__(self, truth, tasks, banner=None, period=0.5):
        self.truth, self.tasks, self.banner, self.period = truth, tuple(tasks), banner, period
        self.seek()

    def seek(self):
        """Forget everything judged so far (replay jumped, or the course changed)."""
        self._frozen, self._open, self._view = {}, {}, None

    def update(self, track, events, t):
        """Results per task and the strip text, for the track and events up to ``t``.

        Rules follow ``judge_mission``: a stage with no start is PENDING, a skipped one
        SKIPPED (FAIL on contact), a finished one judged in full and frozen. A window still
        open is PENDING, or FAIL once it touched something, so a verdict never flips back.
        """
        track = track[:bisect.bisect_right(track, t, key=lambda p: p[0])]
        events = [e for e in events if e.get('time', 0) <= t]
        staged = any(e.get('event') == 'stage_started' for e in events)
        results = {}
        for task in self.tasks:
            start, end = _window(events, task, t) if staged else (0.0, None)
            if start is None:
                results[task] = dict(task=task, verdict='PENDING', checks={})
                continue
            lo = bisect.bisect_left(track, start, key=lambda p: p[0])
            part = track[lo:]
            if end is not None:
                part = part[:bisect.bisect_right(part, end, key=lambda p: p[0])]
            key = (start, end, len(part))
            if end is not None and task in self._frozen:
                results[task] = self._frozen[task]
                continue
            memo = self._open.get(task)
            if memo and (memo[0] == key or (end is None and 0 <= t - memo[1] < self.period)):
                results[task] = memo[2]
                continue
            results[task] = self._judge(task, part, events, start, end)
            self._open[task] = (key, t, results[task])
            if end is not None:
                self._frozen[task] = results[task]
        passed = sum(r['verdict'] == 'PASS' for r in results.values())
        self._view = dict(results=results, strip=f'referee {passed}/{len(results)}')
        return self._view

    def _judge(self, task, part, events, start, end):
        """One window's result; events are cut to the window so a closed stage is bounded."""
        stage = [e for e in events if start <= e.get('time', 0) <= (end if end is not None
                                                                      else math.inf)]
        skipped = any(e.get('event') == 'stage_skipped' and e.get('stage') == task for e in stage)
        if end is not None and not skipped:
            return judging.judge(task, self.truth, part, stage, self.banner)
        touched = judging.contacts(
            part, [e for e in judging.elements(self.truth, task) if e.get('label')])
        verdict = 'FAIL' if touched else 'SKIPPED' if skipped else 'PENDING'
        return dict(task=task, verdict=verdict, checks={'no contact': not touched},
                    contacts=touched)

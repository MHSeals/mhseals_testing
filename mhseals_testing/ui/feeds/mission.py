"""Mission feed: the task runner's current task, its decisions and the task hypotheses; pure, no ROS.

``ingest(kind, message)`` takes one parsed JSON message: ``status`` (/task/status,
``mhseals.mission_status/1``; the older task/status/elapsed/tip keys alone also do), ``event``
(/task/events; only ``decision`` events are kept) or ``hypotheses`` (/task/hypotheses). The ROS
side (ros/mission.py ``MissionFeed``) only parses. Hypotheses and the goal go on into the
``MapState`` as its ``tasks`` and ``mission_goal`` kinds. Callbacks run on the ROS thread, so
every field is replaced, never mutated, and the UI reads a consistent value.
"""

import math
import time

from mhseals_testing.ui.minimap import tasks

FRESH = 5.0     # s: the Drive strip shows only a status younger than this
KEEP = 50       # decision events kept for the log


def goal_fields(goal):
    """MapState ``mission_goal`` fields of a status goal; x None clears the goal."""
    try:
        x, y = float(goal['x']), float(goal['y'])
    except (KeyError, TypeError, ValueError):
        return {'x': None}
    if not (math.isfinite(x) and math.isfinite(y)):
        return {'x': None}
    return {'x': x, 'y': y, 'yaw': float(goal.get('yaw') or 0.0), 'why': str(goal.get('why') or '')}


class MissionState:
    """The latest status and the decision log; a ``ui.feeds`` feed whose view is ``{'mission': self}``."""

    def __init__(self, map_state=None, clock=time.monotonic, wall=time.time):
        self.map_state, self.clock, self.wall = map_state, clock, wall
        self.status, self.received = {}, None
        self.decisions = ()      # (wall s, event), oldest first

    def ingest(self, kind, message):
        if not isinstance(message, dict):
            return
        if kind == 'status':
            self.status, self.received = message, self.clock()
            if self.map_state is not None:
                self.map_state.ingest('mission_goal', **goal_fields(message.get('goal')))
        elif kind == 'event':
            if message.get('event') == 'decision':
                self.decisions = (self.decisions + ((self.wall(), message),))[-KEEP:]
        elif kind == 'hypotheses':
            if self.map_state is not None:
                self.map_state.ingest('tasks', items=tasks.parse(message))
        else:
            raise ValueError(f'unknown mission kind {kind!r}')

    def age(self):
        """Seconds since the last status, inf before the first."""
        return math.inf if self.received is None else self.clock() - self.received

    def fresh(self):
        return bool(self.status) and self.age() < FRESH

    def recent(self, count=5):
        """The last ``count`` decisions as ``(wall s or None, event)``; before any decision event
        arrives, the status's own ``recent`` list (untimed) stands in, so a late start still shows why."""
        if self.decisions:
            return list(self.decisions[-count:])
        recent = self.status.get('recent') or []
        return [(None, e) for e in recent[-count:] if isinstance(e, dict)]

    def ladder(self):
        """``(name, state, attempts, failures)`` per stage, in the publisher's order."""
        stages = self.status.get('stages') or {}
        return [(name, s.get('state', 'pending'), int(s.get('attempts') or 0), int(s.get('failures') or 0))
                for name, s in stages.items() if isinstance(s, dict)]

    def tick(self, app, snapshot, now):
        """Fed by ROS callbacks, not the tick."""

    def view(self):
        return {'mission': self}

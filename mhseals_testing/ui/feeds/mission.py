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
    if not tasks.placed(x, y):
        return {'x': None}
    yaw = goal.get('yaw')
    return {'x': x, 'y': y, 'yaw': float(yaw) if _finite(yaw) else 0.0, 'why': str(goal.get('why') or '')}


SCHEMA = 'mhseals.mission_status/1'


def _count(value):
    """A non-negative int of a count field; anything else (a word, NaN, a list) is 0."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0
    return int(number) if math.isfinite(number) and 0 <= number < 1e9 else 0


def normalise(message):
    """A status with every field the panels read in its expected type: ``stage`` str, ``stages``
    ``{str: {state, attempts, failures}}``, ``recent`` a list of dicts. A malformed field becomes
    its empty value, so the render never meets it."""
    status = dict(message)
    stage = status.get('stage')
    status['stage'] = stage if isinstance(stage, str) else None
    stages = status.get('stages') if isinstance(status.get('stages'), dict) else {}
    status['stages'] = {str(name): {'state': s['state'] if isinstance(s.get('state'), str) else 'pending',
                                    'attempts': _count(s.get('attempts')), 'failures': _count(s.get('failures')),
                                    **({'elapsed': float(s['elapsed'])} if _finite(s.get('elapsed')) else {})}
                        for name, s in stages.items() if isinstance(s, dict)}
    recent = status.get('recent')
    status['recent'] = [e for e in recent if isinstance(e, dict)] if isinstance(recent, list) else []
    for key in ('phase', 'behaviour', 'task', 'tip'):
        if not isinstance(status.get(key), (str, type(None))):
            status[key] = str(status[key])
    if not _finite(status.get('elapsed')):
        status.pop('elapsed', None)
    if not isinstance(status.get('goal'), dict):
        status['goal'] = {}
    return status


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class MissionState:
    """The latest status and the decision log; a ``ui.feeds`` feed whose view is ``{'mission': self}``."""

    def __init__(self, map_state=None, clock=time.monotonic, wall=time.time):
        self.map_state, self.clock, self.wall = map_state, clock, wall
        self.status, self.received = {}, None
        self.decisions = ()      # (wall s, event), oldest first
        self.notice = None       # one line on a status of another schema version, which is ignored

    def ingest(self, kind, message):
        if not isinstance(message, dict):
            return
        if kind == 'status':
            schema = message.get('schema', SCHEMA)
            if schema != SCHEMA:
                self.notice = f'ignoring /task/status {str(schema)[:40]} (reads {SCHEMA})'
                return
            self.status, self.received = normalise(message), self.clock()
            if self.map_state is not None:
                self.map_state.ingest('mission_goal', **goal_fields(self.status['goal']))
        elif kind == 'event':
            if message.get('event') == 'decision':
                stamp = message.get('time')   # the runner's own stamp, else when it arrived
                stamp = float(stamp) if _finite(stamp) and 0 < stamp < 1e10 else self.wall()
                self.decisions = (self.decisions + ((stamp, message),))[-KEEP:]
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

"""Fix-progression timeline from ``gps_fix`` events (gps.md section 6); no ROS imports.

Each event is a transition: ``phase == 'gps_fix'`` with ``fix`` (a name) or ``fix_type``
(the GPS_RAW_INT number) and a ``t`` in seconds, else the ISO ``time``. Time is always
passed in (``now``), so live, replay and tests share one fold.
"""

from datetime import datetime

STATES = ('none', '3D', 'DGPS', 'FLOAT', 'FIXED')  # the strip's order, worst to best
_BY_TYPE = {0: 'none', 1: 'none', 2: '3D', 3: '3D', 4: 'DGPS', 5: 'FLOAT', 6: 'FIXED'}
_ALIASES = {'2d': '3D', '3d': '3D', 'dgps': 'DGPS', 'float': 'FLOAT', 'fixed': 'FIXED'}


def state_of(event):
    """The strip state an event names, or None when it names none."""
    fix = event.get('fix')
    if isinstance(fix, str):
        return 'none' if fix.lower() in ('none', 'no fix') else _ALIASES.get(fix.lower())
    return _BY_TYPE.get(event.get('fix_type', fix))


def stamp(event):
    """Seconds of an event: ``t``, else the wall ``time``; None when neither parses."""
    if isinstance(event.get('t'), (int, float)):
        return float(event['t'])
    try:
        return datetime.fromisoformat(event['time']).timestamp()
    except (KeyError, TypeError, ValueError):
        return None


class Fixlog:
    """Seconds spent in and entries to each state, plus time to the first FIXED."""

    def __init__(self, events, now, start=None):
        steps = []
        for event in events:
            at, state = stamp(event), state_of(event)
            if event.get('phase') == 'gps_fix' and at is not None and state:
                steps.append((at, state))
        steps.sort(key=lambda step: step[0])
        steps = [s for i, s in enumerate(steps) if i == 0 or s[1] != steps[i - 1][1]]
        self.steps = steps
        self.start = start if start is not None else (steps[0][0] if steps else now)
        self.now = max(now, steps[-1][0] if steps else now)
        self.current = steps[-1][1] if steps else None
        self.seconds = dict.fromkeys(STATES, 0.0)
        self.entries = dict.fromkeys(STATES, 0)
        for i, (at, state) in enumerate(steps):
            end = steps[i + 1][0] if i + 1 < len(steps) else self.now
            self.seconds[state] += max(0.0, end - max(at, self.start))
            self.entries[state] += 1
        fixed = [at for at, state in steps if state == 'FIXED']
        self.first_fixed = fixed[0] - self.start if fixed else None

    def bins(self, width):
        """One state (or None before the first event) per column across start..now."""
        span = max(self.now - self.start, 1e-9)
        out = []
        for col in range(width):
            mid = self.start + span * (col + .5) / width
            state = None
            for at, name in self.steps:
                if at <= mid:
                    state = name
            out.append(state)
        return out

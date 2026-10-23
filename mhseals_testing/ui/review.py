"""Review of a past session: the playhead, seeks with a debounce, and the session's timeline.

Pure state, no ROS and no Rich. A reader callback (`load`) fills the dashboard for the
playhead time; it runs once, DEBOUNCE seconds after the last seek key, so holding a seek
key moves only the scrubber and the window loads when the key rests (bags.md 4.1).
"""

from datetime import datetime
from pathlib import Path

from mhseals_testing import data

SMALL, BIG = 5.0, 30.0                     # s per arrow / per { }
RATES = (0.25, 0.5, 1.0, 2.0, 4.0, 8.0)
DEBOUNCE = 0.15                            # s after the last seek key


def event_time(event):
    """Wall seconds of a journal line: t_ns when present, else its ISO time; None if neither."""
    if 't_ns' in event:
        return event['t_ns'] / 1e9
    try:
        return datetime.fromisoformat(event['time']).timestamp()
    except (KeyError, TypeError, ValueError):
        return None


def timeline(events):
    """Markers, procedure windows, recording gaps and health failures from journal events.

    Windows are `<proc>_start` .. `<proc>_stop` pairs (outcome from the stop line);
    gaps are `rec_stop` .. the next `rec_start`. Returns (t0, t1, markers, windows, gaps, fails),
    where markers are (t, note, kind), windows (a, b, outcome, name), gaps (a, b), fails t.
    """
    markers, windows, gaps, fails, open_ = [], [], [], [], {}
    times, gap = [], None
    for event in sorted((e for e in events if event_time(e) is not None), key=event_time):
        t, phase = event_time(event), event.get('phase', '')
        times.append(t)
        if phase == 'marker':
            markers.append((t, event.get('note', ''), event.get('kind', 'user')))
        elif phase == 'rec_stop':
            gap = t
        elif phase == 'rec_start' and gap is not None:
            gaps, gap = gaps + [(gap, t)], None
        elif phase == 'health' and event.get('state') == 'fail':
            fails.append(t)
        elif phase == 'session_start':
            pass                                      # the whole session, not a procedure
        elif phase.endswith('_start'):
            open_[phase[:-6]] = t
        elif phase.endswith('_stop') and phase[:-5] in open_:
            name = phase[:-5]
            windows.append((open_.pop(name), t, str(event.get('outcome', '')).upper(), name))
    for name, a in open_.items():                     # still running when the session ended
        windows.append((a, max(times), '', name))
    return (min(times, default=0.0), max(times, default=0.0), markers, windows, gaps, fails)


def load_session(path):
    """A Review for a session folder, or its flat `<name>.report.json` events."""
    path = Path(path)
    events = data.read_jsonl(path / 'events.jsonl') if (path / 'events.jsonl').exists() else []
    if not events:
        report = data.read_json(path.with_suffix('.report.json'), {})
        events = [dict(phase='event', time=e.get('time'), note=e.get('message', ''))
                  for e in report.get('events', [])]
    t0, t1, markers, windows, gaps, fails = timeline(events)
    if t1 <= t0:
        raise ValueError(f'{path}: no timed events to review')
    host = data.read_json(path / 'session.json', {}).get('host') or next(
        (e['host'] for e in events if e.get('host')), '')
    return Review(t0, t1, markers, windows, gaps, fails, title=path.name, host=host)


def find_session(name, root=None):
    """Session folder for --review: a path, a name under the data root, or the newest one."""
    root = Path(root or data.data_root())
    if name:
        for candidate in (Path(name), root / name):
            if candidate.is_dir():
                return candidate
        raise ValueError(f'no session {name!r} under {root}')
    found = sorted((p for p in root.glob('*') if p.is_dir() and p.name != 'logs'),
                   key=lambda p: p.stat().st_mtime)
    if not found:
        raise ValueError(f'no sessions under {root}')
    return found[-1]


class Review:
    """The playhead over [t0, t1] with play, rate, seeks and mark jumps."""

    def __init__(self, t0, t1, markers=(), windows=(), gaps=(), fails=(), title='', host='',
                 load=None):
        self.t0, self.t1 = t0, t1
        self.markers, self.windows, self.gaps, self.fails = (
            sorted(markers), list(windows), list(gaps), list(fails))
        self.title, self.host, self.load = title, host, load
        self.playhead, self.rate, self.playing = t0, 1.0, False
        self.due = None                  # when the pending seek loads

    @property
    def length(self):
        return self.t1 - self.t0

    def go(self, t, now):
        """Move to `t` (clamped); the window loads DEBOUNCE s after the last move."""
        self.playhead = min(max(t, self.t0), self.t1)
        self.due = now + DEBOUNCE

    def seek(self, seconds, now):
        self.go(self.playhead + seconds, now)

    def jump(self, step, now):
        """To the previous (-1) or next (+1) marker; stays put when there is none."""
        eps = 1e-6
        ahead = [m[0] for m in self.markers if (m[0] > self.playhead + eps if step > 0
                                                else m[0] < self.playhead - eps)]
        if ahead:
            self.go(min(ahead) if step > 0 else max(ahead), now)

    def toggle(self):
        self.playing = not self.playing

    def pause(self):
        self.playing = False

    def faster(self, step=1):
        """Next rate up (+1) or down (-1) the RATES ladder."""
        index = min(max(RATES.index(self.rate) + step, 0), len(RATES) - 1)
        self.rate = RATES[index]

    def tick(self, now, dt):
        """Advance by rate*dt while playing (no reseek) and run a due debounced load."""
        if self.playing:
            self.playhead = min(self.playhead + self.rate * dt, self.t1)
            if self.playhead >= self.t1:
                self.playing = False
        if self.due is not None and now >= self.due:
            self.due = None
            if self.load:
                self.load(self.playhead)

    def window_at(self):
        """(name, outcome) of the procedure under the playhead, or None."""
        for a, b, outcome, name in self.windows:
            if a <= self.playhead <= b:
                return name, outcome
        return None

    def nearest(self):
        """(index, total, note) of the marker nearest the playhead, or None."""
        if not self.markers:
            return None
        i = min(range(len(self.markers)), key=lambda k: abs(self.markers[k][0] - self.playhead))
        return i + 1, len(self.markers), self.markers[i][1]


class Offline:
    """Stands in for the ROS node and the process manager in --review: nothing runs, nothing publishes."""
    fault = ''
    processes = {}

    def emit(self, *args, **details):
        pass

    def running(self, name):
        return False

    def stop(self, *args, **kwargs):
        pass

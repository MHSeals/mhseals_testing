"""Review of a past session: the playhead, seeks with a debounce, and the session's timeline.

Pure state, no ROS and no Rich. A reader callback (`load`) fills the dashboard for the
playhead time; it runs once, DEBOUNCE seconds after the last seek key, so holding a seek
key moves only the scrubber and the window loads when the key rests (bags.md 4.1).
"""

from datetime import datetime
import math
from pathlib import Path

from mhseals_testing import data
from mhseals_testing.harness.bags import RESERVED

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
    times, gap, notes = [], None, {}
    for event in sorted((e for e in events if event_time(e) is not None), key=event_time):
        t, phase = event_time(event), event.get('phase', '')
        times.append(t)
        if phase == 'marker':
            markers.append((t, event.get('note', ''), event.get('kind', 'user'), event.get('id')))
        elif phase == 'marker_note':
            notes[event.get('id')] = event.get('note', '')      # latest edit wins
        elif phase == 'rec_stop':
            gap = t
        elif phase == 'rec_start' and gap is not None:
            gaps, gap = gaps + [(gap, t)], None
        elif phase in ('rec_start', 'rec_stop'):
            pass                                      # recording edges, never a procedure window
        elif phase == 'health' and event.get('state') == 'fail':
            fails.append(t)
        elif phase == 'session_start':
            pass                                      # the whole session, not a procedure
        elif phase.endswith('_start'):
            open_[phase[:-6]] = t
        elif phase.endswith('_stop') and phase[:-5] in open_:
            name = phase[:-5]
            windows.append((open_.pop(name), t, str(event.get('outcome', '')).upper(), name))
    markers = [(t, notes.get(i, note), kind) for t, note, kind, i in markers]
    for name, a in open_.items():                     # still running when the session ended
        windows.append((a, max(times), '', name))
    return (min(times, default=0.0), max(times, default=0.0), markers, windows, gaps, fails)


def review_metadata(path):
    """Validate recorded display metadata before any live view can be replaced."""
    header = data.read_json(path, {})
    if not isinstance(header, dict):
        raise ValueError(f'{path}: session metadata must be a mapping')
    for name in ('host', 'title'):
        if name in header and not isinstance(header[name], str):
            raise ValueError(f'{path}: metadata {name} must be text')
    profile = header.get('profile')
    if profile is not None and not isinstance(profile, (str, dict)):
        raise ValueError(f'{path}: invalid profile metadata')
    if isinstance(profile, dict) and not isinstance(profile.get('name', ''), str):
        raise ValueError(f'{path}: invalid profile name')
    return header


def validate_events(events, path):
    """Reject malformed timeline records at the selected-session input boundary."""
    if not isinstance(events, list):
        raise ValueError(f'{path}: timeline events must be a list')
    for event in events:
        if not isinstance(event, dict) or not isinstance(event.get('phase', ''), str):
            raise ValueError(f'{path}: timeline event and text phase required')
        if event.get('phase') in ('marker', 'marker_note') and not (
                event.get('id') is None or type(event['id']) in (str, int)):
            raise ValueError(f'{path}: marker id must be text or an integer')
        if 't_ns' in event:
            stamp = event['t_ns']
            try:
                valid = type(stamp) in (int, float) and math.isfinite(stamp)
            except OverflowError:
                valid = False
            if not valid:
                raise ValueError(f'{path}: invalid timeline timestamp')


def load_session(path):
    """A Review for a session: its journal (`<name>.events.jsonl` beside the folder, or inside it),
    else the events of `<name>.report.json`."""
    path = Path(path)
    events = []
    for journal in (path.with_suffix('.events.jsonl'), path / 'events.jsonl'):
        events = data.read_jsonl(journal) if journal.exists() else []   # flat, then foldered
        if events:
            break
    if not events:      # report events are the journal's own records (phase, id, kind intact)
        report = data.read_json(path.with_suffix('.report.json'), {})
        if not isinstance(report, dict):
            raise ValueError(f'{path}: report must be a mapping')
        events = report.get('events', [])
    validate_events(events, path)
    header = review_metadata(path / 'session.json')
    t0, t1, markers, windows, gaps, fails = timeline(events)
    if t1 <= t0:
        raise ValueError(f'{path}: no timed events to review')
    host = header.get('host') or next(
        (e['host'] for e in events if e.get('host')), '')
    return Review(t0, t1, markers, windows, gaps, fails, title=path.name, host=host)


def _session_dirs(root):
    """Session folders: `sessions/<id>/` plus legacy top-level ones (not the reserved folders)."""
    folders = sorted((root / 'sessions').glob('*')) if (root / 'sessions').is_dir() else []
    folders += [p for p in root.glob('*') if p.name not in RESERVED]
    return [p for p in folders if p.is_dir()]


def _session_time(path):
    """When the session last wrote: its events file, else the folder."""
    events = path / 'events.jsonl'
    return (events if events.exists() else path).stat().st_mtime


def find_session(name, root=None):
    """Session folder for --review: a path, an id under `sessions/` or the root, or the newest."""
    root = Path(root or data.data_root())
    if name:
        for candidate in (Path(name), root / 'sessions' / name, root / name):
            if candidate.is_dir() and (candidate == Path(name) or candidate.name not in RESERVED):
                return candidate
        raise ValueError(f'no session {name!r} under {root}')
    found = _session_dirs(root)
    if not found:
        raise ValueError(f'no sessions under {root}')
    return max(found, key=_session_time)


class Review:
    """The playhead over [t0, t1] with play, rate, seeks and mark jumps."""

    def __init__(self, t0, t1, markers=(), windows=(), gaps=(), fails=(), title='', host='',
                 load=None):
        self.t0, self.t1 = t0, t1
        self.markers, self.windows, self.gaps, self.fails = (
            sorted(markers), list(windows), list(gaps), list(fails))
        self.title, self.host, self.load = title, host, load
        self.playhead, self.rate, self.playing = t0, 1.0, False
        self.loupe = 720.0               # s, the Activity loupe (z cycles it; panels.scrubber.LOUPES)
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

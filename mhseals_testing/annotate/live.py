"""The automatic segment edges, journaled by a feed ticked with the TUI's snapshot.

Each live tick it compares the snapshot with the last one and journals what changed:
    gnss_fix   the rover's fix tier (after it held HOLD seconds, so a flicker is one edge)
    hold_on/off  a mux hold by cause (a link-loss hold folds into a ``link`` segment)
    owner      who drives (a person over automation folds into an ``intervention``)
    steady     a command held within tolerance for ``steady.min`` s while armed: one line when it ends,
               with its start, mean command, mean and spread of the measured velocity, and the fix
Procedures, steps, arming and fence exits are journaled where they happen. Never touches control.
"""

import math
import time

from mhseals_testing.annotate import config

HOLD = 2.0                     # s a new fix tier must hold before it is journaled
FIX_WORDS = {0: 'none', 1: 'none', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'FLOAT', 6: 'FIXED'}
TRUSTED = (None, 'none', 'hold', 'keyboard', 'procedure')   # owners whose command is app.command


def fix_word(app, snapshot):
    rtk = getattr(getattr(app, 'rtk', None), 'current', None)
    if rtk is not None and rtk.fix:
        return rtk.fix
    row = snapshot.get('sensors', {}).get('gpsraw') or {}
    fix = (row.get('values') or {}).get('fix')
    return FIX_WORDS.get(fix) if row.get('status', 'LIVE') == 'LIVE' and fix is not None else None


def owner_of(app):
    from mhseals_testing.ui import feeds
    record = feeds.merged(app.feeds).get('authority_record') if getattr(app, 'arbiter', None) is not None else None
    if record:
        return record.get('owner') or 'none'
    return 'keyboard' if app.manual else 'procedure' if app.running else 'none'


class Steady:
    """One window of a held command: sums for the mean and spread of the measured velocity."""

    def __init__(self, command, t_ns, fix):
        self.command, self.t0_ns, self.fix = tuple(command), t_ns, fix
        self.n, self.sum, self.sq = 0, [0.0] * 3, [0.0] * 3

    def add(self, velocity):
        if velocity is None:
            return
        self.n += 1
        for k, v in enumerate(velocity):
            self.sum[k] += v
            self.sq[k] += v * v

    def record(self, t_ns):
        mean = [s / self.n for s in self.sum] if self.n else None
        spread = [math.sqrt(max(0.0, q / self.n - m * m)) for q, m in zip(self.sq, mean)] if self.n else None
        return dict(t0_ns=self.t0_ns, t_ns=t_ns, command=[round(c, 3) for c in self.command], fix=self.fix,
                    velocity=mean and [round(v, 4) for v in mean], spread=spread and [round(v, 4) for v in spread],
                    samples=self.n)


class AnnotateFeed:
    """Ticked live; journals edges through ``app.emit``. ``wall`` is the journal's clock (ns)."""

    def __init__(self, wall=time.time_ns):
        self.wall = wall
        self.fix, self.fix_seen, self.holds, self.owner, self.steady = None, None, set(), None, None
        self.settings = config.load()['steady']

    def tick(self, app, snapshot, now):
        self._fix(app, fix_word(app, snapshot), now)
        holds = {str(h.get('cause') or h.get('by') or 'hold') for h in (snapshot.get('control') or {}).get('holds', ())}
        for cause in sorted(holds - self.holds):
            app.emit('hold_on', cause=cause)
        for cause in sorted(self.holds - holds):
            app.emit('hold_off', cause=cause)
        self.holds = holds
        owner = owner_of(app)
        if owner != self.owner:
            if self.owner is not None or owner not in (None, 'none'):
                app.emit('owner', owner=owner, prev=self.owner)
            self.owner = owner
        self._steady(app, snapshot, owner)

    def _fix(self, app, word, now):
        if word is None or word == self.fix:
            self.fix_seen = None
            return
        if self.fix_seen is None or self.fix_seen[0] != word:
            self.fix_seen = (word, now)
        if self.fix is None or now - self.fix_seen[1] >= HOLD:
            app.emit('gnss_fix', fix=word, prev=self.fix)
            self.fix, self.fix_seen = word, None

    def _steady(self, app, snapshot, owner):
        trusted = app.armed and (owner in TRUSTED or app.running or app.manual)
        command = tuple(app.command) if trusted else None
        window = self.steady
        if window is not None and (command is None or any(
                abs(a - b) > self.settings['tolerance'] for a, b in zip(command, window.command))):
            self.end(app)
            window = None
        if command is not None and window is None:
            self.steady = Steady(command, self.wall(), self.fix)
        elif window is not None:
            values = snapshot.get('sensors', {}).get('odometry', {}).get('values') or {}
            velocity = [values.get(k) for k in ('vx', 'vy', 'wz')]
            window.add(velocity if all(isinstance(v, (int, float)) and math.isfinite(v) for v in velocity) else None)

    def end(self, app):
        """Close the open window (journaled when long enough); the session's close calls it too."""
        window, self.steady = self.steady, None
        if window is not None:
            t_ns = self.wall()
            if (t_ns - window.t0_ns) / 1e9 >= self.settings['min']:
                app.emit('steady', **window.record(t_ns))

    def view(self):
        return {}

"""Which correction source feeds the FCU: the local base receiver or NTRIP. No ROS imports.

Rule (rtk.yaml ``priority: auto``): NTRIP when its best station is under ``near_km`` (15 km) and
the local base is not at a fixed site, otherwise the local base; either alone when the other is
unavailable. ``priority: base`` or ``ntrip`` puts that one first.

Only the active source reaches the sinks: two bases mixed at the FCU spoil the rover's solution.
In ``auto`` an active source is kept until it dies, since every switch moves the RTK fix by the
base survey error or datum offset; ``base`` or ``ntrip`` switches to that choice at once.
``switches`` counts changes from one source to the other (the TUI raises a notice on each).
"""

import threading
import time

# Base phases that are, or will shortly be, a correction stream
BASE_UP = ('configuring', 'surveying', 'ready', 'fixed')
NTRIP_PHASES = dict(streaming='ready', connecting='connecting', standby='connecting', off='off')


def choose(priority, base_ok, base_fixed, ntrip_ok, ntrip_km, near_km=15.0):
    """'base', 'ntrip' or None (nothing available)."""
    if not ntrip_ok:
        return 'base' if base_ok else None
    if not base_ok:
        return 'ntrip'
    if priority in ('base', 'ntrip'):
        return priority
    near = ntrip_km is not None and ntrip_km < near_km
    return 'ntrip' if near and not base_fixed else 'base'


class Corrections:
    """Gate between the sources and the sinks; ``gate(name)`` is each source's sink."""

    def __init__(self, sinks, priority='auto', near_km=15.0, dead_after=5.0, config=None, clock=time.monotonic):
        self.sinks, self.priority, self.near_km = list(sinks), priority, near_km
        self.dead_after, self.clock = dead_after, clock
        self.config = config or {}          # the base settings, for StatusTracker's survey terms
        self.base = self.ntrip = self.rough = None
        self.active = self.used = None      # used: the last source that was active
        self.last, self.switches = {}, 0
        self.routes = [dict(sent=0, failed=0, error='', error_at=None) for _ in self.sinks]
        self.lock = threading.RLock()

    def attach(self, base=None, ntrip=None, rough=None):
        self.base, self.ntrip, self.rough = base, ntrip, rough

    def gate(self, name):
        return lambda frame, stamp: self.offer(name, frame, stamp)

    def offer(self, name, frame, stamp):
        """A frame from source ``name``; forwarded to every sink only while that source is active."""
        with self.lock:
            self.last[name] = stamp
            if self.active is None:
                self.evaluate()
            if name != self.active:
                return
        for index, sink in enumerate(self.sinks):
            route = self.routes[index]
            try:
                sink(frame, stamp)
            except (OSError, RuntimeError, ValueError) as error:
                route.update(failed=route['failed'] + 1, error=str(error), error_at=stamp)
            else:
                route['sent'] += 1

    def _alive(self, name, now):
        return now - self.last.get(name, float('-inf')) <= self.dead_after

    def evaluate(self):
        """Re-decide the active source (call at about 1 Hz); returns it."""
        with self.lock:
            now = self.clock()
            base = self.base.snapshot() if self.base else None
            if base and self.rough is not None and base.get('rough'):
                r = base['rough']
                self.rough.offer('base', r['latitude'], r['longitude'], r['height'])
            base_ok = bool(base) and base['phase'] in BASE_UP
            ntrip_ok = self.ntrip is not None and self.ntrip.available
            km = self.ntrip.best_km() if ntrip_ok else None
            want = choose(self.priority, base_ok, bool(base) and base['phase'] == 'fixed', ntrip_ok, km, self.near_km)
            if want != self.active and (self.active is None or not self._alive(self.active, now)
                                        or self.priority != 'auto'):
                self._switch(want)
            if self.ntrip is not None:      # NTRIP connects only while it is in use
                if self.active == 'ntrip':
                    self.ntrip.wanted.set()
                else:
                    self.ntrip.wanted.clear()
            return self.active

    def _switch(self, name):
        if name is not None:
            if self.used is not None and name != self.used:
                self.switches += 1
            self.used = name
        self.active = name

    def snapshot(self):
        """The BaseSession-shaped state of the active source, plus ``source`` for the status document."""
        with self.lock:
            active, switches = self.active, self.switches
        base = self.base.snapshot() if self.base else dict(
            phase='off', detail='no base receiver', received=0, delivered=0, errors=0, model='', survey=None,
            base_position=None, base_satellites=None, site=None, rough=None, routes=[])
        net = self.ntrip.snapshot() if self.ntrip else None
        kind = active or ('base' if self.base else 'ntrip' if self.ntrip else 'base')
        state = dict(base)
        if kind == 'ntrip' and net:
            state.update(phase=NTRIP_PHASES.get(net['phase'], 'connecting'), detail=net['detail'], model='NTRIP',
                         survey=None, base_position=None, base_satellites=None, site=None,
                         received=net['received'], delivered=net['received'])
        state['routes'] = [dict(r) for r in self.routes]
        state['source'] = dict(kind=kind, priority=self.priority, switches=switches,
                               base_phase=base['phase'], base_model=base.get('model', ''), site=base.get('site'),
                               ntrip=None if not net else {k: net[k] for k in (
                                   'phase', 'mount', 'caster', 'km', 'vrs', 'station', 'candidates', 'failovers',
                                   'last_error', 'gga_sent')})
        return state

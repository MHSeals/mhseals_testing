"""The single command loop every motion procedure goes through.

``Runner.hold(command, seconds)`` publishes one command at a fixed rate and,
on every tick, re-checks the interlocks and the abort keys. Any blocker,
abort or exception stops the loop, and the plant is sent neutral before the
error propagates. Procedures therefore never write their own publish loops,
and can't leave the boat driving.

A human override (spec authority_hotreload section 5) does not abort a hold:
``hold`` pauses at neutral, waits for the mux to read auto again and the resume
gate to clear, settles, then re-runs the interrupted hold from its start. The
interrupted attempt and every hold overlapping the pause are reported by
``Runner.invalid`` so the caller drops them from its fit.
"""

from contextlib import contextmanager
import time

ABORT_KEYS = ('space', 'q', 'escape')
NEUTRAL = (0.0, 0.0, 0.0)


def fold(key):
    """Single letters (and press:<letter>) compare lower-case, so Shift or Caps Lock cannot defeat a key."""
    return key.lower() if key and (len(key) == 1 or key.startswith('press:')) else key


class Aborted(RuntimeError):
    """Motion stopped by the operator or an interlock; the plant is neutral."""


class Overridden(Aborted):
    """A human took over the command mux; ``hold`` handles it, other loops abort."""


STALE_MUX_S = 3.0
PAUSE_LIMIT_S = 600.0
GUARD_S = 0.5  # holds this close before the override edge are invalid too


def override_cause(mux, age_s=0.0):
    """Why the mux is not under automation ('' when it is): rc/manual/hold, or unknown."""
    if mux is None or age_s > STALE_MUX_S:
        return 'unknown'
    return '' if mux.get('input') in ('auto', 'idle') else str(mux.get('input'))


class Runner:
    """Drive ``plant`` (``publish(command)`` / ``neutral()``) under interlocks.

    ``blockers()`` returns the reasons motion must stop (empty when ready),
    ``keys.read(timeout)`` returns a key name or None and paces the loop,
    ``show(command, remaining, label)`` refreshes the UI, and ``emit(phase,
    **details)`` records events into the session bag.

    ``override()`` returns the override cause or '' (see ``override_cause``);
    ``gate()`` returns reasons the boat is not yet still (resume waits for
    none). ``on_pause()`` restores the real correction C for the human and
    ``on_resume()`` re-stages identity; both are optional. A pause longer than
    ``pause_limit`` seconds aborts with ``override_timeout``.
    ``supervise`` (navigation) pauses instead while ``override()`` names a cause.
    """

    def __init__(self, plant, blockers, keys, show=lambda *args: None,
                 emit=lambda *args, **kwargs: None, clock=time.monotonic, rate=10.0,
                 override=lambda: '', gate=lambda: [], on_pause=lambda: None,
                 on_resume=lambda: None, settle=5.0, pause_limit=PAUSE_LIMIT_S):
        self.plant = plant
        self.blockers = blockers
        self.keys = keys
        self.show = show
        self.emit = emit
        self.clock = clock
        self.period = 1.0 / rate
        self.command = NEUTRAL
        self.override, self.gate = override, gate
        self.on_pause, self.on_resume = on_pause, on_resume
        self.settle, self.pause_limit = settle, pause_limit
        self.overrides = []   # {by, start, end, paused_s, window, holds_invalidated}
        self.done = []        # (start, end) of completed holds
        self.paused = 0.0     # s spent overridden inside supervise, summed over the session

    def invalid(self, start, end):
        """True when [start, end] overlaps an override window (the pause plus its settle)."""
        return any(start < o['window'][1] and end > o['window'][0] for o in self.overrides)

    def check(self, overrides=True):
        """Raise on an override (holds re-run; ``supervise`` pauses instead) or an interlock."""
        cause = self.override() if overrides else ''
        if cause:
            raise Overridden(cause)
        reasons = self.blockers()
        if reasons:
            raise Aborted('interlock: ' + '; '.join(reasons))

    def hold(self, command, seconds, label=''):
        """Publish ``command`` for ``seconds``; returns (start, end) clock times.

        An override pauses and re-runs the hold from its start; the returned
        window is the clean run.
        """
        while True:
            try:
                window = self._hold(command, seconds, label)
            except Overridden as error:
                self._pause(str(error), command, label)
            else:
                self.done.append(window)
                return window
            if self.settle:  # fresh neutral window; outside the handler so it may itself be overridden
                self.hold(NEUTRAL, self.settle, 'settle')

    def _pause(self, by, command, label):
        """Neutral, restore C, wait for release and the gate, settle; log the override."""
        began = self.clock()
        self.emit('override', by=by, label=label, command=list(command))
        self.emit('pause', by=by)
        self.on_pause()
        while self.override() or self.gate():
            if self.clock() - began > self.pause_limit:
                self.emit('pause_end', outcome='override_timeout')
                raise Aborted('override_timeout')
            if fold(self.keys.read(timeout=self.period)) in ABORT_KEYS:
                self.emit('pause_end', outcome='aborted')
                raise Aborted('aborted')
        resumed = self.clock()
        self.on_resume()
        entry = dict(by=by, start=began, end=resumed, paused_s=resumed - began,
                     window=(began - GUARD_S, resumed + self.settle), holds_invalidated=[])
        self.overrides.append(entry)
        entry['holds_invalidated'] = [w for w in self.done if self.invalid(*w)]
        self.emit('pause_end', outcome='resume', paused_s=entry['paused_s'])

    def _hold(self, command, seconds, label):
        command = tuple(float(value) for value in command)
        start = self.clock()
        deadline = start + seconds
        try:
            while True:
                now = self.clock()
                if now >= deadline:
                    return start, now
                self.check()
                self.plant.publish(command)
                self.command = command
                self.show(command, deadline - now, label)
                key = self.keys.read(timeout=min(self.period, deadline - now))
                if fold(key) in ABORT_KEYS:
                    raise Aborted(f'{label or "motion"} cancelled by operator')
        except BaseException:
            self.stop()
            raise

    def supervise(self, seconds, done=lambda: False, label='', sample=None,
                  on_pause=lambda: None, on_resume=lambda: None):
        """Watch while something else drives (Nav2): interlocks and abort keys, no publishing.

        Ends when ``done()`` is true or after ``seconds``; ``sample(t)`` runs each
        tick with the clock minus the paused time; ``label`` may be a callable for live
        text. Returns (start, end, finished).

        While ``override()`` is set the run is paused: ``on_pause()`` fires once (cancel
        the goal), nothing is sampled, the deadline slides by the paused time, and
        ``on_resume()`` fires when control returns (re-send the goal). The returned
        ``end`` excludes the paused time, so durations and metrics drop it.
        """
        start = self.clock()
        deadline = start + seconds
        paused_at, paused_before = None, self.paused
        try:
            while True:
                now = self.clock()
                cause = self.override()
                if paused_at is None and cause:
                    paused_at = now
                    self.emit('pause', cause=cause)
                    on_pause()
                elif paused_at is not None and not cause:
                    self.paused += now - paused_at
                    deadline += now - paused_at
                    self.emit('pause_end', outcome='resume', paused_s=now - paused_at)
                    paused_at = None
                    on_resume()
                lost = self.paused - paused_before
                if paused_at is not None:
                    lost += now - paused_at
                if paused_at is None and done():
                    return start, now - lost, True
                if paused_at is None and now >= deadline:
                    return start, now - lost, False
                self.check(overrides=False)   # an override pauses here, it does not abort
                if paused_at is not None:
                    self.show(self.command, deadline - paused_at, 'paused: manual override')
                    key = self.keys.read(timeout=self.period)
                    if fold(key) in ABORT_KEYS:
                        self.emit('pause_end', outcome='aborted', paused_s=now - paused_at)
                        raise Aborted('supervision cancelled by operator')
                    continue
                if sample is not None:
                    sample(now - lost)
                text = label() if callable(label) else label
                self.show(self.command, deadline - now, text)
                key = self.keys.read(timeout=min(self.period, deadline - now))
                if fold(key) in ABORT_KEYS:
                    raise Aborted(f'{text or "supervision"} cancelled by operator')
        except BaseException:
            self.stop()
            raise

    def stop(self):
        self.command = NEUTRAL
        self.plant.neutral()

    @contextmanager
    def session(self, name, **details):
        """Bracket a procedure: start/stop events, neutral on every exit path."""
        self.check()
        self.emit(name + '_start', **details)
        outcome = 'complete'
        try:
            yield self
        except Aborted as error:
            outcome = str(error)
            raise
        except BaseException as error:
            outcome = f'error: {error}'
            raise
        finally:
            self.stop()
            self.emit(name + '_stop', outcome=outcome)

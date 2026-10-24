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
    """Motion stopped by the operator or an interlock; the plant is neutral.

    ``by`` and ``cause`` say who ended it for the timeline: the operator's abort
    key (``space``) or a guard (``guard:<first blocker>``).
    """

    def __init__(self, message, by='operator', cause='space'):
        super().__init__(message)
        self.by, self.cause = by, cause


def interlock_abort(reasons):
    """The Aborted for a tripped interlock: stopped by the guard, caused by the first blocker's head."""
    return Aborted('interlock: ' + '; '.join(reasons), by='guard',
                   cause='guard:' + reasons[0].split(':')[0])


class Overridden(Aborted):
    """A human took over the command mux; ``hold`` handles it, other loops abort."""


STALE_MUX_S = 3.0
PAUSE_LIMIT_S = 600.0
GUARD_S = 0.5  # holds this close before the override edge are invalid too


def override_cause(mux, age_s=0.0):
    """Why the mux is not under automation ('' when it is): rc/manual/hold, or unknown."""
    if mux is None or not 0 <= age_s <= STALE_MUX_S:
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
    ``paused_tick(cause, elapsed)`` polls/repaints without commanding; ``key_action``
    handles the host's configured controls and returns 'abort' for abort keys.
    ``supervise`` (navigation) uses the same fresh-still neutral settle before resend.
    """

    def __init__(self, plant, blockers, keys, show=lambda *args: None,
                 emit=lambda *args, **kwargs: None, clock=time.monotonic, rate=10.0,
                 override=lambda: '', gate=lambda: [], on_pause=lambda: None,
                 on_resume=lambda: None, settle=5.0, pause_limit=PAUSE_LIMIT_S,
                 paused_tick=lambda cause, elapsed: None, key_action=None):
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
        self.paused_tick, self.key_action = paused_tick, key_action
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
            raise interlock_abort(reasons)

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

    def read_key(self, timeout):
        key = self.keys.read(timeout=timeout)
        action = self.key_action(key) if self.key_action else (
            'abort' if fold(key) in ABORT_KEYS else None)
        if action == 'abort':
            self.operator_stop(key)
            raise Aborted('motion cancelled by operator')

    def _wait_resume(self, by, began=None):
        """Poll without commanding; every loss of authority or stillness restarts settle."""
        began = self.clock() if began is None else began
        still_since = None
        while True:
            now = self.clock()
            self.paused_tick(by, now - began)
            now = self.clock()
            if now - began > self.pause_limit:
                self.emit('pause_end', outcome='override_timeout')
                raise Aborted('override_timeout')
            cause = self.override()
            reasons = self.gate()
            if cause or reasons:
                still_since = None
            elif still_since is None:
                still_since = now
            if still_since is not None and now - still_since >= self.settle:
                self.check(overrides=False)
                return began, now
            try:
                self.read_key(self.period)
            except Aborted:
                self.emit('pause_end', outcome='aborted')
                raise

    def _pause(self, by, command, label):
        """Neutral, restore C, wait for fresh stillness and full settle, then log."""
        began = self.clock()
        self.emit('override', by=by, label=label, command=list(command))
        self.emit('pause', by=by)
        self.stop()
        try:
            self.on_pause()
            _, resumed = self._wait_resume(by, began)
            self.on_resume()
        except BaseException:
            self.stop()
            raise
        entry = dict(by=by, start=began, end=resumed, paused_s=resumed - began,
                     window=(began - GUARD_S, resumed), holds_invalidated=[])
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
                self.read_key(min(self.period, deadline - now))
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
        paused_before = self.paused
        try:
            while True:
                now = self.clock()
                cause = self.override()
                if cause:
                    self.emit('pause', cause=cause)
                    on_pause()
                    began, resumed = self._wait_resume(cause, now)
                    self.paused += resumed - began
                    deadline += resumed - began
                    self.emit('pause_end', outcome='resume', paused_s=resumed - began)
                    on_resume()
                    now = self.clock()
                lost = self.paused - paused_before
                if done():
                    return start, now - lost, True
                if now >= deadline:
                    return start, now - lost, False
                self.check(overrides=False)
                if sample is not None:
                    sample(now - lost)
                text = label() if callable(label) else label
                self.show(self.command, deadline - now, text)
                self.read_key(min(self.period, deadline - now))
        except BaseException:
            self.stop()
            raise

    def operator_stop(self, key):
        """Space holds automation even while paused; hold errors still end at neutral."""
        if fold(key) == 'space':
            try:
                self.plant.hold('keys')
            finally:
                self.stop()

    def stop(self):
        self.command = NEUTRAL
        self.plant.neutral()

    @contextmanager
    def session(self, name, **details):
        """Bracket a procedure: start/stop events, neutral on every exit path."""
        self.check()
        self.emit(name + '_start', **details)
        outcome, ended = 'complete', dict(by='procedure')
        try:
            yield self
        except Aborted as error:
            outcome, ended = str(error), dict(by=error.by, cause=error.cause)
            raise
        except BaseException as error:
            outcome, ended = f'error: {error}', dict(by='error')
            raise
        finally:
            self.stop()
            self.emit(name + '_stop', outcome=outcome, **ended)

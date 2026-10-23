"""The single command loop every motion procedure goes through.

``Runner.hold(command, seconds)`` publishes one command at a fixed rate and,
on every tick, re-checks the interlocks and the abort keys. Any blocker,
abort or exception stops the loop, and the plant is sent neutral before the
error propagates. Procedures therefore never write their own publish loops,
and can't leave the boat driving.
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


class Runner:
    """Drive ``plant`` (``publish(command)`` / ``neutral()``) under interlocks.

    ``blockers()`` returns the reasons motion must stop (empty when ready),
    ``keys.read(timeout)`` returns a key name or None and paces the loop,
    ``show(command, remaining, label)`` refreshes the UI, and ``emit(phase,
    **details)`` records events into the session bag.
    """

    def __init__(self, plant, blockers, keys, show=lambda *args: None,
                 emit=lambda *args, **kwargs: None, clock=time.monotonic, rate=10.0):
        self.plant = plant
        self.blockers = blockers
        self.keys = keys
        self.show = show
        self.emit = emit
        self.clock = clock
        self.period = 1.0 / rate
        self.command = NEUTRAL

    def check(self):
        reasons = self.blockers()
        if reasons:
            raise Aborted('interlock: ' + '; '.join(reasons))

    def hold(self, command, seconds, label=''):
        """Publish ``command`` for ``seconds``; returns (start, end) clock times."""
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

    def supervise(self, seconds, done=lambda: False, label='', sample=None):
        """Watch while something else drives (Nav2): interlocks and abort keys, no publishing.

        Ends when ``done()`` is true or after ``seconds``; ``sample(now)`` runs each
        tick; ``label`` may be a callable for live text. Returns (start, end, finished).
        """
        start = self.clock()
        deadline = start + seconds
        try:
            while True:
                now = self.clock()
                if done():
                    return start, now, True
                if now >= deadline:
                    return start, now, False
                self.check()
                if sample is not None:
                    sample(now)
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

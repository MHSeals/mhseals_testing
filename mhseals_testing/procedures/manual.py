"""Manual control on the page: deadman keyboard driving under the interlocks.

The key handling is mhseals_hardware's ManualState, the same as keyboard_control:
held keys drive, releasing them (or the repeat timeout in plain terminals)
stops, Space clears everything, x / m / Esc leaves. It runs inside the page with a
key reader that reports key releases, so the dashboard stays on screen.
"""

import threading
import time

from mhseals_hardware.manual_state import ManualState
from mhseals_testing.runner import Aborted, fold, interlock_abort

EXIT_KEYS = ('x', 'm', 'escape', 'press:x', 'press:m', 'press:escape')
STOP_KEYS = ('space', 'press:space', 'focus-out')


CHECK_S = 0.2        # s between the starts of two evaluations of the slow interlocks
CHECK_STALE_S = 1.0  # s from an evaluation's start; older (a stuck, dead or > 0.5 s check) blocks like a tripped interlock


def start_thread(work):
    threading.Thread(target=work, name='manual-interlocks', daemon=True).start()


class Interlocks:
    """Manual's interlocks: ``now`` read on every call, ``checks`` evaluated beside the key loop.

    ``now`` (armed, a ROS fault) is read from memory, so a disarm or a stop blocks the very next
    press. ``checks`` ask the ROS graph and read files, too slow for every publish on a loaded
    computer (the command would go stale at the mux): a thread (``spawn``) evaluates them every
    CHECK_S from manual's start to its end, and a call answers from the last evaluation. One that
    raised, or one older than CHECK_STALE_S (``clock``, wall time), is a blocker, never clear.
    """

    def __init__(self, now, checks, clock=time.monotonic, spawn=None):
        self.now, self.checks, self.clock = now, checks, clock
        self.spawn = spawn or start_thread
        self.done = threading.Event()
        self.result = (float('-inf'), ['interlocks not checked yet'])

    def check(self):
        begun = self.clock()         # the answer is as old as its start
        try:
            reasons = list(self.checks())
        except Exception as error:   # a check that cannot answer blocks
            reasons = [f'interlock check failed: {error}']
        self.result = (begun, reasons)

    def loop(self):
        """Evaluate every CHECK_S, counted from each start, so a slow evaluation is followed at once."""
        while not self.done.wait(max(0.0, CHECK_S - (self.clock() - self.result[0]))):
            self.check()

    def __enter__(self):
        self.check()                 # the first press finds a current answer
        self.spawn(self.loop)
        return self

    def __exit__(self, *exc):
        self.done.set()              # an evaluation under way finishes; nothing reads it
        return False

    def __call__(self):
        reasons = list(self.now())
        when, checked = self.result
        age = self.clock() - when
        if not age <= CHECK_STALE_S:
            reasons.append(f'interlock check stale: {age:.1f} s old')
        return reasons + list(checked)


class GuardedPublisher:
    """Non-neutral commands must pass the interlocks; neutral always goes out."""

    def __init__(self, plant, blockers):
        self.plant = plant
        self.blockers = blockers
        self.command = (0.0, 0.0, 0.0)

    def hold(self):
        """Space: ask the mux to hold so auto commands cannot resume behind the stop."""
        try:
            self.plant.hold('keys')
        finally:
            self.publish((0.0, 0.0, 0.0))

    def publish(self, command):
        command = tuple(float(value) for value in command)
        if any(command):
            reasons = self.blockers()
            if reasons:
                self.plant.neutral()
                raise interlock_abort(reasons)
        self.command = command
        self.plant.publish(command, manual=True)


DRAIN_S = 0.1   # s after the last publish: the queued batch is read before the next, bounded by time, not count
MIN_DRAIN_S = 0.02   # s; after a long stall the budget is spent already, yet a stale burst must still be read whole


def drive(keys, state, publisher, show, clock=time.monotonic, annotate=None):
    """Read keys until an exit key; publish one coherent command per batch of events.

    ``annotate(key)`` sees every key after the drive state did (marks while driving); it only journals."""
    published = clock()
    while True:
        key = keys.read(timeout=0.05)
        command = state.update(None, clock()) if key is None else None
        # Consume every queued event before publishing, so no intermediate command goes out
        # between a press and its release: a burst of stale repeats after a link stall must
        # meet its release before anything is published.
        # The bound counts from the last publish, so the gap between commands stays under the deadman
        # even when the first read waited; MIN_DRAIN_S keeps a stalled link's burst whole.
        deadline = max(published + DRAIN_S, clock() + MIN_DRAIN_S)
        while key is not None:
            if fold(key) in EXIT_KEYS:
                return
            command = state.update(key, clock())
            if fold(key) in ('space', 'press:space'):
                publisher.hold()
            if key in STOP_KEYS:
                break  # Stop must not be hidden by queued input.
            if annotate is not None:
                annotate(key)   # after the stop: a mark never delays it
            if clock() >= deadline:
                break      # the rest stays queued for the next batch
            key = keys.read(timeout=0)
        publisher.publish(command)
        published = clock()
        show(command, state.amplitude, state.chords)


def run(ctx):
    manual = ctx.services['manual']
    config = ctx.services['hardware_config']
    kept = manual.get('speed', lambda: None)()   # the speed from the last manual session, if any
    state = ManualState(config['manual_amplitude'] if kept is None else kept, config['manual_repeat_timeout'],
                        config['manual_initial_timeout'], config['manual_yaw_scale'])
    clock = manual.get('clock', time.monotonic)
    interlocks = Interlocks(manual['blockers'], manual['checks'], spawn=manual.get('spawn'))
    publisher = GuardedPublisher(manual['plant'], interlocks)
    emit = manual.get('emit', lambda *args, **details: None)
    emit('manual_start')
    outcome = dict(outcome='complete', by='operator', cause='exit')
    try:
        with interlocks, manual['open_events']() as keys:
            drive(keys, state, publisher, manual['show'], clock, manual.get('annotate'))
    except Aborted as error:
        outcome = dict(outcome=str(error), by=error.by, cause=error.cause)
        raise
    except BaseException as error:
        outcome = dict(outcome=f'error: {error}', by='error')
        raise
    finally:
        manual['plant'].neutral()
        emit('manual_stop', **outcome)
        manual['show']((0.0, 0.0, 0.0), state.amplitude, state.chords, done=True)

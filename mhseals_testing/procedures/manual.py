"""Manual control on the page: deadman keyboard driving under the interlocks.

The key handling is mhseals_hardware's ManualState, the same as keyboard_control:
held keys drive, releasing them (or the repeat timeout in plain terminals)
stops, Space clears everything, x / m / Esc leaves. It runs inside the page with a
key reader that reports key releases, so the dashboard stays on screen.
"""

import time

from mhseals_hardware.manual_state import ManualState
from mhseals_testing.runner import Aborted, fold, interlock_abort

EXIT_KEYS = ('x', 'm', 'escape', 'press:x', 'press:m', 'press:escape')
STOP_KEYS = ('space', 'press:space', 'focus-out')


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


def drive(keys, state, publisher, show, clock=time.monotonic):
    """Read keys until an exit key; publish one coherent command per batch of events."""
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
    publisher = GuardedPublisher(manual['plant'], manual['blockers'])
    emit = manual.get('emit', lambda *args, **details: None)
    emit('manual_start')
    outcome = dict(outcome='complete', by='operator', cause='exit')
    try:
        with manual['open_events']() as keys:
            drive(keys, state, publisher, manual['show'], manual.get('clock', time.monotonic))
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

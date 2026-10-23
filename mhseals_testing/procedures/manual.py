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
        self.plant.hold('keys')

    def publish(self, command):
        command = tuple(float(value) for value in command)
        if any(command):
            reasons = self.blockers()
            if reasons:
                self.plant.neutral()
                raise interlock_abort(reasons)
        self.command = command
        self.plant.publish(command, manual=True)


def drive(keys, state, publisher, show, clock=time.monotonic):
    """Read keys until an exit key; publish one coherent command per batch of events."""
    while True:
        key = keys.read(timeout=0.05)
        command = state.update(None, clock()) if key is None else None
        # Consume already queued events before publishing, so no intermediate
        # neutral goes out between the press and release of a chord.
        for _ in range(64):
            if key is None:
                break
            if fold(key) in EXIT_KEYS:
                return
            command = state.update(key, clock())
            if fold(key) in ('space', 'press:space'):
                publisher.hold()
            if key in STOP_KEYS:
                break  # Stop must not be hidden by queued input.
            key = keys.read(timeout=0)
        publisher.publish(command)
        show(command, state.amplitude, state.chords)


def run(ctx):
    manual = ctx.services['manual']
    config = ctx.services['hardware_config']
    state = ManualState(config['manual_amplitude'], config['manual_repeat_timeout'],
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

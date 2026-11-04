"""Hold-to-confirm (ui/popups/hold.py, D-20261101-28): Enter, the action's key or a mouse button held until the
bar fills confirms; a tap, Esc or Space never does; a held key's repeats never reach the page afterwards."""

import pytest

from mhseals_hardware.keyboard import Mouse, Terminal
from mhseals_testing.ui import input as key_input
from mhseals_testing.ui.popups import hold


class Reader:
    """Scripted (delay, key) pairs on the app's clock; past the script it waits, and fails after 30 s."""

    def __init__(self, app, script):
        self.app, self.script, self.left = app, list(script), []
        self.terminal, self.report_events = Terminal('legacy'), False

    def read(self, timeout=None):
        if self.app.now > 30:
            raise AssertionError('the hold popup never ended')
        if self.script and self.script[0][0] <= timeout:
            delay, key = self.script.pop(0)
            self.app.now += delay
            return key
        if self.script:
            self.script[0] = (self.script[0][0] - timeout, self.script[0][1])
        self.app.now += timeout
        return None


class App:
    def __init__(self, script):
        self.now, self.modal, self.notice, self.stops, self.revokes = 0.0, None, '', 0, 0
        self.keys = Reader(self, script)
        self.shown = []

    def clock(self):
        return self.now

    def refresh(self):
        if self.modal is not None:
            self.shown.append(self.modal['title'])

    def tick(self):
        pass

    def operator_stop(self):
        self.stops += 1

    def revoke_agent(self):
        self.revokes += 1


def repeats(key, seconds, first=0.5, every=0.03):
    """A held key in a legacy terminal: the press, the OS's delay, then its auto-repeat."""
    return [(0.0, key), (first, key)] + [(every, key)] * round((seconds - first) / every)


@pytest.mark.smoke
def test_a_held_enter_confirms_and_its_repeats_never_reach_the_page():
    app = App(repeats('enter', 2.5) + [(0.05, 'x')])
    assert hold.confirm(app, 'Arm', 'arm', rows=[('Outputs', 'pins')], key='a') is True
    assert app.shown and app.modal is None
    assert app.now >= 1.5                          # the repeat delay, then the full hold
    assert app.keys.script == []                   # every leftover Enter (and the key after it) was swallowed


@pytest.mark.parametrize('key', ['a', 'U', 'O', 'q'])
def test_the_actions_own_key_held_confirms(key):
    app = App(repeats(key, 2.0))
    assert hold.confirm(app, 'Act', 'act', key=key) is True


def test_a_tap_never_confirms_by_waiting_and_esc_declines():
    app = App([(0.0, 'enter'), (3.0, 'escape')])
    assert hold.confirm(app, 'Arm', 'arm', key='a') is False and app.stops == 0


def test_a_hold_broken_by_another_key_starts_over():
    script = repeats('enter', 0.9) + [(0.03, 'z')] + repeats('enter', 0.9) + [(0.05, 'escape')]
    app = App(script)
    assert hold.confirm(app, 'Arm', 'arm') is False


def test_space_stops_at_once_and_declines():
    app = App(repeats('enter', 0.8) + [(0.01, 'space')])
    assert hold.confirm(app, 'Release hold?', 'release', key='U') is False and app.stops == 1


def test_e_still_revokes_the_agent_inside_the_popup():
    app = App([(0.0, 'E'), (0.1, 'escape')])
    assert hold.confirm(app, 'Arm', 'arm', key='a') is False and app.revokes == 1


def test_a_mouse_press_and_hold_confirms_and_an_early_release_does_not():
    held = App([(0.0, Mouse('press', 'left', 10, 5))])
    assert hold.confirm(held, 'Take control', 'take control', key='O') is True
    early = App([(0.0, Mouse('press', 'left', 10, 5)), (0.5, Mouse('release', 'left', 10, 5)), (2.0, 'escape')])
    assert hold.confirm(early, 'Take control', 'take control', key='O') is False


def test_the_session_keys_pass_the_mouse_to_the_popup():
    app = App([(0.0, Mouse('press', 'left', 10, 5))])
    app.keys = key_input.Keys(app.keys)
    assert hold.confirm(app, 'Quit while armed', 'quit', key='q') is True
    assert app.keys.passing == 0


def test_the_bar_fills_with_the_hold():
    assert hold.fill(0.0).strip('▕▏ ') == '' and hold.fill(1.0).count('█') == 24
    assert hold.label('a') == 'Enter or a' and hold.label(None) == 'Enter'

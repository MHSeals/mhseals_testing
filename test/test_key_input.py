"""The TUI's input layer (ui/input.py): one reader across manual mode, the terminal it reports,
hold timing per terminal (config/input.yaml) and the chords-unavailable indicator."""

import pytest

from mhseals_hardware.keyboard import KeyReader, Terminal
from mhseals_testing.ui import input as key_input
from mhseals_testing.ui.panels import conditions


@pytest.mark.smoke
def test_shipped_input_yaml_times_every_terminal():
    timing = key_input.load()
    assert set(timing) == set(Terminal.KINDS)


def test_a_malformed_input_yaml_is_refused(tmp_path):
    bad = tmp_path / 'input.yaml'
    bad.write_text('schema: mhseals.input/1\nterminals:\n  kitty: {confirm_hold: 1.0}\n')
    with pytest.raises(ValueError, match='terminals must be'):
        key_input.load(bad)


def test_manual_events_use_the_session_reader():
    reader = KeyReader()
    assert key_input.events(reader).reader is reader
    assert isinstance(key_input.events(None), KeyReader)    # no session reader: one of its own


def test_attach_reads_the_session_probe():
    reader = KeyReader()
    key_input.attach(reader, {'done': True, 'keyboard': 1}, env={})
    assert reader.terminal.kind == 'kitty' and key_input.chords_unavailable(reader) is None
    key_input.attach(reader, {'done': True, 'keyboard': 1}, env={'TMUX': '/tmp/t'})
    assert key_input.chords_unavailable(reader) == 'tmux'
    assert key_input.chords_unavailable(object()) is None    # a fake reader: unknown


def test_hold_timing_follows_the_terminal():
    timing = {kind: dict(confirm_hold=0.5, first_gap=0.4, gap=0.1) for kind in Terminal.KINDS}
    reader = KeyReader()
    reader.terminal = Terminal('legacy')
    hold = key_input.hold('y', reader, timing)
    assert (hold.hold_s, hold.releases, hold.first_gap_s, hold.gap_s) == (0.5, False, 0.4, 0.1)
    assert key_input.hold('mouse-left', reader, timing).releases     # the mouse always reports releases
    reader.terminal = Terminal('kitty')
    assert not key_input.hold('y', reader, timing).releases     # plain keys: a tap must not confirm by waiting
    reader.report_events = True
    assert key_input.hold('y', reader, timing).releases          # key events on: the release ends it


def test_a_kitty_tap_outside_manual_never_confirms_by_waiting():
    timing = {kind: dict(confirm_hold=1.0, first_gap=0.65, gap=0.15) for kind in Terminal.KINDS}
    reader = KeyReader()
    reader.terminal = Terminal('kitty')
    hold = key_input.hold('enter', reader, timing)
    hold.feed('enter', 0.0)
    assert not hold.tick(2.0) and hold.started is None


def test_input_yaml_names_the_hold_actions_and_the_mouse(tmp_path):
    shipped = key_input.load_file()
    assert set(shipped['hold']) <= set(key_input.ACTIONS) and isinstance(shipped['mouse'], bool)
    bad = tmp_path / 'input.yaml'
    bad.write_text(key_input.FILE.read_text().replace('take: true', 'tkae: true'))
    with pytest.raises(ValueError, match='hold maps'):
        key_input.load_file(bad)
    assert key_input.by_hold('take', bad) is False and key_input.mouse_on(bad) is False   # broken: the popups


class Script:
    """A reader that returns its keys in order, then None."""

    def __init__(self, keys, report_events=False):
        self.keys, self.report_events = list(keys), report_events
        self.terminal = Terminal('legacy')

    def read(self, timeout=None):
        return self.keys.pop(0) if self.keys else None

    def events(self):
        from contextlib import contextmanager

        @contextmanager
        def block():
            before, self.report_events = self.report_events, True
            yield self
            self.report_events = before
        return block()


def test_the_wheel_scrolls_and_other_mouse_reports_are_dropped():
    from mhseals_hardware.keyboard import Mouse
    clicks = [Mouse('wheel', 'up', 3, 4), Mouse('wheel', 'down', 3, 4), Mouse('press', 'left', 3, 4), 'a']
    keys = key_input.Keys(Script(clicks))
    assert [keys.read(0.1) for _ in range(4)] == ['up', 'down', None, 'a']
    keys = key_input.Keys(Script(clicks))
    with key_input.mouse(keys):                    # a hold popup takes the button
        assert [keys.read(0.1) for _ in range(3)][2] == 'press:mouse-left'
    keys = key_input.Keys(Script(clicks))
    with key_input.events(keys) as driving:        # manual driving: the wheel never becomes an arrow
        assert driving is keys and [keys.read(0.1) for _ in range(4)] == [None, None, None, 'a']
    keys.terminal = Terminal('kitty')              # the reader's own attributes pass through
    assert keys.reader.terminal.kind == 'kitty' and key_input.terminal(keys).kind == 'kitty'


def test_mouse_reports_turn_on_only_for_a_real_terminal(monkeypatch):
    written = []
    monkeypatch.setattr(key_input, 'terminal_write', written.append)
    with key_input.Keys(KeyReader()):              # not entered: no tty
        pass
    reader = KeyReader()
    reader.fd = 0
    with key_input.Keys(reader):
        assert written == [key_input.MOUSE_ON]
    assert written == [key_input.MOUSE_ON, key_input.MOUSE_OFF]


def test_the_driver_shows_when_chords_are_unavailable():
    assert conditions.mode_label(False, unavailable='tmux') == 'one key (tmux)'
    assert conditions.mode_label(True, unavailable='tmux') == 'combos'
    view = dict(authority=('human', 'KEYS'), chords=False, chords_unavailable='tmux')
    assert conditions.driver(view)[0].endswith('one key')

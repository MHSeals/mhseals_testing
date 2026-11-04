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
    assert key_input.hold('y', reader, timing).releases


def test_the_driver_shows_when_chords_are_unavailable():
    assert conditions.mode_label(False, unavailable='tmux') == 'one key (tmux)'
    assert conditions.mode_label(True, unavailable='tmux') == 'combos'
    view = dict(authority=('human', 'KEYS'), chords=False, chords_unavailable='tmux')
    assert conditions.driver(view)[0].endswith('one key')

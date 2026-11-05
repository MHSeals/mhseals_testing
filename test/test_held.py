"""Held panels (ui/held.py, perf.yaml hold): inside continuous frames a slow panel reuses its last layout until its
period passes; any other frame, or a new size, lays it out again, so a key's change shows in its own frame."""
from rich.console import Console
from rich.text import Text

from mhseals_testing.ui import held


def lines(renderable, width=20, height=2):
    console = Console(width=width, height=height, color_system=None)
    return [''.join(s.text for s in line) for line in console.render_lines(renderable, console.options.update(height=height))]


def test_continuous_frames_reuse_the_layout_until_the_period_and_others_lay_out_again():
    now, built = [0.0], []

    def panel():
        built.append(1)
        return Text(f'v{len(built)}')
    h = held.Held('t', panel, 0.5, clock=lambda: now[0])
    assert lines(h)[0].startswith('v1')
    with held.continuous():
        now[0] = 0.2
        assert lines(h)[0].startswith('v1') and len(built) == 1          # within the period: the last layout
        now[0] = 0.6
        assert lines(h)[0].startswith('v2')                               # past it: laid out again
        assert lines(h, width=30)[0].startswith('v3')                     # a new size: at once
    assert lines(h)[0].startswith('v4')                                   # a key's (forced) frame: at once


def test_hold_wraps_only_listed_panels_and_keeps_an_empty_slot_empty():
    assert isinstance(held.hold('status', lambda: Text('x'), {'status': 0.5}), held.Held)
    assert isinstance(held.hold('motion', lambda: Text('x'), {'status': 0.5}), Text)
    assert held.hold('status', None, {'status': 0.5}) is None
    assert set(held.periods()) >= {'status', 'procedures', 'activity'}

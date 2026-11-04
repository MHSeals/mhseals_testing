"""The image tier decision for every way the TUI is run on test day (theme.yaml graphics: auto, the default):
a plain kitty or foot, inside the operator's tmux or cx, boat_tui's tmux through docker exec (also over ssh,
`boat_tui view` and attach), and the preview. Images are the default; braille only with a reason."""

import os
from types import SimpleNamespace

import pytest

from mhseals_testing.ui import imaging as pixels
from mhseals_testing.ui import palette

pytestmark = pytest.mark.smoke   # tools/smoke

KITTY_CAPS = dict(done=True, kitty=True, sixel=False, cell=(10, 20), xtversion='kitty(0.42.0)')
FOOT_CAPS = dict(done=True, kitty=False, sixel=True, cell=(12, 24), xtversion='foot(1.28.0)')
XTERM_CAPS = dict(done=True, kitty=False, sixel=False, cell=(9, 18), xtversion='XTerm(390)')
TMUX_SIXEL = dict(done=True, kitty=False, sixel=True, cell=(12, 24), xtversion='tmux 3.4')
TMUX_PLAIN = dict(TMUX_SIXEL, sixel=False)
PANE = {'TMUX': '/tmp/tmux-1/default,1,0', 'TMUX_PANE': '%3'}
DOCKER = {'TERM': 'tmux-256color', 'TERM_PROGRAM': 'tmux'}      # what boat_tui hands into docker exec


def clients(*terms, passthrough=True, cell=(12, 24)):
    features = {'foot': frozenset({'RGB', 'sixel'})}
    return dict(window='@1', passthrough=passthrough,
                clients=tuple((str(i), t, features.get(t.split('(')[0], frozenset({'RGB'})), cell)
                              for i, t in enumerate(terms)))


@pytest.mark.parametrize('case, caps, env, view, tier', [
    ('kitty', KITTY_CAPS, {'TERM': 'xterm-kitty'}, None, 'kitty'),
    ('foot', FOOT_CAPS, {'TERM': 'foot'}, None, 'sixel'),
    ('cx or tmux pane, kitty client', TMUX_PLAIN, PANE, clients('kitty(0.42.0)'), 'kitty'),
    ('cx or tmux pane, foot client', TMUX_SIXEL, PANE, clients('foot(1.28.0)'), 'sixel'),
    ('boat_tui up/view through docker exec, tmux with sixel', TMUX_SIXEL, DOCKER, None, 'sixel'),
    ('the same before tmux answered XTVERSION', dict(TMUX_SIXEL, xtversion=None), DOCKER, None, 'sixel'),
    ('ssh from dingus to the odroid, attach', TMUX_SIXEL, DOCKER, None, 'sixel'),
    ('preview: tmux -L preview, docker exec', TMUX_SIXEL, DOCKER, None, 'sixel'),
    ('xterm without sixel', XTERM_CAPS, {'TERM': 'xterm-256color'}, None, 'text'),
    ('docker exec behind a tmux without sixel', TMUX_PLAIN, DOCKER, None, 'text'),
    ('NO_COLOR', FOOT_CAPS, {'NO_COLOR': '1'}, None, 'text'),
])
def test_auto_picks_an_image_tier_wherever_the_terminal_can_show_one(case, caps, env, view, tier):
    assert pixels.choose('auto', caps, env, view=view) == tier, case
    why = pixels.reason('auto', caps, env, view=view)
    assert bool(why) == (tier == 'text'), (case, why)          # braille always says why


def test_the_shipped_default_is_auto():
    assert palette.load_theme(overlay=False)[0].graphics == 'auto'


def test_a_detached_start_finds_the_client_cell_when_it_attaches(monkeypatch):
    """boat_tui up starts detached; the operator attaches later (the window resizes): the TUI asks tmux
    again and the replies come back through the key reader, never as keys."""
    monkeypatch.setattr(os, 'isatty', lambda fd: True)
    monkeypatch.setattr(pixels, 'probe', lambda fd, write, env=None: (dict(TMUX_SIXEL, cell=None), b''))
    now = [100.0]
    monkeypatch.setattr(pixels, 'clock', lambda: now[0])
    console = SimpleNamespace(size=(200, 50), color_system='256', file=None)
    written = []
    keys = SimpleNamespace(fd=5, buffer=bytearray(), absorb=lambda value: value.startswith(b'\x1b['))
    try:
        assert pixels.start(keys, console, written.append, 'auto', env=dict(DOCKER)) == 'text'
        assert 'cell size' in pixels.WHY
        console.size = (160, 45)                          # dingus attached
        pixels.watch()
        assert written == [pixels.PROBE]
        for reply in (b'\x1b[6;24;12t', b'\x1b[?62;4c'):
            assert keys.absorb(reply)                     # the key reader drops it, the round keeps it
        assert pixels.watch() is True
        assert pixels.TIER == 'sixel' and pixels.CELL == (12, 24) and pixels.WHY == ''
    finally:
        pixels.stop(None, lambda t: None)
        pixels.forget()


def test_the_tui_turns_passthrough_on_for_its_own_pane_only():
    calls = []
    pixels.tmux.passthrough(PANE, run=lambda argv, **kw: calls.append(argv))
    assert calls == [['tmux', 'set-option', '-p', '-t', '%3', 'allow-passthrough', 'on']]
    pixels.tmux.passthrough(DOCKER, run=lambda argv, **kw: pytest.fail('no tmux to ask'))

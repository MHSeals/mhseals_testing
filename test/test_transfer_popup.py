"""Send popup: frames fit, stall after 5 s, the y flow and the quit guard (bags.md B13)."""
import importlib
import io
import re

import pytest
from rich.console import Console

from mhseals_testing.ui import theme
from mhseals_testing.ui.popups import transfer as tp


def lines(t, now, width):
    panel, panel_width = tp.frame(t, now, width)
    assert panel_width <= width
    console = Console(width=panel_width, file=io.StringIO(), record=True)
    console.print(panel)
    return console.export_text().splitlines()


def running():
    t = tp.Transfer('boat_20261023_1402', 'tranquility.local', 'characterize, square', '192.168.0.10')
    t.update(0.0, done=0, total=4 << 30, stage='bags', passes=(2, 3), file='bag_01/bag_01_3.mcap')
    t.update(1.0, done=40 << 20)
    t.update(3.0, done=120 << 20)
    return t


@pytest.mark.parametrize('width', [48, 80, 120])
@pytest.mark.parametrize('state', ['running', 'done', 'failed'])
def test_frames_fit_and_have_no_background(width, state):
    t = running()
    if state != 'running':
        t.finish(state, 'rsync: connection unexpectedly closed (0 bytes received so far) [sender]')
    t.tick()
    out = lines(t, 3.0, width)
    assert out and all(len(line) <= width for line in out)
    panel, _ = tp.frame(t, 3.0, width)
    console = Console(width=width, force_terminal=True, color_system='standard', file=io.StringIO())
    console.print(panel)
    assert not re.search(r'\x1b\[[\d;]*\b(4[0-7]|48|10[0-7])[;m]', console.file.getvalue())  # no background


def test_stalled_after_five_seconds_only():
    t = running()
    assert not t.stalled(7.9) and 'stalled' not in '\n'.join(lines(t, 7.9, 80))
    assert t.stalled(8.1) and 'stalled' in '\n'.join(lines(t, 8.1, 80))
    t.update(8.2, done=130 << 20)
    assert not t.stalled(9.0)


def test_eta_waits_two_seconds_and_bar_eases():
    t = tp.Transfer('s', 'h')
    t.update(0.0, done=0, total=1000)
    t.update(1.0, done=100)
    assert t.eta(1.0) is None and t.eta(3.0) is not None
    for _ in range(40):
        t.tick()
    assert abs(t.shown - 0.1) < 0.01
    t.finish('done')
    t.tick()
    assert t.shown == 1.0


def test_ascii_frame_is_ascii(monkeypatch):
    monkeypatch.setenv('MHS_ASCII', '1')
    importlib.reload(theme)
    try:
        importlib.reload(tp)
        out = '\n'.join(lines(running(), 3.0, 80))
        assert out.isascii() and '[' in out
    finally:
        monkeypatch.delenv('MHS_ASCII')
        importlib.reload(theme)
        importlib.reload(tp)


def test_send_flow_steps():
    assert tp.send_steps(['a'], False, 'x') == ['index', 'send']
    assert tp.send_steps(['a', 'b'], True, '') == ['target', 'stop_recording', 'title', 'index', 'send']


def test_quit_guard_only_while_running():
    t = running()
    assert tp.quit_guard([t])['options'] == ['Wait', 'Cancel (resumable)']
    t.finish('done')
    assert tp.quit_guard([t]) is None
    assert tp.completion_toast(t)[0] == 'ok'

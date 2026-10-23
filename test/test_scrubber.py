"""Footer scrubber goldens (118 and 78 wide, Unicode and ASCII) and loupe geometry."""

import os
from pathlib import Path

import pytest

from mhseals_testing.ui.panels import scrubber
from mhseals_testing.ui.strip import FAIL, WARN, Band, Point, Span

GOLDEN = Path(__file__).parent / 'golden' / 'timeline'
T0 = 15 * 3600
M = 60
END = T0 + 192 * M
PLAY = T0 + 71 * M


def fmt(t):
    return f'{int(t) // 3600 % 24:02d}:{int(t) // 60 % 60:02d}'


def scene():
    spans = [Span(T0 + 5 * M, T0 + 20 * M, 'g'), Span(T0 + 40 * M, T0 + 62 * M, 's'),
             Span(T0 + 75 * M, T0 + 90 * M, 'f', end='abort', bad=(T0 + 88 * M, T0 + 90 * M, WARN)),
             Span(T0 + 150 * M, T0 + 165 * M, 'f')]
    points = [Point(T0 + 10 * M, 'mark'), Point(T0 + 33 * M, 'config'), Point(T0 + 70 * M, 'health', FAIL)]
    bands = [Band(T0 + 68 * M, T0 + 74 * M, FAIL), Band(T0 + 100 * M, T0 + 120 * M, 'recoff')]
    return dict(spans=spans, points=points, bands=bands, armed=[(T0 + 5 * M, T0 + 165 * M)])


def render(width, ascii_mode, size=720.0):
    return [line.plain for line in scrubber.footer(
        width, T0, END, PLAY, size, fmt=fmt, ascii_mode=ascii_mode, color=True, **scene())]


@pytest.mark.parametrize('ascii_mode', [False, True])
@pytest.mark.parametrize('width', [118, 78])
def test_footer_golden(width, ascii_mode, monkeypatch):
    monkeypatch.setattr('mhseals_testing.ui.theme.ASCII', ascii_mode)
    lines = render(width, ascii_mode)
    assert {len(line) for line in lines} == {width}
    path = GOLDEN / f'footer_{width}{"_ascii" if ascii_mode else ""}.txt'
    text = '\n'.join(lines) + '\n'
    if os.environ.get('UPDATE_GOLDEN'):
        path.write_text(text)
    assert path.read_text() == text
    if ascii_mode:
        assert text.isascii()


def test_bracket_matches_the_loupe_and_playhead_sits_inside():
    axis_row = render(118, False)[2]
    left, right = axis_row.index('├'), axis_row.index('┤')
    assert left < axis_row.index('●') < right
    a, b = scrubber.loupe_window(PLAY, 720.0, T0, END)
    assert (b - a, (a + b) / 2) == (720.0, PLAY)
    # 12 m at about 70 s per column (folds shrink the scale)
    assert 8 <= right - left + 1 <= 12


def test_loupe_clamps_to_the_session_and_z_cycles():
    assert scrubber.loupe_window(T0, 720.0, T0, END) == (T0, T0 + 720)
    assert scrubber.loupe_window(END, 720.0, T0, END) == (END - 720, END)
    assert scrubber.loupe_window(T0 + 10, 720.0, T0, T0 + 100) == (T0, T0 + 100)
    sizes = [720.0]
    for _ in range(3):
        sizes.append(scrubber.next_loupe(sizes[-1]))
    assert sizes == [720.0, 1800.0, 120.0, 720.0]
    axis = scrubber.loupe_axis(PLAY, 720.0, T0, END, 48)
    assert axis.folds == [] and axis.scale == 15.0


def test_forced_ascii_footer_has_ascii_status():
    lines = scrubber.footer(78, 0, 3600, 1800, 720, ascii_mode=True)
    assert '·' not in lines[-1].plain and '▶' not in lines[-1].plain

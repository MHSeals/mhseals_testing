"""Width-exact goldens of the timeline strip; UPDATE_GOLDEN=1 rewrites them."""

import os
from pathlib import Path

import pytest

from mhseals_testing.ui import theme
from mhseals_testing.ui.strip import FAIL, WARN, Band, Point, Span, rows
from mhseals_testing.ui.timeaxis import Axis

GOLDEN = Path(__file__).parent / 'golden' / 'timeline'
T0 = 14 * 3600
M = 60


def fmt(t):
    return f'{int(t) // 3600 % 24:02d}:{int(t) // 60 % 60:02d}'


def abort_scene():
    """Characterize aborted with Space, a hold, then a rerun still going (timeline.md 3.1)."""
    g = Span(T0 + 3 * M, T0 + 5 * M, 'g')
    f1 = Span(T0 + 6 * M, T0 + 9 * M + 40, 'f', end='abort', bad=(T0 + 8 * M + 40, T0 + 9 * M + 40, WARN))
    f2 = Span(T0 + 10 * M, T0 + 24 * M, 'f', running=True)
    points = [Point(T0 + 1 * M, 'config'), Point(T0 + 8 * M, 'config'), Point(T0 + 15 * M, 'health', WARN),
              Point(T0 + 16 * M, 'mark'), Point(T0 + 16 * M + 40, 'auth')]
    return dict(spans=[g, f1, f2], points=points, armed=[(T0 + 2 * M, T0 + 24 * M)],
                holds=[(T0 + 9 * M + 40, T0 + 10 * M)], bands=[Band(T0 + 15 * M, T0 + 18 * M, WARN)],
                sel=(T0 + 6 * M, T0 + 9 * M + 40)), T0, T0 + 24 * M


def idle_scene():
    return dict(armed=[(T0, T0 + 5 * M)], bands=[Band(T0, T0 + 5 * M, 'armrecoff')]), T0, T0 + 5 * M


SCENES = {'abort': abort_scene, 'idle': idle_scene}


def render(name, width, ascii_mode, **opts):
    kw, t0, end = SCENES[name]()
    sel = kw.pop('sel', None)
    busy = [(s.a, s.b) for s in kw.get('spans', [])]
    axis = Axis(t0, end, width, busy)
    return [r.plain for r in rows(axis, sel=sel, fmt=fmt, ascii_mode=ascii_mode, **kw, **opts)]


@pytest.mark.parametrize('ascii_mode', [False, True])
@pytest.mark.parametrize('width', [48, 46, 118])
@pytest.mark.parametrize('name', SCENES)
def test_golden(name, width, ascii_mode):
    lines = render(name, width, ascii_mode, color=True)
    assert {len(line) for line in lines} == {width}
    path = GOLDEN / f'{name}_{width}{"_ascii" if ascii_mode else ""}.txt'
    text = '\n'.join(lines) + '\n'
    if os.environ.get('UPDATE_GOLDEN'):
        path.write_text(text)
    assert path.read_text() == text
    if ascii_mode:
        assert text.isascii()


def test_no_color_texture_replaces_the_bad_slice_colour():
    colored = render('abort', 48, False, color=True)[1]
    plain = render('abort', 48, False, color=False)[1]
    assert '▓' in plain and '▓' not in colored


def test_styles_cap_bold_warn_and_accent_only_on_the_bracket():
    kw, t0, end = abort_scene()
    sel = kw.pop('sel')
    axis = Axis(t0, end, 48, [(s.a, s.b) for s in kw['spans']])
    out = rows(axis, sel=sel, fmt=fmt, ascii_mode=False, color=True, **kw)
    cap = next(s for s in out[1].spans if out[1].plain[s.start] == '×')
    assert cap.style == f'bold {theme.WARN}'
    for line in out[:4]:
        assert not any(theme.ACCENT in str(s.style) for s in line.spans)
    assert any(theme.ACCENT in str(s.style) for s in out[4].spans)


def test_one_row_form_and_cluster_count():
    kw, t0, end = abort_scene()
    axis = Axis(t0, end, 48, [(s.a, s.b) for s in kw['spans']])
    one = rows(axis, spans=kw['spans'], form='one', ascii_mode=False)
    assert len(one) == 1 and len(one[0].plain) == 48
    pts = [Point(T0 + 20 * M + i * 5, 'config') for i in range(6)]
    events = rows(axis, points=pts, ascii_mode=False)[0].plain
    assert 'Δ6' in events


@pytest.mark.parametrize('ascii_mode', [False, True])
def test_review_chrome_has_no_object_glyphs(ascii_mode):
    """Playhead and one-column selection use chrome glyphs, never the map's ●/○/▲ (ui U3)."""
    axis = Axis(T0, T0 + 30 * M, 60, [(T0, T0 + 30 * M)])
    lines = rows(axis, form='footer', window=(T0 + 5 * M, T0 + 20 * M), play=T0 + 10 * M,
                 sel=(T0 + 12 * M, T0 + 12 * M), fmt=fmt, ascii_mode=ascii_mode, color=False)
    text = '\n'.join(str(x) for x in lines)
    assert not set(text) & set('●○▲')
    assert ('|' if ascii_mode else '┃') in text

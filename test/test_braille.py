import importlib
import math

from mhseals_testing.ui import braille, theme


def text(canvas):
    return [row.plain for row in canvas.rows()]


def sine():
    return [(i / 20, math.sin(i / 20 * 2 * math.pi)) for i in range(0, 201)]


def test_sine_fills_a_10x2_canvas_without_overflow():
    c = braille.canvas(10, 2)
    assert c.line(sine(), 0, 10, -1.2, 1.2) == set()
    rows = text(c)
    assert len(rows) == 2 and all(len(row) == 10 for row in rows)
    assert all(ch != ' ' for row in rows for ch in row)
    assert all(0x2800 <= ord(ch) <= 0x28FF for row in rows for ch in row)


def test_clipped_samples_sit_on_the_edge_and_report_marks():
    c = braille.canvas(4, 1)
    marks = c.line([(0.1, 5.0), (9.9, -5.0)], 0, 10, -1, 1)
    assert marks == {'▲', '▼'}
    assert (0, 0) in c.dots['meas'] and (7, 3) in c.dots['meas']


def test_gaps_are_not_interpolated():
    # both cases span adjacent dot columns; only the time gap differs
    gapped = braille.canvas(1, 1)
    gapped.line([(0, 1), (60, -1)], 0, 100, -1, 1)
    joined = braille.canvas(1, 1)
    joined.line([(0, 1), (0.3, -1)], 0, 0.5, -1, 1)
    assert gapped.dots['meas'] == {(0, 0), (1, 3)}
    assert len(joined.dots['meas']) == 4


def test_sparse_samples_still_draw_a_continuous_line():
    c = braille.canvas(40, 1)
    series = [(i / 10, math.sin(i / 10 * 2 * math.pi)) for i in range(100)]
    c.line(series, 0, 10, -1.2, 1.2)
    assert {x for x, _ in c.dots['meas']} == set(range(80))
    gapped = braille.canvas(40, 1)
    gapped.line([(0, 0), (0.1, 0), (3, 0), (3.1, 0)], 0, 10, -1, 1)
    assert not any(1 < x < 24 for x, _ in gapped.dots['meas'])


def test_vertical_join_between_neighbouring_columns():
    c = braille.canvas(2, 2)
    c.line([(0.0, 1.0), (0.3, -1.0)], 0, 1, -1, 1)
    assert len(c.dots['meas']) > 2


def test_two_layers_prefer_the_accent_and_hline_only_in_empty_cells():
    c = braille.canvas(2, 2)
    c.line([(0.1, 1.0)], 0, 1, -1, 1, 'pred')
    c.line([(0.1, 1.0)], 0, 1, -1, 1, 'meas')
    c.hline(0.0, -1, 1)
    top, bottom = c.rows()
    assert top.spans[0].style == theme.ACCENT
    assert top.plain[0] != ' ' and bottom.plain == '┈┈'


def test_ascii_fallback_uses_dots(monkeypatch):
    monkeypatch.setattr(theme, 'ASCII', True)
    c = braille.canvas(2, 1)
    c.hline(0.0, -2, 2)
    clipped = c.line([(0.1, 3.0), (0.9, -3.0)], 0, 1, -2, 2)
    assert clipped == {'^', 'v'}
    assert set(text(c)[0]) <= {"'", '.', ':', ' '} and text(c)[0].strip()
    empty = braille.canvas(2, 1)
    empty.hline(0.0, -1, 1)
    assert text(empty)[0] == '--'


def test_cx_ascii_alias_and_spinner(monkeypatch):
    monkeypatch.setenv('CX_ASCII', '1')
    try:
        importlib.reload(theme)
        assert theme.ASCII and theme.spinner(0.15) == '/'
    finally:
        monkeypatch.delenv('CX_ASCII')
        importlib.reload(theme)
    assert theme.spinner(0.0) == '⠋' and theme.spinner(1.0) == '⠋'


def test_nan_and_infinite_samples_are_skipped():
    c = braille.canvas(4, 1)
    c.line([(1, float('nan')), (2, 0.0), (3, float('inf')), (4, 0.5)], 0, 10, -1, 1)
    assert len(c.dots['meas']) == 2


def test_flat_scale_draws_on_the_centre_row_without_dividing_by_zero():
    c = braille.canvas(4, 2)
    assert c.line([(1, 3.0), (9, 3.0)], 0, 10, 3.0, 3.0) == set()
    assert {y for _, y in c.dots['meas']} == {3}
    c.hline(3.0, 3.0, 3.0)
    assert c.lines == [(0, None)]


def test_non_finite_scale_or_reference_line_draws_on_the_centre_row():
    nan, inf = float('nan'), float('inf')
    series = [(1, 0.0), (5, 0.5)]
    for low, high in ((nan, 1), (-1, inf), (-1, nan), (-inf, inf)):
        c = braille.canvas(4, 2)
        c.line(series, 0, 6, low, high)
        assert {y for _, y in c.dots['meas']} == {3}
    c = braille.canvas(4, 2)
    c.hline(nan, -1, 1)
    c.hline(0, nan, nan)
    assert c.lines == [(0, None), (0, None)]


def test_infinite_or_overflowing_windows_plot_nothing():
    inf = float('inf')
    for t0, t1 in ((-inf, 10), (0, inf), (-1e308, 1e308)):
        c = braille.canvas(4, 1)
        assert c.line([(0, 0.0), (5, 0.5)], t0, t1, -1, 1) == set()
        assert not c.dots['meas']


def test_cx_ascii_alias_is_read(monkeypatch):
    monkeypatch.delenv('MHS_ASCII', raising=False)
    monkeypatch.setenv('CX_ASCII', '1')
    try:
        importlib.reload(theme)
        assert theme.ASCII and theme.spinner(float('nan')) == '|'
    finally:
        monkeypatch.delenv('CX_ASCII')
        importlib.reload(theme)

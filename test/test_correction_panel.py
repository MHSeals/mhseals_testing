import io
import os
from pathlib import Path

import pytest
from rich.console import Console

from mhseals_testing.ui import fitrows, theme
from mhseals_testing.ui.panels import correction
from mhseals_testing.ui.panels.correction import correction_view, details, move, render, worst_column

MATRIX = [(1, 1, -1), (1, -1, 1), (1, 1, 1), (1, -1, -1)]
C = [[1, .0, .0], [.031, 1, .02], [-.042, .084, 1]]


def fit(hi=96, lo=91, **extra):
    base = dict(trust_hi=hi, trust_lo=lo, sigma=.006, age='2h', residual='.2°/s 1%', iterations=1,
                fit=91, fit_label='good', ceiling=99, peak=.71, v_lo=.21, v_hi=.38, d1=.9, d2=2.1,
                t95=2.1, vmax=.42, tracking=.91,
                holds=[dict(a=.6, main=-.21, cross=.012, j=.31), dict(a=.6, main=-.22, cross=-.004, j=.18, limited=True)])
    base.update(extra)
    return base


def data(**fits):
    return dict(matrix=MATRIX, c_positive=C, c_negative=None,
                fits={k: fit() for k in correction.DIRECTIONS} | fits,
                header=dict(fitted='10-22 14:05', activity='refine surge+ · hold 7/14'),
                readiness=dict(verdict='good', rows=[('Verdict', 'good 94%')], gates=[('good', 'afloat')]),
                tracking=dict(version='cal v c0150', rows=[('surge', '91%', '88%', 'flat')], notes=['5 sessions']),
                plan='refine sway− (71%)')


def text(renderable, width, height=40):
    console = Console(file=io.StringIO(), width=width, height=height, color_system=None)
    console.print(renderable)
    return console.file.getvalue()


@pytest.mark.parametrize('width,height', [(120, 40), (100, 30), (80, 24)])
def test_renders_inside_width_and_height(width, height):
    view = correction_view(data(**{'sway-': fit(71, None, mark='?')}), cursor=3)
    lines = text(render(view, width, height), width, height).splitlines()
    page = '\n'.join(lines)
    assert all(len(line) <= width for line in lines) and len(lines) <= height
    for needle in ('surge+', theme.pick('yaw−', 'yaw-'), 'Trust hi/lo', 'Plan', '71/' + theme.DASH, theme.pick('sway− ?', 'sway- x'.replace('x', '?'))):
        assert needle in page
    assert ('Readiness' in page) == (width >= 110)


def test_scope_tags_sit_on_the_first_row_of_each_block():
    lines = text(render(correction_view(data()), 120), 120).splitlines()
    tagged = [line[2:5].strip() for line in lines if line[2:5].strip() in ('hw', 'cal')]
    assert tagged == ['hw', 'cal', 'cal']


def test_cross_entries_follow_the_columns_and_mark_limited():
    view = correction_view(data(**{'surge+': fit(hi=70, limited=['ψ'])}))
    surge = view['columns'][0]['cells']
    assert surge['cross'] == ['+.031', theme.pick('−', '-') + '.042 L', theme.DASH]
    assert view['columns'][2]['cells']['cross'][0] == theme.DASH   # the sway column has no sway entry
    assert view['header']['min_trust'] == 70


def test_cursor_moves_clamped_and_defaults_to_least_trust():
    view = correction_view(data(**{'yaw+': fit(55, 80)}), cursor=0)
    assert worst_column(view) == 4
    assert move(view, -1) == 0 and move(correction_view(data(), cursor=5), 1) == 5


def test_details_lists_the_cursor_columns_holds():
    page = text(details(correction_view(data(), cursor=2)), 40)
    assert 'sway+' in page and '.21' in page and 'L' in page
    assert 'no holds yet' in text(details(correction_view(dict(matrix=MATRIX), cursor=1)), 40)


def test_empty_data_renders_dashes_before_the_first_fit():
    view = correction_view(dict(matrix=MATRIX))
    page = text(render(view, 80, 24), 80, 24)
    assert 'not fitted' in page and 'Trust hi/lo' in page


def test_numbers_keep_integer_zeros():
    from mhseals_testing.ui.panels.correction import _num
    assert _num(10.0, 2, False) == '10.00' and _num(-20.5) == theme.pick('−', '-') + '20.500'
    assert _num(.031) == '+.031' and _num(-.042) == theme.pick('−', '-') + '.042'


def test_side_column_fits_a_short_terminal():
    big = data()
    big['readiness']['rows'] = [('row', 'x')] * 8
    big['tracking']['rows'] = [('surge', '9', '8', 'flat')] * 3
    lines = text(render(correction_view(big), 110, 24), 110, 24).splitlines()
    assert len(lines) <= 24 and 'holds' in '\n'.join(lines)


def test_cursor_tint_on_256_colours(monkeypatch):
    monkeypatch.setattr(theme, 'ASCII', False)
    monkeypatch.delenv('NO_COLOR', raising=False)
    monkeypatch.delenv('COLORTERM', raising=False)
    monkeypatch.setenv('TERM', 'xterm-256color')
    assert correction._tint() == 'on grey15'


# --- UIW22: scope tags from fields.yaml, fit under / ceiling now, header marks ---

def scope_data():
    fits = {k: fit() for k in correction.DIRECTIONS}
    fits['surge-'] = fit(ceiling=98, hdg=True)
    fits['sway+'] = fit(70, 80, fit=66, fit_label='marg', ceiling=93)   # clearly better now, low trust: up mark
    fits['sway-'] = fit(mark='?')
    fits['yaw+'] = fit(mark='✗')
    fits['yaw-'] = {}   # never fitted
    return data(**fits)


@pytest.mark.skipif(theme.ASCII, reason='the Unicode golden')
def test_scope_120_golden():
    golden = Path(__file__).parent / 'golden' / 'correction' / 'scope_120.txt'
    actual = text(render(correction_view(scope_data(), cursor=3), 120, 40), 120, 40)
    if os.environ.get('UPDATE_GOLDEN'):
        golden.write_text(actual)
    assert actual == golden.read_text(), 'scope_120.txt differs; UPDATE_GOLDEN=1 to regenerate'


def test_scope_tags_come_from_the_fields_scope():
    assert [correction.scope_tag(k) for k in ('thruster_matrix', 'decoupling', 'characterization.speed_curves',
                                              'command_timeout', '')] == ['hw', 'cal', 'cal', '', '']


def test_fit_rows_and_header_marks():
    view = correction_view(scope_data())
    cols = {c['key']: c for c in view['columns']}
    assert cols['surge+']['cells']['fit'] == 'good 91' and cols['surge-']['cells']['ceiling'] == '98 hdg'
    assert cols['sway+']['cells']['fit'] == 'marg 66' and cols['sway+']['cells']['ceiling'].endswith(fitrows.UP)
    assert cols['yaw-']['cells']['fit'] == theme.DASH == cols['yaw-']['cells']['ceiling']   # never fitted
    head = next(l for l in text(render(view, 120, 40), 120).splitlines() if 'yaw' in l and 'surge+' in l)
    assert f'sway+ {fitrows.UP}' in head and 'sway' in head and ' ?' in head and ('✗' in head or 'x' in head)


def test_scope_tags_never_take_a_status_hue():
    status = {theme.OK, theme.WARN, theme.FAIL} if hasattr(theme, 'OK') else {theme.WARN}
    assert correction.SCOPE == 'italic bright_black' and not any(h in correction.SCOPE for h in status)


def test_rendered_scope_tag_spans_are_slate_italic():
    tags = [seg for seg in Console(width=120).render(correction.render(correction_view(scope_data()), 120, 40))
            if seg.text.strip() in ('hw', 'cal')]
    assert tags and all(seg.style.italic and seg.style.color.name == 'bright_black' for seg in tags)

"""Drive at every width keeps Motion (ui.md 4.5), Conditions is a focus target and Enter opens its popup."""

import io

import pytest
from rich.console import Console

from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import layout, popups, theme
from mhseals_testing.ui.layout import render

from test_app import make_app
from test_dashboard import monitor, view

pytestmark = pytest.mark.skipif(theme.ASCII, reason='box and focus glyphs are Unicode')


def page(width, height, **kw):
    console = Console(width=width, height=height, file=io.StringIO(), force_terminal=False)
    console.print(render(monitor().snapshot(), view(load_profile('boat'), **kw), width, height))
    return console.file.getvalue()


def panel_rows(text, title):
    """The text lines of the panel titled ``title``, from its top border to its bottom border."""
    lines = text.splitlines()
    top = next(i for i, line in enumerate(lines) if '╭─' in line and title in line)
    left = lines[top].index(title)
    bottom = next(i for i in range(top, len(lines)) if '╰' in lines[i][:left])
    return lines[top:bottom + 1]


@pytest.mark.parametrize('width,height', [(120, 40), (110, 40), (100, 36), (90, 32), (80, 30), (70, 30)])
def test_motion_and_conditions_show_from_70_columns(width, height):
    out = page(width, height)
    assert 'Motion' in out and 'Conditions' in out


def test_the_row_tier_gives_motion_and_the_right_column_the_top_rows_at_100_columns():
    out = page(100, 36)
    assert len(panel_rows(out, 'Motion')) == layout.TOP_ROW
    assert len(panel_rows(out, 'Conditions')) == layout.COND_ROWS


def test_stacked_motion_is_three_one_row_axes():
    rows = panel_rows(page(80, 30), 'Motion')
    assert len(rows) == layout.MOTION_COMPACT
    assert [name in row for name, row in zip(('Surge', 'Sway', 'Yaw'), rows[1:-1])] == [True] * 3


def test_the_stack_drops_motion_when_the_body_is_too_short():
    assert 'Motion' not in page(60, 24)
    assert 'Conditions' in page(60, 24)


@pytest.mark.parametrize('width,height,motion', [(120, 40, True), (80, 30, True), (80, 26, False), (60, 24, False)])
def test_panels_follow_what_is_on_screen(tmp_path, width, height, motion):
    app, _ = make_app(tmp_path)
    app.console.size = (width, height)
    assert ('motion' in app.panels()) == motion and 'conditions' in app.panels()


def test_conditions_takes_focus_and_enter_opens_its_popup(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path)
    app.console.size = (120, 40)
    opened = []
    monkeypatch.setitem(popups.OPENERS, 'conditions', lambda a: opened.append(a))
    app.dispatch('ctrl-l')
    app.dispatch('ctrl-l')
    assert app.focus == 'conditions'
    app.dispatch('enter')
    assert opened == [app]
    assert '▸ Conditions' in page(120, 40, focus='conditions')

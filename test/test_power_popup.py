"""popup.power: golden from fixed battery facts, Enter on the Battery row, and no source."""
import io
from pathlib import Path
from types import SimpleNamespace

import pytest
from rich.console import Console

from mhseals_testing.analysis import power
from mhseals_testing.ui import layout, theme
from mhseals_testing.ui.popups import power as popup
from test_app import make_app

GOLDEN = Path(__file__).parent / 'golden' / 'power'
needs_unicode = pytest.mark.skipif(theme.ASCII, reason='goldens are the Unicode glyph set')


def facts_after_swap():
    """A 4S pack at rest, loaded, then a fuller pack fitted: the facts after the swap."""
    tracker = power.Power({'cells': 4, 'capacity_mah': 5000})
    sample = lambda t, v, a: power.from_mavros(SimpleNamespace(voltage=v, current=a, percentage=float('nan')), t)
    for t, v, a in ((0, 15.0, 0.1), (5, 15.0, 0.1), (20, 14.6, 10.0), (30, 14.5, 10.0), (40, 15.9, 0.1), (45, 15.8, 4.0)):
        out = tracker.update(sample(t, v, a))
    return out


def paint(modal, cols=100):
    panel, width = layout.popup(modal, cols, 30)
    out = io.StringIO()
    Console(width=width, file=out, color_system=None).print(panel)
    return out.getvalue()


def test_rows_show_pack_cell_percent_current_swap_and_source():
    text = '\n'.join(f'{a} {b}' for a, b in popup.rows(facts_after_swap()))
    for want in ('mavros', '15.80 V', '4S', '3.95 V min', 'Percent', '+4.0 A', '1 detected', 'Sag'):
        assert want in text


def test_no_source_names_the_flag():
    assert popup.rows(None)[0] == ('Source', 'no_voltage_source')


@needs_unicode
def test_golden(request):
    out = paint(popup.modal(facts_after_swap()))
    path = GOLDEN / 'popup_100.txt'
    if request.config.getoption('--update-goldens'):
        GOLDEN.mkdir(exist_ok=True)
        path.write_text(out)
    assert out == path.read_text()


def open_on_battery(app):
    seen = []
    app.popup = lambda modal, handle, refresh=None: seen.append((modal['title'], handle('x'), handle('escape')))
    app.dispatch('enter')
    return seen


def status_cursor(app, name):
    from mhseals_testing.ui.panels.status import status_rows
    rows = status_rows(app.snapshot(), app.profile.sensors)
    names = [r['name'] for r in rows]
    return names.index(name) if name in names else None


def test_enter_on_the_battery_row_opens_the_popup(tmp_path):
    app, _ = make_app(tmp_path)
    app.monitor.advise('Battery', 'OK', '15.8 V')
    app.ros.power_facts = facts_after_swap()
    with app.attached():
        app.focus = 'status'
        app.cursors['status'] = status_cursor(app, 'Battery')
        assert open_on_battery(app) == [('Power', None, True)]


def test_without_a_source_there_is_no_row_and_nothing_opens(tmp_path):
    app, _ = make_app(tmp_path)
    with app.attached():
        assert status_cursor(app, 'Battery') is None
        app.focus = 'status'
        app.cursors['status'] = 0
        titles = []
        app.popup = lambda modal, handle, refresh=None: titles.append(modal['title'])
        app.dispatch('enter')
    assert 'Power' not in titles

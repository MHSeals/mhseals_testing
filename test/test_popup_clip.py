"""Popups never outgrow a short terminal: the choices and hint stay, the body scrolls."""
import io
from types import SimpleNamespace

from rich.console import Console

from mhseals_testing.ui import app, dashboard


def draw(modal, width, height):
    panel, panel_width = dashboard.popup(modal, width, height)
    console = Console(width=panel_width, file=io.StringIO(), record=True)
    console.print(panel)
    return console.export_text().splitlines()


def arm_modal():
    rows = [(f'Interlock {n}', 'ok') for n in range(12)]
    capture = SimpleNamespace(popup=lambda modal, handle: modal, scroll=app.App.scroll)
    return app.App.choose(capture, 'Arm', ['y Arm', 'n Cancel'], rows=rows, width=64,
                          message='Outputs: simulator')


def test_arm_popup_keeps_choices_and_hint_at_40x12():
    lines = draw(arm_modal(), 40, 12)
    text = '\n'.join(lines)
    assert len(lines) <= 12 and 'y Arm' in text and 'n Cancel' in text and 'scroll' in text


def test_help_popup_fits_60x20_and_scrolls():
    modal = dict(kind='message', title='Help', rows=app.HELP, width=76, hint='Any key closes')
    first = draw(modal, 60, 20)
    assert len(first) <= 20 and 'Any key closes' in '\n'.join(first)
    modal['scroll'] = 10 ** 6  # clamped to the last line, never past the end
    last = draw(modal, 60, 20)
    assert last != first and modal['scroll'] < 10 ** 6


def test_roomy_terminal_is_not_clipped():
    modal = arm_modal()
    draw(modal, 120, 60)
    assert not modal['clipped']

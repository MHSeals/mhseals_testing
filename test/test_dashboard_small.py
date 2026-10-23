"""The dashboard module imports and swaps the page for a hint in a too-small terminal."""
from rich.console import Console

from mhseals_testing.ui import dashboard


def test_too_small_terminal_gets_enlarge_hint():
    width, height = dashboard.MIN_SIZE[0] - 1, dashboard.MIN_SIZE[1]
    page = dashboard.render({}, {}, width=width, height=height)  # returns before reading either
    console = Console(width=40, record=True)
    console.print(page)
    assert 'enlarge terminal' in console.export_text()


def _hint(view):
    page = dashboard.render({}, view, width=dashboard.MIN_SIZE[0] - 1, height=dashboard.MIN_SIZE[1])
    console = Console(width=60, record=True)
    console.print(page)
    return console.export_text()


def test_too_small_hint_shows_armed_state_and_running_trial():
    assert 'disarmed' in _hint({})
    text = _hint({'armed': True, 'running': 'surge'})
    assert 'ARMED' in text and 'surge' in text

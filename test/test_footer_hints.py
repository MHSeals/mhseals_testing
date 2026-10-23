"""The footer cuts hints to fit but keeps Stop, Help, Quit and Leave manual."""
from mhseals_testing.ui import app, dashboard


def keys(focus, manual, capacity):
    hints = app.MANUAL_HINTS if manual else app.PANEL_HINTS[focus] + app.GLOBAL_HINTS
    return [key for key, _ in dashboard.fit_hints(hints, capacity)]


def test_every_panel_keeps_stop_help_and_quit():
    for focus in app.PANEL_HINTS:
        for capacity in (6, 8):
            shown = keys(focus, False, capacity)
            assert len(shown) <= capacity
            assert {'Space', '?', 'q'} <= set(shown)


def test_manual_keeps_stop_and_leave_when_narrow():
    shown = keys('motion', True, 4)
    assert len(shown) == 4 and {'Space', 'x / Esc'} <= set(shown)


def test_short_lists_are_untouched():
    assert dashboard.fit_hints([('a', 'A')], 6) == [('a', 'A')]

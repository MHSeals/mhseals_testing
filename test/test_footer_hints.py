"""The footer cuts hints to fit but keeps Stop, Help, Quit and Leave manual."""
from mhseals_testing.ui import app, layout


def keys(focus, manual, capacity):
    hints = app.MANUAL_HINTS if manual else app.PANEL_HINTS[focus] + app.GLOBAL_HINTS
    return [key for key, _ in layout.fit_hints(hints, capacity)]


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
    assert layout.fit_hints([('a', 'A')], 6) == [('a', 'A')]


def test_footer_columns_follow_the_width():
    from rich.console import Console
    import io
    view = {'armed': False, 'outputs': None, 'focus': 'motion', 'position': '', 'notice': '',
            'hints': [(f'k{i}', f'l{i}') for i in range(8)]}
    for width, shown in ((59, 4), (60, 6), (105, 6), (106, 8)):
        out = io.StringIO()
        Console(file=out, width=width, color_system=None).print(layout.footer(view, width))
        assert sum(f'k{i}' in out.getvalue() for i in range(8)) == shown, width

"""Procedures panel: the runnable procedures and why one is not ready."""

from rich.console import Group
from rich.text import Text

from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import TITLES, window
from mhseals_testing.ui.theme import ACCENT, MUTED


def procedures_panel(view, height):
    focused = view['focus'] == 'procedures'
    items = view['procedures']
    cursor = min(view['cursors'].get('procedures', 0), max(0, len(items) - 1))
    table = theme.grid({'header': '', 'ratio': 1}, padding=(0, 0))
    table.show_header = False
    start, visible = window(items, cursor, max(1, height - 4))
    for index, item in enumerate(visible, start):
        state = ('RUNNING' if item['title'] == view['running'] else
                 'READY' if not item['reason'] else 'OFF')
        table.add_row(theme.row(state, item['title'], focused and index == cursor,
                                Text(item['key'], style=f'bold {ACCENT}')))
    current = items[cursor] if items else None
    detail = Text((current['reason'] or current['description']) if current else '',
                  style=MUTED, no_wrap=True, overflow=theme.OVERFLOW)
    ready = sum(1 for item in items if not item['reason'])
    return theme.block(Group(table, Text(''), detail), TITLES['procedures'], focused,
                       f'{ready} ready')

"""Procedures panel: the runnable procedures, each with its state icon and, when blocked, why."""

import re

from rich.console import Group
from rich.text import Text

from mhseals_testing.harness.runs import slug
from mhseals_testing.procedures import tree
from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import TITLES, window

# A procedure's state as (glyph, ASCII glyph, palette style name, legend word); running is the spinner.
STATES = {
    'ready': ('○', 'o', 'GOOD', 'ready'),
    'blocked': ('⊘', ':', 'MUTED', 'blocked'),   # ASCII ':' stays apart from the spinner's frames
    'running': (None, None, 'BUSY', 'run'),
    'done': ('✓', '+', 'GOOD', 'ok'),
    'failed': ('✗', 'x', 'BAD', 'fail'),
}
PERCENT = re.compile(r'(\d{1,3})%')     # the long-run place line (ui/progress.py) carries a percentage


def _glyph(state, now):
    glyph, ascii_glyph, _, _ = STATES[state]
    return theme.spinner(now) if state == 'running' else theme.pick(glyph, ascii_glyph)


def _style(state):
    return getattr(theme, STATES[state][2])


def legend(now=0.0):
    """``○ready ⊘blocked ⠋run ✓ok ✗fail``: every icon the panel draws, from STATES (it fits the sidebar's border)."""
    out = Text()
    for i, (state, (_, _, _, word)) in enumerate(STATES.items()):
        out.append(' ' if i else '')
        out.append(_glyph(state, now), style=_style(state))
        out.append(word, style=theme.MUTED)
    return out


def last_verdicts(results):
    """``{slug(title): kind}`` of each procedure's newest result (its run_id is ``NNN-<slug>``)."""
    found = {}
    for result in results or ():
        run = str(result.get('run_id') or '')
        if '-' in run:
            found[run.split('-', 1)[1]] = theme.kind(result.get('verdict', ''))
    return found


def state_of(item, view, verdicts):
    if item['title'] == view.get('running'):
        return 'running'
    if item['reason']:
        return 'blocked'
    return {'bad': 'failed', 'good': 'done', 'warn': 'done'}.get(verdicts.get(slug(item['title'])), 'ready')


def _label(item, state, view):
    label = Text(item['title'])
    if state == 'running':
        found = PERCENT.search(str(view.get('notice') or ''))
        if found:
            label.append(f' {found.group(1)}%', style=theme.BUSY)
    elif state == 'blocked':      # the reason on every blocked row, not only the cursor's
        label.append(f"{theme.SEPARATOR}{item['reason']}", style=theme.MUTED)
    return label


def procedures_panel(view, height, results=()):
    """The procedures by group; ``results`` (the snapshot's) mark the last run of each done or failed."""
    focused = view['focus'] == 'procedures'
    items = view['procedures']
    now = view.get('now', 0.0)
    verdicts = last_verdicts(results)
    cursor = min(view['cursors'].get('procedures', 0), max(0, len(items) - 1))
    table = theme.grid({'header': '', 'ratio': 1}, padding=(0, 0))
    table.show_header = False
    rows = tree([item.get('group', '') for item in items])
    at = next((n for n, (kind, index) in enumerate(rows) if kind == 'item' and index == cursor), 0)
    start, visible = window(rows, at, max(1, height - 4))
    for kind, index in visible:
        if kind == 'heading':
            table.add_row(Text(index, style=f'{theme.MUTED} bold'))
            continue
        item = items[index]
        state = state_of(item, view, verdicts)
        text = Text(theme.CURSOR if focused and index == cursor else '  ',
                    style=f'bold {theme.ACCENT}' if focused and index == cursor else None)
        text.append(_glyph(state, now), style=_style(state))
        text.append(' ')
        text.append(item['key'], style=f'bold {theme.ACCENT}')
        text.append(' ')
        label = _label(item, state, view)
        if focused and index == cursor:
            label.stylize(f'bold {theme.ACCENT} {theme.SEL_BG}' if theme.TINT else f'bold underline {theme.ACCENT}',
                          0, len(item['title']))
        table.add_row(text + label)
    current = items[cursor] if items else None
    detail = Text((current['reason'] or current['description']) if current else '',
                  style=theme.MUTED, no_wrap=True, overflow=theme.OVERFLOW)
    ready = sum(1 for item in items if not item['reason'])
    keys = legend(now).plain if focused else None
    return theme.block(Group(table, Text(''), detail), TITLES['procedures'], focused,
                       f'{ready} ready', keys=keys)

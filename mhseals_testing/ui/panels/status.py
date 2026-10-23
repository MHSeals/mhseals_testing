"""Status panel: sensors, TF chains and advisories as one navigable list."""

from rich.console import Group
from rich.text import Text

from mhseals_testing.health.kinds.topic_rate import classify
from mhseals_testing.readiness import advisories
from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import TITLES, window
from mhseals_testing.ui.theme import MUTED, SEPARATOR

MARKS = {'↓': theme.pick('↓', 'v'), '↑': theme.pick('↑', '^'), '~': '~',
         '⏱': theme.pick('⏱', 't'), '×': theme.pick('×', 'x')}
RATE_STYLE = {'warn': theme.WARN, 'fail': theme.BAD}  # Hz colour; a healthy rate stays muted


def status_rows(snapshot, specs):
    """Sensors, TF chains and advisories as one navigable list."""
    rows = []
    for spec in specs:
        row = snapshot['sensors'].get(spec.name)
        if row is None:
            state, value, detail = ('MISSING' if spec.required else 'OFF'), '', spec.topic
        else:
            state = row['status']
            # A silent sensor reads stale, not 'once': its rate window has simply emptied.
            value = ('stale' if row['status'] == 'STALE' else
                     f"{row['hz']:.0f} Hz" if row['hz'] else 'once')
            detail = row['issue'] or row['values'].get('summary', '') or spec.topic
        rate = ''
        if row is not None and spec.hz and state != 'STALE':
            rate, mark = classify(spec.hz, row['hz'], row['max_gap'], row['stamp_age'],
                                  limit=spec.deadline)
            rate = '' if rate == 'ok' else rate
            value += MARKS.get(mark, '')
        rows.append(dict(group='Sensors', state=state, name=spec.name, value=value, rate=rate,
                         detail=f'{spec.topic}{SEPARATOR}{detail}' if detail != spec.topic
                         else spec.topic, required=spec.required))
    for chain in snapshot['tf']:
        rows.append(dict(group='Frames', state=chain['status'],
                         name=f"{chain['source']} {theme.ARROW} {chain['target']}", value='',
                         detail=chain['detail'], required=True))
    for name, level, detail in advisories(snapshot):
        detail = theme.ascii_safe(detail)
        rows.append(dict(group='System', state=level, name=name, value=short(detail),
                         detail=detail, required=False))
    return rows


def short(detail, limit=10):
    """The telling number of an advisory: '78°C' of 'load 7.8/20, 78°C', '+0.04s' of a clock."""
    summary, separator, _ = str(detail).partition(';')
    if separator and len(summary) <= limit:
        return summary
    for part in reversed(str(detail).split(',')):
        token = part.strip().split(' ')[-1] if part.strip() else ''
        if token and any(c.isdigit() for c in token) and '/' not in token and len(token) <= limit:
            return token
    return ''


def status_panel(snapshot, view, height):
    rows = status_rows(snapshot, view['specs'])
    focused = view['focus'] == 'status'
    cursor = min(view['cursors'].get('status', 0), max(0, len(rows) - 1))
    table = theme.grid({'header': '', 'ratio': 1}, {'header': '', 'justify': 'right'},
                       padding=(0, 0))
    table.show_header = False
    budget = max(1, height - 4)
    start, visible = window(rows, cursor, budget - 3)  # room for group headings
    group = None
    for index, row in enumerate(visible, start):
        if row['group'] != group:
            group = row['group']
            table.add_row(Text(group, style=f'{MUTED} bold'), '')
        name = Text(row['name'], style='bold' if row['required'] else None)
        table.add_row(theme.row(row['state'], name, focused and index == cursor),
                      Text(row['value'], style=RATE_STYLE.get(row.get('rate'), MUTED)))
    detail = Text(rows[cursor]['detail'] if rows else 'No sensors in this profile',
                  style=MUTED, no_wrap=True, overflow=theme.OVERFLOW)
    live = sum(1 for row in rows if row['group'] == 'Sensors' and row['state'] == 'LIVE')
    sensors = sum(1 for row in rows if row['group'] == 'Sensors')
    return theme.block(Group(table, Text(''), detail), TITLES['status'], focused,
                       f'{live}/{sensors} live')

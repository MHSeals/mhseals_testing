"""Activity panel, ``agent`` view (agent-control 5): what agents run on the boat, newest first.

Rows fold from the AgentDesk view (``ui/feeds/agent.py``, ``agent_desk``): a hook note pair
(pre/post, one ``tool_id``) is one shell row ``blastoise$ ros2 node list  ✓ 2.1 s``; the current
act and the queue are verb rows ``▸ run square  ◌ 34 s``; ``recent`` finished acts, when the desk
gives them, keep their result, and a refusal reads warn with its hint. Pure Rich, no ROS. The fold
is cached on the desk's lists, so a frame with nothing new costs nothing.
"""

import json
import re
import time

from rich.text import Text

from mhseals_testing.ui import authority_chip, theme
from mhseals_testing.ui.panels import TITLES, window

WHO_AT = 70        # the who column shows from this panel width
_SSH = re.compile(r'^ssh\s+(?:(?:-[BbcDEeFIiJLlmOoPpQRSWw]\s+\S+|-\w+)\s+)*([\w.@][\w.@-]*)\s+')


def short(cmd):
    """``ssh -t blastoise ros2 node list`` reads ``blastoise$ ros2 node list``; others ``$ cmd``."""
    match = _SSH.match(cmd)
    return f'{match.group(1)}$ {cmd[match.end():]}' if match else f'$ {cmd}'


class Row(dict):
    """``t, who, what, state (good|bad|busy|warn|off), result, lines``; a dict so detail can dump it."""


def _shell(row, line):
    if line.get('phase') == 'text':                          # agent `note TEXT`
        row.update(what=f'" {line.get("cmd", "")}"', state='off', result='note')
        return
    if line.get('cmd') or 'what' not in row:
        row['what'] = short(line.get('cmd') or '?')
    if line.get('exit') is None and line.get('phase') != 'post':
        row.update(state='busy', result='running')
        return
    dur = line.get('dur_s')
    took = f'{dur:.1f} s' if isinstance(dur, (int, float)) else ''
    ok = line.get('exit') in (0, None)
    row.update(state='good' if ok else 'bad', result=took if ok else f"exit {line.get('exit')}")


def _verb(item, state, result, hint=''):
    return Row(t=(item.get('t_ns') or 0) / 1e9, who=item.get('who', '?'), state=state, result=result,
               what=f"{theme.FOCUS} {item.get('label') or item.get('op', '?')}", hint=hint, lines=[item])


def _acts(desk):
    """Verb rows: the current act, then the queue, then finished acts (``recent``), newest first."""
    out = []
    current = desk.get('act')
    if current:
        waiting = current.get('state') in ('starting', 'asking', 'waiting_arm')
        elapsed = current.get('elapsed_s')
        took = authority_chip.age(elapsed) if isinstance(elapsed, (int, float)) else ''
        row = _verb(current, 'warn' if current.get('cause') else 'busy',
                    current.get('cause') or (current['state'].replace('_', ' ') if waiting else took))
        row['t'] = row['t'] or float('inf')        # the act always heads the list
        out.append(row)
    out += [dict(_verb(q, 'busy', 'queued'), t=float('inf')) for q in desk.get('queue') or ()]
    for item in reversed(desk.get('recent') or ()):
        if item.get('ok', True):
            out.append(_verb(item, 'good', 'done'))
        else:
            word, _, why = str(item.get('error') or 'refused').partition(': ')
            out.append(_verb(item, 'warn', word, theme.SEPARATOR.join(filter(None, (why, item.get('hint'))))))
    return out


def fold(desk):
    """Rows, newest first: acts and the queue on top, then one row per ``tool_id`` (or loose note)."""
    rows, order = {}, []
    for note in (desk or {}).get('notes') or ():
        key = (note.get('who'), note.get('tool_id') or id(note))
        row = rows.get(key)
        if row is None:
            row = rows[key] = Row(t=(note.get('t_ns') or 0) / 1e9, who=note.get('who', '?'), lines=[])
            order.append(key)
        row['lines'].append(note)
        _shell(row, note)
    shells = [rows[key] for key in order]
    return sorted(_acts(desk or {}) + shells, key=lambda r: r['t'], reverse=True)


_CACHE = {'key': None, 'rows': []}


def rows(desk):
    """The folded rows of ``desk``, refolded only when its act, queue or newest note changes."""
    desk = desk or {}
    notes, recent = desk.get('notes') or (), desk.get('recent') or ()
    current = desk.get('act') or {}
    key = (len(notes), id(notes[-1]) if notes else None, len(recent), id(recent[-1]) if recent else None,
           tuple(sorted((k, str(v)) for k, v in current.items() if k != 'motion_lease_s')),
           tuple(q.get('id') for q in desk.get('queue') or ()))
    if key != _CACHE['key']:
        _CACHE.update(key=key, rows=fold(desk))
    return _CACHE['rows']


def next_view(current, desk):
    """``v`` cycles results · log, plus agent while an AgentDesk runs (the session owner)."""
    views = ('results', 'log') + (('agent',) if desk is not None else ())
    return views[(views.index(current) + 1) % len(views)] if current in views else views[0]


def clock(t):
    return time.strftime('%H:%M:%S', time.localtime(t)) if t and t != float('inf') else 'now'


class _Table:
    """The table, built at render time so the Agent column shows only where the panel is wide."""

    def __init__(self, items, cursor, height, focused):
        self.args = items, cursor, height, focused

    def __rich_console__(self, console, options):
        yield table(*self.args, wide=options.max_width >= WHO_AT)


def body(view, height, focused=False):
    """The view's renderable and its subtitle."""
    items = rows(view.get('agent_desk'))
    if not items:
        return Text(f'No agent activity{theme.SEPARATOR}boat commands arrive from the Claude Code hook',
                    style=theme.MUTED), 'agent'
    return _Table(items, view['cursors'].get('activity', 0), height, focused), \
        f'agent{theme.SEPARATOR}{len(items)}'


def table(items, cursor, height, focused, wide=True):
    start, visible = window(items, cursor, max(1, height))
    columns = [{'header': '  Time', 'width': 10}] + ([{'header': 'Agent', 'max_width': 14}] if wide else [])
    grid = theme.grid(*columns, {'header': 'Command', 'ratio': 1},
                      {'header': 'Result', 'justify': 'right', 'max_width': 10}, padding=(0, 1))
    for index, row in enumerate(visible, start):
        active = focused and index == cursor
        what = Text(theme.ascii_safe(row['what']), style=theme.WARN if row['state'] == 'warn' else None)
        if row.get('hint'):
            what.append(f"{theme.SEPARATOR}{row['hint']}", style=theme.WARN)
        result = theme.labelled(row['state'], row['result']) if row['result'] else Text('')
        cells = [theme.row(None, clock(row['t']), active)] + (
            [Text(row['who'], style=theme.auth_style('agent'))] if wide else [])
        grid.add_row(*cells, what, result)
    return grid


def detail(app, desk, cursor):
    """Enter on a row: the full command or request and every channel line of it, as JSON."""
    items = rows(desk)
    if not items:
        return None
    row = items[min(cursor, len(items) - 1)]
    lines = [json.dumps({k: v for k, v in line.items() if k != 'schema'}, default=str) for line in row['lines']]
    title = theme.SEPARATOR.join((TITLES['activity'], row['who'], clock(row['t'])))
    return app.show(title, message='\n'.join([row['what'], *lines]), width=100)

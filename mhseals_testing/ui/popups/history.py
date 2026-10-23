"""popup.history: browse the config history (ui.md 4.3, config_history.md 5-6).

``State`` is the whole popup (tab, cursor); ``body`` draws it at a terminal width and
``handle`` applies one key, so both run without a terminal. ``open_history`` is the modal loop.

Glyph grammar (ui.md section 9): the head of a subsystem carries an accent ``now`` tag, a revert
is named with the word ``revert``, ``±`` marks a change made outside boat_test, ``◌`` one still
pending, and the italic slate ``hw`` / ``cal`` tags say which kind of key the change touched.
"""

import dataclasses
import datetime

from rich.text import Text

from mhseals_testing import calibration
from mhseals_testing.history import render
from mhseals_testing.ui import theme

SUBS = ('hardware', 'nav2', 'fcu', 'rtk', 'health', 'keymap')
LABELS = {'hardware': 'Hardware', 'nav2': 'Nav2', 'fcu': 'FCU', 'rtk': 'RTK', 'health': 'Health', 'keymap': 'Keymap'}
EXTERNAL, PENDING = theme.pick('±', '~'), theme.pick('◌', 'o')
SLATE = f"italic {theme.PALETTE.style('slate')}"
ACCENT = f'bold {theme.ACCENT}'
CADENCES = ('site', 'build')


@dataclasses.dataclass
class State:
    """Everything the popup shows."""
    history: object
    tab: int = 0                 # 0 is All, then one per subsystem that has history
    cursor: int = 0
    rows: int = 14               # list rows that fit
    now: datetime.datetime = None
    pins: dict = None
    fields: dict = None
    cache: dict = dataclasses.field(default_factory=dict)

    def tabs(self):
        present = {c['subsystem'] for c in self.history.changes()}
        return [None] + [s for s in SUBS if s in present]

    def entries(self):
        """Changes of the open tab, newest first."""
        return list(reversed(self.history.changes(self.tabs()[self.tab % len(self.tabs())])))

    def selected(self):
        found = self.entries()
        return found[min(self.cursor, len(found) - 1)] if found else None

    def clock(self):
        return self.now or datetime.datetime.now().astimezone()


def _value(state, change, which):
    sha = change[which]
    if sha is None:
        return None
    return state.history.version({'subsystem': change['subsystem'], 'after': sha})['value']


def rows_of(state, change, against=None):
    """Diff rows from ``against`` (default the version before ``change``) to ``change``."""
    key = (change['id'], against and against['id'])
    if key not in state.cache:
        old = _value(state, against, 'after') if against else _value(state, change, 'before')
        state.cache[key] = render.diff(change['subsystem'], old, _value(state, change, 'after'),
                                       state.pins, state.fields)
    return state.cache[key]


def scope_tag(sub, rows, fields):
    """``hw``, ``cal`` or ``cal·site`` over the keys a change touched, '' for plain settings."""
    tags = set()
    for row in rows:
        key = row.path.split('[')[0]
        kind, cadence = calibration.scope_of(f'nav2.{key.rsplit(".", 1)[-1]}' if sub == 'nav2' else key, fields)
        if kind == 'hardware':
            tags.add('hw')
        elif kind == 'calibration':
            tags.add(f'cal{theme.MIDDOT}{cadence}' if cadence in CADENCES else 'cal')
    return ' '.join(sorted(tags))


def name_of(state, change):
    """(text, set_by_person): a person's name, else the generated one."""
    if change.get('name'):
        return change['name'], True
    if change.get('revert_of'):
        return f"revert to {change['revert_of']}", False
    text = render.auto_name(change, rows_of(state, change), ascii_=theme.ASCII)
    return text, False


def day_label(when, now):
    days = (now.astimezone(when.tzinfo).date() - when.date()).days
    stamp = when.strftime('%a %b %d').replace(' 0', ' ')
    return f'Today{theme.SEPARATOR}{stamp}' if days == 0 else 'Yesterday' if days == 1 else stamp


def change_line(state, change, active, width, heads):
    """One list row: cursor, mark, id, time, name, and the right-hand tags."""
    when = datetime.datetime.fromisoformat(change['time'])
    mark = EXTERNAL if change['source'] == 'external' else PENDING if change.get('applied') == 'pending' else ' '
    right = Text()
    tag = scope_tag(change['subsystem'], rows_of(state, change), state.fields)
    if tag:
        right.append(tag, style=SLATE)
    if change['id'] in heads:
        right.append(' now' if tag else 'now', style=ACCENT)
    name, named = name_of(state, change)
    head = Text(f"{mark} {change['id']}  {when:%H:%M}  ")
    room = max(4, width - 2 - head.cell_len - right.cell_len - 1)
    label = Text(name, style='bold' if named else 'dim')
    label.truncate(room, overflow='ellipsis', pad=True)
    line = head + label + Text(' ') + right
    return theme.selected(line, active)


def list_lines(state, width):
    """The list pane: day headers between rows, windowed around the cursor."""
    heads = {state.history.head(s)['id'] for s in SUBS if state.history.head(s)}
    now, lines, at = state.clock(), [], 0
    day = None
    for index, change in enumerate(state.entries()):
        when = datetime.datetime.fromisoformat(change['time'])
        if when.date() != day:
            day = when.date()
            lines.append(Text(day_label(when, now), style='dim'))
        if index == state.cursor:
            at = len(lines)
        lines.append(change_line(state, change, index == state.cursor, width, heads))
    start = max(0, min(at - state.rows // 2, len(lines) - state.rows))
    return lines[start:start + state.rows] or [Text('no changes recorded', style='dim')]


def tab_line(state, width):
    line = Text()
    for index, sub in enumerate(state.tabs()):
        label = 'All' if sub is None else LABELS[sub]
        if index == state.tab:
            line.append(f'[{label}]', style=ACCENT)
        else:
            line.append(f' {label} ', style='dim')
        line.append(' ')
    line.truncate(width, overflow='ellipsis')
    return line


def footer():
    return theme.SEPARATOR.join(('n name', 'N note', '[ ] tab', 'Esc'))


def body(state, cols):
    """The popup body for a ``cols``-column terminal: the tabs, then the list."""
    width = min(cols - 8, 92) - 4 if cols >= 80 else cols - 4
    out = [tab_line(state, width), Text(''), *list_lines(state, width)]
    for line in out:
        line.truncate(width, overflow='ellipsis')
    return Text('\n').join(out)


def modal(state, cols):
    return dict(kind='custom', title='Config history', body=body(state, cols), width=min(cols - 8, 92) if cols >= 80 else cols,
                hint=footer(), danger=False)


def handle(state, key, app):
    """Apply one key; returns True when the popup should close."""
    change = state.selected()
    if key in ('escape', 'q'):
        return True
    elif key in ('[', 'left', ']', 'right'):
        state.tab = (state.tab + (1 if key in (']', 'right') else -1)) % len(state.tabs())
        state.cursor = 0
    elif key in ('up', 'k', 'down', 'j'):
        state.cursor = max(0, min(len(state.entries()) - 1, state.cursor + (1 if key in ('down', 'j') else -1)))
    elif change is None:
        pass
    elif key in ('n', 'N'):
        field = 'name' if key == 'n' else 'note'
        text = app.ask_text(f'Name {change["id"]}' if key == 'n' else f'Note on {change["id"]}', '', change.get(field, ''))
        if text is not None:
            state.history.label(change['id'], **{field: text})
    return False


def open_history(app, sub=None):
    """H: the history popup, optionally on one subsystem's tab."""
    state = State(app.history, fields=render.load_fields())
    if sub in state.tabs():
        state.tab = state.tabs().index(sub)
    shell = {}

    def redraw():
        size = app.console.size
        state.rows = max(6, size.height - 14)
        shell.update(modal(state, size.width))

    def on_key(key):
        done = handle(state, key, app)
        redraw()                      # the next frame shows this key's result, not the last one's
        return done or None

    redraw()
    return app.popup(shell, on_key, redraw)

"""popup.history: browse the config history and revert a subsystem (ui.md 4.3, config_history.md 5-6).

``State`` is the whole popup (tab, cursor, plan); ``body`` draws it at a terminal width and
``handle`` applies one key, so both run without a terminal. ``open_history`` is the modal loop.
Reverting is ``history.revert.plan`` then ``.execute``; nothing here applies a config itself.

Glyph grammar (ui.md section 9): the head of a subsystem carries an accent ``now`` tag, a revert
is named with the word ``revert``, ``±`` marks a change made outside boat_test, ``◌`` one still
pending, and the italic slate ``hw`` / ``cal`` tags say which kind of key the change touched.
"""

import dataclasses
import datetime
import textwrap

from rich.text import Text

from mhseals_testing import calibration
from mhseals_testing.history import render, revert
from mhseals_testing.ui import theme

SUBS = ('hardware', 'nav2', 'fcu', 'rtk', 'health', 'keymap')
LABELS = {'hardware': 'Hardware', 'nav2': 'Nav2', 'fcu': 'FCU', 'rtk': 'RTK', 'health': 'Health', 'keymap': 'Keymap'}
WIDE = 100            # terminal columns from which the detail sits beside the list
LIST_W = 44
EXTERNAL, PENDING = theme.pick('±', '~'), theme.pick('◌', 'o')
CADENCES = ('site', 'build')


@dataclasses.dataclass
class State:
    """Everything the popup shows. ``backends`` and ``env`` are the revert seams."""
    history: object
    backends: dict
    env: object
    tab: int = 0                 # 0 is All, then one per subsystem that has history
    cursor: int = 0
    detail: bool = False         # narrow layout: the detail replaces the list
    base: str = 'current'        # current: what selecting would change; previous: what the change did
    plan: object = None
    message: Text = None
    rows: int = 14               # list rows that fit
    height: int = 30             # terminal rows; the body is fitted to it so the layout never clips
    scroll: int = 0              # first visible diff line (J/K), when the diff outruns the popup
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


def age(when, now):
    seconds = max(0, (now - when).total_seconds())
    for size, unit in ((86400, 'd'), (3600, 'h'), (60, 'min')):
        if seconds >= size:
            return f'{int(seconds // size)} {unit} ago'
    return 'just now'


def change_line(state, change, active, width, heads):
    """One list row: cursor, mark, id, time, name, and the right-hand tags."""
    when = datetime.datetime.fromisoformat(change['time'])
    mark = EXTERNAL if change['source'] == 'external' else PENDING if change.get('applied') == 'pending' else ' '
    right = Text()
    tag = scope_tag(change['subsystem'], rows_of(state, change), state.fields)
    if tag:
        right.append(tag, style=f"italic {theme.PALETTE.style('slate')}")
    if change['id'] in heads:
        right.append(' now' if tag else 'now', style=f'bold {theme.ACCENT}')
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


def plan_lines(plan, width):
    """The revert plan: its head, every changed key wrapped (never collapsed), blockers, the hold."""
    lines = [Text(f'Revert {plan.sub} to {plan.target["id"]}:', style='bold')]
    lines += [Text(f'  {row}', style='bold') for row in textwrap.wrap(', '.join(plan.keys) or 'nothing changes', max(10, width - 2))]
    lines += [Text(f'{theme.DASH} {b}', style=theme.BAD) for b in plan.blockers]
    if plan.hold:
        lines.append(Text('armed: the mux holds neutral during the apply', style='dim'))
    return lines


def detail_lines(state, width, avail=None):
    """The detail pane: what the change is, the revert plan or outcome, then the diff.

    The plan, blockers and message sit above the diff so a long diff cannot push them off the
    screen; with ``avail`` lines the diff is windowed (J/K) to what is left.
    """
    change = state.selected()
    if change is None:
        return []
    when = datetime.datetime.fromisoformat(change['time'])
    now = state.clock()
    name, _ = name_of(state, change)
    lines = [Text(f"{change['id']}{theme.SEPARATOR}{when:%a %b %d %H:%M}{theme.SEPARATOR}{age(when, now)}", style='bold'),
             Text(f"{name}{theme.SEPARATOR}{change['source']}{theme.SEPARATOR}{change.get('applied', '')}")]
    if change.get('note'):
        lines.append(Text(change['note'], style='dim'))
    if state.plan is not None:
        lines += plan_lines(state.plan, width)
    if state.message is not None:
        lines.append(state.message)
    head = state.history.head(change['subsystem'])
    # a plan always shows current -> target, whatever `d` says: that is the diff being applied
    if (state.base == 'current' or state.plan is not None) and head and head['id'] != change['id']:
        rows, vs = rows_of(state, change, head), f"vs current {head['id']}"
    else:
        rows, vs = rows_of(state, change), 'vs previous'
    lines.append(Text(vs, style='dim'))
    diff = render.render(rows, width, ascii_=theme.ASCII).split('\n')
    room = None if avail is None else max(2, avail - len(lines))
    if room is not None and len(diff) > room:
        start = state.scroll = max(0, min(state.scroll, len(diff) - (room - 1)))
        lines += diff[start:start + room - 1]
        lines.append(Text(f'{start + 1}-{start + room - 1} of {len(diff)}{theme.SEPARATOR}J/K scroll', style='dim'))
    else:
        lines += diff
    return lines


def tab_line(state, width):
    line = Text()
    for index, sub in enumerate(state.tabs()):
        label = 'All' if sub is None else LABELS[sub]
        if index == state.tab:
            line.append(f'[{label}]', style=f'bold {theme.ACCENT}')
        else:
            line.append(f' {label} ', style='dim')
        line.append(' ')
    line.truncate(width, overflow='ellipsis')
    return line


def footer(state, cols=120):
    if state.plan is not None:
        return theme.SEPARATOR.join(('y apply', 'Esc cancel'))
    if cols < WIDE:
        return theme.SEPARATOR.join(('Enter revert', 'n name', 'l detail', '[ ] tab', 'Esc'))
    keys = ['Enter revert', 'd vs ' + ('previous' if state.base == 'current' else 'current'), 'n name', 'N note']
    return theme.SEPARATOR.join(keys + ['l detail', '[ ] subsystem', 'Esc close'])


def body(state, cols):
    """The popup body for a ``cols``-column terminal: tabs, then list and detail side by side
    (or one at a time below ``WIDE`` columns)."""
    width = min(cols - 8, 92) - 4 if cols >= 80 else cols - 4
    out = [tab_line(state, width), Text('')]
    avail = max(6, state.height - 7)         # popup border, tab line, blank, hint, slack
    if cols >= WIDE:
        left = list_lines(state, LIST_W)
        right = detail_lines(state, width - LIST_W - 3, avail)
        for i in range(max(len(left), len(right))):
            cell = left[i] if i < len(left) else Text('')
            cell.truncate(LIST_W, overflow='ellipsis', pad=True)
            line = cell + Text(f' {theme.BAR} ', style='dim')
            if i < len(right):
                right[i].truncate(width - LIST_W - 3, overflow='ellipsis')
                line += right[i]
            out.append(line)
    else:
        out += detail_lines(state, width, avail) if state.detail or state.plan else list_lines(state, width)
        for line in out:
            line.truncate(width, overflow='ellipsis')
    return Text('\n').join(out)


def modal(state, cols):
    return dict(kind='custom', title='Config history', body=body(state, cols), width=min(cols - 8, 92) if cols >= 80 else cols,
                hint=footer(state, cols), danger=False)


def say(state, text, bad=False):
    state.message = Text(text, style=theme.BAD if bad else theme.GOOD)


def handle(state, key, app):
    """Apply one key; returns True when the popup should close."""
    change = state.selected()
    if key in ('escape', 'q') and state.plan is not None:
        state.plan = None
    elif key in ('escape', 'q'):
        return True
    elif key in ('[', 'left', ']', 'right'):
        state.tab = (state.tab + (1 if key in (']', 'right') else -1)) % len(state.tabs())
        state.cursor, state.plan, state.message, state.scroll = 0, None, None, 0
    elif key in ('up', 'k', 'down', 'j'):
        state.cursor = max(0, min(len(state.entries()) - 1, state.cursor + (1 if key in ('down', 'j') else -1)))
        state.plan, state.message, state.scroll = None, None, 0
    elif key in ('J', 'K'):
        state.scroll = max(0, state.scroll + (1 if key == 'J' else -1))
    elif key == 'l':
        state.detail = not state.detail
    elif key == 'd':
        state.base = 'previous' if state.base == 'current' else 'current'
    elif change is None:
        pass
    elif key == 'enter':
        state.message, state.scroll = None, 0
        if change['subsystem'] not in state.backends:
            say(state, f'cannot revert {change["subsystem"]}: not wired', bad=True)
            return False
        try:
            state.plan = revert.plan(state.history, state.backends, state.env, change['id'], change['subsystem'])
        except KeyError as error:
            say(state, f'cannot revert {change["subsystem"]}: {error.args[0]}', bad=True)
    elif key == 'y' and state.plan is not None:
        confirm(state, app)
    elif key in ('n', 'N'):
        field = 'name' if key == 'n' else 'note'
        text = app.ask_text(f'Name {change["id"]}' if key == 'n' else f'Note on {change["id"]}', '', change.get(field, ''))
        if text is not None:
            state.history.label(change['id'], **{field: text})
    return False


def confirm(state, app):
    """y: execute the planned revert unless a blocker or no-op stops it; show the outcome."""
    plan, state.plan = state.plan, None
    if not plan.ok:
        say(state, '; '.join(plan.blockers) or 'already at that version', bad=True)
        return
    out = revert.execute(state.history, state.backends, state.env, plan)
    say(state, f"Reverted {plan.sub} to {plan.target['id']}" if out.status == 'reverted' else f'{out.status}: {out.detail}',
        bad=out.status != 'reverted')
    app.notice = state.message.plain


def open_history(app, sub=None):
    """H: the history popup, optionally on one subsystem's tab."""
    state = State(app.history, getattr(app, 'config_backends', {}), app.revert_env(), fields=render.load_fields())
    if sub in state.tabs():
        state.tab = state.tabs().index(sub)
    shell = {}

    def redraw():
        size = app.console.size
        state.height, state.rows = size.height, max(6, size.height - 14)
        shell.update(modal(state, size.width))

    def on_key(key):
        done = handle(state, key, app)
        redraw()                      # the next frame shows this key's result, not the last one's
        return done or None

    redraw()
    return app.popup(shell, on_key, redraw)

"""Page layout: header, footer, popups and the arrangement of the panels.

``render(snapshot, view, width, height)`` draws the monitor snapshot and the app's view
state. Layout, after cx:

  header   name, profile, host; health strip of status icons
  body     Status (sensors, frames, system) | Motion, Thrusters | Procedures, Activity
  footer   armed state, focus, notice; key hints for the focused panel
"""

import io

from rich.console import Console, Group
from rich.layout import Layout
from rich.segment import Segment
from rich.style import Style
from rich.table import Table
from rich.text import Text

from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import TITLES
from mhseals_testing.ui.panels.activity import activity_panel
from mhseals_testing.ui.panels.motion import motion_panel
from mhseals_testing.ui.panels.procedures import procedures_panel
from mhseals_testing.ui.panels.status import status_panel, status_rows
from mhseals_testing.ui.panels.thrusters import thrusters_panel
from mhseals_testing.ui.theme import ACCENT, MUTED, SEPARATOR

PINNED_HINTS = ('Space', '?', 'q', 'x / Esc')  # footer keys that are never cut on narrow terminals
WIDE = 106          # below this the Motion panel folds away
SIDEBAR = 36
FOLD = 60           # below this the Status sidebar folds away so Thrusters keeps its width
TOP_ROW = 8         # Motion and Thrusters height


# --- header, footer, popups ------------------------------------------------------


def health_strip(snapshot, view, width=None):
    rows = status_rows(snapshot, view['specs'])
    sensors = [row for row in rows if row['group'] == 'Sensors' and row['required']]
    sensors_ok = all(row['state'] == 'LIVE' for row in sensors)
    frames_ok = all(row['state'] == 'LIVE' for row in rows if row['group'] == 'Frames')
    parts = [theme.labelled('LIVE' if view['recording'] else 'FAIL', 'rec'),
             theme.labelled('LIVE' if sensors_ok else 'FAIL',
                            f"sensors {sum(r['state'] == 'LIVE' for r in sensors)}/{len(sensors)}"),
             theme.labelled('LIVE' if frames_ok else 'FAIL', 'tf')]
    for row in rows:
        if row['group'] == 'System' and row['state'] != 'OK':
            parts.append(theme.labelled(row['state'], f"{row['name']} {row['value']}".strip()))
    if view['bench_override']:
        parts.append(theme.labelled('WARN', 'bench override'))
    return Text('   ').join(parts) if width is None else fit_strip(parts, width)


def fit_strip(chips, width):
    """Join the chips, replacing those that do not fit with a ``+N`` chip."""
    gap = Text('   ')
    for shown in range(len(chips), 0, -1):
        parts = chips[:shown] + ([Text(f'+{len(chips) - shown}', style=MUTED)] if shown < len(chips) else [])
        line = gap.join(parts)
        if len(line) + 1 <= width:
            return line
    return chips[0]


def arm_state(armed, detail=None):
    """Armed or disarmed, drawn like every other status: a glyph that differs per state and a label."""
    text = Text(theme.icon('warn' if armed else 'good').plain + ' ', style=theme.BAD if armed else theme.GOOD)
    text.append('Armed' if armed else 'Disarmed', style='bold')
    if detail:
        text.append(SEPARATOR + detail, style=MUTED)
    return text


def header(snapshot, view, width=120):
    """Name, profile and host over the health strip; the host's ROS domain goes first when narrow."""
    top = Table.grid(expand=True)
    top.add_column(no_wrap=True, overflow=theme.OVERFLOW)
    top.add_column(justify='right', no_wrap=True, overflow=theme.OVERFLOW)
    title = Text(' boat_test ', style=f'bold {ACCENT}')
    title.append(f" {view['profile']}", style='bold')
    arm = arm_state(view['armed'])
    room = width - len(title) - len(arm) - 4
    host = f"{view['host']}{SEPARATOR}ROS {view['domain']}"
    if len(host) > room:
        host = view['host']
    right = Text(f'{host}  ' if len(host) <= room else '', style=MUTED)
    right.append_text(arm)
    top.add_row(title, right)
    strip = Text(' ')
    strip.append_text(health_strip(snapshot, view, width - 1))
    return Group(top, strip)


def fit_hints(hints, capacity):
    """Cut the hints to ``capacity`` but never the PINNED keys, which keep the last cells."""
    if len(hints) <= capacity:
        return hints
    pinned = [hint for hint in hints if hint[0] in PINNED_HINTS]
    rest = [hint for hint in hints if hint[0] not in PINNED_HINTS]
    return rest[:max(capacity - len(pinned), 0)] + pinned[-capacity:]


def footer(view, width):
    status = Table.grid(expand=True, padding=(0, 1))
    status.add_column(ratio=3, no_wrap=True, overflow=theme.OVERFLOW)
    status.add_column(ratio=3, no_wrap=True, overflow=theme.OVERFLOW)
    status.add_column(ratio=4, justify='right', no_wrap=True, overflow=theme.OVERFLOW)
    position = view['position']
    focus = Text(f"Focus: {TITLES[view['focus']]}", style=f'bold {ACCENT}')
    if position:
        focus.append(f'{SEPARATOR}{position}', style=MUTED)
    status.add_row(arm_state(view['armed'], view['outputs'] if view['armed'] else None),
                   focus, Text(view['notice'], style=MUTED))
    hints = view['hints']
    columns = 4 if width >= WIDE else 3
    grid = Table.grid(expand=True)
    for _ in range(columns):
        grid.add_column(ratio=1, no_wrap=True, overflow=theme.OVERFLOW)
    hints = fit_hints(hints, columns * 2)
    for row in range(2):
        cells = [theme.hint(*hint) for hint in hints[row * columns:(row + 1) * columns]]
        grid.add_row(*(cells + [''] * (columns - len(cells))))
    return Group(status, grid)


def _height(renderable, width):
    """Rows ``renderable`` takes at ``width``."""
    console = Console(width=width, file=io.StringIO())
    return len(console.render_lines(renderable, console.options.update(height=None), pad=False))


class Window:
    """``rows`` lines of ``renderable`` starting at line ``offset``."""

    def __init__(self, renderable, offset, rows):
        self.renderable, self.offset, self.rows = renderable, offset, rows

    def __rich_console__(self, console, options):
        lines = console.render_lines(self.renderable, options.update(height=None), pad=False)
        for line in lines[self.offset:self.offset + self.rows]:
            yield from line
            yield Segment.line()


def popup(modal, width, height=None):
    """A centred dialog: message, list of choices, or a text field.

    Given the terminal ``height`` the dialog never outgrows it: the choices and the hint stay
    pinned and the message and rows scroll (``modal['scroll']``, clamped here).
    """
    head, tail = [], []
    if modal.get('message'):
        head.append(Text(modal['message']))
    if modal.get('rows'):
        rows = theme.grid({'header': '', 'style': f'bold {ACCENT}'},
                          {'header': '', 'no_wrap': False, 'overflow': 'fold', 'ratio': 1},
                          padding=(0, 2))
        rows.show_header = False
        for label, value in modal['rows']:
            rows.add_row(str(label), str(value))
        head.append(rows)
    if modal['kind'] == 'choice':
        if head:
            tail.append(Text(''))
        for index, option in enumerate(modal['options']):
            key = modal.get('keys', [None] * len(modal['options']))[index]
            line = Text(f'{key} ' if key else '', style=f'bold {ACCENT}')
            line.append_text(Text.from_markup(option) if '[' in option else Text(option))
            tail.append(theme.selected(line, index == modal['selected']))
    elif modal['kind'] == 'input':
        tail += [Text(''), Text('> ', style=f'bold {ACCENT}') + Text(modal['text'] + theme.FILL)]
    panel_width = min(width - 4, modal.get('width', 64))
    inner = panel_width - 4
    body = Group(*head)
    modal['clipped'] = False
    hint = modal.get('hint')
    if height is not None:
        total, pinned = _height(body, inner), _height(Group(*tail), inner)
        if hint:
            pinned += 1 + _height(Text(hint), inner)
        if total + pinned > height - 2:
            modal['clipped'] = True
            hint = f"{hint or ''}{SEPARATOR}{modal.get('scroll_hint', 'j/k')} scroll".lstrip(SEPARATOR)
            pinned = _height(Group(*tail), inner) + 1 + _height(Text(hint), inner)
            room = max(1, height - 2 - pinned)
            modal['scroll'] = max(0, min(modal.get('scroll', 0), total - room))
            body = Window(body, modal['scroll'], room)
    if hint:
        tail += [Text(''), Text(hint, style=MUTED)]
    panel = theme.block(Group(body, *tail), modal['title'], True)
    if modal.get('danger'):
        panel.border_style = theme.BAD
    return panel, panel_width


class Overlay:
    """Draw ``top`` centred over ``base``, dimming the base like a modal backdrop."""

    def __init__(self, base, top, top_width, width, height):
        self.base, self.top, self.top_width = base, top, top_width
        self.width, self.height = width, height

    def __rich_console__(self, console, options):
        options = options.update(width=self.width, height=self.height)
        lines = console.render_lines(self.base, options, pad=True)
        top = console.render_lines(self.top, options.update(width=self.top_width, height=None))
        dim = Style(dim=True)
        lines = [list(Segment.apply_style(line, post_style=dim)) for line in lines]
        x = (self.width - self.top_width) // 2
        y = max(0, (self.height - len(top)) // 2)
        for offset, row in enumerate(top[:self.height]):
            line = lines[y + offset]
            left, _, right = Segment.divide(line, [x, x + self.top_width, self.width])
            lines[y + offset] = left + row + right
        for line in lines:
            yield from line
            yield Segment.line()


# --- the page --------------------------------------------------------------------


MIN_SIZE = (36, 10)  # below this no panel is legible; only the stop hint is


def too_small(view):
    """What replaces the page in a terminal too small for it; Space still stops.

    The first line keeps the one fact the operator needs at any size: armed or not, and the trial.
    """
    state = 'ARMED' if view.get('armed') else 'disarmed'
    if view.get('running'):
        state += f"{SEPARATOR}{view['running']}"
    return Text(f'boat_test{SEPARATOR}{state}\nenlarge terminal{SEPARATOR}Space stop{SEPARATOR}q quit')


def render(snapshot, view, width=120, height=34):
    if width < MIN_SIZE[0] or height < MIN_SIZE[1]:
        return too_small(view)
    body_height = max(5, height - 5)
    layout = Layout()
    layout.split_column(Layout(header(snapshot, view, width), size=2), Layout(name='body'),
                        Layout(footer(view, width), size=3))
    body = layout['body']
    if width >= FOLD:
        body.split_row(Layout(status_panel(snapshot, view, body_height),
                              size=SIDEBAR if width >= WIDE else 30), Layout(name='main'))
    else:
        body.split_row(Layout(name='main'))  # Status folds away; the header still shows armed state
    main = body['main']
    lower = body_height - TOP_ROW
    if width >= WIDE:
        main.split_column(Layout(name='top', size=TOP_ROW), Layout(name='bottom'))
        main['top'].split_row(Layout(motion_panel(snapshot, view)),
                              Layout(thrusters_panel(view, width < 120)))
        main['bottom'].split_row(Layout(procedures_panel(view, lower), size=SIDEBAR),
                                 Layout(activity_panel(snapshot, view, lower)))
    else:
        half = max(4, (body_height - TOP_ROW) // 2)
        main.split_column(Layout(thrusters_panel(view, True, pins=False), size=TOP_ROW),
                          Layout(procedures_panel(view, half), size=half),
                          Layout(activity_panel(snapshot, view, body_height - TOP_ROW - half)))
    if view.get('modal'):
        panel, panel_width = popup(view['modal'], width, height)
        return Overlay(layout, panel, panel_width, width, height)
    return layout

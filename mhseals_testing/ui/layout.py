"""Page layout: header, footer, popups and the arrangement of the panels.

``render(snapshot, view, width, height)`` draws the monitor snapshot and the app's view
state. Layout, after cx:

  header   name, profile, host; health strip of status icons
  body     Status (sensors, frames, system) | Motion, Thrusters | Procedures, Activity
  footer   armed state, focus, notice; key hints for the focused panel

The pure decisions (ui.md section 4.5, U13) need no Rich: ``tier`` maps a terminal width to a
density tier, ``fold_groups`` (from ui.panels) picks which quiet Status groups collapse when the
column is short, and ``TabMemory`` keeps the focused panel and every panel's cursor per tab so
switching tabs returns to where the operator was.
"""

import io

from rich.console import Console, Group
from rich.layout import Layout
from rich.segment import Segment
from rich.style import Style
from rich.table import Table
from rich.text import Text

from mhseals_testing.harness import recwatch
from mhseals_testing.ui import chrome, theme
from mhseals_testing.ui.panels import scrubber
from mhseals_testing.ui.panels import TITLES, fold_groups  # noqa: F401  (layout.fold_groups)
from mhseals_testing.ui.panels.activity import activity_panel
from mhseals_testing.ui.panels import correction
from mhseals_testing.ui.panels import bags as bags_panel
from mhseals_testing.ui.panels.minimap import WIDTH as MINIMAP_WIDTH, minimap_panel
from mhseals_testing.ui.panels.motion import motion_panel
from mhseals_testing.ui.panels.procedures import procedures_panel
from mhseals_testing.ui.panels.status import status_panel, status_rows
from mhseals_testing.ui.panels.thrusters import thrusters_panel
from mhseals_testing.ui.theme import ACCENT, MUTED, SEPARATOR

PINNED_HINTS = ('Space', '?', 'q', 'x / Esc')  # footer keys that are never cut on narrow terminals
WIDE = 106          # below this the Motion panel folds away
SIDEBAR = 36
MINIMAP = 150       # from this width Drive gains the minimap column (ui U6)
FOLD = 60           # below this the Status sidebar folds away so Thrusters keeps its width
TOP_ROW = 10        # Motion and Thrusters height: three one-row axes with headers, two drift lines
STRIP_FULL, STRIP_SHORT = 100, 5   # below 100 columns the strip shows its first five items


# --- header, footer, popups ------------------------------------------------------


def strip_item(state, label):
    """A healthy item is plain muted text; anything else gets its status glyph and hue."""
    if theme.kind(state) == 'good':
        return Text(label, style=MUTED)
    return theme.labelled(state, label)


def rec_chip(view):
    """The recorder item: plain muted text when fine, a glyph only when it needs attention."""
    sample = view.get('rec')
    if sample is None:                           # no watcher (tests, headless): running or not
        return strip_item('LIVE' if view['recording'] else 'FAIL', 'rec')
    state, text = recwatch.strip_text(sample, view['armed'], SEPARATOR, theme.pick('≈', '~'))
    if state in ('warn', 'fail'):
        return theme.labelled('WARN' if state == 'warn' else 'FAIL', text)
    return Text(text, style=MUTED if state == 'ok' else 'dim')


def health_strip(snapshot, view, width=None):
    rows = status_rows(snapshot, view['specs'])
    sensors = [row for row in rows if row.get('source') == 'sensor' and row['required']]
    sensors_ok = all(row['state'] == 'LIVE' for row in sensors)
    frames_ok = all(row['state'] == 'LIVE' for row in rows if row.get('source') == 'tf')
    parts = [rec_chip(view),
             strip_item('LIVE' if sensors_ok else 'FAIL',
                        f"sensors {sum(r['state'] == 'LIVE' for r in sensors)}/{len(sensors)}"),
             strip_item('LIVE' if frames_ok else 'FAIL', 'tf')]
    for row in rows:
        if row['group'] == 'System' and row['state'] != 'OK':
            parts.append(strip_item(row['state'], f"{row['name']} {row['value']}".strip()))
    if view['bench_override']:
        parts.append(theme.labelled('WARN', 'bench override'))
    if width is None:
        return Text(SEPARATOR).join(parts)
    cut = 0
    if width < STRIP_FULL:  # only the first five items show below 100 columns; the rest count in +N
        parts, cut = parts[:STRIP_SHORT], max(len(parts) - STRIP_SHORT, 0)
    return fit_strip(parts, width, cut)


def health_total(health):
    """The right end of the strip: ``health 41 ok`` or the fail and warn counts and the worst check."""
    if not health:
        return Text('')
    if not health.get('bad') and not health.get('warn'):
        return Text(f"health {health.get('ok', 0)} ok", style=MUTED)
    total = Text()
    for state, count in (('bad', health.get('bad')), ('warn', health.get('warn'))):
        if count:
            total.append_text(theme.icon(state)).append(f' {count}  ')
    if health.get('worst'):
        total.rstrip()
        total.append(SEPARATOR + health['worst'], style=MUTED)
    return total


def fit_strip(chips, width, extra=0):
    """Join the chips, replacing those that do not fit (and ``extra`` already cut) with a ``+N`` chip."""
    gap = Text(SEPARATOR)
    for shown in range(len(chips), 0, -1):
        hidden = len(chips) - shown + extra
        parts = chips[:shown] + ([Text(f'+{hidden}', style=MUTED)] if hidden else [])
        line = gap.join(parts)
        if len(line) + 1 <= width:
            return line
    return chips[0]


def header(snapshot, view, width=120):
    """Row 1: title, profile, deploy, host and the authority and arm chips; row 2: the health strip or agent banner."""
    top = chrome.top_row(view, width)
    if view.get('agent'):
        return Group(top, chrome.banner(view['agent'], width))
    total = health_total(view.get('health'))
    row = Table.grid(expand=True)
    row.add_column(no_wrap=True, overflow=theme.OVERFLOW)
    row.add_column(justify='right', no_wrap=True)
    row.add_row(Text(' ') + health_strip(snapshot, view, width - len(total) - 3), total + Text(' '))
    return Group(top, row)


def fit_hints(hints, capacity):
    """Cut the hints to ``capacity`` but never the PINNED keys, which keep the last cells."""
    if len(hints) <= capacity:
        return hints
    pinned = [hint for hint in hints if hint[0] in PINNED_HINTS]
    rest = [hint for hint in hints if hint[0] not in PINNED_HINTS]
    return rest[:max(capacity - len(pinned), 0)] + pinned[-capacity:]


def playing(view):
    """The ui.review.Review being played, or None (``review: True`` only asks for the chip)."""
    review = view.get('review')
    return review if review and not isinstance(review, bool) else None


def footer(view, width):
    status = Table.grid(expand=True, padding=(0, 1))
    status.add_column(ratio=4, no_wrap=True, overflow=theme.OVERFLOW)
    status.add_column(ratio=6, justify='right', no_wrap=True, overflow=theme.OVERFLOW)
    position = view['position']
    title, position = view.get('focus_title') or (TITLES.get(view['focus'], view['focus'].title()), position)
    focus = Text(f' Focus {title}', style=f'bold {ACCENT}')
    if position:
        focus.append(f'{SEPARATOR}{position}', style=MUTED)
    status.add_row(focus, Text(view['notice'], style=MUTED))
    review = playing(view)
    if review:  # the strip footer takes the status line's place; the notice rides on its status line
        status = Group(*scrubber.review_footer(review, width, view['notice']))
    hints = view['hints']
    columns = 4 if width >= WIDE else 3 if width >= 60 else 2
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
    if modal.get('body') is not None:       # a popup that draws itself (popup.history)
        head.append(modal['body'])
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
        if total + pinned > height - 2:
            modal['clipped'] = True
            hint = f"{modal.get('scroll_hint', 'j/k')} scroll{SEPARATOR}{hint or ''}".rstrip(SEPARATOR)  # first: the border may crop
            room = max(1, height - 2 - pinned)
            modal['scroll'] = max(0, min(modal.get('scroll', 0), total - room))
            body = Window(body, modal['scroll'], room)
    panel = theme.block(Group(body, *tail), modal['title'], True, keys=hint)  # keys sit on the bottom border
    if modal.get('danger'):
        panel.border_style = theme.BAD
    return panel, panel_width


TOAST_STYLE = {'warn': theme.WARN, 'fail': theme.BAD, 'ok': theme.GOOD, 'info': ACCENT}


TOAST_MIN, TOAST_MAX, TOAST_LINES = 26, 60, 2   # columns; text rows (plus 2 border rows = 4)
FADE = 1.0                                      # seconds a finite toast is dimmed before it expires


def toast_box(toast, width, now=None):
    """One toast as a bordered box at most 4 rows tall, titled in its kind's hue.

    Returns the panel and its height. Text beyond two rows is cut with an ellipsis.
    """
    hue = TOAST_STYLE[toast.kind]
    text = Text(toast.text + (f'{theme.pick(" ×", " x")}{toast.count}' if toast.count > 1 else ''))
    lines = text.wrap(Console(width=width - 4), width - 4)
    for line in lines:
        line.rstrip()
    if len(lines) > TOAST_LINES:
        last = lines[TOAST_LINES - 1]
        ellipsis = theme.pick('…', '...')
        last.truncate(width - 4 - len(ellipsis))      # the box interior is width - 4 columns
        last.append(ellipsis)
        lines = lines[:TOAST_LINES]
    panel = theme.block(Text('\n').join(lines), toast.title.strip())
    panel.title = Text(toast.title.strip(), style=f'bold {hue}')
    panel.border_style = hue
    if now is not None and toast.expires is not None and toast.expires - now <= FADE:
        panel.style = 'dim'
    return panel, len(lines) + 2


def toast_stack(toasts, more, width, now=None, rows=None):
    """The toasts as bordered boxes, oldest on top, then a dim `+N more` line.

    Width follows the longest text within 26-60 columns and the terminal. With ``rows`` the stack
    is clipped to that many rows by dropping the oldest boxes (counted into `more`).
    """
    want = max((len(t.text) + 8 for t in toasts), default=TOAST_MIN)
    panel_width = max(8, min(max(TOAST_MIN, min(want, TOAST_MAX)), width - 2))
    boxes = [toast_box(t, panel_width, now) for t in toasts]
    while rows is not None and boxes and sum(h for _, h in boxes) + bool(more) > rows:
        boxes.pop(0)
        more += 1
    items = [box for box, _ in boxes]
    if more and boxes:
        items.append(Text(f'+{more} more{SEPARATOR}!', style=MUTED, justify='right'))
    return Group(*items), panel_width


class Overlay:
    """Draw ``top`` centred over ``base``, dimming the base like a modal backdrop.

    ``anchor='bottom_right'`` puts it in the body's corner above the footer instead, undimmed.
    """

    def __init__(self, base, top, top_width, width, height, anchor='center'):
        self.base, self.top, self.top_width, self.anchor = base, top, top_width, anchor
        self.width, self.height = width, height

    def __rich_console__(self, console, options):
        options = options.update(width=self.width, height=self.height)
        lines = console.render_lines(self.base, options, pad=True)
        top = console.render_lines(self.top, options.update(width=self.top_width, height=None))
        if self.anchor == 'bottom_right':
            x = self.width - self.top_width - 1
            top = top[max(0, len(top) - (self.height - HEADER_ROWS - FOOTER_ROWS)):]
            y = max(HEADER_ROWS, self.height - FOOTER_ROWS - len(top))
        else:
            dim = Style(dim=True)
            lines = [list(Segment.apply_style(line, post_style=dim)) for line in lines]
            x = (self.width - self.top_width) // 2
            y = max(0, (self.height - len(top)) // 2)
        for offset, row in enumerate(top[:self.height - y]):
            line = lines[y + offset]
            left, _, right = Segment.divide(line, [x, x + self.top_width, self.width])
            lines[y + offset] = left + row + right
        for line in lines:
            yield from line
            yield Segment.line()


# --- the page --------------------------------------------------------------------


HEADER_ROWS = 2
FOOTER_ROWS = 3
REVIEW_ROWS = 3   # events, skyline and axis join the status line
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
    extra = REVIEW_ROWS if playing(view) else 0   # strip footer rows below the focus line
    body_height = max(5, height - 5 - extra)
    layout = Layout()
    layout.split_column(Layout(header(snapshot, view, width), size=2), Layout(name='body'),
                        Layout(footer(view, width), size=3 + extra))
    body = layout['body']
    build = BODIES.get(view.get('tab', TABS[0]), placeholder_body)
    body.update(build(snapshot, view, width, body_height))
    return _with_modal(layout, view, width, height)


def placeholder_body(snapshot, view, width, height):
    """The body of a tab nobody has built yet."""
    return theme.block(Text(f"{view['tab']} is not built yet", style=MUTED), view['tab'], True)


def _cells(slots, sizes=None):
    """Layouts for the slots that hold a panel, in order; an empty slot (None) takes no space."""
    sizes = sizes or {}
    return [Layout(panel, size=sizes.get(name)) for name, panel in slots if panel is not None]


def _drive_dims(width, height):
    """``(wide, lower, half)``: the wide tier, the rows under the top row, and half of them (min 4)."""
    lower = height - TOP_ROW
    return width >= WIDE, lower, max(4, lower // 2)


def drive_slots(snapshot, view, width, height):
    """The Drive panels by named slot for the tier of ``width`` (ui.md 4.5); None leaves a slot empty.

    A later item fills a slot (``minimap``, ``conditions``) by returning a panel here.
    """
    wide, lower, half = _drive_dims(width, height)
    return {
        'status': status_panel(snapshot, view, height) if width >= FOLD else None,
        'motion': motion_panel(snapshot, view, TOP_ROW) if wide else None,
        'conditions': None,
        'thrusters': thrusters_panel(view, width < 120) if wide else thrusters_panel(view, True, pins=False),
        'procedures': procedures_panel(view, lower if wide else half),
        'activity': activity_panel(snapshot, view, lower if wide else height - TOP_ROW - half),
        'minimap': minimap_panel(view, TOP_ROW) if width >= MINIMAP else None,
    }


def drive_body(snapshot, view, width, height):
    """Status column beside the main area: Motion and the right column over Procedures and Activity."""
    slots = drive_slots(snapshot, view, width, height)
    wide, _, half = _drive_dims(width, height)
    main = Layout()
    if wide:
        top = Layout(size=TOP_ROW)
        right = Layout()   # Conditions over Thrusters; an empty slot takes no rows
        right.split_column(*_cells([('conditions', slots['conditions']), ('thrusters', slots['thrusters'])]))
        top.split_row(*_cells([('motion', slots['motion']), ('right', right), ('minimap', slots['minimap'])],
                              {'minimap': MINIMAP_WIDTH}))
        bottom = Layout()
        bottom.split_row(*_cells([('procedures', slots['procedures']), ('activity', slots['activity'])],
                                 {'procedures': SIDEBAR}))
        main.split_column(top, bottom)
    else:
        main.split_column(*_cells([('thrusters', slots['thrusters']), ('procedures', slots['procedures']),
                                   ('activity', slots['activity'])],
                                  {'thrusters': TOP_ROW, 'procedures': half}))
    page = Layout()
    page.split_row(*_cells([('status', slots['status'])], {'status': status_width(width) or 30}), main)
    return page


# Tab -> body function (snapshot, view, width, height) -> renderable. A tab not listed shows the
# placeholder; building a tab is registering its body here.
def correction_body(snapshot, view, width, height):
    """The Correction tab: the table (full width below 110 columns) and its side column."""
    model = correction.tab_view(view.get('hardware') or {}, view.get('correction'), view.get('running'))
    return correction.render(model, width, height)


LAYERS_ROWS = 6   # the Layers box: four interior rows and its border


def map_body(snapshot, view, width, height):
    """The Map tab: the minimap over the full body, with the Layers box under it when there is room."""
    box = view.get('layerbox')
    rows = LAYERS_ROWS if box is not None and height > LAYERS_ROWS + 8 else 0
    page = Layout()
    cells = [Layout(minimap_panel(view, height - rows, width))]
    if rows:
        cells.append(Layout(box.render(view['focus'] == 'map'), size=rows))
    page.split_column(*cells)
    return page


def bags_body(snapshot, view, width, height):
    """The Bags tab: the session list over the selected session's summary."""
    bags = view.get('bags') or []
    cursor = min(view['cursors'].get('list', 0), max(0, len(bags) - 1))
    page = Layout()
    page.split_column(
        Layout(bags_panel.render(bags, view.get('bag_usage') or (0, 0, 0), cursor, root=view.get('bag_root', ''),
                                 focused=view['focus'] == 'list')),
        Layout(bags_panel.summary(bags[cursor] if bags else None), size=5))
    return page


BODIES = {'Drive': drive_body, 'Correction': correction_body, 'Map': map_body, 'Bags': bags_body}


def _with_modal(layout, view, width, height):
    if view.get('modal'):
        panel, panel_width = popup(view['modal'], width, height)
        return Overlay(layout, panel, panel_width, width, height)
    toasts = view.get('toasts')
    if toasts:
        stack, stack_width = toast_stack(toasts, view.get('toasts_more', 0), width, view.get('now'),
                                         height - HEADER_ROWS - FOOTER_ROWS)
        return Overlay(layout, stack, stack_width, width, height, anchor='bottom_right')
    return layout


# --- density tiers and per-tab memory (no Rich) -----------------------------------

TABS = ('Drive', 'Correction', 'Map', 'Bags')
DEFAULT_FOCUS = {'Drive': 'status', 'Correction': 'table', 'Map': 'map', 'Bags': 'list'}

# (minimum width, tier name, Status column width on Drive); Status is hidden in 'single'.
TIERS = ((150, 'wide', 34), (110, 'standard', 34), (90, 'compact', 28),
         (70, 'narrow', 30), (0, 'single', 0))


def tier(width):
    """Name of the density tier for a terminal ``width``."""
    return next(name for floor, name, _ in TIERS if width >= floor)


def status_width(width):
    """Columns the Status panel takes on Drive at ``width`` (0 when it folds away)."""
    return next(columns for floor, _, columns in TIERS if width >= floor)


class TabMemory:
    """Focus and cursors remembered for each tab."""

    def __init__(self, tabs=TABS):
        self.tabs = tuple(tabs)
        self.current = self.tabs[0]
        self._saved = {}

    def switch(self, step, focus, cursors):
        """Save ``focus`` and ``cursors`` for the current tab, move ``step`` tabs on.

        Returns the new tab's remembered ``(focus, cursors)``; a tab seen for the first time gets
        its default focus and zero cursors.
        """
        self._saved[self.current] = (focus, dict(cursors))
        self.current = self.tabs[(self.tabs.index(self.current) + step) % len(self.tabs)]
        focus, cursors = self._saved.get(self.current, (DEFAULT_FOCUS[self.current], {}))
        return focus, dict(cursors)

"""Popups, toasts and the overlay that lays a modal or the toast stack over the page."""

import io

from rich.console import Console, Group
from rich.segment import Segment
from rich.style import Style
from rich.text import Text

from mhseals_testing.ui import theme, widgets
from mhseals_testing.ui.layout.density import FOOTER_ROWS, HEADER_ROWS


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


def popup_rows(rows, width, quiet=False):
    """Label/value grids; a ``(widgets.SECTION, title)`` row starts a new group under a rule.

    ``quiet`` (the arm panel): labels muted and right-aligned, values in fg, rules in bold slate, a
    leading ✓/✗ in its state colour and the Checks rule counting what blocks."""
    out, grid = [], None
    labels = [str(label) for label, _ in rows if label != widgets.SECTION]
    # groups under rules share one label width so their values line up
    column = max(map(len, labels), default=0) if len(labels) < len(rows) else None
    good, bad = theme.icon('good').plain, theme.icon('bad').plain
    for index, (label, value) in enumerate(rows):
        if label == widgets.SECTION:
            note = hue = None
            if quiet and value == 'Checks':
                after = []
                for later, item in rows[index + 1:]:
                    if later == widgets.SECTION:
                        break
                    after.append(str(item))
                blocking = sum(item.startswith(bad + ' ') for item in after)
                note, hue = (f'{blocking} blocking', theme.BAD) if blocking else ('all clear', theme.GOOD)
            out.append(widgets.rule(str(value), width, note, hue, strong=quiet))
            grid = None
            continue
        if grid is None:
            label_style = theme.MUTED if quiet else f'bold {theme.ACCENT}'
            grid = theme.grid({'header': '', 'style': label_style, 'min_width': column,
                               'justify': 'right' if quiet else 'left'},
                              {'header': '', 'no_wrap': False, 'overflow': 'fold', 'ratio': 1},
                              padding=(0, 2))
            grid.show_header = False
            out.append(grid)
        text = str(value)
        if isinstance(value, Text):
            grid.add_row(str(label), value)
        elif quiet:
            cell = Text(style=theme.PALETTE.style('fg'))
            for glyph, state in ((good, 'good'), (bad, 'bad')):
                if text.startswith(glyph + ' '):
                    cell.append_text(theme.icon(state))
                    text = text[len(glyph):]
                    break
            cell.append(text)
            grid.add_row(str(label), cell)
        else:
            grid.add_row(str(label), text)
    return out


def popup(modal, width, height=None):
    """A centred dialog: message, list of choices, or a text field.

    Given the terminal ``height`` the dialog never outgrows it: the choices and the hint stay
    pinned and the message and rows scroll (``modal['scroll']``, clamped here).
    """
    head, tail = [], []
    quiet = modal.get('quiet', False)
    if modal.get('message') and quiet:          # the arm panel's caution line
        caution = Text(theme.pick('▲ ', '! '), style=f'bold {theme.WARN}')
        caution.append(modal['message'], style=theme.WARN)
        head += [caution, Text('')]
    elif modal.get('message'):
        head.append(Text(modal['message']))
    if modal.get('body') is not None:       # a popup that draws itself (popup.history)
        head.append(modal['body'])
    panel_width = min(width - 4, modal.get('width', 64))
    inner = panel_width - 4
    if modal.get('rows'):
        head += popup_rows(modal['rows'], inner, quiet)
    if quiet:
        tail.append(Text(''))
    hint = modal.get('hint')
    if modal['kind'] == 'choice':
        if head and not quiet:
            tail.append(Text(''))
        options = modal['options']
        lines = widgets.choice_list(options, modal.get('keys', [None] * len(options)), modal['selected'],
                                    modal.get('risky', ()), inner)
        if len(lines) == 1 and len(options) > 1 and hint:   # one row of buttons: arrows go sideways
            hint = hint.replace(theme.pick('↑↓', 'Up/Down'), theme.pick('←→', 'Left/Right'))
        tail += lines
        if modal.get('countdown'):
            tail.append(widgets.countdown(*modal['countdown']))
    elif modal['kind'] == 'input':
        word = modal.get('word')
        if word:
            tail.append(widgets.word_field(modal['text'], word, modal.get('word_hue')))
            if modal['text'] == word:
                hint = f'Enter {word.lower()}{theme.SEPARATOR}Esc cancel'
        else:
            tail.append(widgets.field(modal['text'], width=modal.get('field_width', inner),
                                      placeholder=modal.get('placeholder', ''),
                                      state=modal.get('state', 'idle'), note=modal.get('note', ''),
                                      suffix=modal.get('suffix', '')))
    body = Group(*head)
    modal['clipped'] = False
    if height is not None:
        total, pinned = _height(body, inner), _height(Group(*tail), inner)
        if total + pinned > height - 2:
            modal['clipped'] = True
            # first: the border may crop
            hint = f"{modal.get('scroll_hint', 'j/k')} scroll{theme.SEPARATOR}{hint or ''}"
            hint = hint.rstrip(theme.SEPARATOR)
            room = max(1, height - 2 - pinned)
            modal['scroll'] = max(0, min(modal.get('scroll', 0), total - room))
            body = Window(body, modal['scroll'], room)
    panel = theme.block(Group(body, *tail), modal['title'], True, keys=hint, bar=False)  # keys sit on the bottom border
    if quiet:                                   # the title in the danger hue, the profile muted
        words = str(modal['title']).split(theme.SEPARATOR, 1)
        panel.title = Text.assemble((f'{theme.FOCUS} {words[0]}', f'bold {theme.BAD}'.strip()),
                                    *([(theme.SEPARATOR + words[1], theme.MUTED)] if len(words) > 1 else []))
    if modal.get('danger'):
        panel.border_style = theme.BAD
    elif modal.get('border'):
        panel.border_style = modal['border']
    return panel, panel_width


TOAST_STYLE = {'warn': 'WARN', 'fail': 'BAD', 'ok': 'GOOD', 'info': 'ACCENT'}   # theme names


TOAST_MIN, TOAST_MAX, TOAST_LINES = 26, 60, 2   # columns; text rows (plus 2 border rows = 4)
FADE = 1.0                                      # seconds a finite toast is dimmed before it expires


def toast_box(toast, width, now=None):
    """One toast as a bordered box at most 4 rows tall, titled in its kind's hue.

    Returns the panel and its height. Text beyond two rows is cut with an ellipsis.
    """
    hue = getattr(theme, TOAST_STYLE[toast.kind])
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
        items.append(Text(f'+{more} more{theme.SEPARATOR}!', style=theme.MUTED, justify='right'))
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

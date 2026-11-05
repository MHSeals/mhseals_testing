"""The Labels tab: the water conditions, a session timeline and the review list of marks and segments.

Pure rendering of ``labels_model`` (ui/app/actions/annotate.py feeds it); keys belong to the app.
The timeline has three lanes over the session: runs (procedures over armed time), windows (calibration
candidates, solid when usable) and marks (▾ mark, ━ bad data in red, ! intervention, ❝ said, ⌁ link hold,
⊘ fence exit). The selected item is drawn in reverse in its lane, with a ↑ on the axis under it.
Map-object glyphs stay off this tab (ui.md 1, test_ui_grammar).
"""

from datetime import datetime

from rich.console import Group
from rich.table import Table
from rich.text import Text

from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import window

SCALE_WORDS = ('unset', 'calm', 'moderate', 'rough')
GLYPHS = {'mark': ('▾', 'v'), 'bad': ('✕', 'x'), 'intervention': ('!', '!'), 'said': ('❝', '"'),
          'link': ('⌁', '~'), 'fence': ('⊘', 'o'), 'procedure': ('▸', '>'), 'steady': ('▬', '='),
          'neutral': ('▭', 'o')}
KIND_WORDS = {'bad': 'bad data', 'link': 'link hold', 'fence': 'fence exit', 'said': 'said',
              'steady': 'steady', 'neutral': 'neutral'}
LANES = ('runs', 'windows', 'marks')
SIDE = 100              # from this width the conditions sit beside the timeline
CONDITIONS = 30         # their column there
LANE_WIDTH = 7


def glyph(kind):
    return theme.pick(*GLYPHS.get(kind, ('·', '.')))


def quality_style(quality):
    return {'ok': theme.GOOD, 'degraded': theme.WARN, 'suspect': theme.WARN, 'invalid': theme.BAD}.get(quality, theme.MUTED)


def status_cell(status):
    if status == 'accepted':
        return theme.icon('good')
    if status == 'rejected':
        return theme.icon('bad')
    return Text(theme.pick('·', '.'), style=theme.MUTED)


def scale(name, value, active, compact=False):
    """``› Wind     ▰▰▱  moderate``; compact: ``› Wind ▰▰▱``."""
    on, off = theme.pick('▰', '#'), theme.pick('▱', '-')
    text = Text(theme.CURSOR if active else '  ', style=f'bold {theme.ACCENT}' if active else None)
    text.append(f'{name.title()} ' if compact else f'{name.title():<9}', style=f'bold {theme.ACCENT}' if active else None)
    text.append(on * value, style=theme.ACCENT)
    text.append(off * (3 - value), style=theme.MUTED)
    if compact:
        return text + Text('  ')
    text.append(f'  {SCALE_WORDS[value]}', style=theme.MUTED if value == 0 else None)
    if active:
        text.append(f"  {theme.pick('←→', '<>')} set", style=theme.MUTED)
    return text


def lanes(model, width):
    """The three timeline lanes, each ``width`` cells over [t0, t1] (the selected item in reverse), and the axis
    with the session's start and end and a ↑ under the selected item."""
    t0, t1 = model['t0'], max(model['t1'], model['t0'] + 1.0)
    cells = {lane: [(' ', None)] * width for lane in LANES}
    selected = (model.get('selected') or {}).get('id')

    def at(t):
        return max(0, min(width - 1, round((t - t0) / (t1 - t0) * (width - 1))))

    def paint(lane, item, char, style):
        if item.get('id') == selected:
            style = f'reverse bold {theme.ACCENT}'
        for k in range(at(item['t0']), at(item['t1']) + 1):
            cells[lane][k] = (char, style)

    for seg in model['armed']:
        paint('runs', seg, theme.RULE, theme.MUTED)
    for item in model['rows']:
        kind, muted = item['kind'], item['status'] == 'rejected'
        if kind == 'procedure':
            paint('runs', item, theme.pick('━', '='), theme.MUTED if muted else theme.ACCENT)
        elif kind in ('steady', 'neutral'):
            usable = item['id'] in model['calibration'] and not muted
            paint('windows', item, theme.pick('█', '#') if usable else glyph('steady'),
                  theme.GOOD if usable else theme.MUTED)
        elif kind == 'bad':
            paint('marks', item, theme.pick('━', '='), theme.MUTED if muted else theme.BAD)
        else:
            style = theme.MUTED if muted else theme.BAD if kind in ('intervention', 'link') else theme.ACCENT
            paint('marks', dict(item, t1=item['t0']), glyph(kind), style)
    out = []
    for lane, name in zip(LANES, ('runs', 'calib', 'marks')):
        text = Text(f'{name:<{LANE_WIDTH}}', style=theme.MUTED)
        for char, style in cells[lane]:
            text.append(char, style=style)
        out.append(text)
    axis = [' '] * width                      # the start and end clocks at the lane edges, ↑ at the selection
    mark = at(model['selected']['t0']) if model.get('selected') else None
    for start, word in ((0, clock(t0)), (width - len(clock(t1)), clock(t1))):
        if mark is None or not start <= mark < start + len(word):       # a clock under the ↑ gives way
            axis[start:start + len(word)] = word
    text = Text(' ' * LANE_WIDTH, style=theme.MUTED)
    for k, char in enumerate(axis):
        text.append(theme.pick('↑', '^') if k == mark else char,
                    style=f'bold {theme.ACCENT}' if k == mark else theme.MUTED)
    out.append(text)
    return out


def clock(t):
    return datetime.fromtimestamp(t).strftime('%H:%M:%S') if t else '--:--:--'


def duration(item):
    seconds = item['t1'] - item['t0']
    if seconds <= 0:
        return theme.DASH
    return f'{seconds:.0f}s' if seconds < 120 else f'{seconds / 60:.0f}m'


def what(item):
    """``▾ wake``: the kind's glyph and the label, else the kind's word."""
    if item['kind'] == 'said' and not item['label']:
        return Text(f"{glyph('said')} words later (n)", style=theme.MUTED)
    return Text(f"{glyph(item['kind'])} {item['label'] or KIND_WORDS.get(item['kind'], item['kind'])}")


def result(item):
    """Quality of a window, the effect of a mark or span, the outcome of a said item."""
    if item['kind'] == 'said':
        outcome = item.get('outcome')
        return Text(f"{theme.ARROW} {outcome or '?'}", style=None if outcome else theme.MUTED)
    if item['kind'] in ('procedure', 'steady', 'neutral'):
        return Text(item.get('quality', ''), style=quality_style(item.get('quality')))
    effect = item.get('effect', 'none')
    return Text('' if effect == 'none' else effect, style=quality_style(effect))


def item_table(model, rows):
    table = theme.grid('', 'Time', 'What', dict(header='Dur', justify='right'), 'Result', 'Note', padding=(0, 1))
    for index, item in rows:
        active = index == model['cursor']
        lead = Text(theme.CURSOR if active else '  ', style=f'bold {theme.ACCENT}') + status_cell(item['status'])
        flags = ', '.join(item.get('flags') or ())
        note = item.get('note') or (f'by {flags}' if flags else '')
        table.add_row(lead, clock(item['t0']), what(item), duration(item), result(item), note,
                      style=f'bold {theme.ACCENT}' if active else (theme.MUTED if item['status'] == 'rejected' else None))
    return table


def render(model, width, height, focused=True):
    """The whole tab body in one rounded panel; below SIDE columns the conditions take one row over the timeline."""
    scales = model['scales']
    side = width >= SIDE
    lane_width = max(10, width - 4 - LANE_WIDTH - (CONDITIONS + 2 if side else 0))
    timeline = lanes(model, lane_width)
    rows = [scale(name, model['conditions'].get(name, 0), model['cursor'] == k, not side) for k, name in enumerate(scales)]
    if side:
        head = Table.grid(expand=True, padding=(0, 2))
        head.add_column(width=CONDITIONS, no_wrap=True)
        head.add_column(ratio=1, no_wrap=True, overflow=theme.OVERFLOW)
        for a, b in zip(rows + [Text('')] * (len(timeline) - len(rows)), timeline):
            head.add_row(a, b)
    else:
        head = Group(Text('').join(rows), *timeline)
    first = len(scales)
    items = list(enumerate(model['rows'], first))
    room = max(1, height - 2 - len(timeline) - (0 if side else 1) - 1 - 1)   # borders, head, summary, table header
    _, shown = window(items, max(0, model['cursor'] - first), room)
    body = item_table(model, shown) if items else Text('No marks or segments yet: b marks, N bad data, " said',
                                                       style=theme.MUTED)
    sep = theme.SEPARATOR
    summary = Text(f"{model['pending']} to review{sep}{plural(len(model['calibration']), 'calibration window')}"
                   f"{sep}{len(model['commands'])} said", style=theme.MUTED)
    if model['readonly']:
        summary.append(f'{sep}edits after disarm', style=theme.WARN)
    title = f"Labels{sep}{'live' if model['live'] else 'closed'}"
    return theme.block(Group(head, summary, body), title, focused, subtitle=model['session'])


def plural(n, word):
    return f'{n} {word}' + ('' if n == 1 else 's')

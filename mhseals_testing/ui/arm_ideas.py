"""Prototype presentations of the arm panel (``a``) for the gallery; not wired in.

Variant 0 is today's: the modal dicts App.arm_panel builds, through layout.popup. Variants 1-3 keep
the structure and every safety element (typed ARM, Cancel default, the refusal rows, the danger
treatment) and change only colour, alignment, spacing and grouping, still through layout.popup.
"""

from rich.align import Align
from rich.console import Group
from rich.table import Table
from rich.text import Text

from mhseals_testing.ui import layout, theme, widgets
from mhseals_testing.ui.popups import choice_modal

STATES = ('typed', 'blocked', 'armed')
WIDTH = 72
MESSAGE = 'Propellers submerged, area clear, physical power cutoff within reach.'
BLOCKERS = ['FCU heading is unreliable (compass variance 0.41 rad, limit 0.20)',
            'another instance boat_test@jetson is live']


def facts(state):
    """(title, info groups, checks) as App.arm_rows would show them; checks are (state, text)."""
    sim = state == 'blocked'
    name = 'crane_full_stack' if sim else 'boat'
    outputs = 'simulator over ROS (/pwm_cmd)' if sim else 'ODROID PWM pins (real thrusters)'
    driver = 'running' if state == 'armed' else 'starts on arm'
    info = [('Will drive', [('Outputs', outputs), ('Commands', '/thrust_cmd (newtons)'), ('Driver', driver)]),
            ('Setup', [('Config', '~/ros2_ws/config/hardware.yaml'),
                       ('Decoupling', ('ok', 'fitted (Thrusters c clears)'))])]
    checks = [('bad', b) for b in BLOCKERS] if sim else [('ok', 'Interlocks clear')]
    if state != 'blocked':
        checks.append(('ok', 'Pins verified (sysfs readback)'))
    return name, info, checks


def modal(state):
    """Today's modal dict for ``state`` (App.arm_panel / confirm_drive / type_word)."""
    name, info, checks = facts(state)
    rows = []
    for title, items in info:
        rows.append((widgets.SECTION, title))
        for label, value in items:
            if isinstance(value, tuple):
                value = theme.icon(value[0]).plain + ' ' + value[1]
            rows.append((label, value))
    rows.append((widgets.SECTION, 'Checks'))
    blocked = [c for c in checks if c[0] == 'bad']
    if not blocked:
        rows.append(('Interlocks', theme.icon('ok').plain + ' clear'))
    for index, (_, text) in enumerate(blocked):
        rows.append(('Interlocks' if not index else '', theme.icon('bad').plain + ' ' + text))
    if state == 'typed':
        word = 'ARM'
        return dict(kind='input', title=f'Arm{theme.SEPARATOR}{name}', message=MESSAGE, text='AR',
                    danger=True, word=word, rows=rows, width=WIDTH,
                    hint=f'Type {word} and Enter{theme.SEPARATOR}Esc cancel')
    if state == 'blocked':
        return choice_modal(f'Arm{theme.SEPARATOR}{name}', ('Cancel', 'Arm'), {'n': 0, 'y': 1},
                            rows=rows, width=WIDTH)
    return choice_modal(f'Armed{theme.SEPARATOR}{name}', ('Disarm', 'Close'), {'d': 0, 'c': 1},
                        rows=rows, danger=True, width=WIDTH)


def _style(token):
    return theme.PALETTE.style(token)


def _icon(state):
    return theme.icon({'ok': 'good', 'bad': 'bad', 'warn': 'warn'}[state])


def _value(value, strong=False):
    if isinstance(value, tuple):
        out = _icon(value[0]).copy()
        out.append(' ' + value[1], style=_style('fg'))
        return out
    return Text(value, style=f'bold {_style("armed")}' if strong else _style('fg'))


def _rule(label, width, note=None, hue=None):
    dash = theme.RULE
    text = Text(f'{dash * 2} ', style=_style('border'))
    text.append(label, style=f'bold {_style("slate")}')
    used = len(label) + 4
    if note:
        text.append(theme.SEPARATOR, style=_style('border'))
        text.append(note, style=hue or theme.MUTED)
        used += len(theme.SEPARATOR) + len(note)
    text.append(' ' + dash * max(0, width - used), style=_style('border'))
    return text


def _grid(label_width, pad=2):
    grid = Table.grid(padding=(0, pad))
    grid.add_column(width=label_width, justify='right', no_wrap=True)
    grid.add_column(ratio=1, overflow='fold')
    return grid


def body_1(state, inner):
    """Quiet labels, right-aligned to one column; a state gutter for the checks; a counted rule."""
    name, info, checks = facts(state)
    out = []
    if state == 'typed':
        caution = Text(theme.pick('▲ ', '! '), style=f'bold {theme.WARN}')
        caution.append(MESSAGE, style=theme.WARN)
        out += [caution, Text('')]
    for title, items in info:
        out.append(_rule(title, inner))
        grid = _grid(10)
        for label, value in items:
            grid.add_row(Text(label, style=theme.MUTED), _value(value))
        out.append(grid)
    bad = sum(1 for s, _ in checks if s == 'bad')
    out.append(_rule('Checks', inner, f'{bad} blocking' if bad else 'all clear',
                     theme.BAD if bad else theme.GOOD))
    grid = _grid(10)
    for index, (s, text) in enumerate(sorted(checks, key=lambda c: c[0] != 'bad')):
        icon = _icon(s)
        line = Text(text, style=_style('fg') if s == 'bad' else theme.MUTED)
        grid.add_row(icon, line)
    out.append(grid)
    out.append(Text(''))
    return out


def body_2(state, inner):
    """The frame in the armed hue; the outputs value carries the emphasis; a two-column checklist."""
    name, info, checks = facts(state)
    out = []
    if state == 'typed':
        out += [Text(MESSAGE, style=_style('fg')), Text('')]
    grid = _grid(11, 3)
    for title, items in info:
        grid.add_row(Text(title.upper(), style=f'bold {_style("slate")}'), Text(''))
        for label, value in items:
            grid.add_row(Text(label, style=theme.MUTED), _value(value, strong=label == 'Outputs' and 'real' in str(value)))
        grid.add_row('', '')
    out.append(grid)
    out.append(_rule('Checks', inner))
    blocked = any(s == 'bad' for s, _ in checks)
    table = Table.grid(padding=(0, 3), expand=True)
    table.add_column(ratio=1)
    if not blocked:
        table.add_column(ratio=1)
    cells = []
    for s, text in checks:
        item = _icon(s).copy()
        item.append(' ' + text, style=theme.BAD if s == 'bad' else _style('fg'))
        cells.append(item)
    if blocked:
        for item in cells:
            table.add_row(item)
    else:
        for i in range(0, len(cells), 2):
            table.add_row(*(cells[i:i + 2] + [''] * (2 - len(cells[i:i + 2]))))
    out.append(table)
    out.append(Text(''))
    if state == 'typed':
        out.append(Text.assemble(('Type ', theme.MUTED), ('ARM', f'bold {_style("armed")}'),
                                 (' to start the thruster driver', theme.MUTED)))
    return out


def body_3(state, inner):
    """Compact card: small group headings, one tab stop, blockers first, clear checks on one line."""
    name, info, checks = facts(state)
    out = []
    bad = [c for c in checks if c[0] == 'bad']
    if bad:
        for _, text in bad:
            line = _icon('bad').copy()
            line.append(' ' + text, style=theme.BAD)
            out.append(line)
        out.append(Text(''))
    for title, items in info:
        out.append(Text(title, style=f'bold {theme.ACCENT}'))
        grid = _grid(11)
        grid.columns[0].justify = 'left'
        for label, value in items:
            grid.add_row(Text(' ' + label, style=theme.MUTED), _value(value))
        out.append(grid)
    ok = [t for s, t in checks if s == 'ok']
    if ok:
        out.append(Text('Checks', style=f'bold {theme.ACCENT}'))
        line = Text(' ')
        for i, text in enumerate(ok):
            if i:
                line.append(theme.SEPARATOR, style=_style('border'))
            line.append_text(_icon('ok'))
            line.append(' ' + text, style=_style('fg'))
        out.append(line)
    if state == 'typed':
        out += [Text(''), Text(MESSAGE, style=theme.WARN)]
    return out


def red_field(typed, word):
    """widgets.word_field in the ``arm.word`` crimson: border and typed letters; a wrong letter still
    turns ``warn`` with ``! needs``. NO_COLOR: the border and letters are bold (the token's mono form)."""
    red = _style('arm.word')
    slots, wrong = Text(), False
    for index in range(len(word)):
        if index:
            slots.append(' ')
        if index < len(typed):
            wrong = wrong or typed[index] != word[index]
            slots.append(typed[index], style=f'bold {theme.WARN if wrong else red}')
        else:
            slots.append('_', style=theme.MUTED)
    if typed == word:
        note = Text(f'{word} {theme.pick("✓", "+")}', style=f'bold {red}')
    elif wrong:
        note = Text(f'! needs {word}', style=theme.WARN)
    else:
        note = Text.assemble(('needs ', theme.MUTED), (word, f'bold {red}'))
    table = Table.grid(padding=(0, 2))
    table.add_column(no_wrap=True)
    table.add_column(vertical='middle')
    table.add_row(widgets._box(slots, theme.WARN if wrong else (red or 'bold'), width=len(word) * 2 + 3), note)
    return table


def _crule(label, width, note=None, hue=None):
    """``──── Checks · all clear ────``: a rule with its label in the middle."""
    words = Text(label, style=f'bold {_style("slate")}')
    if note:
        words.append(theme.SEPARATOR, style=_style('border'))
        words.append(note, style=hue or theme.MUTED)
    side = max(2, (width - words.cell_len - 2) // 2)
    text = Text(theme.RULE * side + ' ', style=_style('border'))
    text.append_text(words)
    text.append(' ' + theme.RULE * max(2, width - side - words.cell_len - 2), style=_style('border'))
    return text


def body_1c(state, inner):
    """Variant 1 centred: caution, rules, the label/value block, checks and the action on one axis."""
    name, info, checks = facts(state)
    out = []
    if state == 'typed':                     # two balanced lines, each centred
        first, second = MESSAGE.split(', physical')
        caution = Text(theme.pick('▲ ', '! '), style=f'bold {theme.WARN}')
        caution.append(first + ',', style=theme.WARN)
        out += [Align.center(caution), Align.center(Text('physical' + second, style=theme.WARN)), Text('')]
    for title, items in info:
        out.append(_crule(title, inner))
        grid = _grid(10)
        grid.columns[1].overflow = 'ellipsis'
        grid.columns[1].min_width = 30
        for label, value in items:
            grid.add_row(Text(label, style=theme.MUTED), _value(value))
        out.append(Align.center(grid))
    bad = sum(1 for s, _ in checks if s == 'bad')
    out.append(_crule('Checks', inner, f'{bad} blocking' if bad else 'all clear', theme.BAD if bad else theme.GOOD))
    lines = []
    for s, text in sorted(checks, key=lambda c: c[0] != 'bad'):
        line = _icon(s).copy()
        line.append(' ' + text, style=_style('fg') if s == 'bad' else theme.MUTED)
        lines.append(line)
    block = Table.grid()
    block.add_column(max_width=inner - 4)
    for line in lines:
        block.add_row(line)
    out += [Align.center(block), Text('')]
    return out


def _action(m, inner, centred):
    """The field or the buttons, as layout.popup draws them, optionally centred."""
    if m['kind'] == 'input':
        part = red_field(m['text'], m['word'])
        return [Align.center(part) if centred else part]
    lines = widgets.choice_list(m['options'], m['keys'], m['selected'], m['risky'], inner)
    hint = m['hint']
    if len(lines) == 1:
        m['hint'] = hint.replace(theme.pick('↑↓', 'Up/Down'), theme.pick('←→', 'Left/Right'))
    return [Align.center(line) if centred else line for line in lines]


BODIES = {'1': body_1, '1c': body_1c, '2': body_2, '3': body_3}


def render(option, state, width):
    """The arm panel for ``state`` centred in a ``width``-column terminal, as layout.popup draws it."""
    m = modal(state)
    if option != '0':
        inner = min(width - 4, WIDTH) - 4
        parts = BODIES[option](state, inner)
        if option in ('1', '1c'):              # the action drawn here: the red field, or centred buttons
            parts += _action(m, inner, option == '1c')
            m['kind'] = 'custom'
        m['body'] = Group(*parts)
        m['rows'], m['message'] = None, None
        if option == '2':
            m['danger'] = False
            m['border'] = _style('armed')
    panel, panel_width = layout.popup(m, width)
    if option == '2':
        title = m['title'].split(theme.SEPARATOR)
        chip = theme.PALETTE.chip(title[0].upper(), 'armed') if not theme.ASCII else Text(f'[{title[0].upper()}]')
        chip.stylize('bold')
        heading = Text.assemble(chip, (f' {title[1]} ', theme.MUTED))
        panel.title = heading
    elif option in ('1', '1c', '3'):
        title = m['title'].split(theme.SEPARATOR)
        heading = Text(f' {title[0]}', style=f'bold {theme.BAD}')
        heading.append(theme.SEPARATOR + title[1] + ' ', style=theme.MUTED)
        panel.title = heading
        if option == '1c':
            panel.title_align = 'center'
            panel.subtitle_align = 'center'
    panel.width = panel_width
    return Align.center(panel, width=width)

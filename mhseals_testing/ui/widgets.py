"""Popup widgets in the cx look: buttons, choice lists, text fields, letter slots, rules, the stop chip.

Pure functions over ``theme``: ``layout.popup`` calls them only while a popup is open. Every cell
has an ASCII form (``theme.pick`` at call time) and the focused button keeps ``reverse`` with no
colour, so NO_COLOR still shows where Enter lands.
"""

from rich import box
from rich.console import Group
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from mhseals_testing.ui import theme

KINDS = {'plain': 'accent', 'danger': 'warn', 'stop': 'stop'}   # button kind -> fill token
SECTION = '§'   # a popup row ``(SECTION, 'Will drive')`` draws as a rule() between label/value groups
ROW_LABEL_MAX, ROW_OPTIONS_MAX, GAP = 14, 3, '   '               # a choice fits one row of buttons


def _style(token):
    return theme.PALETTE.style(token)


def _fill(token):
    """The solid cell style of ``token``: bg on token, reverse on 16 colours and NO_COLOR."""
    return theme.PALETTE.chip('', token, ascii=False).style or 'reverse'


def _label(label):
    if isinstance(label, Text):
        return label.copy()
    return Text.from_markup(label) if '[' in label else Text(label)


def button(label, key=None, *, focused=False, kind='plain'):
    """``[ y Release ]``, or focused ``[▸ n Cancel ↵]`` as a solid chip; ↵ marks what Enter does."""
    if focused:
        text = Text(theme.pick('[▸', '[>') + (f'{key} ' if key else ''))
        text.append_text(_label(label))
        text.append(theme.pick(' ↵]', ' <]'))
        text.stylize(f'{_fill(KINDS[kind])} bold')
        return text
    edge = _style('border')
    text = Text('[ ', style=edge)
    if key:
        text.append(f'{key} ', style=f'bold {theme.ACCENT}')
    label = _label(label)
    if kind != 'plain':
        label.stylize(_style(KINDS[kind]))
    text.append_text(label)
    text.append(' ]', style=edge)
    return text


def button_row(buttons, width):
    """The buttons on one line with a 3-cell gap; None when they do not fit in ``width``."""
    row = Text(GAP).join(buttons)
    return row if row.cell_len <= width else None


def choice_list(options, keys, selected, danger, width):
    """A short choice as one row of buttons, else a vertical list in the same grammar.

    ``danger`` is the set of option indices drawn as danger buttons (never Cancel).
    """
    short = (len(options) <= ROW_OPTIONS_MAX
             and all(_label(o).cell_len <= ROW_LABEL_MAX for o in options))
    if short:
        row = button_row([button(o, k, focused=i == selected, kind='danger' if i in danger else 'plain')
                          for i, (o, k) in enumerate(zip(options, keys))], width)
        if row is not None:
            return [row]
    lines = []
    for index, (option, key) in enumerate(zip(options, keys)):
        active = index == selected
        line = Text(theme.pick('▸ ', '> ') if active else '  ', style=f'bold {theme.ACCENT}')
        if key:
            line.append(f'{key} ', style=f'bold {theme.ACCENT}')
        label = _label(option)
        if index in danger:
            label.stylize(theme.WARN)
        line.append_text(label)
        if active:
            line.append(theme.pick(' ↵', ' <'), style=f'bold {theme.ACCENT}')
            line.stylize(f'bold {theme.SEL_BG}' if theme.TINT else 'bold underline')
        lines.append(line)
    return lines


def _box(content, border, width=None):
    return Panel(content, box=box.ASCII if theme.ASCII else box.ROUNDED, border_style=border,
                 padding=(0, 1), width=width)


def field(text, *, width, placeholder='', state='idle', note='', suffix='', limit=None):
    """A bordered one-line text field: caret at the end, a muted counter, a note line under it.

    ``state`` 'bad' turns the border and the note ``warn`` with a ``!``; the field is always
    focused (a popup has one), so its idle border is the accent.
    """
    line = Text(text)
    if not text and placeholder:
        line.append(theme.CURSOR_BLOCK)
        line.append(placeholder + theme.pick('…', '...'), style=f'{theme.MUTED} italic')
    else:
        if suffix:
            line.append(f' {suffix}', style=theme.MUTED)
        line.append(theme.CURSOR_BLOCK)
    if limit:
        counter = f'{len(text)} / {limit}'
        line.append(' ' * max(1, width - 4 - line.cell_len - len(counter)))
        line.append(counter, style=theme.MUTED)
    parts = [_box(line, theme.WARN if state == 'bad' else theme.ACCENT, width)]
    if note:
        parts.append(Text(('! ' if state == 'bad' else '') + note,
                          style=theme.WARN if state == 'bad' else theme.MUTED))
    return Group(*parts)


def word_field(typed, word, hue=None):
    """The typed confirm's letter slots, ``A R _``, with what it still needs beside them.

    Letters matching the word so far are ``ok``; from the first wrong one on they are ``warn``.
    ``hue`` (the arm panel's ``arm.word``) draws the box, the matching letters and the word instead.
    """
    slots, wrong = Text(), False
    for index in range(len(word)):
        if index:
            slots.append(' ')
        if index < len(typed):
            wrong = wrong or typed[index] != word[index]
            slots.append(typed[index], style=f'bold {theme.WARN if wrong else hue or theme.GOOD}'.strip())
        else:
            slots.append('_', style=theme.MUTED)
    done = typed == word
    if done:
        note = Text(f'{word} {theme.pick("✓", "+")}', style=f'bold {hue}' if hue else theme.GOOD)
    elif wrong:
        note = Text(f'! needs {word}', style=theme.WARN)
    else:
        note = Text.assemble(('needs ', theme.MUTED), (word, f'bold {hue}'.strip())) if hue is not None \
            else Text(f'needs {word}', style=theme.MUTED)
    table = Table.grid(padding=(0, 2))
    table.add_column(no_wrap=True)
    table.add_column(vertical='middle')
    table.add_row(_box(slots, theme.WARN if wrong else (hue or 'bold') if hue is not None else theme.ACCENT,
                       width=len(word) * 2 + 3), note)
    return table


def rule(label, width, note=None, note_style=None, strong=False):
    """``── Drift ─────``: a quiet section line for popups; ``strong`` labels in bold slate and a
    ``note`` (``2 blocking``) follows the label after a separator."""
    dash = theme.RULE
    text = Text(f'{dash * 2} ', style=_style('border'))
    text.append(label, style=f'bold {_style("slate")}'.strip() if strong else theme.MUTED)
    used = len(label) + 4
    if note:
        text.append(theme.SEPARATOR, style=_style('border'))
        text.append(note, style=note_style or theme.MUTED)
        used += len(theme.SEPARATOR) + len(note)
    text.append(' ' + dash * max(0, width - used), style=_style('border'))
    return text


def stop_chip(cause, age):
    """``⊘ STOP rc 12 s`` as a solid ``stop`` chip; ``[STOP rc 12 s]`` in ASCII."""
    words = ' '.join(str(w) for w in ('STOP', cause, age) if w)
    if theme.ASCII:
        return Text(f'[{words}]', style='bold')
    chip = theme.chip(f'{theme.PALETTE.glyph("stop", ascii=False)} {words}', 'stop')
    chip.stylize('bold')
    return chip


def countdown(left, total, width=10):
    """``▕██████▌   ▏ 14 s`` in muted: an agent popup's time left; ``[=====   ] 14 s`` in ASCII."""
    share = max(0.0, min(1.0, left / total)) if total else 0.0
    if theme.ASCII:
        filled = round(share * width)
        bar = '[' + '=' * filled + ' ' * (width - filled) + ']'
    else:
        eighths = round(share * width * 8)
        bar = '█' * (eighths // 8) + ('▏▎▍▌▋▊▉'[eighths % 8 - 1] if eighths % 8 else '')
        bar = '▕' + bar.ljust(width) + '▏'
    return Text(f'{bar} {max(0, round(left))} s', style=theme.MUTED)

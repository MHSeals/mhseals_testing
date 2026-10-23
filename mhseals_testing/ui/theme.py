"""One look for every panel, after cx: ANSI colors, rounded blocks, status glyphs.

Only the 16 terminal colors are used, so the terminal theme decides how they
look. NO_COLOR is honoured by Rich, so each state has its own glyph (✓ ! × ◌ ○) and
never relies on color alone; TERM=dumb or MHS_ASCII swaps glyphs for ASCII.
"""

from rich import box
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from mhseals_testing.ui import palette

ASCII = palette.ascii_mode()

# Status styles are palette tokens (ui.md U2): ok, warn, fail, busy. The user overlay
# warning, if any, is kept for the UI to toast.
PALETTE, OVERLAY_WARNING = palette.load_theme()
ACCENT, MUTED = PALETTE.style('accent'), PALETTE.style('muted')
GOOD, WARN, BAD, BUSY = (PALETTE.style(token) for token in ('ok', 'warn', 'fail', 'busy'))


def pick(glyph, ascii_glyph):
    """The Unicode glyph, or its ASCII stand-in when ASCII mode is on."""
    return ascii_glyph if ASCII else glyph


def ascii_safe(text):
    """Free text from the suite with °, –, σ and × spelled out in ASCII mode."""
    if not ASCII:
        return text
    text = text.replace('°', ' deg ').replace('–', '-').replace('σ', 'sigma').replace('×', 'x')
    return text.replace('  ', ' ').replace(' ,', ',').strip()


SEPARATOR = pick(' · ', ' | ')
DOT, MIDDOT, DASH = pick('●', '*'), pick('·', '.'), pick('—', '-')
BAR, RULE, FILL = pick('│', '|'), pick('─', '-'), pick('█', '#')
SPARK = pick('▁▂▃▄▅▆▇█', '._-:=+*#')
OVERFLOW = pick('ellipsis', 'crop')   # Rich's ellipsis cell is the Unicode …
FOCUS = '>' if ASCII else '▸'
CURSOR = '> ' if ASCII else '› '
# Cursor-row tint (ui.md U10) from the sel_bg token; 16-colour, NO_COLOR and ASCII underline instead.
TINT = palette.detect_tier() in ('truecolor', '256') and not ASCII
SEL_BG = f"on {PALETTE.style('sel_bg')}" if TINT else ''
ARROW = '->' if ASCII else '→'

# Glyph and color per state; every state the suite reports maps to one of these.
_GLYPHS = {
    'good': ('+' if ASCII else '✓', GOOD),
    'warn': ('!', WARN),
    'bad': ('x' if ASCII else '×', BAD),
    'busy': ('~' if ASCII else '◌', BUSY),
    'off': ('o' if ASCII else '○', MUTED),
    'unknown': ('?', MUTED),
}
_KINDS = {
    'good': {'LIVE', 'OK', 'PASS', 'SIGN MATCH', 'SETTLED', 'HELD', 'REACHED', 'FITTED',
             'SAVED', 'DONE', 'RECORDED', 'SUCCEEDED', 'ARMED', 'READY'},
    'warn': {'WARN', 'PARTIAL', 'STALE', 'LOW RESPONSE', 'NOT SETTLED', 'LEFT TOLERANCE', 'UNAVAILABLE'},
    'bad': {'FAIL', 'MISSING', 'SIGN MISMATCH', 'TIMEOUT', 'ABORTED', 'CANCELED', 'ERROR'},
    'busy': {'RUNNING', 'CHECKING'},
    'off': {'N/A', 'OFF', 'IDLE', 'OPTIONAL'},
}


def kind(state):
    state = str(state).upper()
    for name, states in _KINDS.items():
        if state in states or state.startswith(tuple(states)):
            return name
    return 'unknown'


def icon(state):
    glyph, color = _GLYPHS[state if state in _GLYPHS else kind(state)]
    return Text(glyph, style=color)


def labelled(state, label=None, style=None):
    """``● label`` with the glyph colored by state."""
    text = icon(state)
    text.append(' ' + (label if label is not None else str(state).lower()), style=style)
    return text


# Solid chips (ui spec 1, 2): the word in reverse video on the authority hue; ASCII brackets the word.
CHIPS = {'human': '', 'ours': ACCENT, 'nav': 'magenta', 'agent': 'bold magenta', 'hold': '',
         'unclaimed': BAD, 'armed': BAD}
AGENT_BANNER = 'reverse bold magenta'


def chip(word, kind):
    """``word`` as a solid chip in the ``kind`` hue (reverse video, so NO_COLOR keeps it)."""
    if ASCII:
        return Text(f'[{word}]', style='bold')
    return Text(f' {word} ', style=f'reverse {CHIPS[kind]}'.strip())


def block(renderable, title, focused=False, subtitle=None, keys=None, **kwargs):
    """A rounded panel; the focused one gets a bold border and a ▸ before its title.

    ``keys`` is written on the bottom border (popup key hints).
    """
    heading = Text(f'{FOCUS} {title}' if focused else title, style=f'bold {ACCENT}')
    if subtitle:
        heading.append(SEPARATOR + subtitle, style=MUTED)
    bottom = Text(f' {keys} ', style=MUTED) if keys else None
    return Panel(renderable, title=heading, title_align='left', subtitle=bottom, subtitle_align='left',
                 box=box.ASCII if ASCII else box.ROUNDED,
                 border_style='bold' if focused else 'default', padding=(0, 1), **kwargs)


def grid(*columns, padding=(0, 2)):
    """A borderless table with dim headers, the table style of every panel."""
    table = Table(box=None, expand=True, padding=padding, show_edge=False,
                  header_style=MUTED, pad_edge=False)
    for column in columns:
        options = dict(no_wrap=True, overflow=OVERFLOW)
        options.update(column if isinstance(column, dict) else {'header': column})
        table.add_column(**options)
    return table


def hint(key, label):
    text = Text(f' {key} ', style=f'bold {ACCENT}')
    text.append(label)
    return text


def row(state, label, active=False, prefix=None):
    """One list entry: ``› ● label`` for the cursor row, ``  ● label`` otherwise.

    ``state`` picks the glyph (None for none); ``prefix`` is extra text after the
    glyph, such as a shortcut key. The cursor row is cyan bold underlined, like cx.
    """
    text = Text(CURSOR if active else '  ', style=f'bold {ACCENT}' if active else None)
    if state is not None:
        text.append_text(icon(state))
        text.append(' ')
    if prefix is not None:
        text.append_text(prefix if isinstance(prefix, Text) else Text(prefix))
        text.append(' ')
    label = label if isinstance(label, Text) else Text(str(label))
    if active:
        label.stylize(f'bold {ACCENT} {SEL_BG}' if TINT else f'bold underline {ACCENT}')
    return text + label


def selected(text, active):
    """Highlight a row the way cx does: › prefix, cyan bold underline when focused."""
    prefix = Text(CURSOR if active else '  ')
    if isinstance(text, str):
        text = Text(text)
    if active:
        text.stylize(f'bold {ACCENT}')
        prefix.stylize(f'bold {ACCENT}')
    return prefix + text

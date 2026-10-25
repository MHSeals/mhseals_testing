"""One look for every panel, after cx: ANSI colors, rounded blocks, status glyphs.

Only the 16 terminal colors are used, so the terminal theme decides how they
look. NO_COLOR is honoured by Rich, so each state has its own glyph (✓ ! ✗ ◌ ·) and
never relies on color alone; TERM=dumb or MHS_ASCII swaps glyphs for ASCII.
"""

import math

from rich import box
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from mhseals_testing.ui import palette

ASCII = palette.ascii_mode()

# Status styles are palette tokens (ui.md U2): ok, warn, fail, busy. The user overlay
# warning, if any, is kept for the UI to toast.
PALETTE, OVERLAY_WARNING = palette.load_theme()
AUTHORITY_HUES, _GLYPHS = {}, {}   # filled by use_palette; read as theme.X at call time


def use_palette(new):
    """Make ``new`` the theme: the style names below follow it (the hot reload calls this too)."""
    global PALETTE, ACCENT, MUTED, GOOD, WARN, BAD, BUSY, SEL_BG, AGENT_BANNER, AGENT_LOW, AGENT_DIM
    PALETTE = new
    ACCENT, MUTED = PALETTE.style('accent'), PALETTE.style('muted')
    GOOD, WARN, BAD, BUSY = (PALETTE.style(token) for token in ('ok', 'warn', 'fail', 'busy'))
    SEL_BG = f"on {PALETTE.style('sel_bg')}" if TINT else ''
    # Who drives, as a hue (ui.md U4); the command arrow wears the same one. Never a status hue
    # except unclaimed. Owners are harness.authority.OWNERS.
    AUTHORITY_HUES.update({
        'rc': 'bold', 'keyboard': 'bold', 'hold': 'bright_black', 'agent': 'bright_magenta',
        'task': ACCENT, 'procedure': ACCENT, 'nav': 'magenta', 'external': BAD, 'none': MUTED,
    })
    # The agent banner is the auth.agent chip cell stretched over the row; a low lease is a warn cell.
    AGENT_BANNER = f"{PALETTE.chip('', 'auth.agent', ascii=False).style} bold".strip()
    AGENT_LOW = f"{PALETTE.chip('', 'warn', ascii=False).style} bold".strip()
    AGENT_DIM = PALETTE.style('auth.agent_dim')
    # Glyph and color per state; every state the suite reports maps to one of these.
    _GLYPHS.update({
        'good': ('+' if ASCII else '✓', GOOD),
        'warn': ('!', WARN),
        'bad': ('x' if ASCII else '✗', BAD),
        'busy': ('~' if ASCII else '◌', BUSY),
        'off': (MIDDOT, MUTED),
        'unknown': (MIDDOT, MUTED),
    })


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
MIDDOT, DASH = pick('·', '.'), pick('—', '-')
BAR, RULE, FILL = pick('│', '|'), pick('─', '-'), pick('█', '#')
CURSOR_BLOCK = pick('█', '_')   # a text field's caret; FILL's ASCII '#' would read as content
SPARK = pick('▁▂▃▄▅▆▇█', '._-:=+*#')
OVERFLOW = pick('ellipsis', 'crop')   # Rich's ellipsis cell is the Unicode …
FOCUS = '>' if ASCII else '▸'
CURSOR = '> ' if ASCII else '› '
# Cursor-row tint (ui.md U10) from the sel_bg token; 16-colour, NO_COLOR and ASCII underline instead.
TINT = palette.detect_tier() in ('truecolor', '256') and not ASCII
ARROW = '->' if ASCII else '→'
SPINNER = '|/-\\' if ASCII else '⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏'
SPINNER_HZ = 10
AGENT_GLYPH, WORKING_GLYPH = PALETTE.glyph('agent'), PALETTE.glyph('agent_work')   # driving, working
use_palette(PALETTE)


def spinner(t):
    """The spinner frame for time ``t`` seconds, advancing SPINNER_HZ frames a second."""
    return SPINNER[int(t * SPINNER_HZ) % len(SPINNER)] if math.isfinite(t) else SPINNER[0]

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
    """``✓ label`` with the glyph colored by state."""
    text = icon(state)
    text.append(' ' + (label if label is not None else str(state).lower()), style=style)
    return text


# Solid chips (ui spec 1, 2): the word in reverse video on the authority hue; ASCII brackets the word.
CHIPS = {'human': 'auth.human', 'ours': 'auth.ours', 'nav': 'auth.nav', 'agent': 'auth.agent',
         'hold': 'auth.hold', 'unclaimed': 'auth.unclaimed', 'armed': 'armed',
         'stop': 'stop'}   # chip kind -> palette token


def chip(word, kind):
    """``word`` as a solid chip in the ``kind`` hue (reverse video on 16 colours, so NO_COLOR keeps it)."""
    chip_ = PALETTE.chip(word, CHIPS[kind], ascii=ASCII)
    if ASCII or kind in ('agent', 'stop'):   # these stay bold where colour is absent
        chip_.stylize('bold')
    return chip_


def auth_style(kind):
    """The foreground style of an authority hue: the Driving value, the command arrow, subtitles."""
    return PALETTE.style(CHIPS.get(kind, 'auth.human'))


def agent_style(agent):
    """The hue of a coding agent by vendor (``claude``, ``codex`` or a ``claude-3f2a`` name);
    any other agent wears the agent authority hue."""
    token = f"agent.{str(agent or '').split('-')[0]}"
    return PALETTE.style(token if token in PALETTE.tokens else 'auth.agent')


def block(renderable, title, focused=False, subtitle=None, keys=None, **kwargs):
    """A rounded panel; the focused one gets a bold border and a ▸ before its title.

    ``keys`` is written on the bottom border (popup key hints).
    """
    heading = Text(f'{FOCUS} {title}' if focused else title, style=f'bold {ACCENT}')
    if subtitle:
        heading.append(SEPARATOR, style=MUTED)
        heading.append_text(subtitle) if isinstance(subtitle, Text) else heading.append(subtitle, style=MUTED)
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


def hint(key, label, style=None):
    """`` key label``; ``style`` replaces the accent on the key (Space wears ``stop``)."""
    text = Text(f' {key} ', style=f'bold {style or ACCENT}')
    text.append(label)
    return text


def row(state, label, active=False, prefix=None):
    """One list entry: ``› ✓ label`` for the cursor row, ``  ✓ label`` otherwise.

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


# Activity timeline glyphs (timeline.md section 4, ui U8): name -> (Unicode, ASCII).
_TIMELINE = {
    'body': ('█', '#'), 'bad': ('▓', '%'), 'cap': ('×', 'x'), 'tip': ('◌', '~'),
    'notch': ('▌', '|'), 'clip': ('┄', '.'), 'idle': ('▁', '_'), 'manual1': ('▄', '='),
    'mark': ('▾', 'v'), 'config': ('Δ', '*'), 'health_warn': ('!', '^'),
    'health_fail': ('✗', 'x'), 'auth': ('↔', '@'), 'agent': ('◆', 'A'), 'off_left': ('◂', '<'),
    'ok': ('─', '-'), 'recoff': ('╌', '.'), 'armrecoff': ('╍', ','),
    'warn': ('━', '='), 'fail': ('═', '!'), 'fold': ('┊', ':'),
    'bracket': ('╰─╯', '\\_/'), 'single': ('┴', '+'),
    'loupe_l': ('├', '['), 'loupe_r': ('┤', ']'), 'play': ('┃', '|'),
}


def timeline(ascii_mode=None):
    """Glyph table of the timeline strip, Unicode or ASCII (default: the current mode)."""
    ascii_mode = ASCII if ascii_mode is None else ascii_mode
    return {name: pair[1 if ascii_mode else 0] for name, pair in _TIMELINE.items()}

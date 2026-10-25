"""SGR-run emitter for the map canvas (map.md sections 3 and 9); pure, no ROS.

`emit()` turns the resolved cells of a `Canvas` into one string per row. Neighbouring
cells with the same style share one escape sequence, so a frame stays small. Colours come
from a ``style(token)`` callable returning a Rich-style string (``#rrggbb``, ``color(N)``,
a 16-colour name, plus ``bold``/``dim``/``reverse``), e.g. ``Theme.style`` bound to a tier.

Empty braille cells go out as spaces, which look the same and cost a third of the bytes.

Tiers: ``truecolor`` and ``256`` paint foregrounds, tints and chips; ``16`` drops tints and
turns chips into reverse video; ``mono`` (NO_COLOR) uses only the token's bold/dim/reverse
attributes; ASCII emits no SGR at all and maps braille, blanks and gap marks to ASCII.
"""

from mhseals_testing.ui.minimap.canvas import BLANK, GAP

RESET = '\x1b[0m'
_NAMES = ('black', 'red', 'green', 'yellow', 'blue', 'magenta', 'cyan', 'white')
_ATTRS = {'bold': '1', 'dim': '2', 'italic': '3', 'underline': '4', 'reverse': '7'}


def _colour(word, bg=False):
    """SGR parameters for one colour word, or '' when it is not a colour."""
    base = 48 if bg else 38
    if word.startswith('#') and len(word) == 7:
        r, g, b = (int(word[i:i + 2], 16) for i in (1, 3, 5))
        return f'{base};2;{r};{g};{b}'
    if word.startswith('color(') and word.endswith(')'):
        return f'{base};5;{word[6:-1]}'
    bright = word.startswith('bright_')
    name = word[7:] if bright else word
    if name in _NAMES:
        return str(_NAMES.index(name) + (40 if bg else 30) + (60 if bright else 0))
    return ''


def parse(style):
    """Split a style string into (foreground SGR, attribute SGR list)."""
    fore, attrs = '', []
    for word in (style or '').split():
        if word in _ATTRS:
            attrs.append(_ATTRS[word])
        elif not fore:
            fore = _colour(word)
    return fore, attrs


def _back(style):
    """Background SGR from the first colour word of a style (attributes may come first)."""
    for word in (style or '').split():
        code = _colour(word, bg=True)
        if code:
            return code
    return ''


def _sgr(*params):
    """One run's sequence; it starts with a reset so no attribute or bg leaks from the last run."""
    params = [p for p in params if p]
    return f'\x1b[0;{";".join(params)}m' if params else ''


def _ascii_text(cell):
    if cell.text == GAP:
        return '-'
    if cell.text == BLANK:
        return ' '
    return '.' if 0x2800 <= ord(cell.text) <= 0x28FF else cell.text


def _cell_sgr(cell, style, tier):
    """``(sequence, see_through)`` for one cell ('' = default style). ``see_through`` is True when
    the style only colours glyphs (no background, reverse or underline), so a blank may share it."""
    fore, attrs = parse(style(cell.fg)) if cell.fg else ('', [])
    if cell.chip:
        if tier in ('16', 'mono'):
            return _sgr(fore if tier == '16' else '', *attrs, '7'), False
        back = _back(style(cell.fg))
        return _sgr(parse(style('bg'))[0], back), False
    clear = '4' not in attrs and '7' not in attrs
    if tier == 'mono':
        return _sgr(*attrs), clear
    back = ''
    if cell.bg and tier != '16':
        back = _back(style(cell.bg))
    return _sgr(fore, back, *attrs), clear and not back


def emit(canvas, style, tier='truecolor', ascii=False):
    """One string per canvas row; styles coalesce into runs and each row ends in a reset.

    A blank cell without a tint joins the current run when that run only colours glyphs, so
    sparse dots do not pay a reset and a new sequence each."""
    rows = []
    for row in canvas.cells():
        if ascii:
            rows.append(''.join(_ascii_text(cell) for cell in row))
            continue
        out, active, through = [], '', True
        for cell in row:
            if cell.text == BLANK and not cell.bg and through:
                out.append(' ')   # 1 byte, not 3: half a frame is blank
                continue
            code, see = _cell_sgr(cell, style, tier)
            if code != active:
                out.append(code or RESET)
                active, through = code, see
            out.append(' ' if cell.text == BLANK else cell.text)
        if active:
            out.append(RESET)
        rows.append(''.join(out))
    return rows

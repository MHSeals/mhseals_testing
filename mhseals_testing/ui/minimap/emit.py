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


def _cell_sgr(fg, bg, chip, style, tier):
    """``(sequence, background, clear, foreground, attributes)`` for a cell's look ('' = default
    style). ``background`` is its bg SGR ('' none, None for a chip, which nothing joins or diffs
    against); ``clear`` is False with reverse or underline. A blank shows only its background, so
    it may join a clear run of the same one."""
    fore, attrs = parse(style(fg)) if fg else ('', [])
    if chip:
        if tier in ('16', 'mono'):
            return _sgr(fore if tier == '16' else '', *attrs, '7'), None, False, None, ()
        return _sgr(parse(style('bg'))[0], _back(style(fg))), None, False, None, ()
    clear = '4' not in attrs and '7' not in attrs
    if tier == 'mono':
        return _sgr(*attrs), '', clear, '', tuple(attrs)
    back = _back(style(bg)) if bg and tier != '16' else ''
    return _sgr(fore, back, *attrs), back, clear, fore, tuple(attrs)


def _switch(old, new):
    """The sequence from look ``old`` to ``new``: only the changed colours when both are styled
    plain looks with the same attributes (``39``/``49`` back to default), else a full reset sequence."""
    if not old[0] or old[3] is None or new[3] is None or old[4] != new[4]:
        return new[0] or RESET
    params = ([new[3] or '39'] if new[3] != old[3] else []) + ([new[1] or '49'] if new[1] != old[1] else [])
    return f'\x1b[{";".join(params)}m' if params else ''


def emit(canvas, style, tier='truecolor', ascii=False):
    """One string per canvas row; styles coalesce into runs and each row ends in a reset.

    A blank cell joins the current run when that run is clear (no reverse or underline) and has
    the blank's background, so sparse dots and tinted costmap stretches do not pay a sequence each;
    a change of colour alone sends only that colour. Each distinct look is resolved once per frame."""
    rows, looks, default = [], {}, ('', '', True, '', ())
    for row in canvas.cells():
        if ascii:
            rows.append(''.join(_ascii_text(cell) for cell in row))
            continue
        out, active, prev = [], default, None
        for cell in row:
            if cell is prev:   # the same look again (shared blank cells): only its text
                out.append(text)
                continue
            prev, blank = cell, cell.text == BLANK
            text = ' ' if blank else cell.text
            look = looks.get(cell)
            if look is None:   # by cell, then by its look: braille cells differ only in text
                key = (None if blank else cell.fg, cell.bg, cell.chip)
                look = looks[cell] = looks.get(key) or looks.setdefault(key, _cell_sgr(*key, style, tier))
            if blank and active[2] and look[1] is not None and active[3] is not None \
                    and (active[0] or look[1] == active[1]):
                if look[1] != active[1]:   # a blank shows only its background: keep the glyph colour
                    out.append(f'\x1b[{look[1] or "49"}m')
                    fore, attrs = active[3], active[4]
                    active = (_sgr(fore, look[1], *attrs), look[1], True, fore, attrs)
                out.append(text)   # 1 byte, not 3: half a frame is blank
                continue
            if look[0] != active[0]:
                out.append(_switch(active, look))
                active = look
            out.append(text)
        if active[0]:
            out.append(RESET)
        rows.append(''.join(out))
    return rows

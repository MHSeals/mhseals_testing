"""SGR-run emitter: tiers, run coalescing, chips, NO_COLOR, ASCII and the byte budget."""
import re

from mhseals_testing.ui.minimap import canvas, emit

STYLES = {'trail': '#112233', 'boat': 'color(116)', 'contact': 'red', 'bg': '#000000',
          'cost': 'bright_black bold', 'object': 'green dim'}


def style(token):
    return STYLES.get(token, '')


def sample():
    c = canvas.Canvas(4, 1)
    c.dot(0, 0, 'trail')
    c.glyph(2, 0, 'X', 'contact', chip=True)
    c.tint(3, 0, 'cost')
    return c


def test_same_style_neighbours_share_one_sequence():
    c = canvas.Canvas(3, 1)
    for x in (0, 2, 4):
        c.dot(x, 0, 'trail')
    row = emit.emit(c, style)[0]
    assert row == '\x1b[0;38;2;17;34;51m' + '⠁⠁⠁' + emit.RESET


def test_truecolor_and_16_colours_and_chip():
    row = emit.emit(sample(), style)[0]
    assert '\x1b[0;38;2;17;34;51m⠁' in row
    assert '\x1b[0;38;2;0;0;0;41mX' in row            # chip: bg glyph on the token colour
    assert '\x1b[0;100m ' in row                 # tint painted (a blank cell goes out as a space)
    row16 = emit.emit(sample(), style, '16')[0]
    assert '\x1b[0;31;7mX' in row16                   # chip becomes reverse video
    assert '100' not in row16                       # tints dropped at 16 colours


def test_256_tier_colour_indices():
    row = emit.emit(sample(), lambda t: {'trail': 'color(116)'}.get(t, ''), '256')[0]
    assert '\x1b[0;38;5;116m⠁' in row


def test_styles_reset_between_runs():
    c = canvas.Canvas(3, 1)
    c.tint(0, 0, 'cost')
    c.glyph(1, 0, 'B', 'cost')
    c.dot(4, 0, 'trail')
    row = emit.emit(c, style)[0]
    assert row.endswith('\x1b[0;38;2;17;34;51m⠁' + emit.RESET)
    assert '\x1b[0;100m \x1b[0;90;1mB' in row


def test_attribute_first_tokens_still_give_backgrounds():
    st = {'fail': 'bold red', 'bg': '#000000', 'cost': 'dim #aabbcc'}.get
    c = canvas.Canvas(2, 1)
    c.glyph(0, 0, 'X', 'fail', chip=True)
    c.tint(1, 0, 'cost')
    row = emit.emit(c, lambda t: st(t, ''))[0]
    assert '\x1b[0;38;2;0;0;0;41mX' in row
    assert '48;2;170;187;204' in row


def test_names_and_indices():
    assert emit.parse('bright_black bold') == ('90', ['1'])
    assert emit.parse('color(116)') == ('38;5;116', [])
    assert emit._colour('blue', bg=True) == '44'
    assert emit._colour('bright_red', bg=True) == '101'


def test_no_color_uses_attributes_only():
    out = ''.join(emit.emit(sample(), style, 'mono'))
    assert '38;' not in out and '48;' not in out
    assert '\x1b[0;7mX' in out


def test_ascii_has_no_escapes_and_maps_braille_and_gaps():
    c = canvas.Canvas(3, 1)
    c.dot(0, 0, 'trail')
    c.gap(2, 0)
    assert emit.emit(c, style, ascii=True) == ['. -']


def test_frame_stays_under_the_byte_budget():
    c = canvas.Canvas(56, 30)
    for x in range(c.dots_wide):
        for y in range(0, c.dots_high, 3):
            c.dot(x, y, 'trail' if (x + y) % 2 else 'boat')
    rows = emit.emit(c, style)
    assert sum(len(r.encode()) for r in rows) <= 10 * 1024
    assert not any(re.search(r'\x1b\[[^m]*m\x1b\[', r) for r in rows)


def _painted(row):
    """(char, fg, bg, attrs) per printed cell of an SGR row, as a terminal keeps its state."""
    fg = bg = None
    attrs, out = set(), []
    for seq, text in re.findall(r'(?:\x1b\[([0-9;]*)m)?([^\x1b]*)', row):
        params = [int(p) for p in seq.split(';') if p]   # no sequence before ``text`` reads as ''
        i = 0
        while i < len(params):
            p = params[i]
            if p == 0:
                fg = bg = None
                attrs = set()
            elif p in (38, 48):
                n = 4 if params[i + 1] == 2 else 2
                colour = tuple(params[i + 1:i + 1 + n])
                fg, bg = (colour, bg) if p == 38 else (fg, colour)
                i += n
            elif p in (39, 49):
                fg, bg = (None, bg) if p == 39 else (fg, None)
            elif 30 <= p <= 37 or 90 <= p <= 97:
                fg = p
            elif 40 <= p <= 47 or 100 <= p <= 107:
                bg = p
            else:
                attrs.add(p)
            i += 1
        out += [(ch, fg, bg, frozenset(attrs)) for ch in text]
    return out


def test_colour_deltas_paint_every_cell_as_its_own_full_style():
    """Runs send only the colour that changed, and a blank keeps the glyph colour (it shows only its
    background); a terminal still ends up painting each cell exactly as that cell's own style."""
    st = {'trail': '#112233', 'boat': 'color(116)', 'cost': '#203040', 'deep': '#405060', 'bold': 'bold #aabbcc',
          'contact': 'red', 'bg': '#000000'}.get
    c = canvas.Canvas(12, 2)
    for col in range(12):
        c.tint(col, 0, 'cost' if col < 6 else 'deep')
    for x, layer in ((0, 'trail'), (4, 'boat'), (6, 'trail'), (14, 'trail'), (20, 'boat')):
        c.dot(x, 0, layer)
    c.glyph(9, 0, 'B', 'bold')
    c.glyph(11, 0, 'X', 'contact', chip=True)
    c.dot(2, 4, 'trail')
    c.tint(5, 1, 'cost')
    for tier in ('truecolor', '256', '16'):
        got = [_painted(row) for row in emit.emit(c, lambda t: st(t, ''), tier)]
        for r, cells in enumerate(c.cells()):
            for col, cell in enumerate(cells):
                blank = cell.text == canvas.BLANK
                alone = canvas.Canvas(1, 1)   # the cell emitted on its own, from a reset
                if cell.fg:
                    alone.glyph(0, 0, cell.text, cell.fg, cell.chip)
                if cell.bg:
                    alone.tint(0, 0, cell.bg)
                want = _painted(emit.emit(alone, lambda t: st(t, ''), tier)[0])[0]
                have = got[r][col]
                if blank:     # only the background and reverse/underline show on a blank
                    assert (have[2], have[3] & {4, 7}) == (want[2], want[3] & {4, 7}), (tier, r, col)
                else:
                    assert have == want, (tier, r, col)

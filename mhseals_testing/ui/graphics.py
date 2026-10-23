"""Terminal images (perception.md section 6): kitty graphics with Unicode placeholders, or half-blocks.

Pure and ROS-free: callers hand over PNG bytes (kitty) or an RGB pixel grid (half-blocks) and
write the returned strings themselves. The kitty upload goes to the real stdout after the Rich
refresh; the popup body is placeholder text, so Rich repaints never leave a ghost image.
"""

from rich.color import Color, ColorSystem
from rich.style import Style
from rich.text import Text

ESC = '\x1b'
ST = ESC + '\\'  # string terminator
PLACEHOLDER = '\U0010EEEE'
CHUNK = 4096  # kitty's limit on base64 bytes per escape
IMAGE_ID = 31  # one fixed id, so each frame replaces the pixels and keeps the placement
# The first kitty "rowcolumn diacritics", combining marks that encode a row or column index.
DIACRITICS = tuple(chr(c) for c in (
    0x305, 0x30D, 0x30E, 0x310, 0x312, 0x33D, 0x33E, 0x33F, 0x346, 0x34A, 0x34B, 0x34C, 0x350, 0x351, 0x352,
    0x357, 0x35B, 0x363, 0x364, 0x365, 0x366, 0x367, 0x368, 0x369, 0x36A, 0x36B, 0x36C, 0x36D, 0x36E, 0x36F))
MAX_ROWS = len(DIACRITICS)  # rows an image may span; only column 0 carries a diacritic, so columns are free
KITTY_TERMINALS = ('kitty', 'ghostty')


def kitty_enabled(graphics, xtversion):
    """True when the key reader saw an OK graphics reply (``reader.graphics``) and ``reader.xtversion`` is kitty or ghostty."""
    return bool(graphics) and (xtversion or '').lower().startswith(KITTY_TERMINALS)


def upload(png, cols, rows, image_id=IMAGE_ID):
    """The escape sequence that (re)transmits ``png`` as a placeholder image of cols x rows cells."""
    import base64
    data = base64.standard_b64encode(png).decode('ascii')
    chunks = [data[i:i + CHUNK] for i in range(0, len(data), CHUNK)] or ['']
    out = []
    for i, chunk in enumerate(chunks):
        more = int(i < len(chunks) - 1)
        head = f'a=T,f=100,U=1,i={image_id},c={cols},r={rows},q=2,m={more}' if i == 0 else f'q=2,m={more}'
        out.append(f'{ESC}_G{head};{chunk}{ST}')
    return ''.join(out)


def delete(image_id=IMAGE_ID):
    """The escape sequence that removes the image and frees its pixels (view close, resize, detach, atexit)."""
    return f'{ESC}_Ga=d,d=I,i={image_id},q=2{ST}'


def placeholder_rows(cols, rows, image_id=IMAGE_ID):
    """Rich ``Text`` lines: ``rows`` of ``cols`` placeholder cells, the id in the 256-colour foreground.

    Each row starts with the placeholder, its row diacritic and the column-0 diacritic; kitty infers
    the later columns. Ids 1-255 fit the 256-colour foreground. Rich measures one cell per placeholder.
    """
    if cols < 1 or not 1 <= rows <= MAX_ROWS:
        raise ValueError(f'image spans {cols}x{rows} cells; rows are limited to {MAX_ROWS}')
    if not 1 <= image_id <= 255:
        raise ValueError('image id must be 1-255')
    style = Style(color=Color.from_ansi(image_id))
    return [Text(PLACEHOLDER + DIACRITICS[r] + DIACRITICS[0] + PLACEHOLDER * (cols - 1), style=style)
            for r in range(rows)]


def _eight_bit(rgb):
    return Color.from_rgb(*rgb).downgrade(ColorSystem.EIGHT_BIT)


def half_blocks(pixels):
    """Fallback ``Text`` lines: one row per two pixel rows, '▀' with the top pixel as fg and bottom as bg.

    ``pixels`` is a list of rows of (r, g, b); an odd last row pairs with black. Colours are
    quantised to the 256-colour palette (health.md section A), so tmux and cx render them.
    """
    lines = []
    for y in range(0, len(pixels), 2):
        top = pixels[y]
        bottom = pixels[y + 1] if y + 1 < len(pixels) else [(0, 0, 0)] * len(top)
        line = Text()
        for t, b in zip(top, bottom):
            line.append('▀', Style(color=_eight_bit(t), bgcolor=_eight_bit(b)))
        lines.append(line)
    return lines

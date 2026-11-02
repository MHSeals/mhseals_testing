"""Kitty graphics: Unicode placeholder cells, so images flow, clip and move with the Rich layout.

Two uses. Pooled tiles (the core's surfaces): each tile is transmitted once as zlib RGBA under an id
picked from its pool slot, the id's low byte in the 256-colour foreground and its high byte in the
third diacritic. A whole image (the camera view, perception.md section 6): one fixed id, so each
frame replaces the pixels and keeps the placement, uploaded after the Rich refresh so a repaint
never leaves a ghost image.
"""

import base64
import zlib

from rich.color import Color
from rich.style import Style
from rich.text import Text

from mhseals_testing.ui.imaging.protocol import rgba as _rgba

KITTY_NAMES = ('kitty', 'ghostty')
ESC, ST = '\x1b', '\x1b\\'


PLACEHOLDER = '\U0010EEEE'


# kitty's row/column diacritics (the first 32 of its table)
DIACRITICS = tuple(chr(c) for c in (
    0x305, 0x30D, 0x30E, 0x310, 0x312, 0x33D, 0x33E, 0x33F, 0x346, 0x34A, 0x34B, 0x34C, 0x350, 0x351, 0x352,
    0x357, 0x35B, 0x363, 0x364, 0x365, 0x366, 0x367, 0x368, 0x369, 0x36A, 0x36B, 0x36C, 0x36D, 0x36E, 0x36F,
    0x483, 0x484))


def _kitty_id(n):
    """Pool slot ``n`` (1..POOL) as a kitty id: low byte 1..255 in the colour, high byte >= 1."""
    return ((1 + (n - 1) // 255) << 24) | (1 + (n - 1) % 255)


def _delete(image_id):
    return f'{ESC}_Ga=d,d=I,i={image_id},q=2{ST}'


def kitty_upload(image_id, tile, chunk=4096):
    """Transmit a tile as zlib RGBA with a virtual placement for its Unicode placeholders."""
    h, w = tile.px.shape
    data = base64.standard_b64encode(zlib.compress(_rgba(tile.px, tile.palette).tobytes(), 6)).decode('ascii')
    parts = [data[i:i + chunk] for i in range(0, len(data), chunk)] or ['']
    head = f'a=T,f=32,s={w},v={h},o=z,U=1,i={image_id},c={tile.cols},r={tile.rows},q=2'
    return ''.join(f'{ESC}_G{head if i == 0 else "q=2"},m={int(i < len(parts) - 1)};{p}{ST}'
                   for i, p in enumerate(parts))


def _kitty_env(env):
    return env.get('TERM') == 'xterm-kitty' or 'KITTY_WINDOW_ID' in env



CHUNK = 4096  # kitty's limit on base64 bytes per escape
IMAGE_ID = 31  # one fixed id, so each frame replaces the pixels and keeps the placement
MAX_ROWS = 30  # rows a whole image may span (its first 30 diacritics); only column 0 carries one


def kitty_enabled(graphics, xtversion):
    """True when the key reader saw an OK graphics reply (``reader.graphics``) and ``reader.xtversion`` is kitty or ghostty."""
    return bool(graphics) and (xtversion or '').lower().startswith(KITTY_NAMES)


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
    return _delete(image_id)


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


class Kitty:
    name, rank = 'kitty', 10

    def probe(self, setting, caps, env, colours):
        name = (caps.get('xtversion') or '').lower()
        return setting == 'kitty' or (setting == 'auto' and bool(caps.get('kitty'))
                                      and colours in ('256', 'truecolor', 'eight_bit')
                                      and (name.startswith(KITTY_NAMES) or (not name and _kitty_env(env))))

    def place(self, tile, r, c):
        kid = _kitty_id(tile.id)
        return PLACEHOLDER + DIACRITICS[r] + DIACRITICS[c] + DIACRITICS[kid >> 24], f'color({kid & 0xFF})'

    def clear(self, ids):
        return ''.join(_delete(_kitty_id(i)) for i in ids)

    def cost(self, tile):
        return kitty_upload(_kitty_id(tile.id), tile)


BACKEND = Kitty()

"""The text tier: widgets draw their braille dots (ui/braille.py) and nothing is uploaded; a picture
that must show anyway (the camera view) falls back to half blocks in the 256-colour palette.
"""

from rich.color import Color, ColorSystem
from rich.style import Style
from rich.text import Text


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


class Braille:
    name, rank = 'text', 99

    def probe(self, setting, caps, env, colours):
        return True         # the last resort: every terminal draws text

    def place(self, tile, r, c):
        return ' ', None

    def clear(self, ids):
        return ''

    def cost(self, tile):
        return ''


BACKEND = Braille()

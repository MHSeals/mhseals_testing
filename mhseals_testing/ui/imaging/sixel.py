"""Sixel: private-use marker cells that the core turns into cropped sixel images after each frame.

A marker is MARK0 + id * 128 + row * 16 + col; after Rich lays out a frame the core finds the markers,
splits each tile's visible cells into rectangles and draws those cropped (`sixel`), while its stream
turns the markers into cursor-forward so text never paints over an image.
"""

import re

import numpy as np

from mhseals_testing.ui.imaging.kitty import ESC, ST

MARK0 = 0xF0000            # sixel marker: MARK0 + id * 128 + row * 16 + col (Supplementary PUA-A)


_MARKS = re.compile('[\U000F0000-\U000FFFFD]+')


def sixel(px, palette):
    """A sixel image of indexed pixels ``px`` (0 transparent, P2=1) with run-length encoding."""
    h, w = px.shape
    used = [int(i) for i in np.unique(px) if i]
    head = f'{ESC}P0;1;0q"1;1;{w};{h}' + ''.join(
        f'#{i};2;{r * 100 // 255};{g * 100 // 255};{b * 100 // 255}' for i in used for r, g, b in (palette[i],))
    bands = -(-h // 6)
    stack = np.zeros((bands * 6, w), np.uint8)
    stack[:h] = px
    stack = stack.reshape(bands, 6, w)
    weights = (1 << np.arange(6, dtype=np.uint8))[None, :, None]
    # every colour's sixel characters for every band at once: (colour, band, x)
    chars = np.stack([((stack == i) * weights).sum(1, dtype=np.uint8) + 63 for i in used]) if used else []
    out = []
    for b in range(bands):
        rows = []
        for c, i in enumerate(used):
            line = chars[c, b].tobytes().rstrip(b'?')      # no trailing blanks; a colour absent here: none
            if line:
                rows.append(f'#{i}' + _RUN.sub(_run, line.decode('ascii')))
        out.append('$'.join(rows))
    while out and not out[-1]:       # no empty bands, and no line feed, below the last pixels
        out.pop()
    return head + '-'.join(out) + ST


_RUN = re.compile(r'(.)\1{3,}')     # four or more of a character go out as !N<char>


def _run(match):
    return f'!{len(match.group())}{match.group(1)}'


class Sixel:
    name, rank = 'sixel', 20

    def probe(self, setting, caps, env, colours):
        # a guessed cell lets tiles spill past their cells, over the arm and stop popups
        return bool(caps.get('cell')) and (setting == 'sixel' or bool(caps.get('sixel')))

    def place(self, tile, r, c):
        return chr(MARK0 + tile.id * 128 + r * 16 + c), ''

    def clear(self, ids):
        return ''           # a sixel image is pixels on the screen: the next frame paints over it

    def cost(self, tile):
        return sixel(tile.px, tile.palette)


BACKEND = Sixel()

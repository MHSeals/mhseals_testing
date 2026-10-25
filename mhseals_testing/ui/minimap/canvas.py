"""Map canvas cell resolution (map.md section 4.1); pure, no ROS and no colour codes.

Layers paint braille dots, glyphs, gap marks and background tints onto a grid of cells.
`Canvas.cells()` resolves each cell to one `Cell`: the layer names stay symbolic, so the
palette (and the colourless tiers) are applied later by the emitter.
"""

from collections import namedtuple
from functools import lru_cache

# Layer priority, highest first (map.md 4.1, tui-visual 2.3). A layer not listed ranks below all of these.
PRIORITY = (
    'chrome', 'contact', 'cursor', 'goal', 'home', 'label', 'bow', 'boat', 'track.vel',
    'object', 'dock_target',
    'ref_touched', 'ref_failed', 'ref_next', 'ref_passed', 'ref_pending', 'ref_skipped',
    'wake', 'fov', 'track.pred', 'wall', 'ring', 'grid', 'truth_dot', 'raw_det', 'truth_line', 'hits',
    'trail', 'acc', 'plan', 'local_plan', 'costmap',
)
_RANK = {name: i for i, name in enumerate(PRIORITY)}
GAP = '┄'
BLANK = '⠀'  # empty braille cell
# Braille dot bit for (dx, dy) inside a cell: 2 dots wide, 4 high.
BITS = {(0, 0): 0x01, (0, 1): 0x02, (0, 2): 0x04, (0, 3): 0x40,
        (1, 0): 0x08, (1, 1): 0x10, (1, 2): 0x20, (1, 3): 0x80}

# text/fg: the glyph and the layer (optionally ``layer.token``) owning its colour; chip: fg is painted as the cell's bg.
Cell = namedtuple('Cell', 'text fg chip bg')


@lru_cache(maxsize=1024)
def rank(layer):
    """Sort key where a smaller number wins. A ``layer:token`` suffix names the colour only;
    a dotted name ranks as itself when listed (``track.vel``), else by its head (``trail.gps.fix``)."""
    layer = layer.split(':')[0].split('@')[0]
    if layer in _RANK:
        return _RANK[layer]
    return _RANK.get(layer.split('.')[0], len(PRIORITY))


class Canvas:
    """A ``cols`` x ``rows`` grid of cells, each 2 x 4 braille dots."""

    def __init__(self, cols, rows):
        self.cols, self.rows = max(cols, 0), max(rows, 0)
        self._dots = {}    # (col, row) -> {bit: layer}
        self._glyphs = {}  # (col, row) -> (char, layer, chip)
        self._gaps = set()
        self._tint = {}    # (col, row) -> background token

    def copy(self):
        """An independent canvas with the same content, so a cached layer is never painted on."""
        other = Canvas(self.cols, self.rows)
        other._dots = {pos: dict(bits) for pos, bits in self._dots.items()}
        other._glyphs = dict(self._glyphs)
        other._gaps = set(self._gaps)
        other._tint = dict(self._tint)
        return other

    def overlay(self, other):
        """Merge ``other`` into this canvas by the same priority rules the painters use.

        Dot bits are ORed and each bit keeps the higher-priority layer; the higher-priority
        glyph wins a cell; gaps are unioned; ``other``'s tints replace ours.
        """
        for pos, bits in other._dots.items():
            mine = self._dots.setdefault(pos, {})
            for bit, layer in bits.items():
                if bit not in mine or rank(layer) < rank(mine[bit]):
                    mine[bit] = layer
        for pos, (char, layer, chip) in other._glyphs.items():
            old = self._glyphs.get(pos)
            if old is None or rank(layer) < rank(old[1]):
                self._glyphs[pos] = (char, layer, chip)
        self._gaps |= other._gaps
        self._tint.update(other._tint)

    @property
    def dots_wide(self):
        return self.cols * 2

    @property
    def dots_high(self):
        return self.rows * 4

    def _inside(self, col, row):
        return 0 <= col < self.cols and 0 <= row < self.rows

    def dot(self, x, y, layer):
        """OR one dot at dot coordinates (x right, y down); off-canvas dots are dropped."""
        col, row = x // 2, y // 4
        if not self._inside(col, row):
            return
        bits = self._dots.setdefault((col, row), {})
        bit = BITS[(x % 2, y % 4)]
        if bit not in bits or rank(layer) < rank(bits[bit]):
            bits[bit] = layer

    def dots(self, xs, ys, layers):
        """``dot`` for many points at once (the trail): same rules, one Python loop."""
        cols, rows, store = self.cols, self.rows, self._dots
        for x, y, layer in zip(xs, ys, layers):
            col, row = x // 2, y // 4
            if 0 <= col < cols and 0 <= row < rows:
                bits = store.setdefault((col, row), {})
                bit = BITS[(x % 2, y % 4)]
                if bit not in bits or rank(layer) < rank(bits[bit]):
                    bits[bit] = layer

    def glyph(self, col, row, char, layer, chip=False):
        """Replace a cell with ``char``; the higher-priority glyph keeps a contested cell."""
        if not self._inside(col, row):
            return
        old = self._glyphs.get((col, row))
        if old is None or rank(layer) < rank(old[1]):
            self._glyphs[(col, row)] = (char, layer, chip)

    def gap(self, col, row):
        """Mark a cell where the trail has no data; drawn only if nothing stronger is there."""
        if self._inside(col, row):
            self._gaps.add((col, row))

    def tint(self, col, row, token):
        """Set the background tint token of a cell (costmap); chips override it."""
        if self._inside(col, row):
            self._tint[(col, row)] = token

    def _resolve(self, pos):
        bg = self._tint.get(pos)
        if pos in self._glyphs:
            char, layer, chip = self._glyphs[pos]
            return Cell(char, layer, chip, None if chip else bg)
        bits = self._dots.get(pos)
        if bits:
            top = min(bits.values(), key=rank)
            if pos in self._gaps and rank(top) >= rank('trail'):
                return Cell(GAP, 'trail', False, bg)
            return Cell(chr(0x2800 + sum(bits)), top, False, bg)
        if pos in self._gaps:
            return Cell(GAP, 'trail', False, bg)
        return Cell(BLANK, None, False, bg)

    def cells(self):
        """Rows of resolved `Cell`s, top row first."""
        return [[self._resolve((c, r)) for c in range(self.cols)] for r in range(self.rows)]

    def text(self):
        """The canvas as plain strings, one per row, for tests and the colourless dump."""
        return [''.join(cell.text for cell in row) for row in self.cells()]

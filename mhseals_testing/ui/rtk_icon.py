"""The correction source as a small base-station icon inside a ring (gps.md 7 companion).

Ring colour = the rover's RTK state (theme tokens ``rtk.fixed`` / ``rtk.float`` / ``rtk.dgps`` /
``rtk.none``). Ring fill = how good the link is: survey progress while the base surveys, else the
correction freshness (full at <= 1 s, empty at the 5 s rtk.age fail), times the station distance
for NTRIP (full at <= 10 km, a quarter at 40 km and beyond). While the base is being identified
the lit segment turns (the cx spinner rate). Fill is drawn as shape, not colour, so NO_COLOR and
ASCII keep it: ``○◔◑◕●`` / ``.:oO@`` in one cell, and lit vs dashed segments in the 3-row icon.
"""

from rich.text import Text

from mhseals_testing.ui import theme
from mhseals_testing.ui.rtk_row import DEAD, _state

RING_TOKENS = {'FIXED': 'rtk.fixed', 'FLOAT': 'rtk.float', 'DGPS': 'rtk.dgps', '3D': 'rtk.dgps',
               '2D': 'rtk.none', 'none': 'rtk.none'}
FRESH = 1.0             # s: a 1 Hz stream is this old just before the next frame
NEAR_KM, FAR_KM, FAR_FILL = 10.0, 40.0, 0.25
TURN = ('◜◝◞◟', '|/-\\')
# 3x5 icon: the ring's 12 cells clockwise from the top middle, as (row, col)
PERIMETER = ((0, 2), (0, 3), (0, 4), (1, 4), (2, 4), (2, 3), (2, 2), (2, 1), (2, 0), (1, 0), (0, 0), (0, 1))
LIT = {(0, 0): '┏', (0, 4): '┓', (2, 0): '┗', (2, 4): '┛', 'h': '━', 'v': '┃'}
UNLIT = {(0, 0): '┌', (0, 4): '┐', (2, 0): '└', (2, 4): '┘', 'h': '┄', 'v': '┆'}


def distance_fill(km):
    if km is None or km <= NEAR_KM:
        return 1.0
    return max(FAR_FILL, 1.0 - (1.0 - FAR_FILL) * (km - NEAR_KM) / (FAR_KM - NEAR_KM))


def fill(view):
    """0..1 ring fill, or None while identifying (the ring turns instead)."""
    link, _ = _state(view)
    if link == 'identifying':
        return None
    if link == 'surveying':
        return max(0.0, min(1.0, view.survey))
    if link == 'external':
        return 1.0 if view.fix in ('FLOAT', 'FIXED') else 0.5      # no age to see: the FCU fix tells
    if link in ('off', 'unused', 'error') or view.age is None:
        return 0.0
    fresh = max(0.0, min(1.0, (DEAD - view.age) / (DEAD - FRESH)))
    return fresh * (distance_fill(view.station_km) if view.kind == 'ntrip' else 1.0)


def ring_style(view):
    return theme.PALETTE.style(RING_TOKENS.get(view.fix, 'rtk.none'))


def ring_glyph(view, ascii=False):
    """One cell: the ring filled by quarter, or a turning arc while identifying."""
    amount = fill(view)
    if amount is None:
        frames = TURN[1 if ascii else 0]
        return frames[view.tick % len(frames)]
    levels = theme.PALETTE.glyph('rtk_ring', ascii)
    return levels[_cells(amount, len(levels) - 1)]


def _cells(amount, total):
    """Lit cells, floored: the ring is full only at 100% (a 99% survey is not done)."""
    return int(amount * total + 1e-9)


def label(view):
    """What feeds the rover: ``Here4 site dock``, ``Here+ svin``, ``NTRIP FPRN/TAMP 12km`` ..."""
    if view.source == 'external' and view.kind != 'ntrip' and view.base in ('none', 'error'):
        return 'external'
    if view.source == 'none' and view.base in ('none', 'error'):
        return 'no corrections'
    if view.kind == 'ntrip':
        if not view.mount:
            return 'NTRIP picking'
        km = '' if view.station_km is None else f' {view.station_km:.0f}km'
        return f"NTRIP {view.mount}{' VRS' if view.vrs else ''}{km}"
    model = view.base_model or 'base'
    if view.base == 'none':
        return 'no base'
    if view.site:
        return f'{model} site {view.site}'
    return {'surveying': f'{model} svin', 'streaming': model}.get(view.base, f'{model} {view.base}')


def cell(view, ascii=False):
    """Status row value: the coloured ring, then the source label."""
    text = Text()
    text.append(ring_glyph(view, ascii), style=ring_style(view))
    text.append(' ' + label(view))
    return text


def source_row(view, width, ascii=False, cursor=None):
    """The Status row in the RTK row's grammar: gutter, the ring where the glyph goes, ``Source``, the label."""
    row = Text(' ')
    row.append(cursor if cursor else '  ', style='bold cyan')
    row.append_text(cell(view, ascii)[:1])
    row.append(' Source ')
    row.append(label(view))
    row.truncate(max(width - 1, 0), overflow='ellipsis' if not ascii else 'crop')
    return row


class SourceCell:
    """Rich renderable for the Status table, drawn at whatever width the column gives it."""

    def __init__(self, view, ascii=False, cursor=None):
        self.view, self.ascii, self.cursor = view, ascii, cursor

    def __rich_console__(self, console, options):
        yield source_row(self.view, options.max_width, self.ascii, self.cursor)


def icon(view, ascii=False):
    """Three Text lines, 5 cells wide: the base-station glyph inside its ring."""
    amount = fill(view)
    lit_cells = len(PERIMETER) if amount is None else _cells(amount, len(PERIMETER))
    turning = amount is None
    style, faint = ring_style(view), theme.PALETTE.style('rtk.unlit')
    grid = [[' '] * 5 for _ in range(3)]
    marks = [[None] * 5 for _ in range(3)]
    for index, (row, col) in enumerate(PERIMETER):
        on = (index - view.tick) % len(PERIMETER) < 3 if turning else index < lit_cells
        shape = LIT if on else UNLIT
        if ascii:
            char = '#' if on else '.'
        else:
            char = shape.get((row, col)) or shape['v' if row == 1 else 'h']
        grid[row][col], marks[row][col] = char, on
    lines = []
    for row in range(3):
        line = Text()
        for col in range(5):
            if row == 1 and col == 2:
                line.append(theme.PALETTE.glyph('rtk_base', ascii), style='bold')
            elif marks[row][col] is None:
                line.append(grid[row][col])
            else:
                line.append(grid[row][col], style=style if marks[row][col] else faint)
        lines.append(line)
    return lines


def pane_lines(view, ascii=False):
    """The icon beside the source label, the rover state and the ring's meaning, for the popup."""
    amount = fill(view)
    meaning = ('identifying the base' if amount is None else
               f'survey {amount:.0%}' if view.base == 'surveying' else
               f'link {amount:.0%}' + ('' if view.age is None else f' (age {view.age:.1f}s)'))
    beside = [Text(label(view), style='bold'), Text(f'rover {view.fix}', style=ring_style(view)), Text(meaning)]
    return [line + Text('  ') + extra for line, extra in zip(icon(view, ascii), beside)]

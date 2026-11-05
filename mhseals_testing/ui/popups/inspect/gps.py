"""Inspect section 4: the GPS fix ladder and the uncertainty circle drawn to scale."""

import math

from rich.console import Group
from rich.table import Table
from rich.text import Text

from mhseals_testing.analysis import fixlog
from mhseals_testing.ui import sparks, theme
from mhseals_testing.ui.popups.inspect.common import (
    FIX_STEPS, FIX_TOKENS, MOVING, SEP, WINDOW, duration, kv, length, mark, nice, recent, sensor, strip)


# --- 4. GPS fix and uncertainty ---------------------------------------------------------------

def fix_step(fix_type):
    return dict(zip(fixlog.STATES, FIX_STEPS)).get(fixlog.state_of({'fix_type': fix_type}), 'none')


def uncertainty(h_acc, points, cols, rows, style):
    """A to-scale box: the h_acc circle around the centre, positions as dots, and the metres per box.

    ``points`` are (east, north) metres from the centre. Returns (rows of Text, half-height m)."""
    grid = sparks.Dots(cols, rows, name='inspect.fix')
    k, w, h = grid.k, cols * grid.sx, rows * grid.sy
    cx, cy = (w - 1) / 2, (h - 1) / 2
    reach = max([h_acc or 0] + [math.hypot(e, n) for e, n in points])
    half = nice(1.25 * reach) if reach else 1.0
    ppm = (h / 2 - k) / half
    cross, dots = grid.layer(theme.MUTED), grid.layer(theme.PALETTE.style('slate'))
    circle = grid.layer(style)
    grid.segment(cross, cx - 2 * k, cy, cx + 2 * k, cy)
    grid.segment(cross, cx, cy - 2 * k, cx, cy + 2 * k)
    for e, n in points:
        x, y = round(cx + e * ppm), round(cy - n * ppm)
        if 0 <= x < w and 0 <= y < h:
            dots.add((x, y))
    if h_acc:
        grid.ring(circle, cx, cy, h_acc * ppm)
    return grid.rows_text(), half, ppm / k     # per braille dot (2 a cell), as the scale bar counts


BANDS = {'fix': 'RTK fixed', 'float': 'RTK float', 'dgps': 'DGPS', '3d': '3D', 'none': 'none'}


def gps_part(state, snapshot, inner):
    """Accuracy from the map's shared ``MapState.accuracy()`` when it has one (GPSRAW h_acc, else
    the navsat covariance), else the GPS raw row; the fix ladder; the to-scale circle; h_acc over time."""
    gps = sensor(snapshot, state.specs, 'gpsraw')
    shared = state.accuracy() if state.accuracy else None
    if not gps and not shared:
        return [kv('GPS', Text('no GPS raw sensor and no navsat covariance yet', style=theme.MUTED))]
    v = dict(gps['values']) if gps else {}
    step = fix_step(v['fix']) if 'fix' in v else BANDS.get(shared[1], 'none')
    if shared:
        v['h_acc'] = shared[0]
    style = theme.PALETTE.style(FIX_TOKENS[step])
    since = ''
    changes = state.fix_changes or []
    if changes:
        since = f' for {duration(state.wall() - changes[-1][0])}'
    pm = theme.pick('±', '+/-')
    out = [kv('Accuracy', Text(pm + length(v.get('h_acc')), style=f'bold {style}'),
              SEP, Text(step + since, style=style), f'{SEP}{v.get("sats", 0)} sats'
              + (f'{SEP}vel {length(v["vel_acc"])}/s' if v.get('vel_acc') else '')),
           kv('Fix', sparks.ladder(FIX_STEPS, step, {s: theme.PALETTE.style(t) for s, t in FIX_TOKENS.items()}))]
    now = state.now
    pts = [(e, n) for t, e, n in state.positions if t >= now - WINDOW]
    centre = (sum(e for e, _ in pts) / len(pts), sum(n for _, n in pts) / len(pts)) if pts else (0.0, 0.0)
    rel = [(e - centre[0], n - centre[1]) for e, n in pts]
    cols, rows = (25, 7) if state.compact else (31, 9)
    box, half, ppm = uncertainty(v.get('h_acc'), rel, cols, rows, style)
    bar_m = nice(half / 2)
    bar_cells = max(2, round(bar_m * ppm / 2))
    scale = Text(theme.pick('├', '|') + theme.RULE * (bar_cells - 2) + theme.pick('┤', '|'), style=theme.MUTED)
    scale.append(f' {length(bar_m)}', style=theme.MUTED)
    legend = [mark(style) + Text(f' h_acc {length(v.get("h_acc"))}'),
              Text(theme.pick('⠂', '.'), style=theme.PALETTE.style('slate'))
              + Text(f' {len(pts)} fixes, {WINDOW:.0f} s'),
              Text('+ mean position', style=theme.MUTED)]
    if len(rel) > 2:
        radii = sorted(math.hypot(e, n) for e, n in rel)
        legend.append(Text(f'  CEP50 {length(radii[len(radii) // 2])}'))
        legend.append(Text(f'  max   {length(radii[-1])}'))
    if v.get('speed') is not None and v['speed'] >= MOVING:
        legend.append(Text(f'  moving {v["speed"]:.1f} m/s: wander is travel', style=theme.WARN))
    legend += [Text(''), scale]
    table = Table.grid(padding=(0, 2))
    table.add_column(no_wrap=True)
    table.add_column(no_wrap=True)
    table.add_row(Group(*box), Group(*legend))
    out += [Text(''), table, Text('')]
    out += strip('h_acc', recent(state.series.get('h_acc', ()), now), now, inner, 'm', style=style, fmt=length)
    return out

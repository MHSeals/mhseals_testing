"""Inspect section 3: the Compass/Yaw heading dial and field gauge."""

import math

from rich.console import Group
from rich.table import Table
from rich.text import Text

from mhseals_testing.analysis import magnetic
from mhseals_testing.ui import sparks, theme
from mhseals_testing.ui.popups.inspect.common import MOVING, SEP, bearing, hue, kv, mark, number, sensor


# --- 3. compass dial --------------------------------------------------------------------------

def compass_heading(v, site=magnetic.SITE):
    """True heading of a level compass from /mavros/imu/mag (ROS FLU body, so y and z flip to FRD)."""
    return (magnetic.heading((v['x'], -v['y'], -v['z']), 0.0, 0.0) + site['declination_deg']) % 360


def dial(marks, cols, rows):
    """A braille rose ``cols`` x ``rows`` cells: cardinal letters and ticks, a needle for the first
    mark and radial ticks for the rest. ``marks`` is [(bearing deg or None, style, dotted)]."""
    grid = sparks.Dots(cols, rows, name='inspect.dial')
    k, w, h = grid.k, cols * grid.sx, rows * grid.sy     # 2 x 4 dots a cell, or 2K x 4K as an image
    cx, cy = (w - 1) / 2, (h - 1) / 2
    r = min(w / 2 - 3 * k, h / 2 - 5 * k)
    ring = ticks = grid.layer(theme.PALETTE.style('faint'))
    grid.ring(ring, cx, cy, r)

    def at(deg, f):
        a = math.radians(deg)
        return cx + f * r * math.sin(a), cy - f * r * math.cos(a)

    for deg in range(0, 360, 45):
        grid.segment(ticks, *at(deg, 0.75 if deg % 90 == 0 else 0.85), *at(deg, 1.0))
    for i, (deg, style, dotted) in enumerate(reversed(marks)):
        if deg is None:
            continue
        layer = grid.layer(style)
        needle = i == len(marks) - 1
        grid.segment(layer, *at(deg, 0.0 if needle else 0.45), *at(deg, 0.95 if needle else 1.0))
        if dotted:      # shape-coded, so the mark reads without colour
            layer.difference_update(d for i, d in enumerate(sorted(layer)) if (i // k) % 2)
    ccol, crow = int(cx // grid.sx), int(cy // grid.sy)
    half_cols, half_rows = int(r / grid.sx) + 1, int(r / grid.sy) + 1
    grid.label(ccol, max(0, crow - half_rows), 'N', f'bold {theme.ACCENT}')
    grid.label(ccol, min(rows - 1, crow + half_rows), 'S', theme.MUTED)
    grid.label(max(0, ccol - half_cols - 1), crow, 'W', theme.MUTED)
    grid.label(min(cols - 1, ccol + half_cols + 1), crow, 'E', theme.MUTED)
    return grid.rows_text()


def compass_part(snapshot, specs, inner, compact=False, site=magnetic.SITE):
    mag = sensor(snapshot, specs, 'mag')
    if not mag or mag['values'].get('x') is None:
        return [kv('Compass', Text('no compass reading', style=theme.MUTED))]
    v = mag['values']
    odom = sensor(snapshot, specs, 'odometry')
    gps = sensor(snapshot, specs, 'gpsraw')
    compass = compass_heading(v, site)
    yaw = (90 - math.degrees(odom['values']['yaw'])) % 360 \
        if odom and odom['values'].get('yaw') is not None and odom['status'] == 'LIVE' else None
    g = (gps or {}).get('values', {})
    course = g.get('cog') if g.get('speed') is not None and g['speed'] >= MOVING else None
    # Neutral marks, never an authority or scope hue: bold yaw, dotted muted course (dial.* when themed).
    styles = (theme.ACCENT, hue('dial.yaw', 'bold'), hue('dial.course', theme.MUTED))
    cols, rows = (15, 7) if compact else (21, 9)
    rose = dial([(compass, styles[0], False), (yaw, styles[1], False), (course, styles[2], True)], cols, rows)
    known = [d for d in (compass, yaw, course) if d is not None]
    spread = max(abs(magnetic.wrap(a - b)) for a in known for b in known)
    legend = [Text(bearing(compass), style=f'bold {theme.ACCENT}') + Text('  compass', style=theme.MUTED),
              Text(''),
              mark(styles[0]) + Text(f' Compass  {bearing(compass)}'),
              mark(styles[1]) + Text(f' Yaw      {bearing(yaw)}' if yaw is not None else ' Yaw      ' + theme.DASH),
              mark(styles[2], True) + Text(f' Course   {bearing(course)}  {g["speed"]:.1f} m/s' if course is not None
                                           else ' Course   stationary' if g else ' Course   no GPS'),
              Text(theme.ascii_safe(f"  Spread   {theme.pick('Δ', 'd')} {spread:.0f}°") if len(known) > 1 else '',
                   style=theme.WARN if spread > magnetic.DISAGREE_MAX else None)]
    if mag['status'] != 'LIVE':
        legend.append(Text(f"  compass {mag['status'].lower()}", style=theme.WARN))
    table = Table.grid(padding=(0, 2))
    table.add_column(no_wrap=True)
    table.add_column(no_wrap=True)
    table.add_row(Group(*rose), Group(*legend))
    field = magnetic.field_mg(v['x'], v['y'], v['z'])
    ratio = field / site['field_mg']
    ok = magnetic.FIELD_RATIO[0] <= ratio <= magnetic.FIELD_RATIO[1]
    band = tuple(f * site['field_mg'] for f in magnetic.FIELD_RATIO)
    gauge = sparks.hbar(field, 0, 2 * site['field_mg'], 24, band=band, style=theme.GOOD if ok else theme.BAD)
    quality, reason = magnetic.yaw_quality(v.get('compass_gyro'), v.get('magcal_residual'), v.get('yaw_sigma'), ratio)
    checks = [f"comp–gyro {number(v.get('compass_gyro'))}°", f"magcal {number(v.get('magcal_residual'))}°",
              f"σ {math.degrees(v['yaw_sigma']):.1f}°" if v.get('yaw_sigma') is not None else f'σ {theme.DASH}']
    return [table, Text(''),
            kv('Field', gauge, f"  {field:.0f}/{site['field_mg']:.0f} mG  ({ratio:.2f}{theme.pick('×', 'x')})"),
            kv('Yaw', Text(theme.ascii_safe(reason) or 'ok', style=theme.GOOD if quality == 'ok' else theme.WARN),
               Text(SEP + theme.ascii_safe(SEP.join(checks)), style=theme.MUTED))]

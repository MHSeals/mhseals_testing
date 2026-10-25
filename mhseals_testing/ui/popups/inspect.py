"""popup.inspect: Enter on a Status row that has no popup of its own (tui-visual.md section 4).

The popup picks a builder by the row: a sensor gets its 60 s arrival-rate strip, a TF chain its
frame diagram, Compass/Yaw a heading dial and field gauge, GPS the fix ladder and an uncertainty
circle drawn to scale, RC its rate and latency, any other advisory its number over time. Each body
ends with the row's Detail line.

Everything is sampled only while the popup is open (``Inspect.update`` from the popup refresh),
into rings that die with it, so a closed popup costs nothing. Builders are pure functions of the
``Inspect`` state, the snapshot and the inner width, which the tests drive directly.
"""

from bisect import bisect_left, bisect_right
from collections import deque
import math
import time

from rich.console import Group
from rich.table import Table
from rich.text import Text

from mhseals_testing.analysis import fixlog, magnetic
from mhseals_testing.profiles import MIN_DEADLINE
from mhseals_testing.runner import fold
from mhseals_testing.ui import sparks, theme
from mhseals_testing.ui.panels.status import short, status_rows

WINDOW = 60.0       # s shown by every strip
RING = 600          # samples kept per series while open
ARRIVALS = 12000    # receive times kept while open (60 s at 200 Hz)
EVERY = 0.25        # s between rebuilds; the popup refreshes at 10 Hz
MOVING = 0.3        # m/s of GPS speed above which its course is drawn
WIDTH = 76
LABEL = 9           # label column
SEP = theme.SEPARATOR
FIX_STEPS = ('none', '3D', 'DGPS', 'RTK float', 'RTK fixed')
FIX_TOKENS = dict(zip(FIX_STEPS, ('gps.none', 'gps.3d', 'gps.dgps', 'gps.float', 'gps.fix')))
POINTS = ('N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW')


# --- formatting ------------------------------------------------------------------------------

def length(metres):
    """``2 cm``, ``0.15 m``, ``12 m``: whole cm (at least 1, as the minimap's ``accuracy_text``) below
    10 cm, else 2 significant figures; the unit is picked from the rounded value."""
    if metres is None or not math.isfinite(metres):
        return theme.DASH
    cm = round(metres * 100)
    if cm < 10:
        return f'{max(1, cm)} cm'
    rounded = float(f'{max(metres, 0.1):.2g}')
    return f'{rounded:g} m' if rounded < 10 else f'{metres:.0f} m'


def duration(seconds):
    seconds = max(0, int(seconds))
    if seconds < 60:
        return f'{seconds} s'
    if seconds < 3600:
        return f'{seconds // 60} min {seconds % 60:02d} s'
    return f'{seconds // 3600} h {seconds // 60 % 60:02d} min'


def cardinal(deg):
    return POINTS[round(deg % 360 / 22.5) % 16]


def bearing(deg):
    """``247° WSW``."""
    return theme.ascii_safe(f'{deg % 360:03.0f}° {cardinal(deg)}')


def nice(value):
    """The 1-2-5 step at or above ``value``."""
    if value <= 0 or not math.isfinite(value):
        return 1.0
    base = 10 ** math.floor(math.log10(value))
    return next(step * base for step in (1, 2, 5, 10) if step * base >= value * 0.999)


def number(value):
    if value is None:
        return theme.DASH
    if isinstance(value, float):
        return f'{value:.8g}' if abs(value) >= 10 else f'{value:.3g}'
    return str(value)


def kv(label, *parts):
    """A label column in the popup-row style, then the value parts (str or Text), wrapping under itself."""
    text = Text()
    for part in parts:
        text.append_text(part) if isinstance(part, Text) else text.append(str(part))
    table = Table.grid()
    table.add_column(width=LABEL, style=f'bold {theme.ACCENT}', no_wrap=True)
    table.add_column(ratio=1)
    table.add_row(label, text)
    return table


def mark(style, dotted=False):
    return Text(theme.pick('┅', ':') if dotted else theme.pick('━', '='), style=style)


def hue(token, fallback):
    """The style of ``token`` when the theme defines it, else of ``fallback`` (a plain Rich style)."""
    return theme.PALETTE.style(token) if token in theme.PALETTE.tokens else fallback


# --- sampling ---------------------------------------------------------------------------------

def find(rows, row):
    """The current version of ``row`` among fresh status rows (same name and source)."""
    return next((r for r in rows if r['name'] == row['name'] and r.get('source') == row.get('source')), row)


class Inspect:
    """The rings one open popup fills; ``update`` samples, ``body`` draws."""

    def __init__(self, row, specs, monitor=None, live=True, publishers=None, fix_changes=None,
                 compact=False, wall=time.time):
        self.row, self.specs, self.monitor, self.live = row, specs, monitor, live
        self.publishers, self.fix_changes, self.compact, self.wall = publishers, fix_changes, compact, wall
        self.spec = next((s for s in specs if s.name == row['name']), None) if row.get('source') == 'sensor' else None
        self.times = deque(maxlen=ARRIVALS)
        self.series = {}      # name -> deque of (t, value)
        self.positions = deque(maxlen=RING)   # (t, east m, north m) of GPS raw
        self.origin = None
        self.accuracy = None  # MapState.accuracy, () -> (h_acc m, band) or None, when the map feeds exist
        self.now = 0.0

    def watched(self):
        """The profile sensor whose arrivals the strip shows, or None."""
        if self.spec:
            return self.spec
        want = {'Compass': 'mag', 'Yaw': 'mag', 'GPS': 'gpsraw'}.get(self.row['name'])
        return next((s for s in self.specs if s.kind == want), None) if want else None

    def push(self, name, t, value):
        if value is not None and math.isfinite(value):
            ring = self.series.setdefault(name, deque(maxlen=RING))
            if not ring or t > ring[-1][0]:
                ring.append((t, float(value)))

    def update(self, snapshot, now):
        """Take what arrived since the last call; only ever called while the popup is open."""
        self.now = now
        spec = self.watched()
        if spec and self.monitor is not None and self.live:
            self.times.extend(self.monitor.arrivals(spec.name, self.times[-1] if self.times else float('-inf')))
        gps = sensor(snapshot, self.specs, 'gpsraw')
        if gps and (self.row['name'] == 'GPS' or (self.spec and self.spec.kind == 'gpsraw')):
            v = gps['values']
            self.push('h_acc', gps['received'], v.get('h_acc'))
            if v.get('lat') is not None and (not self.positions or gps['received'] > self.positions[-1][0]):
                if self.origin is None:
                    self.origin = (v['lat'], v['lon'])
                north = math.radians(v['lat'] - self.origin[0]) * 6371000.0
                east = math.radians(v['lon'] - self.origin[1]) * 6371000.0 * math.cos(math.radians(v['lat']))
                self.positions.append((gps['received'], east, north))
        own = snapshot['sensors'].get(self.spec.name) if self.spec else None
        if own and self.spec.kind == 'imu':
            self.push('wz', own['received'], own['values'].get('wz'))
        elif own and self.spec.kind == 'odometry' and own['values'].get('vx') is not None:
            self.push('speed', own['received'], math.hypot(own['values']['vx'], own['values'].get('vy') or 0.0))
        if self.row['name'] == 'RC':
            rc = (snapshot.get('control') or {}).get('rc') or {}
            self.push('rate', now, rc.get('rate_hz'))
            self.push('latency', now, rc.get('latency_ms'))
        elif self.row.get('group') == 'System':
            row = find(status_rows(snapshot, self.specs), self.row)   # the detail now, not at open
            self.push('value', now, lead(short(row.get('detail', ''))))


def lead(token):
    """The leading number of ``'+0.04s'`` or ``'78°C'``; None without one."""
    digits = ''
    for char in str(token):
        if char.isdigit() or char in '+-.' and not digits.endswith(('.',)):
            digits += char
        elif digits:
            break
    try:
        return float(digits)
    except ValueError:
        return None


def sensor(snapshot, specs, kind):
    for spec in specs:
        if spec.kind == kind:
            return snapshot['sensors'].get(spec.name)
    return None


def recent(ring, now):
    return [(t, v) for t, v in ring if t >= now - WINDOW]


def columns(series, now, cells):
    """``series`` [(t, v)] as one value per dot column over the last WINDOW seconds (last sample wins)."""
    n = cells * 2
    out = [None] * n
    for t, v in series:
        x = min(n - 1, int((t - (now - WINDOW)) / WINDOW * n))
        if 0 <= x:
            out[x] = v
    return out


def strip(label, series, now, cells, unit, band=None, style=None, fmt=None):
    """A labelled 2-row spark of ``series`` with its range (``fmt`` formats a value), or a note while it fills."""
    fmt = fmt or (lambda v: f'{v:.3g} {unit}'.strip())
    values = [v for _, v in series]
    if not values:
        return [kv(label, Text('waiting for samples', style=theme.MUTED))]
    lo, hi = min(values + list(band or ())), max(values + list(band or ()))
    pad = (hi - lo) * 0.1 or abs(hi) * 0.1 or 1.0
    lo, hi = lo - pad, hi + pad
    rows = sparks.spark(columns(series, now, cells - LABEL), cells - LABEL, lo, hi, band=band, style=style)
    span = f"{fmt(min(values))} {theme.pick('–', 'to')} {fmt(max(values))}"
    head = kv(label, f'{fmt(values[-1])} now{SEP}{span}{SEP}{WINDOW:.0f} s')
    return [head] + [kv('', r) for r in rows]


# --- 1. sensor rate strip ---------------------------------------------------------------------

def rate_columns(times, now, cells, hz, gap):
    """(Hz per dot column, gap flag per column) over the last WINDOW seconds of arrival ``times``."""
    n = cells * 2
    win = max(1.0, 5 / hz) if hz else 2.0
    rates, gaps = [], []
    for x in range(n):
        tc = now - WINDOW + (x + 1) * WINDOW / n
        if not times or tc < times[0]:
            rates.append(None)
            gaps.append(False)
            continue
        count = bisect_right(times, tc) - bisect_right(times, tc - win)
        rates.append(count / win if tc - win >= times[0] else None)
        after = bisect_right(times, tc)
        prev, nxt = times[after - 1], times[after] if after < len(times) else now
        gaps.append(nxt - prev > gap)
    return rates, gaps


def sensor_part(state, spec, row, inner):
    """Rate strip, rate figures, topic and publishers, then the latest values."""
    times, now = list(state.times), state.now
    times = times[bisect_left(times, now - WINDOW):]
    hz = spec.hz
    gap = max(3 / hz, MIN_DEADLINE) if hz else (spec.deadline or 2.0)
    cells = inner - 5
    out = []
    if not state.live:
        out.append(kv('Rate', Text('review: no arrivals are kept for a past session', style=theme.MUTED)))
    elif len(times) < 2:
        out.append(kv('Rate', Text(f'no arrivals in the last {WINDOW:.0f} s', style=theme.BAD)))
    else:
        rates, gaps = rate_columns(times, now, cells, hz, gap)
        seen = [r for r in rates if r is not None]
        dts = [b - a for a, b in zip(times, times[1:])]
        median = sorted(dts)[len(dts) // 2]
        jitter = sorted(abs(d - median) for d in dts)[int(0.95 * (len(dts) - 1))]
        hi = math.ceil(max([(hz or 0) * 1.3] + [r * 1.1 for r in seen] + [1.0]))
        stamp = (row or {}).get('stamp_age')
        figures = [f"{(row or {}).get('hz', 0):.0f} Hz"]
        if seen:
            figures.append(f'{min(seen):.0f}–{max(seen):.0f}')
        figures.append(f'max gap {max(dts):.2f} s')
        if stamp is not None:
            figures.append(f'stamp {stamp * 1000:.0f} ms')
        figures.append(f"{theme.pick('±', '+/-')}{jitter * 1000:.0f} ms p95")
        out.append(kv('Rate', theme.ascii_safe(SEP.join(figures))))
        band = (0.85 * hz,) if hz else None      # the warn floor of topic_rate.classify
        lines = sparks.spark(rates, cells, 0, hi, band=band, gaps=gaps, height=3)
        axis = theme.pick('┤', '|')
        for value, line in zip((f'{hi:>3.0f} {axis}', f'    {theme.BAR}', f'{0:>3.0f} {axis}'), lines):
            out.append(Text(value, style=theme.MUTED) + line)
        tail = (f"expected {hz:g} Hz{SEP}{theme.pick('┈', '-')} {0.85 * hz:.3g} warn floor" if hz
                else f'typical {1 / median:.3g} Hz (no nominal rate)')
        axis_line = Text(f'    -{WINDOW:.0f} s', style=theme.MUTED)
        axis_line.append(' ' * max(1, cells - len(tail) - 10) + tail + '  now', style=theme.MUTED)
        out.append(axis_line)
        if any(gaps):
            note = f"{theme.pick('┄', '.')} silence over {gap:.1f} s (the stale deadline)"
            out.append(kv('', Text(note, style=theme.BAD)))
    pubs = state.publishers
    who = (f'{len(pubs)} publisher{"s" * (len(pubs) != 1)}' + (f' ({", ".join(pubs[:3])})' if pubs else '')
           if pubs is not None else 'publishers unknown')
    out.append(kv('Topic', f'{spec.topic}{SEP}{who}'))
    values = {k: v for k, v in ((row or {}).get('values') or {}).items() if k != 'summary'}
    if values:
        out.append(kv('Values', SEP.join(f'{k} {number(v)}' for k, v in values.items())))
    return out


# --- 2. TF chain ------------------------------------------------------------------------------

def chain(edges, source, target):
    """The hops source -> target over every edge ever seen: ([(parent, child, [edge dicts])], found).

    Without a full chain the hops reach the deepest frame source does reach (the partial chain)."""
    by_parent = {}
    for edge in edges:
        by_parent.setdefault(edge['parent'], {}).setdefault(edge['child'], []).append(edge)
    pending, seen, deepest = [(source, [])], {source}, []
    while pending:
        frame, path = pending.pop(0)
        if frame == target:
            return path, True
        deepest = path if len(path) > len(deepest) else deepest
        for child, group in sorted(by_parent.get(frame, {}).items()):
            if child not in seen:
                seen.add(child)
                pending.append((child, path + [(frame, child, group)]))
    return deepest, False


def tf_part(edges, source, target, inner):
    """``map ━━▶ odom ━━▶ base_link`` with each hop's publisher and age; a missing hop is ``╌✗╌`` with the
    publisher once seen sending the target, if any; every edge into the chain below."""
    hops, found = chain(edges, source, target)
    line, pubs, ages = Text(source), Text(' ' * len(source)), Text(' ' * len(source))

    def hop(arrow, child, who, age, style, who_style, age_style, child_style=None):
        width = max(len(arrow) + len(child), len(who) + 2, len(age) + 2)
        line.append(arrow, style=style)
        line.append(child.ljust(width - len(arrow)), style=child_style)
        pubs.append(('  ' + who).ljust(width), style=who_style)
        ages.append(('  ' + age).ljust(width), style=age_style)

    for parent, child, group in hops:
        fresh = [e for e in group if e['fresh']]
        names = sorted({e['authority'] or '?' for e in fresh or group})
        style = theme.BAD if not fresh else theme.WARN if len(names) > 1 else theme.GOOD
        best = min(group, key=lambda e: e['seen'])
        age = ('static' if best['static'] else f"{best['stamp_age']:.2f} s dyn") if fresh \
            else f"last seen {best['seen']:.0f} s ago"
        hop(f" {theme.pick('━━▶', '-->')} ", child, f'! {len(names)} publishers' if len(names) > 1 else names[0],
            age, style, style if len(names) > 1 else theme.MUTED, theme.MUTED if fresh else theme.BAD)
    if not found:
        into = [e for e in edges if e['child'] == target]
        known = sorted({e['authority'] or '?' for e in into})
        who = f"expected: {', '.join(known)}" if known else f'no publisher of {target} seen'
        last = f"last seen {min(e['seen'] for e in into):.0f} s ago" if into else 'never seen'
        hop(f" {theme.pick('╌✗╌', '-x-')} ", target, who, last, theme.BAD, theme.MUTED, theme.BAD, theme.BAD)
    out = [line, pubs, ages]
    frames = {source, target} | {child for _, child, _ in hops}
    table = theme.grid({'header': 'edge'}, {'header': 'publisher'}, {'header': 'age', 'justify': 'right'},
                       {'header': ''})
    for e in sorted(edges, key=lambda e: (e['child'], e['parent'], e['authority'])):
        if e['child'] in frames:
            state = 'OK' if e['fresh'] else 'STALE'
            table.add_row(f"{e['parent']} {theme.ARROW} {e['child']}", e['authority'] or '?',
                          'static' if e['static'] else f"{e['stamp_age']:.2f} s", theme.icon(state))
    return out + [Text(''), table]


# --- 3. compass dial --------------------------------------------------------------------------

def compass_heading(v, site=magnetic.SITE):
    """True heading of a level compass from /mavros/imu/mag (ROS FLU body, so y and z flip to FRD)."""
    return (magnetic.heading((v['x'], -v['y'], -v['z']), 0.0, 0.0) + site['declination_deg']) % 360


def dial(marks, cols, rows):
    """A braille rose ``cols`` x ``rows`` cells: cardinal letters and ticks, a needle for the first
    mark and radial ticks for the rest. ``marks`` is [(bearing deg or None, style, dotted)]."""
    grid = sparks.Dots(cols, rows)
    w, h = cols * 2, rows * 4
    cx, cy = (w - 1) / 2, (h - 1) / 2
    r = min(w / 2 - 3, h / 2 - 5)
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
            layer.difference_update(sorted(layer)[1::2])
    ccol, crow = int(cx // 2), int(cy // 4)
    half_cols, half_rows = int(r / 2) + 1, int(r / 4) + 1
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


# --- 4. GPS fix and uncertainty ---------------------------------------------------------------

def fix_step(fix_type):
    return dict(zip(fixlog.STATES, FIX_STEPS)).get(fixlog.state_of({'fix_type': fix_type}), 'none')


def uncertainty(h_acc, points, cols, rows, style):
    """A to-scale box: the h_acc circle around the centre, positions as dots, and the metres per box.

    ``points`` are (east, north) metres from the centre. Returns (rows of Text, half-height m)."""
    grid = sparks.Dots(cols, rows)
    w, h = cols * 2, rows * 4
    cx, cy = (w - 1) / 2, (h - 1) / 2
    reach = max([h_acc or 0] + [math.hypot(e, n) for e, n in points])
    half = nice(1.25 * reach) if reach else 1.0
    ppm = (h / 2 - 1) / half
    cross, dots = grid.layer(theme.MUTED), grid.layer(theme.PALETTE.style('slate'))
    circle = grid.layer(style)
    grid.segment(cross, cx - 2, cy, cx + 2, cy)
    grid.segment(cross, cx, cy - 2, cx, cy + 2)
    for e, n in points:
        x, y = round(cx + e * ppm), round(cy - n * ppm)
        if 0 <= x < w and 0 <= y < h:
            dots.add((x, y))
    if h_acc:
        grid.ring(circle, cx, cy, h_acc * ppm)
    return grid.rows_text(), half, ppm


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


# --- 5. RC and advisories ---------------------------------------------------------------------

def rc_part(state, snapshot, inner):
    rc = (snapshot.get('control') or {}).get('rc')
    if not rc:
        return [kv('RC', Text('no RC_CHANNELS seen', style=theme.MUTED))]
    out = [kv('Mode', rc.get('mode') or theme.DASH, SEP, 'valid' if rc.get('valid', True) else 'lost')]
    out += strip('Rate', recent(state.series.get('rate', ()), state.now), state.now, inner, 'Hz')
    out += strip('Latency', recent(state.series.get('latency', ()), state.now), state.now, inner, 'ms')
    for i, us in enumerate((rc.get('channels') or [])[:8]):
        out.append(kv(f'ch{i + 1}', sparks.hbar(us, 1000, 2000, 30), f"  {us} {theme.pick('µs', 'us')}"))
    return out


def advisory_part(state, snapshot, inner):
    adv = snapshot.get('advisories', {}).get(state.row['name'])
    out = []
    if adv:
        out.append(kv('Level', theme.labelled(adv['level']), f'{SEP}updated {duration(adv["age"])} ago'))
    if state.series.get('value'):
        out += strip('Value', recent(state.series['value'], state.now), state.now, inner, '')
    return out


# --- the popup --------------------------------------------------------------------------------

def body(state, snapshot, inner=WIDTH - 4, edges=()):
    """The renderables for ``state.row`` (refreshed from ``snapshot``)."""
    row = find(status_rows(snapshot, state.specs), state.row)
    name, out = row['name'], []
    spec = state.watched()
    if row.get('source') == 'tf':
        source, _, target = name.partition(f' {theme.ARROW} ')
        out += tf_part(edges, source, target, inner)
    elif name in ('Compass', 'Yaw') or (state.spec and state.spec.kind == 'mag'):
        out += compass_part(snapshot, state.specs, inner, state.compact)
    elif name == 'GPS' or (state.spec and state.spec.kind == 'gpsraw'):
        out += gps_part(state, snapshot, inner)
    elif name == 'RC':
        out += rc_part(state, snapshot, inner)
    elif row.get('group') == 'System':
        out += advisory_part(state, snapshot, inner)
    elif state.spec and state.spec.kind in ('imu', 'odometry'):
        key, label, unit = ('wz', 'Gyro z', 'rad/s') if state.spec.kind == 'imu' else ('speed', 'Speed', 'm/s')
        out += strip(label, recent(state.series.get(key, ()), state.now), state.now, inner, unit)
    if spec is not None:
        if out:
            out.append(Text(''))
        out += sensor_part(state, spec, snapshot['sensors'].get(spec.name), inner)
    out.append(kv('Detail', theme.ascii_safe(str(row.get('detail', '')))))
    return Group(*out)


def title(row, spec):
    return f"{row['name']}{SEP}{spec.topic}" if spec else row['name']


def open_inspect(app, row=None):
    """Open the inspect popup on ``row`` (default: the Status cursor row); any key but j/k closes."""
    specs = app.profile.sensors
    if row is None:
        rows = status_rows(app.snapshot(), specs)
        if not rows:
            return
        row = rows[min(app.cursors['status'], len(rows) - 1)]
    live = bool(getattr(app.source, 'live', True))
    probe = Inspect(row, specs, app.monitor, live, compact=app.console.size.height < 32 or app.console.size.width < 100)
    spec = probe.watched()
    if spec and live:
        try:   # one graph query at open, never per refresh
            probe.publishers = list(app.ros.publishers(spec.topic))
        except Exception:   # noqa: BLE001  no graph (review, tests): say unknown
            probe.publishers = None
    feeds = getattr(app.ros, 'map', None) if live else None
    probe.fix_changes = getattr(feeds, 'fix_changes', None)
    probe.accuracy = getattr(getattr(feeds, 'state', None), 'accuracy', None)   # the minimap's shared reading
    modal = dict(kind='message', title=title(row, spec), width=WIDTH, hint='Any key closes')   # j/k added when clipped
    last = [None]

    def refresh():
        now = app.monitor.clock()
        if last[0] is not None and now - last[0] < EVERY:
            return
        last[0] = now
        snapshot = app.snapshot()
        probe.update(snapshot, now)
        edges = app.monitor.tf_edges() if row.get('source') == 'tf' and live else ()
        modal['body'] = body(probe, snapshot, WIDTH - 4, edges)

    def handle(key):
        return not (modal.get('clipped') and app.scroll(modal, fold(key), ('k', 'j')))
    refresh()
    app.popup(modal, handle, refresh)

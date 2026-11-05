"""popup.inspect's shared pieces: sizes, number and length formatting, and the sampling rings (``Inspect``)
with the strip every section draws."""

import math
import time
from collections import deque
from rich.table import Table
from rich.text import Text

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

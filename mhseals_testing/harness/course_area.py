"""The day's course area: lat/lon corners the operator sets in the TUI (owner decision 2026-11-01). ROS-free.

fence_watchdog (mhseals_nav) enforces it: ``course.corners`` [lat1, lon1, lat2, lon2, ...] is a live parameter,
validated (at least 3 corners, a simple polygon) and applied whole or refused; ``on_exit`` (alert | return) is live
too. The TUI applies through SetParameters, reads the corners back and keeps them in the operator's fence file
(``~/.config/mhseals/fence.yaml``, ``course: {corners: [...]}``), which fence_watchdog reads at start.

``Corners`` is the editor's model: add a corner at the boat's GPS fix, remove, reorder and nudge one by metres
north or east. ``status_line`` turns /fence/status into what the status area says; ``preview`` places corners in the
map frame through the boat's own fix and map position, for the minimap's dashed preview.
"""

import math
from pathlib import Path

import yaml

FENCE_FILE = Path('~/.config/mhseals/fence.yaml')
EARTH_M = 6378137.0
ON_EXIT = ('alert', 'return')


def metres(lat):
    """(m per degree of latitude, m per degree of longitude) at ``lat``."""
    return math.radians(1) * EARTH_M, math.radians(1) * EARTH_M * math.cos(math.radians(lat))


def offset(corner, north, east):
    """``corner`` (lat, lon) moved ``north`` and ``east`` metres."""
    per_lat, per_lon = metres(corner[0])
    return corner[0] + north / per_lat, corner[1] + east / max(per_lon, 1e-9)


def enu(origin, corner):
    """(east, north) metres of ``corner`` from ``origin``, both (lat, lon); flat earth, fine over a course."""
    per_lat, per_lon = metres(origin[0])
    return (corner[1] - origin[1]) * per_lon, (corner[0] - origin[0]) * per_lat


def problem(corners):
    """Why ``corners`` is no course area ('' when it is): fence_watchdog's own check when importable."""
    flat = flatten(corners)
    try:
        from mhseals_nav.fence_watchdog import problem as watchdog_problem
    except ImportError:
        return '' if len(corners) >= 3 else 'at least 3 corners'
    return watchdog_problem('latlon', flat)


def parse(words):
    """``(corners, '')`` from ``LAT,LON`` words, or ``([], why)``."""
    corners = []
    for word in words:
        try:
            lat, lon = (float(v) for v in str(word).split(','))
        except ValueError:
            return [], f'{word!r}: expected LAT,LON in degrees'
        if not (math.isfinite(lat) and math.isfinite(lon) and abs(lat) <= 90 and abs(lon) <= 180):
            return [], f'{word!r}: not a latitude, longitude'
        corners.append((lat, lon))
    return corners, ''


def flatten(corners):
    return [float(v) for corner in corners for v in corner]


def pairs(flat):
    """[(lat, lon)] of a flat list; [] for anything else."""
    if not isinstance(flat, (list, tuple)) or len(flat) % 2:
        return []
    return [(float(a), float(b)) for a, b in zip(flat[::2], flat[1::2])]


class Corners:
    """The corners being edited and the selected one."""

    def __init__(self, corners=()):
        self.items, self.at = [tuple(c) for c in corners], 0

    def add(self, fix):
        """A corner at ``fix`` (lat, lon) after the selected one; it becomes the selection."""
        self.at = len(self.items) and self.at + 1
        self.items.insert(self.at, (float(fix[0]), float(fix[1])))

    def remove(self):
        if self.items:
            del self.items[self.at]
            self.at = max(0, min(self.at, len(self.items) - 1))

    def select(self, step):
        if self.items:
            self.at = (self.at + step) % len(self.items)

    def move(self, step):
        """Reorder: the selected corner swaps with its neighbour ``step`` (+1 or -1) away."""
        if len(self.items) > 1:
            other = (self.at + step) % len(self.items)
            self.items[self.at], self.items[other] = self.items[other], self.items[self.at]
            self.at = other

    def nudge(self, north, east):
        if self.items:
            self.items[self.at] = offset(self.items[self.at], north, east)

    def set(self, fix):
        """The selected corner to ``fix``: the boat sits on that corner now."""
        if self.items:
            self.items[self.at] = (float(fix[0]), float(fix[1]))
        else:
            self.add(fix)


def load(path=FENCE_FILE):
    """The corners the operator's fence file holds (``course.corners``), [] when none."""
    try:
        day = yaml.safe_load(Path(path).expanduser().read_text()) or {}
    except (OSError, yaml.YAMLError):
        return []
    course = day.get('course') if isinstance(day, dict) else None
    return pairs((course or {}).get('corners')) if isinstance(course, dict) else []


def save(corners=None, path=FENCE_FILE, on_exit=None):
    """Keep ``corners`` as ``course.corners`` (and ``on_exit`` as ``course.on_exit``) in the fence file, the rest of
    it unchanged; written whole."""
    path = Path(path).expanduser()
    try:
        day = yaml.safe_load(path.read_text()) or {}
    except (OSError, yaml.YAMLError):
        day = {}
    day = day if isinstance(day, dict) else {}
    course = dict(day.get('course') or {})
    if corners is not None:
        course['corners'] = flatten(corners)
    if on_exit is not None:
        course['on_exit'] = on_exit
    day['course'] = course
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix('.yaml.tmp')
    temp.write_text(yaml.safe_dump(day, sort_keys=False))
    temp.replace(path)
    return path


def same(left, right, tolerance=1e-7):
    """Two corner lists agree within ``tolerance`` degrees (the readback)."""
    return len(left) == len(right) and all(abs(a - b) <= tolerance for p, q in zip(left, right)
                                           for a, b in zip(p, q))


STALE_S = 5.0          # fence_watchdog sends /fence/status at status_rate (2 Hz) and on every change


def fresh(status, at, now):
    """``status`` while it is younger than STALE_S, else it marked ``stale`` (inside unknown): a dead
    fence_watchdog never leaves the row saying inside."""
    if not isinstance(status, dict) or (at is not None and 0 <= now - at <= STALE_S):
        return status
    return dict(status, inside=None, stale=True)


def status_line(status, held=True):
    """``(state, text)`` of /fence/status: ``ok``, ``warn`` (no position or stale) or ``bad`` (outside, with what
    to do: U releases the hold only when the mux ``held`` one)."""
    if not isinstance(status, dict):
        return 'off', 'course area: no status'
    edge = status.get('distance_to_edge')
    far = f' {edge:.1f} m' if isinstance(edge, (int, float)) else ''
    mode = status.get('on_exit') or 'alert'
    if status.get('stale'):
        return 'warn', f'course area: no fresh status from fence_watchdog ({mode} on exit)'
    if status.get('inside') is None:
        return 'warn', f'course area: no position ({mode} on exit)'
    if status.get('inside'):
        return 'ok', f'inside the course area,{far} to the edge ({mode} on exit)'
    if status.get('returning') or mode == 'return':
        return 'bad', f'OUTSIDE the course area by{far}: returning inside slowly (on_exit return)'
    return 'bad', f'OUTSIDE the course area by{far}: drive the boat back inside' + (
        ', then U releases the hold' if held else '')


def preview(corners, fix, boat):
    """``corners`` in map (x, y), placed through the boat: its ``fix`` (lat, lon) and ``boat`` (x, y) in map."""
    if not fix or boat is None:
        return []
    return [(boat[0] + e, boat[1] + n) for e, n in (enu(fix, c) for c in corners)]

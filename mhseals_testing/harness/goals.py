"""Cursor goals: the gates, the goal cell check and send/cancel (map.md section 6).

Pure and ROS-free. The caller passes ``Facts`` it gathered from the app and
Nav2, and an injected Nav2 (``plan``, ``navigate``) that returns a Goal with
``done``/``status``/``cancel``. Every refusal is a reason string, shown as a
toast or in the confirm popup. Goals are attributed to the claim that sent
them (``claim`` is on both journal lines), so the authority record shows
whose goal is executing.
"""

from dataclasses import dataclass, field
import math

import numpy as np
import yaml

REQUIRED = ('bt_navigator', 'controller_server', 'planner_server', 'velocity_smoother')
MAX_POSE_AGE_S = 0.5
MIN_FIX = 3  # 3D
MIN_DIST_M = 0.5  # goal checker xy 0.4 plus margin
LETHAL = 99  # 3x3 maximum at or above this is lethal or inscribed
EDGE_M = 2.0  # kept clear of the costmap half-width


@dataclass
class Facts:
    """What the gates read; one snapshot taken by the caller."""
    live: bool = False
    owner: bool = False  # this instance owns the session
    nav_running: bool = False
    lifecycle: dict = field(default_factory=dict)  # server -> 'active' | ...
    unit: str = ''  # command unit: velocity (m/s) or effort; empty fails closed
    armed: bool = False
    driving: str = ''  # name of a running procedure, manual control or mission
    pose_age_s: float = math.inf
    fix: int = 0
    yaw_quality: str = 'ok'  # ok | warn | fail
    costmap_age_s: float = math.inf
    fence: dict = None  # fence_watchdog's last fence message (see check_fence), None when none came
    fence_age_s: float = math.inf  # its receipt age
    to_ll: object = None  # map [(x, y)] -> [(lat, lon)] (navsat toLL) for a lat/lon fence; raises on failure


@dataclass
class Grid:
    """A costmap: ``data`` is (height, width) int8, cell (0, 0) at ``origin`` (x, y)."""
    data: np.ndarray
    resolution: float
    origin: tuple

    def cell(self, x, y):
        return int(math.floor((y - self.origin[1]) / self.resolution)), \
            int(math.floor((x - self.origin[0]) / self.resolution))

    def half_width(self):
        return min(self.data.shape[1], self.data.shape[0]) * self.resolution / 2


def gate(facts, auto=False):
    """The first gate that fails as a reason, or None. ``auto`` skips the confirm-only ones."""
    if not facts.live:
        return 'not live: goals need a live source'
    if not facts.owner:
        return 'this instance does not own the session'
    if not facts.nav_running:
        return 'Navigation is not running'
    down = [s for s in REQUIRED if facts.lifecycle.get(s) != 'active']
    if down:
        return f'Nav2 not active: {", ".join(down)}'
    if facts.unit != 'velocity':
        return f'command unit is {facts.unit}, goals need the velocity controller'
    if not facts.armed:
        return 'outputs are not armed'
    if facts.driving:
        return f'{facts.driving} is driving'
    if auto:
        return None
    if facts.pose_age_s > MAX_POSE_AGE_S:
        return f'pose is stale ({facts.pose_age_s:.1f} s)'
    if facts.fix < MIN_FIX:
        return f'GPS fix {facts.fix} is below 3D'
    if facts.yaw_quality == 'fail':
        return 'yaw quality is failing'
    limit = costmap_limit_s()
    if limit is None:
        return "costmap freshness limit unreadable (mhseals_nav costmap_watchdog.yaml max_age_s)"
    if facts.costmap_age_s > limit:
        return 'global costmap is stale'
    return None


_LIMIT = []  # the first good read; a failed read is retried on the next goal


def costmap_limit_s():
    """mhseals_nav's costmap_watchdog ``max_age_s``, the one costmap freshness limit; None when unreadable."""
    if _LIMIT:
        return _LIMIT[0]
    from mhseals_testing.health.config import share_dir
    try:
        params = yaml.safe_load((share_dir('mhseals_nav') / 'config' / 'costmap_watchdog.yaml').read_text())
        limit = float(params['/**']['ros__parameters']['max_age_s'])
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return None
    if not math.isfinite(limit) or limit < 0:
        return None
    _LIMIT.append(limit)
    return limit


def check_goal(goal, boat, grid):
    """Refusal reason for a goal (x, y) against the boat (x, y) and the global costmap."""
    x, y = goal[0], goal[1]
    if not (math.isfinite(x) and math.isfinite(y)):
        return 'goal is not finite'
    height, width = grid.data.shape
    # bounds in metres first: a huge x would overflow the cell index (int(inf))
    if not (0 <= x - grid.origin[0] < width * grid.resolution and 0 <= y - grid.origin[1] < height * grid.resolution):
        return 'goal is outside the costmap'
    row, col = grid.cell(x, y)
    if not (0 <= row < height and 0 <= col < width):
        return 'goal is outside the costmap'
    dist = math.hypot(x - boat[0], y - boat[1])
    if dist > grid.half_width() - EDGE_M:
        return f'goal is {dist:.1f} m away, over the {grid.half_width() - EDGE_M:.0f} m limit'
    if dist < MIN_DIST_M:
        return f'goal is {dist:.2f} m from the boat, under {MIN_DIST_M} m'
    window = grid.data[max(row - 1, 0):row + 2, max(col - 1, 0):col + 2]
    if window.max() >= LETHAL:
        return 'goal is on a lethal or inscribed cell'
    return None


_FENCE = []  # (topic, stale_s), the first good read


def fence_config():
    """(topic, stale_s) of mhseals_nav's fence_watchdog.yaml: where it publishes the fence it
    enforces, and three of its periods as the age past which the fence counts as missing; None
    when unreadable."""
    if _FENCE:
        return _FENCE[0]
    from mhseals_testing.health.config import share_dir
    try:
        params = yaml.safe_load((share_dir('mhseals_nav') / 'config' / 'fence_watchdog.yaml').read_text())
        params = params['/**']['ros__parameters']
        found = str(params['fence_topic']), 3.0 / float(params['fence_rate_hz'])
    except (OSError, KeyError, TypeError, ValueError, ZeroDivisionError, yaml.YAMLError):
        return None
    if not found[0] or not math.isfinite(found[1]) or found[1] <= 0:
        return None
    _FENCE.append(found)
    return found


def edges(p, poly):
    """[(distance, unit normal toward the edge)] from ``p`` to each edge of ``poly``."""
    from mhseals_nav.fence_watchdog import closest      # the watchdog's own geometry
    out = []
    for c in closest(p, poly):
        dx, dy = c[0] - p[0], c[1] - p[1]
        d = math.hypot(dx, dy)
        out.append((d, (dx / d, dy / d) if d > 1e-9 else (0.0, 0.0)))
    return out


AHEAD_M = 1.0  # a lat/lon fence: the boat's heading is toLL of a point this far ahead


def fence_plane(goal, boat, fence, to_ll):
    """(polygon, goal, boat (x, y), boat yaw) in the fence's own plane, or a refusal reason.

    A map fence (the sim) is checked in map. A lat/lon fence is checked in lat/lon, never through a
    map copy that a rotated map would shift: the goal, the boat and a point AHEAD_M ahead of it go
    through ``to_ll`` (navsat's toLL, the EKF's own transform), then into the watchdog's projection
    from the fence's first vertex. No ``to_ll``, or a failed one, refuses.
    """
    from mhseals_nav.fence_watchdog import polygon, project
    poly = polygon(fence.get('frame'), fence.get('points'))
    if poly is None:
        return f"fence_watchdog has no valid fence: {fence.get('error') or 'no points'}"
    if fence['frame'] == 'map':
        return poly, (goal[0], goal[1]), (boat[0], boat[1]), boat[2]
    if to_ll is None:
        return 'lat/lon fence: no toLL to place the goal in lat/lon, goals are refused'
    ahead = (boat[0] + AHEAD_M * math.cos(boat[2]), boat[1] + AHEAD_M * math.sin(boat[2]))
    try:
        found = to_ll([(goal[0], goal[1]), (boat[0], boat[1]), ahead])
    except Exception as error:  # noqa: BLE001 - any failure refuses (fail closed)
        return f'lat/lon fence: toLL failed ({error}), goals are refused'
    if len(found) != 3 or not all(math.isfinite(v) for ll in found for v in ll) or \
            any(abs(lat) < 1e-9 and abs(lon) < 1e-9 for lat, lon in found):
        return 'lat/lon fence: toLL gave no position (navsat has no datum yet), goals are refused'
    origin = fence['points'][:2]
    g, b, a = (project(origin, lat, lon) for lat, lon in found)
    return poly, g, b, math.atan2(a[1] - b[1], a[0] - b[0])


def check_fence(goal, boat, fence, age_s, to_ll=None):
    """Refusal reason for a goal against fence_watchdog's fence, by its own rules; None when allowed.

    ``fence`` is the watchdog's message (``valid``, ``frame`` latlon or map, its raw ``points``,
    ``margin_m``, ``outward_angle_deg``); ``boat`` is (x, y, yaw) in map; ``to_ll`` maps map points
    to (lat, lon) (``fence_plane``). A missing, stale or invalid fence refuses. A goal must lie
    inside the fence and outside its margin. Within the margin the watchdog holds while the boat
    faces outward, so there a goal is refused when the boat faces outward, and allowed only when it
    lies farther from every edge than the boat (inward).
    """
    from mhseals_nav.fence_watchdog import inside
    found = fence_config()
    if found is None:
        return 'fence unknown: mhseals_nav fence_watchdog.yaml has no fence_topic/fence_rate_hz'
    if not isinstance(fence, dict) or age_s > found[1]:
        return f'no fence from fence_watchdog ({found[0]}): goals are refused without one'
    if not fence.get('valid'):
        return f"fence_watchdog has no valid fence: {fence.get('error') or 'no points'}"
    plane = fence_plane(goal, boat, fence, to_ll)
    if isinstance(plane, str):
        return plane
    poly, (x, y), boat_p, yaw = plane
    boat = (*boat_p, yaw)
    margin = float(fence['margin_m'])
    if not inside((x, y), poly):
        return 'goal is outside the fence'
    goal_m = min(d for d, _ in edges((x, y), poly))
    if not inside(boat[:2], poly):
        return 'the boat is outside the fence: the operator drives it back in, then releases the hold (U)'
    near = [(d, n) for d, n in edges(boat[:2], poly) if d < margin]
    if not near:
        if goal_m < margin:
            return f'goal is within the {margin:g} m fence margin ({goal_m:.1f} m from the edge)'
        return None
    boat_m = min(d for d, _ in near)
    cos_limit = math.cos(math.radians(float(fence['outward_angle_deg'])))
    heading = (math.cos(boat[2]), math.sin(boat[2]))
    if any(heading[0] * n[0] + heading[1] * n[1] > cos_limit for _, n in near):
        return (f'the boat is in the fence margin ({boat_m:.1f} m from the edge) facing outward: the fence '
                'watchdog holds as soon as Nav2 drives; the operator turns it inward (manual), then U')
    if goal_m <= boat_m:
        return (f'the boat is in the fence margin ({boat_m:.1f} m from the edge): only goals farther '
                f'inward are allowed (this one is {goal_m:.1f} m from the edge)')
    return None


def bearing(boat, goal):
    """Degrees counter-clockwise from east, boat to goal."""
    return math.degrees(math.atan2(goal[1] - boat[1], goal[0] - boat[0])) % 360


class Goals:
    """Sends and cancels cursor goals through an injected Nav2; one goal at a time."""

    def __init__(self, nav2, emit, clock):
        self.nav2, self.emit, self.clock = nav2, emit, clock
        self.active = None  # {'goal': Nav2 Goal, 'claim', 'x', 'y', 'yaw'}

    def preview(self, goal, boat, grid, facts, timeout=2.0):
        """(path, None) for the confirm popup, or (None, reason). ``boat`` is (x, y, yaw) in map."""
        reason = gate(facts) or check_goal(goal, boat, grid) or check_fence(goal, boat, facts.fence,
                                                                            facts.fence_age_s, facts.to_ll)
        if reason:
            return None, reason
        try:
            path = self.nav2.plan(goal, timeout=timeout)
        except Exception as error:  # noqa: BLE001 - any planner failure is a refusal
            return None, f'planner failed: {error}'
        if not path:
            return None, 'planner returned no path'
        return path, None

    def send(self, goal, boat, grid, facts, claim, path_m):
        """Re-check every gate, then navigate. Returns None or the refusal reason."""
        reason = gate(facts) or check_goal(goal, boat, grid) or check_fence(goal, boat, facts.fence,
                                                                            facts.fence_age_s, facts.to_ll)
        if reason is None and self.active:
            reason = 'a goal is already active'
        if reason:
            return reason
        try:
            handle = self.nav2.navigate([tuple(goal)])
        except Exception as error:  # noqa: BLE001
            return f'Nav2 refused: {error}'
        self.active = {'goal': handle, 'claim': claim, 'x': goal[0], 'y': goal[1], 'yaw': goal[2]}
        self.emit('cursor_goal_start', {'x': goal[0], 'y': goal[1], 'yaw': goal[2], 'frame': 'map',
                                        'path_m': path_m, 'claim': claim, 't_ns': self.clock()})
        return None

    def cancel(self, cause):
        """Cancel the active goal; True if there was one."""
        if not self.active:
            return False
        try:
            self.active['goal'].cancel()
        except Exception:  # noqa: BLE001 - the stop line is still written
            pass
        self._stop('canceled', cause)
        return True

    def poll(self, facts):
        """Journal the end of a finished goal, and cancel when a gate no longer holds."""
        if not self.active:
            return
        reason = gate(facts, auto=True)
        if reason:
            self.cancel(f'gate: {reason}')
        elif self.active['goal'].done:
            self._stop(self.active['goal'].status.lower(), '')

    def _stop(self, outcome, cause):
        done, self.active = self.active, None
        self.emit('cursor_goal_stop', {'outcome': outcome, 'cause': cause, 'claim': done['claim'],
                                       't_ns': self.clock()})

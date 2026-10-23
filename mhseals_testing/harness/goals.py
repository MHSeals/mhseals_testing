"""Cursor goals: the gates, the goal cell check and send/cancel (map.md section 6).

Pure and ROS-free. The caller passes ``Facts`` it gathered from the app and
Nav2, and an injected Nav2 (``plan``, ``navigate``) that returns a Goal with
``done``/``status``/``cancel``. Every refusal is a reason string, shown as a
toast or in the confirm popup. Goals are attributed to the claim that sent
them (``claim`` is on both journal lines), so the authority record shows
whose goal is executing.
"""

import math
from dataclasses import dataclass, field

import numpy as np

REQUIRED = ('bt_navigator', 'controller_server', 'planner_server', 'velocity_smoother')
MAX_POSE_AGE_S = 0.5
MAX_COSTMAP_AGE_S = 3.0
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
    if facts.costmap_age_s > MAX_COSTMAP_AGE_S:
        return 'global costmap is stale'
    return None


def check_goal(goal, boat, grid):
    """Refusal reason for a goal (x, y) against the boat (x, y) and the global costmap."""
    x, y = goal[0], goal[1]
    if not (math.isfinite(x) and math.isfinite(y)):
        return 'goal is not finite'
    row, col = grid.cell(x, y)
    height, width = grid.data.shape
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


def bearing(boat, goal):
    """Degrees counter-clockwise from east, boat to goal."""
    return math.degrees(math.atan2(goal[1] - boat[1], goal[0] - boat[0])) % 360


class Goals:
    """Sends and cancels cursor goals through an injected Nav2; one goal at a time."""

    def __init__(self, nav2, emit, clock):
        self.nav2, self.emit, self.clock = nav2, emit, clock
        self.active = None  # {'goal': Nav2 Goal, 'claim', 'x', 'y', 'yaw'}

    def preview(self, goal, boat, grid, facts):
        """(path, None) for the confirm popup, or (None, reason)."""
        reason = gate(facts) or check_goal(goal, boat, grid)
        if reason:
            return None, reason
        try:
            path = self.nav2.plan(goal, timeout=2.0)
        except Exception as error:  # noqa: BLE001 - any planner failure is a refusal
            return None, f'planner failed: {error}'
        if not path:
            return None, 'planner returned no path'
        return path, None

    def send(self, goal, boat, grid, facts, claim, path_m):
        """Re-check every gate, then navigate. Returns None or the refusal reason."""
        reason = gate(facts) or check_goal(goal, boat, grid)
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

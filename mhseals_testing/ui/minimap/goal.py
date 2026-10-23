"""The ``minimap.goal`` modal state: cursor, goal yaw and the subtitle readout (map.md section 6).

Pure. The cursor is in map metres and starts at the boat; one move is 1 dot
(``hjkl``) or 8 dots (``HJKL``), a dot being ``dot_m`` metres at the current
zoom. Yaw is the bearing from the boat unless rotated with ``,`` ``.`` (15
degrees) and returned to auto with ``/``. Space, mark and the other
passthroughs never reach this class.
"""

import math

from ...harness.goals import bearing, check_goal
from .. import theme

MOVES = {'left': (-1, 0), 'right': (1, 0), 'up': (0, 1), 'down': (0, -1)}
FAST = 8  # dots per shifted move
YAW_STEP = 15.0


class GoalCursor:
    def __init__(self, boat, dot_m):
        self.boat = tuple(boat)
        self.dot_m = dot_m
        self.pos = tuple(boat)
        self.yaw_set = None  # degrees, or None for auto

    def move(self, direction, fast=False, up_yaw=None):
        """Step in the screen frame; ``up_yaw`` is the heading drawn straight up (heading-up view), None for north-up."""
        dx, dy = MOVES[direction]
        if up_yaw is not None:  # undo view.rotate: screen offset back to map axes
            a = math.radians(90 - up_yaw)
            dx, dy = dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a)
        step = self.dot_m * (FAST if fast else 1)
        self.pos = (self.pos[0] + dx * step, self.pos[1] + dy * step)

    def rotate(self, sign):
        self.yaw_set = ((self.yaw() if self.yaw_set is None else self.yaw_set) + sign * YAW_STEP) % 360

    def auto_yaw(self):
        self.yaw_set = None

    def yaw(self):
        if self.yaw_set is not None:
            return self.yaw_set
        return bearing(self.boat, self.pos) if self.pos != self.boat else 0.0

    def goal(self):
        """(x, y, yaw in radians) as Nav2 takes it."""
        return self.pos[0], self.pos[1], math.radians(self.yaw())

    def subtitle(self, grid):
        """`goal 12.4 m @045° · cost 0 · in map` (the goal yaw after @), with the refusal reason when it would be refused."""
        dist = math.hypot(self.pos[0] - self.boat[0], self.pos[1] - self.boat[1])
        row, col = grid.cell(*self.pos)
        inside = 0 <= row < grid.data.shape[0] and 0 <= col < grid.data.shape[1]
        cost = int(grid.data[row, col]) if inside else None
        sep = theme.SEPARATOR
        text = (f'goal {dist:.1f} m @{round(self.yaw()) % 360:03d}\N{DEGREE SIGN}{sep}'
                f'cost {cost if cost is not None else "-"}{sep}{"in map" if inside else "off map"}')
        reason = check_goal(self.goal(), self.boat, grid)
        return theme.ascii_safe(f'{text}{sep}{reason}' if reason else text)

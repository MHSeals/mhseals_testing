"""What more than one part of the app shares: constants and small helpers."""

from mhseals_testing.ui.panels import PANELS


# Ctrl+h/j/k/l and Ctrl+arrows move focus to the panel in that direction, by layout:
#   row (>= 90 columns): Status | Motion     | Conditions, Thrusters      stack: Status | Motion, Conditions,
#                                | Procedures | Activity                            | Thrusters, Procedures, Activity
DIRECTIONS = {'ctrl-h': 'left', 'ctrl-left': 'left', 'ctrl-l': 'right', 'ctrl-right': 'right',
              'ctrl-k': 'up', 'ctrl-up': 'up', 'ctrl-j': 'down', 'ctrl-down': 'down'}


PULSE_STEP = 10          # µs per ←/→ in the Thrusters panel


class Detach(KeyboardInterrupt):
    """SIGUSR1: another instance is taking over. Unwinds like Ctrl+C, then hands the children on."""


CURSORS = (*PANELS, 'list')   # every cursor an App keeps, the Bags tab's 'list' included

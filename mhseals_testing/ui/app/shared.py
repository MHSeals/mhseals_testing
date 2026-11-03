"""What more than one part of the app shares: constants and small helpers."""

from mhseals_hardware.menu import Cancelled
from mhseals_hardware.thruster_mixer import IDENTITY
from mhseals_testing.runner import Aborted
from mhseals_testing.ui.panels import PANELS


# Ctrl+h/j/k/l and Ctrl+arrows move focus to the panel in that direction, by layout:
#   row (>= 90 columns): Status | Motion     | Conditions, Thrusters      stack: Status | Motion, Conditions,
#                                | Procedures | Activity                            | Thrusters, Procedures, Activity
DIRECTIONS = {'ctrl-h': 'left', 'ctrl-left': 'left', 'ctrl-l': 'right', 'ctrl-right': 'right',
              'ctrl-k': 'up', 'ctrl-up': 'up', 'ctrl-j': 'down', 'ctrl-down': 'down'}


PULSE_STEP = 10          # µs per ←/→ in the Thrusters panel


class Detach(KeyboardInterrupt):
    """SIGUSR1: another instance is taking over. Unwinds like Ctrl+C, then hands the children on."""


def decoupled(config):
    """The active decoupling matrix, or None when both C+ and C- are the identity."""
    identity = sum(IDENTITY, ())
    values = tuple(float(value) for value in config.get('decoupling', identity))
    negative = config.get('decoupling_negative')
    if negative is not None and tuple(float(value) for value in negative) != identity:
        return values
    return None if values == identity else values


# Errors that stop a procedure: the plant goes neutral and outputs disarm.
STOPPING_ERRORS = (Aborted, Cancelled, RuntimeError, OSError, ValueError, KeyError)


CURSORS = (*PANELS, 'list', 'labels')   # every cursor an App keeps: the Bags tab's 'list', the Labels tab's

"""Procedure registry.

A procedure is one ``run(ctx)`` function plus what it needs from the plant.
The menu lists every procedure and disables (with the reason) those the
current profile or state can't run, so procedures never branch on "boat or
simulator" themselves.
"""

from dataclasses import dataclass, field

# What a procedure may need:
#   armed     thruster outputs running (H)
#   effort    commands are normalized effort (open-loop thruster tests)
#   velocity  commands are m/s and rad/s (controller and Nav2 tests)
#   hardware  real PWM pins (identification, readback)
#   sensors   motion scoring, so the bench override is not enough
NEEDS = frozenset({'armed', 'effort', 'velocity', 'hardware', 'sensors'})


@dataclass(frozen=True)
class Procedure:
    key: str            # menu shortcut
    title: str
    run: object         # run(ctx) -> result dict or None
    needs: frozenset = field(default_factory=frozenset)
    description: str = ''

    def __post_init__(self):
        unknown = set(self.needs) - NEEDS
        if unknown:
            raise ValueError(f'{self.title}: unknown needs {sorted(unknown)}')


def unavailable(procedure, profile, armed, bench_override=False):
    """Why ``procedure`` can't run now, or '' when it can."""
    needs = procedure.needs
    if 'armed' in needs and not armed:
        return 'arm first (H)'
    if 'effort' in needs and profile.command_units != 'effort':
        return 'needs effort commands; run without --velocity-control'
    if 'velocity' in needs and profile.command_units != 'velocity':
        return 'needs velocity commands; use --velocity-control or a velocity profile'
    if 'hardware' in needs and profile.outputs != 'odroid':
        return 'needs the ODROID PWM outputs'
    if 'sensors' in needs and bench_override:
        return 'scores motion; not available with the bench override'
    return ''


# Filled as procedures are added; order is menu order.
PROCEDURES = ()

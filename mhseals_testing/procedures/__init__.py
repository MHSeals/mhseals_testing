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
#   outputs   the profile has thruster outputs to drive (pins or a simulator topic)
#   hardware  real PWM pins
#   sensors   motion scoring, so the bench override is not enough
NEEDS = frozenset({'armed', 'effort', 'velocity', 'outputs', 'hardware', 'sensors'})


@dataclass
class Settings:
    """Trial timing and size; amplitude is in the profile's command units."""
    amplitude: float = 0.25
    baseline: float = 5.0      # s of neutral before each command, to measure drift
    command: float = 3.0       # s the command is held
    settle: float = 5.0        # s of neutral after, so the next trial starts at rest
    repeats: int = 1


@dataclass
class Context:
    """What a procedure can use. The app builds it; tests pass fakes.

    ``motion(name, **details)`` is a context manager yielding a Runner inside the
    live dashboard; ``prompt``/``choose`` talk to the operator outside it;
    ``record(result)`` stores a result in the monitor, the bag and the report.
    """
    profile: object
    monitor: object
    settings: Settings
    motion: object
    prompt: object
    choose: object
    record: object
    stale_after: float = 2.0
    services: dict = field(default_factory=dict)  # optional extras (manual, arming, config)

    def history(self, since=float('-inf'), until=float('inf'), name='odometry'):
        return self.monitor.history(name, since, until)


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
    if 'outputs' in needs and profile.outputs == 'none':
        return 'the profile has no thruster outputs'
    if 'hardware' in needs and profile.outputs != 'odroid':
        return 'needs the ODROID PWM outputs'
    if 'sensors' in needs and bench_override:
        return 'scores motion; not available with the bench override'
    return ''


def _registry():
    # Imported here: the procedure modules use the classes above.
    from functools import partial

    from mhseals_testing.procedures import (
        axis, characterize, combined, controller, manual, thrusters,
    )
    trial = frozenset({'armed', 'sensors'})
    return (
        Procedure('1', 'Surge trial', partial(axis.run, axis='surge'), trial,
                  'forward and astern; scores the drift-corrected response sign'),
        Procedure('2', 'Sway trial', partial(axis.run, axis='sway'), trial,
                  'port and starboard'),
        Procedure('3', 'Yaw trial', partial(axis.run, axis='yaw'), trial,
                  'counterclockwise and clockwise'),
        Procedure('c', 'Combined trials', combined.run, trial,
                  'diagonal and turning commands'),
        Procedure('k', 'Characterize', characterize.run, trial | {'effort', 'outputs'},
                  'fit the velocity model and measured limits'),
        Procedure('v', 'Controller steps', controller.run, trial | {'velocity'},
                  'step response of the calibrated velocity controller'),
        Procedure('m', 'Manual control', manual.run, frozenset({'armed'}),
                  'deadman keyboard driving under the interlocks'),
        Procedure('i', 'Thruster mapping', thrusters.run, frozenset({'outputs'}),
                  'identify outputs with pulses or enter the mapping (disarms)'),
    )


PROCEDURES = _registry()

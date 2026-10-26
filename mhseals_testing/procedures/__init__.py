"""Procedure registry.

A procedure is one ``run(ctx)`` function plus what it needs from the plant.
The menu lists every procedure and disables (with the reason) those the
current profile or state can't run, so procedures never branch on "boat or
simulator" themselves.
"""

from dataclasses import dataclass, field

from mhseals_testing.harness.settings import Settings  # noqa: F401  (re-export)

# What a procedure may need:
#   armed     thruster outputs running (a, the arm panel)
#   effort    commands are normalized effort (open-loop thruster tests)
#   velocity  commands are m/s and rad/s (controller and Nav2 tests)
#   outputs   the profile has thruster outputs to drive (pins or a simulator topic)
#   hardware  real PWM pins
#   sensors   motion scoring, so the bench override is not enough
NEEDS = frozenset({'armed', 'effort', 'velocity', 'outputs', 'hardware', 'sensors'})


@dataclass
class Context:
    """What a procedure can use. The app builds it; tests pass fakes.

    ``motion(name, **details)`` is a context manager yielding a Runner inside the
    live dashboard; ``prompt``/``choose`` talk to the operator outside it;
    ``record(result)`` stores a result in the monitor, the bag and the report.
    ``supervise(name)`` is ``motion`` for when something else (Nav2) drives, and
    ``observe(name)`` a passive window that needs neither arming nor readiness.
    """
    profile: object
    monitor: object
    settings: Settings
    motion: object
    prompt: object
    choose: object
    record: object
    supervise: object = None
    observe: object = None
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
    group: str = ''     # heading it sits under in the menu; a group's procedures are adjacent

    def __post_init__(self):
        unknown = set(self.needs) - NEEDS
        if unknown:
            raise ValueError(f'{self.title}: unknown needs {sorted(unknown)}')


def unavailable(procedure, profile, armed, bench_override=False):
    """Why ``procedure`` can't run now, or '' when it can."""
    needs = procedure.needs
    if 'armed' in needs and not armed:
        return 'arm first (a)'
    if 'effort' in needs and profile.command_units != 'effort':
        return 'needs effort commands; use --set command_mode=effort'
    if 'velocity' in needs and profile.command_units != 'velocity':
        return 'needs velocity commands; use --set command_mode=velocity'
    if 'outputs' in needs and profile.outputs == 'none':
        return 'the profile has no thruster outputs'
    if 'hardware' in needs and profile.outputs != 'odroid':
        return 'needs the ODROID PWM outputs'
    if 'sensors' in needs and bench_override:
        return 'scores motion; not available with the bench override'
    return ''


def _registry():
    # Imported here: the procedure modules use the classes above.
    from mhseals_testing.procedures import (
        characterize, controller, interference, magcal, manual, mission, nav, perception, physics,
        thrusters,
    )
    trial = frozenset({'armed', 'sensors'})
    return (
        Procedure('f', 'Characterize', characterize.run, trial | {'effort', 'outputs'},
                  'survey the boat, refine the thrust correction per direction', 'Calibrate'),
        Procedure('d', 'Dock physics', physics.dock, frozenset(),
                  'the dock step in Physics: dock_step (weigh-in, sensor noise); nothing moves', 'Calibrate'),
        Procedure('y', 'Thrust physics', physics.thrust, trial | {'effort', 'outputs'},
                  'the thrust step in Physics: thrust_step (bollard, pulses, survey, maneuvers); keeps C', 'Calibrate'),
        Procedure('g', 'Compass calibration', magcal.run, frozenset(),
                  'handheld bearing -> fixed-yaw calibration (42006), sent once you type FCU',
                  'Calibrate'),
        Procedure('e', 'Compass interference', interference.run, frozenset({'armed', 'outputs'}),
                  'pulse each output; fit the compass field against thruster effort', 'Calibrate'),
        Procedure('v', 'Controller steps', controller.run, trial | {'velocity'},
                  'step response of the calibrated velocity controller', 'Calibrate'),
        Procedure('m', 'Manual control', manual.run, frozenset({'armed'}),
                  'deadman keyboard driving under the interlocks', 'Drive'),
        Procedure('i', 'Thruster set', thrusters.run, frozenset({'armed', 'outputs'}),
                  'pulse each output; map and reverse thrusters', 'Drive'),
        Procedure('n', 'Nav2 bring-up', nav.bringup, frozenset(),
                  'lifecycle, costmap and a plan, without moving', 'Navigate'),
        Procedure('w', 'Waypoint square', nav.waypoints, trial | {'velocity'},
                  'Nav2 through four waypoints; cross-track error', 'Navigate'),
        Procedure('s', 'Station keeping', nav.station, trial | {'velocity'},
                  'hold position with Nav2; radius and drift', 'Navigate'),
        Procedure('t', 'Mission rehearsal', mission.run, trial | {'velocity'},
                  "run one of mhseals_nav's task trees and score it", 'Mission'),
        Procedure('p', 'Perception', perception.run, frozenset(),
                  'camera, LiDAR, detection and track rates; costmap obstacles', 'Perception'),
    )


PROCEDURES = _registry()


def tree(groups):
    """Menu rows for procedures with these group names: ``('heading', group)`` before each
    group's first procedure, else ``('item', index)``. Headings are never selectable."""
    rows, current = [], None
    for index, group in enumerate(groups):
        if group != current:
            current = group
            if group:
                rows.append(('heading', group))
        rows.append(('item', index))
    return rows

"""Characterization: drive a schedule of commands, fit the velocity model, save it.

The fit (analysis/characterization.py) gives the calibrated controller its
gains, the driver its decoupling, and Nav2 its measured limits. A steady turn
at the end shows the fit every heading, so it can tell a world-fixed current
from the boat's own bias. Nothing is written until the operator reviews it.
"""

import yaml

from mhseals_hardware.configuration import update_config
from mhseals_hardware.thruster_mixer import AXES, axis_command
from mhseals_testing.analysis.characterization import (
    characterize_samples, nav2_profile, samples_from_windows,
)
from mhseals_testing.procedures.combined import VECTORS, scaled
from mhseals_testing.runner import NEUTRAL

LEVELS = (0.5, 1.0)  # fractions of the amplitude, so the fit sees more than one speed


def schedule(amplitude):
    """Commands to hold: each axis both ways at each level, then the combined vectors."""
    commands = [axis_command(axis, direction * level * amplitude)
                for level in LEVELS for axis in AXES for direction in (1, -1)]
    return commands + [scaled(vector, amplitude) for vector in VECTORS]


def single_decoupling(config):
    """True when one C serves both directions, the only case this fit can describe."""
    negative = config.get('decoupling_negative')
    return negative is None or list(negative) == list(config['decoupling'])


def run(ctx):
    if not single_decoupling(ctx.services['hardware_config']):
        # A fit with one C beside a sign-split C is a config the controller refuses.
        refusal = dict(procedure='characterization', verdict='REFUSED',
                       reason='decoupling_negative differs from decoupling; '
                              'clear it to characterize with a single C')
        ctx.record(refusal)
        return refusal
    settings = ctx.settings
    commands = schedule(settings.amplitude)
    ctx.prompt(f'Characterization holds {len(commands)} commands for '
               f'{settings.command:g}s each, in every direction. Clear all sides and '
               'press Enter... ')
    windows = []
    with ctx.motion('characterization', commands=len(commands)) as runner:
        for command in commands:
            # The neutral before each command doubles as a zero-command sample window.
            windows.append((NEUTRAL, *runner.hold(NEUTRAL, settings.settle, 'settle')))
            windows.append((command, *runner.hold(command, settings.command, 'command')))
        # Turning shows every heading, which separates the current from the boat.
        turn = axis_command('yaw', settings.amplitude)
        windows.append((turn, *runner.hold(turn, settings.turn, 'turn')))
        runner_end = runner.hold(NEUTRAL, settings.settle, 'settle')[1]
    history = ctx.history(windows[0][1], runner_end)
    config = ctx.services['hardware_config']
    result = characterize_samples(samples_from_windows(history, windows),
                                  config['decoupling'], source='mhseals_testing session',
                                  motion=samples_from_windows(history, windows, transient=0.0))
    summary = dict(procedure='characterization', verdict='FITTED',
                   samples=result['sample_count'],
                   velocity_limit=result['motion_limits']['velocity'],
                   drift=result['drift'])
    nav2_params, overlay = ctx.services.get('nav2_params'), ctx.services.get('nav2_overlay')
    if nav2_params and overlay:
        # Only measured keys; navigation.launch.py merges these with current defaults.
        overlay.write_text(yaml.safe_dump(nav2_profile(result, nav2_params), sort_keys=False))
        summary['nav2_overlay'] = str(overlay)
    ctx.record(dict(summary, characterization=result))
    # No decouple option: the secant C over-corrects (see thrust.md section 1, R17).
    choice = ctx.choose('Save the characterization to the hardware config?',
                        ('no', 'save'), {'n': 0, 's': 1})
    if choice != 'no':
        path = ctx.services['user_config']
        saved = update_config(dict(characterization=result), path)
        ctx.record(dict(procedure='characterization', verdict='SAVED', path=str(saved)))
    return result

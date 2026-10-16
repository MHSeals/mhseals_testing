"""Characterization: drive a schedule of commands, fit the velocity model, save it.

The fit (analysis/characterization.py) gives the calibrated controller its
gains and Nav2 its measured limits. Nothing is written until the operator
reviews the result.
"""

from mhseals_hardware.configuration import load_config, save_config
from mhseals_hardware.thruster_mixer import AXES, axis_command
from mhseals_testing.analysis.characterization import characterize_samples, samples_from_windows
from mhseals_testing.procedures.combined import VECTORS, scaled
from mhseals_testing.runner import NEUTRAL

LEVELS = (0.5, 1.0)  # fractions of the amplitude, so the fit sees more than one speed


def schedule(amplitude):
    """Commands to hold: each axis both ways at each level, then the combined vectors."""
    commands = [axis_command(axis, direction * level * amplitude)
                for level in LEVELS for axis in AXES for direction in (1, -1)]
    return commands + [scaled(vector, amplitude) for vector in VECTORS]


def run(ctx):
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
        runner_end = runner.hold(NEUTRAL, settings.settle, 'settle')[1]
    history = ctx.history(windows[0][1], runner_end)
    config = ctx.services['hardware_config']
    result = characterize_samples(samples_from_windows(history, windows),
                                  config['thruster_matrix'], source='mhseals_testing session')
    summary = dict(procedure='characterization', verdict='FITTED',
                   samples=result['sample_count'],
                   velocity_limit=result['motion_limits']['velocity'])
    ctx.record(dict(summary, characterization=result))
    if ctx.choose('Save the characterization to the hardware config?',
                  ('no', 'save'), {'n': 0, 's': 1}) == 'save':
        path = ctx.services['user_config']
        saved = save_config(dict(load_config(path), characterization=result), path)
        ctx.record(dict(procedure='characterization', verdict='SAVED', path=str(saved)))
    return result

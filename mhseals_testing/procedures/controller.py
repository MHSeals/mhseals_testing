"""Velocity-controller validation: step each axis and measure how it tracks.

Needs velocity commands (--set command_mode=velocity or a velocity profile), so the
calibrated controller closes the loop on odometry. Metrics: analysis/step_response.py.
"""

from mhseals_hardware.thruster_mixer import AXES, axis_command
from mhseals_testing.analysis.step_response import step_metrics
from mhseals_testing.analysis.trial import COMPONENT
from mhseals_testing.runner import NEUTRAL


def run(ctx, axes=AXES):
    settings = ctx.settings
    ctx.prompt(f'Controller steps of {settings.amplitude:g} (m/s, rad/s) on '
               f"{', '.join(axes)}, both directions. Clear the area and press Enter... ")
    results = []
    for axis in axes:
        for direction in (1, -1):
            target = direction * settings.amplitude
            with ctx.motion('controller_step', axis=axis, target=target) as runner:
                runner.hold(NEUTRAL, settings.baseline, 'rest')
                start, end = runner.hold(axis_command(axis, target), settings.command,
                                         f'{axis} step {target:+g}')
                runner.hold(NEUTRAL, settings.settle, 'settle')
            key = COMPONENT[axis]
            samples = [(t, values.get(key)) for t, values in ctx.history(start, end)]
            result = dict(step_metrics(samples, target, start, end), procedure='controller',
                          axis=axis, direction=direction)
            ctx.record(result)
            results.append(result)
    return results

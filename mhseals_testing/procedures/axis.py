"""Open-loop axis trials: does each command axis move the boat the way it should?

Each trial holds neutral (to measure drift), then the command, then neutral
again, and scores the drift-corrected velocity response (analysis/trial.py).
"""

from mhseals_hardware.thruster_mixer import axis_command
from mhseals_testing.analysis.trial import summarize_trial
from mhseals_testing.runner import NEUTRAL

GUIDANCE = {
    'surge': {1: ('ahead', 'move forward'), -1: ('astern', 'reverse')},
    'sway': {1: ('on the port/left side', 'translate port/left'),
             -1: ('on the starboard/right side', 'translate starboard/right')},
    'yaw': {1: ('around the boat', 'rotate counterclockwise in place'),
            -1: ('around the boat', 'rotate clockwise in place')},
}


def trial(ctx, axis, direction, repeat=1):
    """One baseline/command/settle trial; returns its scored result."""
    settings = ctx.settings
    with ctx.motion('axis_trial', axis=axis, direction=direction, repeat=repeat,
                    amplitude=settings.amplitude) as runner:
        baseline_start, _ = runner.hold(NEUTRAL, settings.baseline, 'baseline')
        start, end = runner.hold(axis_command(axis, direction * settings.amplitude),
                                 settings.command, f"{axis} {'+' if direction > 0 else '-'}")
        runner.hold(NEUTRAL, settings.settle, 'settle')
    result = summarize_trial(ctx.history(baseline_start, end), axis, direction,
                             baseline_start, start, end, ctx.stale_after)
    result.update(procedure=axis, repeat=repeat, amplitude=settings.amplitude,
                  units=ctx.profile.command_units)
    ctx.record(result)
    return result


def run(ctx, axis):
    results = []
    for repeat in range(1, ctx.settings.repeats + 1):
        for direction in (1, -1):
            clearance, movement = GUIDANCE[axis][direction]
            ctx.prompt(f"{axis.upper()} {'+' if direction > 0 else '-'} "
                       f'({repeat}/{ctx.settings.repeats}): clear the area {clearance}; '
                       f'the boat should {movement}. Press Enter to run... ')
            results.append(trial(ctx, axis, direction, repeat))
    return results

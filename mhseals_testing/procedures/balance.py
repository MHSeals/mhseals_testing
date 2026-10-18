"""Measure each thruster alone, review attenuation, then invalidate the old boat model."""

from copy import deepcopy

from mhseals_hardware.configuration import load_config
from mhseals_hardware.thruster_mixer import (
    IDENTITY, NEUTRAL_PWM, POSITIONS, PWM_SCALE, effort_to_pulse,
)
from mhseals_testing.analysis import thruster_balance


def run(ctx):
    settings, services = ctx.settings, ctx.services
    config = deepcopy(services['hardware_config'])
    _, _, directions = thruster_balance.layout(config['thruster_matrix'])
    if settings.command < 1 or settings.baseline < 1 or settings.repeats < 1:
        raise ValueError('balance needs --command-duration / --baseline >= 1 s and --repeats >= 1')
    levels = sorted({(effort_to_pulse(settings.amplitude * level) - NEUTRAL_PWM) / PWM_SCALE
                     for level in (.5, 1)})
    if len(levels) != 2 or levels[0] <= 0:
        raise ValueError('increase amplitude to obtain two distinct balance pulse levels')
    neutral, trials = [NEUTRAL_PWM] * 4, []
    try:
        for index, position in enumerate(POSITIONS):
            output = config['channel_map'][index]
            ctx.prompt(f'{position.upper()} (output {output}): test alone in both directions at '
                       f'{levels[0]:.0%} / {levels[1]:.0%} for {settings.command:g}s each, '
                       f'{settings.repeats} repeat(s). Boat must float freely; clear all sides.',
                       title='Thruster balance')
            for repeat in range(settings.repeats):
                for effort in levels:
                    for direction in (1, -1):
                        pulse = effort_to_pulse(effort * direction *
                                                (-1 if config['reversed'][index] else 1))
                        with ctx.motion('thruster_balance_pulse', position=position, output=output,
                                        effort=effort, direction=direction, pulse_us=pulse) as runner:
                            try:
                                services['pulses'](neutral)
                                start, end, _ = runner.supervise(settings.baseline, label='baseline')
                                before = thruster_balance.baseline(ctx.history(start, end), start, end)
                                pulses = list(neutral)
                                pulses[output - 1] = pulse
                                services['pulses'](pulses)
                                start, end, _ = runner.supervise(
                                    settings.command, label=f'{position.upper()} {effort * direction:+.0%}')
                            finally:
                                services['pulses'](neutral)
                            measurement = thruster_balance.response(
                                ctx.history(start, end), start, end, before, direction, directions[index])
                            runner.supervise(settings.settle, label='settling')
                        trial = dict(position=position, output=output, effort=effort,
                                     direction=direction, repeat=repeat + 1, **measurement)
                        trials.append(trial)
                        ctx.record(dict(procedure='thruster balance pulse', verdict='RECORDED', **trial))
    except ValueError as error:
        detail = f'{position.upper()} (output {output}): {error}'
        ctx.record(dict(procedure='thruster balance', verdict='INCONCLUSIVE', detail=detail))
        raise ValueError(detail) from error
    finally:
        services['pulses'](None)
    result = thruster_balance.fit(trials, config['thruster_matrix'])
    ctx.record(dict(procedure='thruster balance', verdict='FITTED', **result))
    rows = [(r['position'].upper(), f"+{r['positive']:.3f} / -{r['negative']:.3f} m/s²; "
             f"row ×{r['scale']:.2f}") for r in result['rows']]
    choice = ctx.choose('Save thruster balance', ('discard', 'save'), {'d': 0, 's': 1}, rows=rows,
                        message='Save attenuates matrix rows, disarms, and clears the old velocity '
                        'model/decoupling. Run Characterize (f) again before velocity control / Nav2.')
    if choice == 'save':
        current = load_config(services['user_config'])
        if any(current[key] != config[key] for key in ('thruster_matrix', 'channel_map', 'reversed')):
            raise RuntimeError('hardware mapping/matrix changed during balance; retest before saving')
        services['disarm']('saving thruster balance')
        services['save_hardware'](thruster_matrix=result['thruster_matrix'],
                                  decoupling=list(sum(IDENTITY, ())), characterization={})
        ctx.record(dict(procedure='thruster balance', verdict='SAVED', **result))
    return result

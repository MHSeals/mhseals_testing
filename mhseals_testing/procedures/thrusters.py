"""Thruster set: find which output drives each thruster, and which ones are reversed.

Each output is pulsed alone through the armed driver's raw-pulse input; the
operator says which thruster moved and whether its thrust pointed the expected
way. The result is saved to the hardware YAML, which the driver hot-reloads.
The same flow exists without the suite as mhseals_hardware's thruster_identify.
"""

from mhseals_hardware.identification import IDENTIFY_EFFORT, POSITION_NAMES, PULSE_SECONDS
from mhseals_hardware.thruster_mixer import (
    NEUTRAL_PWM, POSITIONS, channel_map_from_observations, effort_to_pulse,
)

SETTLE_SECONDS = 1.0


def run(ctx):
    services = ctx.services
    # With a command mux the raw pulses reach the thrusters in auto; nothing publishes
    # /cmd_vel during a pulse test, so the mux then reads idle, which is also fine.
    mux_input = services.get('mux_input')
    mode = mux_input() if mux_input is not None else 'auto'
    if mode not in ('auto', 'idle'):
        raise RuntimeError(f'thruster set needs the mux in auto, not {mode}; '
                           'release the override first')
    neutral = [NEUTRAL_PWM] * 4
    observations, flipped = {}, set()
    try:
        for output in range(1, 5):
            ctx.prompt(f'Pulse output {output} at {IDENTIFY_EFFORT:.0%} for '
                       f'{PULSE_SECONDS:g} s. Watch which thruster moves.', title='Thruster set')
            pulses = list(neutral)
            pulses[output - 1] = effort_to_pulse(IDENTIFY_EFFORT)
            with ctx.observe('thruster_pulse', output=output) as runner:
                services['pulses'](pulses)
                runner.supervise(PULSE_SECONDS, label=f'output {output} pulsing')
                services['pulses'](neutral)
                runner.supervise(SETTLE_SECONDS, label='settling')
            remaining = [p for p in POSITIONS if p not in observations.values()]
            labels = [POSITION_NAMES[p] for p in remaining] + ['nothing moved']
            answer = ctx.choose(f'Output {output} moved', labels,
                                {str(i + 1): i for i in range(len(labels))})
            if answer == 'nothing moved':
                raise RuntimeError(f'output {output} drove no thruster; check its wiring')
            position = remaining[labels.index(answer)]
            expected = 'aft' if position.startswith('f') else 'forward'
            if ctx.choose(f'{answer.capitalize()} thrust', ('yes', 'no, reversed'),
                          {'y': 0, 'n': 1}, message=f'Did it push water {expected}?') != 'yes':
                flipped.add(position)
            observations[output] = position
    finally:
        services['pulses'](None)
    channel_map = channel_map_from_observations(observations)
    reversed_ = [p in flipped for p in POSITIONS]
    summary = ', '.join(f"{p.upper()}→{o}{' −' if r else ''}"
                        for p, o, r in zip(POSITIONS, channel_map, reversed_))
    result = dict(procedure='thruster set', verdict='DONE', channel_map=list(channel_map),
                  reversed=reversed_, detail=summary)
    if ctx.choose('Save mapping', ('save', 'discard'), {'s': 0, 'd': 1}, message=summary) == 'save':
        services['save_hardware'](channel_map=list(channel_map), reversed=reversed_)
        result['verdict'] = 'SAVED'
    ctx.record(result)
    return result

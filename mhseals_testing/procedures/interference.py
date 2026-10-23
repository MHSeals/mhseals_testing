"""Compass interference: pulse each thruster output and fit the field change.

The boat is tied up and still (props off or restrained). Every pulse runs in a
``ctx.motion`` session, so each tick passes the same interlocks as any motion
(sensors, TF, command-topic conflicts, Space aborts) and ends at neutral on
every path. Needs ``services['mag']()`` -> latest (x, y, z) in mG or None.
"""
from mhseals_hardware.thruster_mixer import NEUTRAL_PWM, effort_to_pulse
from mhseals_testing.analysis.interference import OUTPUTS, fit, verdict

LEVELS = (0.25, 0.5, 0.75)
PULSE_SECONDS = 2.0
GAP_SECONDS = 2.0
SETTLE_SECONDS = 0.5   # of each pulse, ignored while the current ramps
NEUTRAL = [NEUTRAL_PWM] * OUTPUTS


def schedule():
    """(output index, effort) for every pulse: each output, each level, both signs."""
    return [(i, sign * level) for i in range(OUTPUTS) for level in LEVELS for sign in (1, -1)]


def mean_xy(mags):
    return tuple(sum(m[i] for m in mags) / len(mags) for i in (0, 1))


def pulse_samples(segments):
    """Field changes per pulse, each against the neutral holds either side of it.

    A tied boat swings, so a run-wide neutral mean would put heading drift into
    every k; the neighbouring holds remove it (gps 4.4). Returns ([(efforts, dB)],
    mean neutral field xy).
    """
    samples, neutrals = [], []
    for i, (efforts, mags) in enumerate(segments):
        if efforts is None:
            neutrals += mags
            continue
        around = [m for j in (i - 1, i + 1) if 0 <= j < len(segments) and segments[j][0] is None
                  for m in segments[j][1]]
        if not around:
            continue
        base = mean_xy(around)
        samples += [(efforts, (m[0] - base[0], m[1] - base[1])) for m in mags]
    return samples, (mean_xy(neutrals) if neutrals else None)


def run(ctx):
    services = ctx.services
    if 'mag' not in services:
        raise RuntimeError('no compass feed in this session (/mavros/imu/mag not wired)')
    ctx.prompt('Props off or boat tied up and still. Each output pulses to +-25/50/75% '
               'for 2 s with 2 s of neutral between. Press Enter to run... ')
    segments = []   # (efforts or None for neutral, [field samples (x, y, z)]) in run order
    clock = ctx.monitor.clock

    def hold(runner, pulses, seconds, label, efforts=None):
        start, mags = clock(), []

        def sample(now):
            mag = services['mag']()
            if mag is not None and now - start >= SETTLE_SECONDS:
                mags.append(tuple(mag))
        services['pulses'](pulses)
        runner.supervise(seconds, label=label, sample=sample)
        segments.append((efforts, mags))

    try:
        with ctx.motion('interference', levels=list(LEVELS)) as runner:
            hold(runner, NEUTRAL, GAP_SECONDS, 'baseline')
            for output, effort in schedule():
                pulses, efforts = list(NEUTRAL), [0.0] * OUTPUTS
                pulses[output] = effort_to_pulse(effort)
                efforts[output] = effort
                hold(runner, pulses, PULSE_SECONDS, f'output {output + 1} {effort:+.0%}', efforts)
                hold(runner, NEUTRAL, GAP_SECONDS, 'neutral')
    finally:
        services['pulses'](None)
    samples, base = pulse_samples(segments)
    if not samples:
        raise RuntimeError('no compass samples: is /mavros/imu/mag publishing?')
    rows = fit(samples)
    if rows is None:
        raise RuntimeError('too few compass samples to fit')
    level, worst = verdict(rows)
    detail = '; '.join(f"out{i + 1} {r['pos'][0]:+.0f},{r['pos'][1]:+.0f}/{r['neg'][0]:+.0f},{r['neg'][1]:+.0f}"
                       for i, r in enumerate(rows))
    result = dict(procedure='compass interference', verdict=level, worst_deg=worst,
                  outputs=rows, field=base, detail=f'{worst:.1f} deg at full effort; {detail} mG/100%')
    ctx.record(result)
    return result

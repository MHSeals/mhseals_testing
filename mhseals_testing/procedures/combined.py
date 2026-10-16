"""Combined-axis trials: response to mixed commands, for the characterization fit."""

from mhseals_testing.analysis.trial import window_mean
from mhseals_testing.runner import NEUTRAL

# Unit directions; each is scaled so its largest component equals the amplitude.
VECTORS = ((1, 1, 0), (1, -1, 0), (1, 0, 1), (1, 0, -1), (0, 1, 1), (0, 1, -1), (1, 1, 1))


def scaled(vector, amplitude):
    peak = max(abs(value) for value in vector)
    return tuple(amplitude * value / peak for value in vector)


def run(ctx, vectors=VECTORS):
    settings = ctx.settings
    ctx.prompt('Combined trials move the boat diagonally and while turning. '
               'Clear all sides and press Enter... ')
    results = []
    for vector in vectors:
        command = scaled(vector, settings.amplitude)
        with ctx.motion('combined_trial', command=command) as runner:
            runner.hold(NEUTRAL, settings.baseline, 'baseline')
            start, end = runner.hold(command, settings.command, 'combined')
            runner.hold(NEUTRAL, settings.settle, 'settle')
        samples = ctx.history(start, end)
        result = dict(procedure='combined', command=command, samples=len(samples),
                      mean_velocity=window_mean(samples, start, end), window=(start, end),
                      verdict='RECORDED' if samples else 'INCONCLUSIVE')
        ctx.record(result)
        results.append(result)
    return results

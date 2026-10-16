"""Step-response metrics for velocity-controller validation (docs/math.tex, "Step response").

A step from rest to ``target`` is applied at ``start``. Values are normalized
by the target's sign, so the same metrics describe positive and negative steps.
"""

import numpy as np

RISE_LOW, RISE_HIGH = 0.1, 0.9
SETTLING_BAND = 0.05     # fraction of the target
SETTLING_FLOOR = 0.02    # m/s or rad/s; small targets can't settle tighter than sensor noise
TAIL_FRACTION = 0.2      # last part of the step used for steady-state error


def _first_crossing(times, values, level):
    above = np.nonzero(values >= level)[0]
    return float(times[above[0]]) if above.size else None


def step_metrics(samples, target, start, end):
    """``samples`` are (time, value) pairs; returns rise, overshoot, settling, SSE and IAE."""
    window = [(t - start, v) for t, v in samples if start <= t <= end and v is not None]
    result = dict(target=target, samples=len(window), verdict='INCONCLUSIVE')
    if len(window) < 5 or target == 0:
        return result
    times = np.array([t for t, _ in window])
    sign, reference = np.sign(target), abs(target)
    values = sign * np.array([v for _, v in window])
    low = _first_crossing(times, values, RISE_LOW * reference)
    high = _first_crossing(times, values, RISE_HIGH * reference)
    tolerance = max(SETTLING_BAND * reference, SETTLING_FLOOR)
    outside = np.nonzero(np.abs(values - reference) > tolerance)[0]
    if not outside.size:
        settling = 0.0
    elif outside[-1] + 1 < len(times):
        settling = float(times[outside[-1] + 1])
    else:
        settling = None  # still outside the band when the step ended
    tail = values[int(len(values) * (1 - TAIL_FRACTION)):]
    error = np.abs(reference - values)
    result.update(
        rise_time=high - low if low is not None and high is not None else None,
        overshoot=max(0.0, float(values.max() - reference) / reference),
        settling_time=settling,
        steady_state_error=float(reference - tail.mean()),  # positive: falls short
        # Trapezoidal integral of |error| over the step.
        iae=float(np.sum((error[1:] + error[:-1]) / 2 * np.diff(times))),
        verdict='SETTLED' if settling is not None else 'NOT SETTLED')
    return result

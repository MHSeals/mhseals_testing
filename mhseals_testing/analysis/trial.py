"""Score an open-loop axis trial from odometry samples (see docs/math.tex, "Trial response").

The response is the mean body-frame velocity while driven minus the mean
before the command (drift from current or wind). The verdict says whether the
boat moved the commanded way; it is never a calibration pass.
"""

import numpy as np

COMPONENT = {'surge': 'vx', 'sway': 'vy', 'yaw': 'wz'}
# Below this change (m/s or rad/s) the boat did not visibly respond.
MIN_RESPONSE = 0.03
MIN_DRIVEN, MIN_BASELINE = 5, 3


def summarize_trial(samples, axis, direction, baseline_start, command_start, command_end,
                    stale_after=2.0):
    """``samples`` are (time, values) odometry pairs, e.g. ``monitor.history('odometry')``."""
    component = COMPONENT[axis]
    baseline = [(t, v) for t, v in samples
                if baseline_start <= t < command_start and v.get(component) is not None]
    # A sample stamped at the instant the command starts can't reflect it yet.
    driven = [(t, v) for t, v in samples
              if command_start < t <= command_end and v.get(component) is not None]
    result = dict(axis=axis, direction=direction, samples=len(driven),
                  verdict='INCONCLUSIVE', detail='insufficient fresh odometry')
    if len(driven) < MIN_DRIVEN or len(baseline) < MIN_BASELINE:
        return result
    times = np.array([command_start] + [t for t, _ in driven] + [command_end])
    max_gap = float(np.diff(times).max())
    frames = {(v.get('frame'), v.get('child')) for _, v in baseline + driven}
    if max_gap > stale_after or len(frames) != 1:
        result['detail'] = 'odometry gap or frame change during trial'
        return result
    drift = float(np.mean([v[component] for _, v in baseline]))
    mean = float(np.mean([v[component] for _, v in driven]))
    delta = mean - drift
    verdict = ('LOW RESPONSE' if abs(delta) < MIN_RESPONSE else
               'SIGN MATCH' if delta * direction > 0 else 'SIGN MISMATCH')
    result.update(verdict=verdict, baseline=drift, mean=mean, response=delta,
                  peak=max(abs(v[component]) for _, v in driven), max_sample_gap=max_gap,
                  detail='body-frame velocity response; not a calibration pass',
                  mean_velocity={key: float(np.mean([v.get(key) or 0. for _, v in driven]))
                                 for key in ('vx', 'vy', 'wz')})
    first, last = driven[0][1], driven[-1][1]
    if all(first.get(key) is not None and last.get(key) is not None for key in ('x', 'y')):
        result['displacement'] = dict(frame=first.get('frame'),
                                      dx=last['x'] - first['x'], dy=last['y'] - first['y'])
    return result


def window_mean(samples, start, end, keys=('vx', 'vy', 'wz')):
    """Mean of each key over samples in [start, end], or None without samples."""
    window = [v for t, v in samples if start <= t <= end]
    return {key: float(np.mean([v[key] for v in window if v.get(key) is not None]))
            if any(v.get(key) is not None for v in window) else None for key in keys}

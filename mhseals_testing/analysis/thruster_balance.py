"""Relative thruster strength from isolated, paired acceleration trials.

For the boat's symmetric X layout, equal-magnitude translational directions
see the same mass/drag. Compare baseline-corrected translation, not a norm
mixing m/s² with rad/s². Fit a common a = k |effort|**p curve at two levels;
matrix row attenuation is then (weakest / strength)**(1/p).
"""

import math

import numpy as np

from mhseals_hardware.thruster_mixer import POSITIONS, validate_mixer


def layout(matrix):
    matrix = np.array(validate_mixer(matrix))
    sizes = np.max(np.abs(matrix[:, :2]), axis=1)
    if np.any(sizes <= 0) or np.linalg.matrix_rank(matrix) != 3:
        raise ValueError('balance needs a full-rank X thruster matrix')
    directions = matrix[:, :2] / np.linalg.norm(matrix[:, :2], axis=1)[:, None]
    if not np.allclose(np.abs(directions), math.sqrt(.5), atol=.05):
        raise ValueError('balance needs the symmetric 45-degree X layout')
    return matrix, sizes / max(1.0, max(sizes)), directions


def motion_window(history, start, end, max_gap=.25):
    """Regress world-frame velocity; a turning boat must not turn current into thrust."""
    samples = [(t, v) for t, v in history if start < t <= end]
    if len(samples) < 6:
        raise ValueError('balance needs at least six fresh odometry samples per window')
    times = np.array([t for t, _ in samples])
    gaps = np.diff([start, *times, end])
    frames = {(v.get('frame'), v.get('child')) for _, v in samples}
    values = np.array([[v.get(key, np.nan) for key in ('vx', 'vy', 'wz', 'yaw')]
                       for _, v in samples], dtype=float)
    if (len(frames) != 1 or not all(next(iter(frames))) or not np.isfinite(values).all()
            or min(np.diff(times)) <= 0 or max(gaps) > max_gap):
        raise ValueError('balance odometry has a gap, invalid values or a frame change')
    yaw = np.unwrap(values[:, 3])
    if np.ptp(yaw) > math.pi / 4:
        raise ValueError('too much rotation during balance; lower amplitude or pulse duration')
    c, s = np.cos(yaw), np.sin(yaw)
    velocity = np.column_stack((c * values[:, 0] - s * values[:, 1],
                                s * values[:, 0] + c * values[:, 1], values[:, 2]))
    centered = times - times.mean()
    slope = centered @ velocity / (centered @ centered)
    residual = velocity - velocity.mean(axis=0) - centered[:, None] * slope
    error = np.sqrt(np.sum(residual ** 2, axis=0) / (len(times) - 2)
                    / (centered @ centered))
    return dict(acceleration=slope, error=error, yaw=float(yaw.mean()),
                yaw_rate=float(np.max(np.abs(values[:, 2]))), frame=next(iter(frames)))


def baseline(history, start, end):
    result = motion_window(history, start, end)
    if np.linalg.norm(result['acceleration'][:2]) > .05 or result['yaw_rate'] > .1:
        raise ValueError('boat still accelerating or turning; increase --settle / --baseline')
    return result


def response(history, start, end, before, direction, expected):
    driven = motion_window(history, start, end)
    if driven['frame'] != before['frame']:
        raise ValueError('odometry frame changed between baseline and pulse')
    world = driven['acceleration'][:2] - before['acceleration'][:2]
    c, s = math.cos(driven['yaw']), math.sin(driven['yaw'])
    body = np.array((c * world[0] + s * world[1], -s * world[0] + c * world[1]))
    along = float(direction * np.dot(body, expected))
    error = float(np.linalg.norm(np.hypot(driven['error'][:2], before['error'][:2])))
    if along <= max(.005, 3 * error):
        raise ValueError('weak/noisy or reversed response; check odometry, mapping and propeller')
    if abs(expected[0] * body[1] - expected[1] * body[0]) > .5 * along:
        raise ValueError('response is not along the expected thruster direction; check mapping')
    return dict(response=along, uncertainty=error,
                acceleration=body.tolist(),
                yaw_acceleration=float(driven['acceleration'][2] - before['acceleration'][2]))


def fit(trials, matrix):
    """Recommend row scaling; reject data a single row factor cannot explain."""
    original, sizes, _ = layout(matrix)
    levels = sorted({trial['effort'] for trial in trials})
    if len(levels) != 2 or not 0 < levels[0] < levels[1] <= 1:
        raise ValueError('balance needs two distinct positive effort levels')
    measured, powers = {}, []
    for index, position in enumerate(POSITIONS):
        for direction in (1, -1):
            strengths = []
            for level in levels:
                values = [t['response'] for t in trials if
                          (t['position'], t['direction'], t['effort']) ==
                          (position, direction, level)]
                if not values or not all(math.isfinite(v) and v > 0 for v in values):
                    raise ValueError(f'{position.upper()}: incomplete/invalid balance trials')
                if max(values) / min(values) > 1.35:
                    raise ValueError(f'{position.upper()}: inconsistent repeats; retest in calm water')
                strengths.append(float(np.median(values)))
            power = math.log(strengths[1] / strengths[0]) / math.log(levels[1] / levels[0])
            if not .7 <= power <= 3.5:
                raise ValueError(f'{position.upper()}: unusable response curve; adjust test amplitude')
            powers.append(power)
            measured[index, direction] = strengths[1]
    power = float(np.median(powers))
    if max(abs(p - power) for p in powers) > .5:
        raise ValueError('thruster curves differ too much for one matrix scale; inspect hardware')
    scales = []
    for direction in (1, -1):
        strengths = np.array([measured[i, direction] for i in range(4)]) * sizes ** power
        scales.append((min(strengths) / strengths) ** (1 / power))
    scale = np.sqrt(scales[0] * scales[1])
    scale /= max(scale)  # never amplify an existing matrix row
    if np.min(levels[1] * sizes * scale) < levels[0] - 1e-9:
        raise ValueError('balance would go below the measured effort range; inspect the weak thruster')
    for direction in (1, -1):
        predicted = [measured[i, direction] * (sizes[i] * scale[i]) ** power for i in range(4)]
        if max(predicted) / min(predicted) > 1.25:
            raise ValueError('forward/reverse imbalance needs different corrections; inspect hardware')
    rows = [dict(position=p, scale=float(scale[i]),
                 positive=measured[i, 1], negative=measured[i, -1])
            for i, p in enumerate(POSITIONS)]
    return dict(rows=rows, exponent=power, tested_effort=levels,
                previous_matrix=original.flatten().tolist(),
                thruster_matrix=(original * scale[:, None]).flatten().tolist(),
                detail=', '.join(f"{r['position'].upper()} ×{r['scale']:.2f}" for r in rows))

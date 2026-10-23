import math

from pytest import approx

from mhseals_testing.analysis.characterization import characterize_samples
from mhseals_testing.analysis.thrust import speed_curve, speed_curves


def settle(effort, d1=0.5, d2=1.5):
    """Steady speed where d1*v + d2*v*v = effort."""
    return (-d1 + math.sqrt(d1 * d1 + 4 * d2 * effort)) / (2 * d2)


def test_curve_recovers_quadratic_drag():
    curve = speed_curve([(settle(e), e) for e in (0.3, 0.6)])
    assert curve['d1'] == approx(0.5) and curve['d2'] == approx(1.5)


def test_single_level_and_noise_stay_monotone():
    assert speed_curve([(0.4, 0.5)])['d2'] == 0.0
    # Speed rising faster than effort would need a negative d2: fall back to linear.
    curve = speed_curve([(0.2, 0.3), (0.7, 0.6)])
    assert curve['d1'] > 0 and curve['d2'] >= 0
    assert speed_curve([]) is None


def rows():
    out = []
    for stamp, (axis, sign, level) in enumerate(
            (a, s, l) for a in range(3) for s in (1, -1) for l in (0.3, 0.6)):
        command = [0, 0, 0]
        command[axis] = sign * level
        velocity = [0, 0, 0]
        velocity[axis] = sign * settle(level)
        out += [{'time': stamp + k / 10, 'command': tuple(command), 'velocity': tuple(velocity)}
                for k in range(3)]
    # A combined command and a neutral hold must not enter any curve.
    out += [{'time': 99 + k, 'command': (0.5, 0.5, 0), 'velocity': (0.3, 0.3, 0)}
            for k in range(2)]
    return out


def test_characterization_writes_curves_that_beat_the_linear_gain():
    result = characterize_samples(rows())
    curves = result['speed_curves']
    assert set(curves) == {'surge', 'sway', 'yaw'} and set(curves['surge']) == {'positive', 'negative'}
    curve = curves['sway']['negative']
    # Unfitted midpoint: the curve predicts it, the secant gain at 0.6 does not.
    d1, d2, effort = curve['d1'], curve['d2'], 0.45
    predicted = (-d1 + math.sqrt(d1 * d1 + 4 * d2 * effort)) / (2 * d2)
    assert predicted == approx(settle(effort), rel=1e-6)
    linear = effort * settle(0.6) / 0.6
    assert abs(linear - settle(effort)) / settle(effort) > 0.05

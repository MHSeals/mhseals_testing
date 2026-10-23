import math

from pytest import approx

from mhseals_testing.analysis.characterization import characterize_samples
from mhseals_testing.analysis.thrust import speed_curve, speed_curves


def settle(effort, d1=0.5, d2=1.5):
    """Steady speed where d1*v + d2*v*v = effort**2 (thrust goes with e|e|)."""
    return (-d1 + math.sqrt(d1 * d1 + 4 * d2 * effort ** 2)) / (2 * d2)


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
    assert set(curves) == {a + s for a in ('surge', 'sway', 'yaw') for s in '+-'}
    curve = curves['sway-']
    # Unfitted midpoint: the curve predicts it, the secant gain at 0.6 does not.
    d1, d2, effort = curve['d1'], curve['d2'], 0.45
    predicted = (-d1 + math.sqrt(d1 * d1 + 4 * d2 * effort ** 2)) / (2 * d2)
    assert predicted == approx(settle(effort), rel=1e-6)
    linear = effort * settle(0.6) / 0.6
    assert abs(linear - settle(effort)) / settle(effort) > 0.05


def test_speed_curves_round_trip_through_hardware_validation():
    from mhseals_hardware.configuration import load_defaults, validate_config
    from mhseals_hardware.velocity_law import Plant
    curves = characterize_samples(rows())['speed_curves']
    assert all(set(curve) == {'d1', 'd2'} for curve in curves.values())
    config = validate_config(dict(load_defaults(), characterization=dict(
        speed_curves=curves, motion_limits=dict(velocity=dict(surge=1.0, sway=1.0, yaw=1.0)))))
    assert Plant(config).curves['surge+'] == approx((curves['surge+']['d1'], curves['surge+']['d2']))


def test_speed_curve_drops_non_finite_samples():
    clean = [(0.5, 0.2), (1.0, 0.5), (1.5, 0.9)]
    noisy = clean + [(math.inf, 0.4), (0.8, math.inf), (math.nan, 0.3)]
    assert speed_curve(noisy) == speed_curve(clean)


def test_fitted_curves_are_saved_and_drive_the_plant(tmp_path):
    from mhseals_hardware.configuration import load_config, update_config
    from mhseals_hardware.velocity_law import Plant
    result = characterize_samples(rows())
    limits = dict(surge=1.0, sway=1.0, yaw=1.0)
    result['motion_limits'] = dict(velocity=limits)
    path = tmp_path / 'boat.yaml'
    update_config(dict(characterization=result), path)
    plant = Plant(load_config(path))
    assert plant.curves['sway-'] == approx((result['speed_curves']['sway-']['d1'],
                                            result['speed_curves']['sway-']['d2']))


def test_curves_remove_the_body_projected_drift():
    drift, yaw = (0.15, 0.0), math.pi / 2  # current along odom x is -sway when facing +y
    samples = []
    for sign in (1, -1):
        for level in (0.3, 0.6):
            speed = settle(level)
            # Sway hold: through-water speed plus the body-frame current (0, -0.15).
            samples.append(((0, sign * level, 0), (0, sign * speed - 0.15, 0), yaw))
    curves = speed_curves(samples, drift)
    for sign in '+-':
        assert curves['sway' + sign]['d1'] == approx(0.5) and curves['sway' + sign]['d2'] == approx(1.5)
    assert speed_curves(samples)['sway+']['d1'] != approx(0.5)


def test_quadratic_thrust_boat_is_recovered_at_an_unfitted_level():
    # Thrust ~ e|e| against linear plus quadratic drag: the curve must predict a level it never saw.
    curve = speed_curve([(settle(e, 0.8, 0.3), e) for e in (0.2, 0.3, 0.6, 0.8)])
    d1, d2 = curve['d1'], curve['d2']
    for effort in (0.25, 0.45):
        predicted = (-d1 + math.sqrt(d1 * d1 + 4 * d2 * effort ** 2)) / (2 * d2)
        assert predicted == approx(settle(effort, 0.8, 0.3), rel=0.05)

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


# --- holds -----------------------------------------------------------------------------

import numpy
from mhseals_testing.analysis.thrust import Hold, body_velocity, residual, step_response

FLOOR = numpy.array([0.002, 0.01, 0.01])      # wave_floor order: yaw, surge, sway


def make_hold(direction='surge+', body=(0.5, 0.0, 0.0), heading=0.7, drift=(0.0, 0.0), turn=0.0):
    """A 20 s hold at 10 Hz: thrust on at 2 s, cut at 18 s, constant body velocity, heading falling with ``turn``."""
    t = numpy.arange(0, 20, 0.1)
    u, v, r = body
    psi = heading - (r + turn) * numpy.clip(t - 2, 0, None)
    east = u * numpy.sin(psi) - v * numpy.cos(psi) + drift[0]
    north = u * numpy.cos(psi) + v * numpy.sin(psi) + drift[1]
    return Hold(direction, 0.6, (0, 0), t, (r + turn) * (t >= 2), numpy.column_stack([east, north]),
                heading, 2.0, 8.0, 18.0, 16.0)


def test_body_velocity_removes_drift_and_rotates():
    hold = make_hold(body=(0.5, 0.1, 0.0), drift=(0.05, -0.03))
    v = body_velocity(hold, (0.05, -0.03))
    assert v[:, 0] == approx(0.5) and v[:, 1] == approx(0.1)


def test_body_velocity_follows_gyro_heading_and_lever():
    hold = make_hold(body=(0.4, 0.0, 0.0), turn=0.05)          # turning left at 0.05 rad/s
    # The world velocity here was built with the true turning heading, so propagating the gyro recovers it.
    v = body_velocity(hold, (0, 0))
    assert v[:, 0] == approx(0.4, abs=1e-3) and abs(v[:, 1]).max() < 1e-3
    # The antenna 1 m ahead moves 1*r to the left; asking for that lever removes it.
    hold.vel_world = hold.vel_world.copy()
    shifted = body_velocity(hold, (0, 0), lever_arm=(1.0, 0.0))
    on = hold.t >= 2
    assert shifted[on, 1] == approx(v[on, 1] - 0.05)


def test_clean_hold_is_trusted_and_not_limited():
    result = residual(make_hold(), (0, 0), FLOOR / 4, 0.0)
    assert result.main == approx(0.5) and result.trust > 0.99 and not result.limited.any()


def test_trust_formula_matches_the_gaussian_product():
    # Surge 0.5 with 0.10 m/s sway (crab 0.2 >> 0.05) and a yaw rate 0.005 inside 0.0075.
    hold = make_hold(body=(0.5, 0.10, 0.005))
    result = residual(hold, (0, 0), FLOOR, 0.0)
    phi = lambda x: 0.5 * (1 + math.erf(x / math.sqrt(2)))
    crab = phi((0.025 - 0.10) / 0.01) - phi((-0.025 - 0.10) / 0.01)
    yaw = phi((0.0075 - 0.005) / 0.002) - phi((-0.0075 - 0.005) / 0.002)
    assert result.cross == approx([0.10, 0.005])
    assert result.trust == approx(crab * yaw) and result.trust < 1e-6


def test_noise_limited_flag_and_heading_error_term():
    # Window mean sigma 0.01 against tau*V = 0.05*0.5 = 0.025: 2*sigma = 0.02 < 0.025, not limited ...
    assert not residual(make_hold(), (0, 0), FLOOR, 0.0).limited[0]
    # ... but a 5 deg heading error adds 0.5*0.087 = 0.044 m/s and limits the crab entry only.
    result = residual(make_hold(), (0, 0), FLOOR, math.radians(5))
    assert list(result.limited) == [True, False]          # sway first, then yaw (index order)
    assert result.se[0] == approx(math.hypot(0.01, 0.5 * math.radians(5)))
    # A noisier floor limits both, and trust falls.
    noisy = residual(make_hold(), (0, 0), FLOOR * 4, 0.0)
    assert noisy.limited.all() and noisy.trust < 0.99


def test_yaw_hold_uses_rotation_centre_tolerance():
    # 0.3 rad/s about a centre 0.2 m off (surge speed 0.06 vs 0.10*0.3 = 0.03 allowed).
    result = residual(make_hold('yaw-', body=(0.06, 0.0, -0.3)), (0, 0), FLOOR, 0.0)
    assert result.main == approx(0.3) and result.trust < 0.2


def test_dead_hold_has_no_trust():
    result = residual(make_hold(body=(0.0, 0.0, 0.0)), (0, 0), FLOOR, 0.0)
    assert result.trust == 0.0 and result.limited.all()


def test_step_response_times_and_edge_rates():
    t = numpy.arange(0, 20, 0.05)
    y = numpy.where(t < 2, 0.0, 1 - numpy.exp(-(t - 2) / 1.0))
    y = numpy.where(t < 14, y, (1 - numpy.exp(-12.0)) * numpy.exp(-(t - 14) / 1.0))
    step = step_response(t, y, 2.0, 14.0)
    assert step.steady == approx(1.0, abs=0.01) and step.t95 == approx(3.0, abs=0.1)
    assert step.onset_rate == approx(0.8, abs=0.2) and step.cut_rate < -0.5

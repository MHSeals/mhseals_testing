import math
import random

import pytest

from mhseals_testing.analysis.geo import enu, fit_rigid, geodetic

ORIGIN = (42.3601, -71.0589)


def test_enu_axes_and_scale():
    east, north = enu(ORIGIN[0] + 0.001, ORIGIN[1], ORIGIN)
    assert east == pytest.approx(0, abs=1e-9) and north == pytest.approx(111.32, abs=0.1)
    east, _ = enu(ORIGIN[0], ORIGIN[1] + 0.001, ORIGIN)
    assert east == pytest.approx(111.32 * math.cos(math.radians(ORIGIN[0])), abs=0.1)


def test_geodetic_inverts_enu():
    lat, lon = geodetic(*enu(42.3650, -71.0540, ORIGIN), ORIGIN)
    assert (lat, lon) == pytest.approx((42.3650, -71.0540), abs=1e-9)


def test_fit_rigid_recovers_rotation_and_translation():
    cos, sin = math.cos(0.7), math.sin(0.7)
    source = [(0, 0), (10, 0), (10, 5), (-3, 8)]
    pairs = [((x, y), (cos * x - sin * y + 4, sin * x + cos * y - 2)) for x, y in source]
    theta, tx, ty, rms = fit_rigid(pairs)
    assert (theta, tx, ty) == pytest.approx((0.7, 4, -2))
    assert rms == pytest.approx(0, abs=1e-9)


def test_fit_rigid_reports_residual_and_needs_two_pairs():
    pairs = [((0, 0), (0, 0)), ((10, 0), (10, 0)), ((5, 5), (5, 9))]
    assert fit_rigid(pairs)[3] > 0.5
    with pytest.raises(ValueError):
        fit_rigid(pairs[:1])


def test_fit_rigid_rejects_coincident_and_non_finite_input():
    with pytest.raises(ValueError):
        fit_rigid([((1, 1), (5, 5)), ((1, 1), (6, 6))])
    with pytest.raises(ValueError):
        fit_rigid([((0, 0), (0, 0)), ((10, float('nan')), (10, 0))])


def test_fit_rigid_on_rotated_noisy_pairs_recovers_theta_within_point_two_degrees():
    rng = random.Random(7)
    cos, sin = math.cos(0.5), math.sin(0.5)
    source = [(0, 0), (20, 0), (20, 15), (-5, 18), (8, -9), (30, 10)]
    pairs = [((x, y), (cos * x - sin * y + 4 + rng.gauss(0, 0.2),
                       sin * x + cos * y - 2 + rng.gauss(0, 0.2))) for x, y in source]
    theta, _, _, rms = fit_rigid(pairs)
    assert math.degrees(abs(theta - 0.5)) < 0.2
    assert 0 < rms < 0.5


def _stream(rotation, start=0.0, zeros=2.6, rate=10, seconds=40, **kw):
    """Drive MapRotation with a boat moving 0.5 m/s east, odom frame rotated 2 degrees."""
    import math as m
    from mhseals_testing.analysis.geo import MapRotation
    fit = MapRotation(**kw)
    cos, sin = m.cos(m.radians(rotation)), m.sin(m.radians(rotation))
    for i in range(seconds * rate):
        t = start + i / rate
        x, y = 0.5 * t, 0.1 * t
        gps = (cos * x - sin * y, sin * x + cos * y)
        fit.add(t, (0.0, 0.0) if t < zeros else gps, (gps[0] + 1, gps[1]), (x, y))
    return fit


def test_map_rotation_ignores_zero_start_samples():
    import math as m
    theta, rms = _stream(2.0).fit()
    assert abs(m.degrees(theta) - 2.0) < 0.01
    assert rms < 0.01


def test_map_rotation_waits_for_global_to_settle():
    fit = _stream(2.0, seconds=12, spacing=0)
    assert len(fit._pairs) == 20  # 12 s of stream, the first 10 s dropped


def test_map_rotation_resets_on_map_jump():
    from mhseals_testing.analysis.geo import MapRotation
    fit = _stream(2.0)
    assert fit.fit() is not None
    fit.add(100.0, (50.0, 50.0), (60.0, 50.0), (0.0, 0.0))
    assert fit.fit() is None


@pytest.mark.parametrize('stream', [0, 1, 2])
@pytest.mark.parametrize('value', [math.nan, math.inf])
def test_map_rotation_ignores_non_finite_samples_in_any_stream(stream, value):
    fit = _stream(2.0)
    before = fit.fit()
    streams = [(1.0, 1.0), (2.0, 1.0), (3.0, 1.0)]
    streams[stream] = (value, 1.0)
    fit.add(50.0, *streams)
    assert fit.fit() == before


def test_map_rotation_keeps_fit_through_a_slow_hold():
    from mhseals_testing.analysis.geo import MapRotation
    fit = MapRotation(settle=0)
    x = 0.0
    for i in range(1, 2001):  # 0.2 m/s at 5 Hz, standing still from 100 s to 160 s
        t = i / 5
        x += 0.0 if 100 < t < 160 else 0.04
        fit.add(t, (x, 0.0), (x + 1, 0.0), (x, 0.0))
        if 60 < t < 160 and t > 70:
            assert fit.fit() is not None, t


def test_enu_takes_the_short_way_across_the_antimeridian():
    east, _ = enu(0, -179.9999, (0, 179.9999))
    assert east == pytest.approx(22.24, abs=0.1)
    assert enu(0, 179.9999, (0, -179.9999))[0] == pytest.approx(-22.24, abs=0.1)

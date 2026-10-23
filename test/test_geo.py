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

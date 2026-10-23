import numpy as np
import pytest

from mhseals_testing.analysis import conditions as cond
from mhseals_testing.analysis import readiness as r

TOL = r.load_tolerances()
FLOOR = np.array([0.0025, 0.01, 0.01])      # yaw rad/s, surge, sway m/s


def test_shipped_tolerances_load():
    assert TOL == {'psi': 0.015, 'ell': 0.10}


@pytest.mark.parametrize('speed, expected', [(0.38, 0.98), (0.21, 0.79), (0.16, 0.66), (0.09, 0.41)])
def test_spec_worked_numbers(speed, expected):
    assert r.entry_trust(TOL['psi'], speed, 0.0025) == pytest.approx(expected, abs=0.005)


def test_drift_trust_at_spec_se():
    assert r.drift_trust(0.010, 0.02) == pytest.approx(0.86, abs=0.005)
    assert r.drift_trust(0.015, 0.02) == pytest.approx(0.59, abs=0.005)
    assert r.drift_trust(None, 0.02) is None


def test_gyro_bias_error_lowers_translation_ceiling_but_drift_does_not():
    d = r.Direction('surge+', 'surge', 0.38)
    base = r.direction_ceiling(d, FLOOR, 0.0, TOL)
    assert r.direction_ceiling(d, FLOOR, 0.05, TOL) == base
    assert r.direction_ceiling(d, FLOOR, 0.0, TOL, gyro_bias_se=0.003) < base


def test_yaw_ceiling_falls_with_drift_error():
    d = r.Direction('yaw+', 'yaw', 0.3)
    assert r.direction_ceiling(d, FLOOR, 0.03, TOL) < r.direction_ceiling(d, FLOOR, 0.0, TOL)


def test_one_direction_plan_gates_on_its_own_direction():
    weak, strong = r.Direction('sway-', 'sway', 0.09), r.Direction('surge+', 'surge', 0.38)
    both = r.ceilings([weak, strong], FLOOR, 0.0, TOL)
    assert min(both.values()) == both['sway-'] < 0.6
    assert r.ceilings([strong], FLOOR, 0.0, TOL).keys() == {'surge+'}
    assert min(r.ceilings([strong], FLOOR, 0.0, TOL).values()) > 0.9


def _est(noise, n=8, drift=(0.0, 0.0), seed=1):
    rng = np.random.default_rng(seed)
    s = cond.load_settings()
    t = np.arange(0, 8 * n, 0.2)
    vel = np.array(drift) + rng.normal(0, noise, (len(t), 2))
    rate = rng.normal(0, noise / 4, len(t))
    heading = np.zeros(len(t))
    windows = cond.build_windows(t, vel, heading, rate, np.ones(len(t), bool), s)
    return cond.estimate(windows, s), s, 8 * n


def test_rate_unknown_without_an_estimate():
    est = cond.Estimate('unknown')
    assert r.rate(est, [r.Direction('surge+', 'surge', 0.38)], 0)[0] == 'unknown'


def test_rate_grades_by_planned_direction_and_names_it():
    est, s, now = _est(0.002)
    good = r.rate(est, [r.Direction('surge+', 'surge', 0.38)], now, settings=s)
    assert good[0] == 'good' and good[1] == ''
    weak = r.rate(est, [r.Direction('surge+', 'surge', 0.38), r.Direction('sway-', 'sway', 0.02)], now, settings=s)
    assert weak[0] == 'poor' and weak[1].startswith('sway- ceiling')
    rerun = r.rate(est, [r.Direction('surge+', 'surge', 0.38)], now, settings=s)
    assert rerun[0] == 'good'


def test_material_trend_is_poor_whatever_the_ceiling():
    est, s, now = _est(0.002)
    est = cond.Estimate('trending', est.drift, est.trend, 9.0, est.pool, size=0.03)
    grade, reason, *_ = r.rate(est, [r.Direction('surge+', 'surge', 0.38)], now, settings=s)
    assert grade == 'poor' and reason.startswith('trending')

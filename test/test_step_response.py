import math

import numpy as np
import pytest

from mhseals_testing.analysis.step_response import step_metrics


def second_order(target, zeta=0.4, omega=3.0, start=1.0, end=11.0, dt=0.02):
    """Underdamped unit-step response with analytic overshoot exp(-pi zeta / sqrt(1-zeta^2))."""
    wd = omega * math.sqrt(1 - zeta ** 2)
    phi = math.acos(zeta)
    samples = []
    for t in np.arange(0, end, dt):
        tau = t - start
        y = 0.0 if tau < 0 else 1 - math.exp(-zeta * omega * tau) * math.sin(wd * tau + phi) / math.sqrt(1 - zeta ** 2)
        samples.append((float(t), target * y))
    return samples


@pytest.mark.parametrize('target', [0.5, -0.5])
def test_underdamped_step_matches_analytic_overshoot(target):
    metrics = step_metrics(second_order(target), target, 1.0, 11.0)
    expected = math.exp(-math.pi * 0.4 / math.sqrt(1 - 0.4 ** 2))
    assert metrics['overshoot'] == pytest.approx(expected, abs=0.01)
    # 5% settling of a second-order system is about 3 / (zeta * omega).
    assert metrics['settling_time'] == pytest.approx(3 / (0.4 * 3.0), rel=0.25)
    assert metrics['steady_state_error'] == pytest.approx(0, abs=0.01)
    assert metrics['verdict'] == 'SETTLED' and metrics['rise_time'] > 0


def test_first_order_has_no_overshoot_and_known_iae():
    tau, target = 0.5, 1.0
    samples = [(t, target * (1 - math.exp(-t / tau))) for t in np.arange(0, 6, 0.001)]
    metrics = step_metrics(samples, target, 0.0, 6.0)
    assert metrics['overshoot'] == 0
    assert metrics['rise_time'] == pytest.approx(tau * math.log(9), rel=0.01)  # 10-90%
    assert metrics['iae'] == pytest.approx(tau, rel=0.01)  # integral of exp(-t/tau)


def test_unsettled_and_empty_steps():
    ramp = [(t, 0.1 * t) for t in np.arange(0, 3, 0.1)]
    assert step_metrics(ramp, 1.0, 0, 3)['verdict'] == 'NOT SETTLED'
    assert step_metrics([], 1.0, 0, 3)['verdict'] == 'INCONCLUSIVE'
    assert step_metrics(ramp, 0.0, 0, 3)['verdict'] == 'INCONCLUSIVE'

import numpy as np
import pytest

from mhseals_testing.analysis import conditions as c

S = c.load_settings()
HZ = S.neutral_gps_hz


def ar1(n, tau, sigma, rng, columns=2):
    """Unit-step AR(1) noise whose correlation time is ``tau`` s."""
    rho = np.exp(-1 / HZ / tau)
    x = np.zeros((n, columns))
    x[0] = rng.normal(0, sigma, columns)
    for i in range(1, n):
        x[i] = rho * x[i - 1] + rng.normal(0, sigma * np.sqrt(1 - rho ** 2), columns)
    return x


def neutral(seconds, drift=(0.08, -0.05), tau=1.0, sigma=0.03, seed=0, heading=0.0, t0=0.0, ramp=(0, 0)):
    """Samples of a drifting boat at neutral: (t, vel, heading, rate, mask)."""
    rng = np.random.default_rng(seed)
    t = t0 + np.arange(int(seconds * HZ)) / HZ
    vel = np.array(drift) + np.outer(t - t0, ramp) + ar1(len(t), tau, sigma, rng)
    return t, vel, np.full(len(t), heading), rng.normal(0, 0.002, len(t)), np.ones(len(t), bool)


def windows_of(samples, hold=None):
    return c.build_windows(*samples, S, hold=hold)


def test_settings_load_from_the_yaml():
    assert (S.window_s, S.batch_s, S.coast_max_drop, S.drift_trend_z) == (8, 4, 2, 3)





def test_coast_time_and_mask():
    assert c.coast_time(10, 0.3, S) == pytest.approx(12.0)          # floor of 2 s
    assert c.coast_time(10, 3.0, S) == pytest.approx(10 + 1.33 * 3)
    t = np.arange(10.0)
    mask = c.neutral_mask(t, t > 1, np.where(t == 6, 0.5, 0.1), 3.0, S)
    assert list(np.nonzero(mask)[0]) == [3, 4, 5, 7, 8, 9]


def test_coast_settle_drops_decay_and_keeps_flat_series():
    direction, sigma = (1.0, 0.0), 0.005
    decay = [[0.3 / (1 + 0.5 * 4 * i), 0, 0] for i in range(5)]    # v0 / (1 + t / T)
    drop, settled = c.settle_coast(decay, direction, sigma, S)
    assert drop == 2 and not settled
    drop, settled = c.settle_coast([[0.0 / (1 + i) + 0.02, 0, 0] for i in range(5)], direction, sigma, S)
    assert (drop, settled) == (0, True)
    late = [[0.2, 0, 0], [0.09, 0, 0], [0.085, 0, 0], [0.08, 0, 0]]
    assert c.settle_coast(late, direction, sigma, S) == (1, True)


def test_build_windows_flags_coast_and_trims_leading_batches():
    t, vel, heading, rate, mask = neutral(40, sigma=0.0)
    vel[:, 0] += 0.3 / (1 + 0.5 * t)
    windows = c.build_windows(t, vel, heading, rate, mask, S, hold=((1.0, 0.0), 0.005))
    assert windows[0].flag == 'coast' and windows[0].t0 == pytest.approx(8.0)
    assert all(w.flag == '' for w in windows[1:])


def test_window_gates_coverage_and_spin_and_gaps_split_runs():
    t, vel, heading, rate, mask = neutral(16, sigma=0.0)
    thin = np.arange(len(t)) % 3 != 0                      # 3.3 Hz of a 5 Hz stream
    sparse = c.build_windows(t[thin], vel[thin], heading[thin], rate[thin], mask[thin], S)
    assert {w.flag for w in sparse} == {'sparse'}
    spin = c.build_windows(t, vel, heading, np.full(len(t), 0.1), mask, S)
    assert {w.flag for w in spin} == {'spin'}
    assert [w.flag for w in c.build_windows(t, vel, heading, rate, mask, S)] == ['', '']
    # A 2 s dropout splits the run: the 2 s before it is too short, the rest restarts at 4 s.
    keep = ~((t > 2) & (t < 4))
    gap = c.build_windows(t[keep], vel[keep], heading[keep], rate[keep], mask[keep], S)
    assert [w.t0 for w in gap] == pytest.approx([4.0, 12.0])





def test_gps_truncation_bias_is_below_half_a_centimetre():
    rng = np.random.default_rng(0)
    speed = rng.uniform(0.0, 0.5, 20000)
    truncated = np.floor(speed * 100) / 100             # vel arrives in whole cm/s
    assert 0 <= (speed - truncated).mean() <= 0.005







def test_settings_ignore_keys_of_other_parts(tmp_path):
    path = tmp_path / 'c.yaml'
    path.write_text(c._config_path().read_text() + 'windage: {k: 1}\nhint: x\ndrift_tau: 90\n')
    assert c.load_settings(path) == S

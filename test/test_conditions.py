import numpy as np
import pytest

from mhseals_testing.analysis import conditions as c

S = c.load_settings()
HZ = 5          # the rate the helpers generate at; the estimator measures it from the timestamps


def ar1(n, tau, sigma, rng, columns=2, hz=HZ):
    """Unit-step AR(1) noise whose correlation time is ``tau`` s."""
    rho = np.exp(-1 / hz / tau)
    x = np.zeros((n, columns))
    x[0] = rng.normal(0, sigma, columns)
    for i in range(1, n):
        x[i] = rho * x[i - 1] + rng.normal(0, sigma * np.sqrt(1 - rho ** 2), columns)
    return x


def neutral(seconds, drift=(0.08, -0.05), tau=1.0, sigma=0.03, seed=0, heading=0.0, t0=0.0, ramp=(0, 0), hz=HZ):
    """Samples of a drifting boat at neutral: (t, vel, heading, rate, mask)."""
    rng = np.random.default_rng(seed)
    t = t0 + np.arange(int(seconds * hz)) / hz
    vel = np.array(drift) + np.outer(t - t0, ramp) + ar1(len(t), tau, sigma, rng, hz=hz)
    return t, vel, np.full(len(t), heading), rng.normal(0, 0.002, len(t)), np.ones(len(t), bool)


def windows_of(samples, hold=None):
    return c.build_windows(*samples, S, hold=hold)


def test_settings_load_from_the_yaml():
    assert (S.window_s, S.batch_s, S.coast_max_drop, S.drift_trend_z) == (8, 4, 2, 3)


@pytest.mark.parametrize('tau', [0.5, 1.0, 2.0])
def test_ar1_correction_recovers_spectral_level(tau):
    sigma = 0.03
    truth = 2 * sigma ** 2 * tau            # long-run level of an AR(1) with correlation time tau
    levels = [c.spectral_level(windows_of(neutral(8 * 300, tau=tau, sigma=sigma, seed=seed)), S)
              for seed in range(6)]
    assert np.mean(levels) == pytest.approx(truth, rel=0.10)


@pytest.mark.parametrize('hz', [5, 10, 20])
def test_spectral_level_follows_the_sample_rate_of_the_data(hz):
    truth = 2 * 0.03 ** 2 * 1.0
    levels = [c.spectral_level(windows_of(neutral(8 * 100, sigma=0.03, seed=seed, hz=hz)), S) for seed in range(6)]
    assert np.mean(levels) == pytest.approx(truth, rel=0.10)


def test_circular_heading_mean_and_vector_drift():
    # Heading wobbles across north; a plain mean would give 180 degrees.
    t, vel, _, rate, mask = neutral(8 * 6, sigma=0.0)
    heading = np.where(np.arange(len(t)) % 2, 0.1, 2 * np.pi - 0.1)
    windows = c.build_windows(t, vel, heading, rate, mask, S)
    assert windows[0].heading == pytest.approx(0.0, abs=1e-9)
    # Averaged components keep a drift that speed averaging would inflate.
    noisy = neutral(8 * 40, drift=(0.0, 0.0), sigma=0.05, seed=1)
    drift = c.world_drift(windows_of(noisy), S)
    assert np.linalg.norm(drift.vector) < 3 * drift.se


def test_a_stationary_gps_bias_is_reported_as_drift_not_removed():
    # Recorded Here GPS at rest read 0.038 m/s (se 0.002): the estimator must pass it on for the
    # readiness gate to judge, not treat it as zero.
    drift = c.world_drift(windows_of(neutral(8 * 12, drift=(0.038, 0.0), seed=7)), S)
    assert np.hypot(*drift.vector) == pytest.approx(0.038, abs=3 * drift.se)
    assert drift.se < 0.005


def test_drift_matches_truth_within_its_error():
    drift = c.world_drift(windows_of(neutral(8 * 12, seed=5)), S)
    assert drift.n == 12
    assert drift.vector == pytest.approx([0.08, -0.05], abs=3 * drift.se)
    assert drift.dispersion < 3


def test_coast_time_and_mask():
    assert c.coast_time(10, 0.3, S) == pytest.approx(12.0)          # floor of 2 s
    assert c.coast_time(10, 3.0, S) == pytest.approx(10 + 1.33 * 3)
    t = np.arange(10.0)
    mask = c.neutral_mask(t, t > 1, np.full(10, 0.1), 3.0, S)
    assert list(np.nonzero(mask)[0]) == [3, 4, 5, 6, 7, 8, 9]
    assert not c.neutral_mask(t, t > 1, np.full(10, 0.5), 3.0, S).any()      # poor accuracy throughout


def test_patchy_vel_acc_gates_on_the_window_median_not_each_sample():
    # Real Here GPS at rest: a third of the samples over 0.3 m/s, often two in a row.
    t, vel, heading, rate, _ = neutral(80, seed=3)
    acc = np.where(np.arange(len(t)) % 3 == 0, 0.36, 0.22)
    acc[1::9] = 0.36                                              # some pairs in a row
    assert (acc > S.neutral_vel_acc_max).mean() > 0.3
    mask = c.neutral_mask(t, np.ones(len(t), bool), acc, 0.0, S)
    assert mask.all()
    assert len(c.build_windows(t, vel, heading, rate, mask, S)) == 10
    bad = c.neutral_mask(t, np.ones(len(t), bool), np.full(len(t), 0.36), 0.0, S)
    assert not bad.any()                                          # a genuinely poor fix still gates


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
    fast = neutral(16, sigma=0.0, hz=20)
    thin = fast[0] % 1.5 >= 0.45                           # 0.45 s dropouts, short enough not to split the run
    sparse = c.build_windows(*(a[thin] for a in fast), S)
    assert {w.flag for w in sparse} == {'sparse'}
    t, vel, heading, rate, mask = neutral(16, sigma=0.0)
    spin = c.build_windows(t, vel, heading, np.full(len(t), 0.1), mask, S)
    assert {w.flag for w in spin} == {'spin'}
    assert [w.flag for w in c.build_windows(t, vel, heading, rate, mask, S)] == ['', '']
    # A 2 s dropout splits the run: the 2 s before it is too short, the rest restarts at 4 s.
    keep = ~((t > 2) & (t < 4))
    gap = c.build_windows(t[keep], vel[keep], heading[keep], rate[keep], mask[keep], S)
    assert [w.t0 for w in gap] == pytest.approx([4.0, 12.0])


def test_isolated_masked_samples_do_not_split_a_window():
    t, vel, heading, rate, mask = neutral(80, seed=3)
    mask[::17] = False                        # one rejected sample in 17, a 0.4 s gap at 5 Hz
    masked = c.build_windows(t, vel, heading, rate, mask, S)
    removed = c.build_windows(t[mask], vel[mask], heading[mask], rate[mask], mask[mask], S)
    assert len(masked) == len(removed) == 10
    assert {w.flag for w in masked} == {''}


def test_trend_and_change_flags():
    steady = c.estimate(windows_of(neutral(8 * 15, seed=2)), S)
    assert steady.state == 'steady' and steady.drift.n == 15
    ramp = c.estimate(windows_of(neutral(8 * 15, seed=2, ramp=(0.0006, 0))), S)
    assert ramp.state == 'trending' and ramp.trend[0] == pytest.approx(0.0006, rel=0.3)
    first, second = neutral(8 * 6, seed=2), neutral(8 * 6, drift=(0.25, -0.05), seed=4, t0=8 * 6)
    stepped = c.estimate(windows_of(tuple(np.concatenate(p) for p in zip(first, second))), S)
    assert stepped.state == 'changed'
    assert stepped.drift.vector[0] == pytest.approx(0.25, abs=0.04)


def test_too_few_windows_is_unknown():
    assert c.estimate(windows_of(neutral(16)), S).state == 'unknown'
    assert c.world_drift(windows_of(neutral(16)), S) is None


def test_wave_floor_follows_gyro_and_velocity_scatter():
    samples = neutral(8 * 20, sigma=0.01, seed=7)
    t, vel, heading, rate, mask = samples
    # A gust: alternate windows lean on the gyro, which window means must show.
    rate = rate + np.where((t // 8) % 2 == 0, 0.01, -0.01)
    floor = c.wave_floor(c.build_windows(t, vel, heading, rate, mask, S), S)
    assert floor[0] == pytest.approx(0.01, rel=0.2)
    assert floor[1] < 0.02 and floor[2] < 0.02
    assert c.wave_floor([], S) is None


def test_gps_truncation_bias_is_below_half_a_centimetre():
    rng = np.random.default_rng(0)
    speed = rng.uniform(0.0, 0.5, 20000)
    truncated = np.floor(speed * 100) / 100             # vel arrives in whole cm/s
    assert 0 <= (speed - truncated).mean() <= 0.005


def stepped(jump, sigma, seed=2):
    first = neutral(8 * 8, sigma=sigma, seed=seed)
    second = neutral(8 * 8, drift=(0.08 + jump, -0.05), sigma=sigma, seed=seed + 2, t0=8 * 8)
    return tuple(np.concatenate(p) for p in zip(first, second))


def test_a_drift_step_does_not_inflate_the_spectral_level():
    level = c.spectral_level(windows_of(stepped(0.17, 0.03)), S)
    assert level == pytest.approx(c.spectral_level(windows_of(stepped(0.0, 0.03)), S), rel=0.10)


def test_calm_water_step_is_changed_not_trending():
    assert c.estimate(windows_of(stepped(0.2, 0.01)), S).state == 'changed'


def test_one_outlier_after_a_restart_is_not_a_second_change():
    samples = list(stepped(0.25, 0.02))
    t, vel = samples[0], samples[1].copy()
    vel[(t >= 8 * 11) & (t < 8 * 12)] += (0.06, 0.0)             # window 11, the first a pool of three tests
    windows = windows_of((t, vel, *samples[2:]))
    est = c.estimate(windows, S)
    assert est.state == "changed" and est.pool[0].t0 == pytest.approx(8 * 8)


def test_wave_floor_ignores_retired_windows():
    samples = stepped(0.25, 0.01)
    both = c.wave_floor(windows_of(samples), S)
    after = c.wave_floor([w for w in windows_of(samples) if w.t0 >= 8 * 8], S)
    assert both[1] < 0.02 and both == pytest.approx(after, rel=0.5)


def test_settings_ignore_keys_of_other_parts(tmp_path):
    path = tmp_path / 'c.yaml'
    path.write_text(c._config_path().read_text() + 'windage: {k: 1}\nhint: x\ndrift_tau: 90\n')
    assert c.load_settings(path) == S


def test_wave_floor_keeps_surge_and_sway_levels_apart():
    # Heading 0: surge is north, sway is minus east.
    rng = np.random.default_rng(3)
    t, vel, heading, rate, mask = neutral(8 * 40, sigma=0.0)
    noise = ar1(len(t), 1.0, 1.0, rng)
    vel = vel + noise * np.array([0.06, 0.005])
    floor = c.wave_floor(c.build_windows(t, vel, heading, rate, mask, S), S)
    truth = lambda sd: np.sqrt(2 * sd ** 2 * 1.0 / S.window_s)
    assert floor[1] == pytest.approx(truth(0.005), rel=0.5)
    assert floor[2] == pytest.approx(truth(0.06), rel=0.5)


def test_wave_floor_survives_a_pool_of_short_windows():
    # The Characterize shape: full windows, then a drift step seen only through 5 s windows.
    parts = [neutral(8 * 3, seed=1)]
    for i in range(4):
        parts.append(neutral(5, drift=(0.3, 0.1), seed=10 + i, t0=24 + 10 * i + 3))
    samples = tuple(np.concatenate(p) for p in zip(*parts))
    windows = windows_of(samples)
    assert c.estimate(windows, S).state == 'changed'
    assert np.all(np.isfinite(c.wave_floor(windows, S)))


@pytest.mark.parametrize('tau, count', [(2.0, 5), (2.0, 3), (3.0, 10)])
def test_spectral_level_stays_bounded_with_few_windows(tau, count):
    truth = 2 * 0.03 ** 2 * tau
    ratio = [c.spectral_level(windows_of(neutral(8 * count, tau=tau, seed=seed)), S) / truth
             for seed in range(150)]
    assert np.percentile(ratio, 95) < 3


@pytest.mark.parametrize('tau, count', [(0.5, 3), (1.0, 4), (1.0, 6)])
def test_trend_false_alarms_stay_rare_on_stationary_data(tau, count):
    alarms = sum(c.estimate(windows_of(neutral(8 * count, tau=tau, seed=seed)), S).state == 'trending'
                 for seed in range(300))
    assert alarms <= 3


@pytest.mark.parametrize('column', ['vel', 'heading', 'rate'])
def test_a_non_finite_sample_is_dropped_not_propagated(column):
    t, vel, heading, rate, mask = neutral(8 * 12, seed=5)
    {'vel': vel, 'heading': heading, 'rate': rate}[column][400] = np.nan
    est = c.estimate(windows_of((t, vel, heading, rate, mask)), S)
    assert est.state == 'steady'
    assert np.isfinite(est.drift.vector).all()
    assert np.isfinite(c.wave_floor(windows_of((t, vel, heading, rate, mask)), S)).all()


def test_noise_free_constant_drift_is_steady_with_finite_error():
    t, vel, heading, _, mask = neutral(8 * 10, sigma=0.0)
    samples = (t, vel, heading, np.zeros(len(t)), mask)
    with np.errstate(all='raise'):
        est = c.estimate(windows_of(samples), S)
    assert est.state == 'steady'
    assert est.drift.s0 == S.noise_s0_floor
    assert np.isfinite(est.drift.dispersion) and est.drift.se > 0
    assert est.drift.vector == pytest.approx([0.08, -0.05])
    assert np.all(np.isfinite(c.wave_floor(windows_of(samples), S)))


def _same_windows(a, b):
    assert len(a) == len(b) > 0
    for x, y in zip(a, b):
        assert x.t0 == y.t0 and x.flag == y.flag and x.hz == pytest.approx(y.hz)
        assert x.mean == pytest.approx(y.mean)


def test_duplicated_samples_give_the_clean_windows():
    clean = neutral(60)
    _same_windows(windows_of([np.repeat(a, 2, axis=0) for a in clean]), windows_of(clean))


def test_locally_swapped_samples_give_the_clean_windows():
    clean = neutral(60)
    order = np.arange(len(clean[0]))
    order[0:-1:3], order[1::3] = order[1::3].copy(), order[0:-1:3].copy()
    _same_windows(windows_of([a[order] for a in clean]), windows_of(clean))


def test_reversed_samples_give_the_clean_windows():
    clean = neutral(60)
    _same_windows(windows_of([a[::-1] for a in clean]), windows_of(clean))


def noise_free_step(jump):
    first = neutral(8 * 8, sigma=0.0)
    second = neutral(8 * 8, drift=(0.08 + jump, -0.05), sigma=0.0, t0=8 * 8)
    return tuple(np.concatenate(p) for p in zip(first, second))


def test_a_millimetre_step_in_noise_free_sim_is_changed_but_not_material():
    est = c.estimate(windows_of(noise_free_step(0.003)), S)
    assert est.state == 'changed' and est.size == pytest.approx(0.003, abs=5e-4)
    assert not est.material(S)


def test_a_practical_step_is_material():
    est = c.estimate(windows_of(noise_free_step(0.05)), S)
    assert est.material(S)


def _same_windows(a, b):
    assert len(a) == len(b) > 0
    for x, y in zip(a, b):
        assert x.t0 == y.t0 and x.flag == y.flag and x.hz == pytest.approx(y.hz)
        assert x.mean == pytest.approx(y.mean)


def test_duplicated_samples_give_the_clean_windows():
    clean = neutral(60)
    _same_windows(windows_of([np.repeat(a, 2, axis=0) for a in clean]), windows_of(clean))


def test_locally_swapped_samples_give_the_clean_windows():
    clean = neutral(60)
    order = np.arange(len(clean[0]))
    order[0:-1:3], order[1::3] = order[1::3].copy(), order[0:-1:3].copy()
    _same_windows(windows_of([a[order] for a in clean]), windows_of(clean))


def test_reversed_samples_give_the_clean_windows():
    clean = neutral(60)
    _same_windows(windows_of([a[::-1] for a in clean]), windows_of(clean))


def _stamped(t):
    """Windows over gapless-by-construction samples stamped at ``t``."""
    n = len(t)
    return c.build_windows(t, np.tile([0.08, -0.05], (n, 1)), np.zeros(n), np.zeros(n), np.ones(n, bool), S)


@pytest.mark.parametrize('name', ['alternating', 'jitter', 'bursts', 'one_dropped'])
def test_rate_estimate_is_not_fooled_by_stamp_pattern(name):
    rng = np.random.default_rng(1)
    n = 20 * 40
    if name == 'alternating':                              # sim header stamps on 10 ms /clock ticks
        t = np.cumsum(np.tile([0.04, 0.06], n // 2))
    elif name == 'jitter':                                 # +/-40 % receipt jitter on a 20 Hz stream
        t = np.cumsum(0.05 * rng.uniform(0.6, 1.4, n))
    elif name == 'bursts':                                 # delivered in pairs
        t = np.repeat(np.arange(n // 2) * 0.1, 2) + np.tile([0.0, 0.002], n // 2)
    else:                                                  # one sample lost per window
        t = np.delete(np.arange(n) / 20, np.arange(100, n, 160))
    windows = _stamped(t)
    assert len(windows) >= 4 and {w.flag for w in windows} == {''}
    assert windows[0].hz == pytest.approx(20, rel=0.1)

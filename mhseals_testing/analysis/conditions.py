"""Water conditions at neutral: world drift, wave floor and stationarity (conditions spec 1.1 to 1.4).

ROS-free. Inputs are arrays: ``t`` in s, world velocity ``vel`` (E, N) in m/s, compass heading
``heading`` in rad (clockwise from north) and gyro yaw rate ``rate`` in rad/s. Thrust is dead
in the samples used here, so the mean GPS velocity is the boat's drift.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
from functools import lru_cache
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class Settings:
    window_s: float
    min_window_s: float
    batch_s: float
    coast_min_s: float
    coast_t95_mult: float
    coast_settle_sigma: float
    coast_max_drop: int
    neutral_vel_acc_max: float
    neutral_yaw_rate_max: float
    neutral_coverage: float
    neutral_max_gap_s: float
    drift_trend_z: float
    drift_change_chi2: float
    noise_s0_floor: float


def _config_path():
    """config/conditions.yaml in the source tree, else the installed share directory."""
    source = Path(__file__).resolve().parent.parent.parent / 'config' / 'conditions.yaml'
    if source.exists():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('mhseals_testing')) / 'config' / 'conditions.yaml'


def load_settings(path=None) -> Settings:
    """Settings from the YAML; a group's keys are prefixed with its name (coast: {min_s} -> coast_min_s)."""
    raw = yaml.safe_load(Path(path or _config_path()).read_text())
    flat = {}
    for key, value in raw.items():
        flat.update({f'{key}_{k}': v for k, v in value.items()} if isinstance(value, dict) else {key: value})
    known = {f.name for f in fields(Settings)}      # the file is shared with the readiness parts
    return Settings(**{k: v for k, v in flat.items() if k in known})


@dataclass
class Window:
    """One neutral window: the means and sub-batch means of (E, N, yaw rate)."""
    t0: float
    duration: float
    mean: np.ndarray        # (3,) E, N, r
    heading: float          # circular mean of the compass
    sub: np.ndarray         # (k, 3) means of consecutive batch_s slices
    moments: np.ndarray     # (5, 3) per channel: sum x[i]x[i-1], sum x^2, sum x, x[0]+x[-1], count
    hz: float               # sample rate measured from the timestamps
    flag: str = ''          # '' usable; sparse, spin or coast keeps it out of the pool


@dataclass
class Drift:
    vector: np.ndarray      # (2,) world drift E, N in m/s
    se: float               # per component, inflated by overdispersion
    n: int                  # windows pooled
    dispersion: float       # D = chi2 / df
    s0: float               # spectral level of the velocity noise, m^2/s


@dataclass
class Estimate:
    state: str              # unknown, steady, trending or changed
    drift: Drift | None = None
    trend: np.ndarray | None = None   # (2,) m/s per s
    drift_trend_z: float = 0.0
    pool: tuple = ()        # the windows behind the drift: those since the last change


# --- neutral samples and windows -------------------------------------------------------

def coast_time(t_cut, t95, s: Settings):
    """First time after the last non-zero command at which the boat counts as coasted."""
    return t_cut + max(s.coast_min_s, s.coast_t95_mult * t95)


def neutral_mask(t, ok, vel_acc, t_coast, s: Settings):
    """Samples that are neutral: ``ok`` (zero command, neutral PWM, afloat, 3D fix) and settled."""
    return np.asarray(ok, bool) & (np.asarray(vel_acc) <= s.neutral_vel_acc_max) & (np.asarray(t) >= t_coast)


def settle_coast(sub_means, direction, sigma_sub, s: Settings):
    """Leading batches to drop, and whether the coast has settled.

    Quadratic drag decays as 1/t, so a batch still differs from the next by more than
    ``coast_settle_sigma`` sigma along the last hold direction while the boat is coasting.
    """
    along = np.asarray(sub_means)[:, :2] @ np.asarray(direction, float)
    for drop in range(s.coast_max_drop + 1):
        if drop + 1 >= len(along):
            return drop, False
        if abs(along[drop] - along[drop + 1]) <= s.coast_settle_sigma * sigma_sub:
            return drop, True
    return s.coast_max_drop, False


def _window(t, vel, heading, rate, hz, s: Settings, flag=''):
    """Window statistics for the samples of one chunk."""
    x = np.column_stack([vel, rate])
    duration = float(t[-1] - t[0]) + 1 / hz
    eps = 0.5 / hz                     # half a sample absorbs float error in t
    cuts = int((duration + eps) // s.batch_s)
    parts = [x[(t >= t[0] + i * s.batch_s - eps) & (t < t[0] + (i + 1) * s.batch_s - eps)] for i in range(cuts)]
    mean = x.mean(axis=0)
    if not flag and len(t) < s.neutral_coverage * hz * duration:
        flag = 'sparse'
    elif not flag and abs(mean[2]) >= s.neutral_yaw_rate_max:
        flag = 'spin'
    return Window(
        t0=float(t[0]), duration=duration, mean=mean,
        heading=float(np.arctan2(np.sin(heading).mean(), np.cos(heading).mean())),
        sub=np.array([p.mean(axis=0) for p in parts if len(p)]).reshape(-1, 3),
        moments=np.array([(x[1:] * x[:-1]).sum(axis=0), (x ** 2).sum(axis=0), x.sum(axis=0),
                          x[0] + x[-1], np.full(3, len(x))]), hz=hz, flag=flag)


def build_windows(t, vel, heading, rate, mask, s: Settings, hold=None):
    """Back-to-back windows over each neutral run; masked samples shorter than max_gap_s do not split it.

    ``hold`` is (direction, sigma_sub) of the last hold: leading batches that have not settled
    are dropped, and a first window that still fails is flagged ``coast``.
    """
    t, vel, heading, rate = (np.asarray(a, float) for a in (t, vel, heading, rate))
    finite = np.isfinite(vel).all(axis=1) & np.isfinite(heading) & np.isfinite(rate) & np.isfinite(t)
    idx = np.nonzero(np.asarray(mask, bool) & finite)[0]       # a garbled sample splits the run, never poisons it
    runs = np.split(idx, np.nonzero(np.diff(t[idx]) > s.neutral_max_gap_s)[0] + 1)
    steps = np.diff(t[idx])
    steps = steps[steps <= s.neutral_max_gap_s]
    if not len(steps):
        return []
    hz = 1 / float(np.median(steps))                           # the velocity source sets the rate, not the yaml
    windows = []
    for run in (r for r in runs if len(r)):
        start, unsettled = t[run[0]], False
        if hold is not None:
            batches = [run[(t[run] >= start + i * s.batch_s) & (t[run] < start + (i + 1) * s.batch_s)]
                       for i in range(s.coast_max_drop + 2)]
            subs = [np.column_stack([vel[b], rate[b]]).mean(axis=0) for b in batches if len(b)]
            if len(subs) >= 2:
                drop, settled = settle_coast(subs, *hold, s)
                start, unsettled = start + drop * s.batch_s, not settled
        while True:
            chunk = run[(t[run] >= start) & (t[run] < start + s.window_s)]
            if not len(chunk) or t[chunk[-1]] - start + 1 / hz < s.min_window_s:
                break
            windows.append(_window(t[chunk], vel[chunk], heading[chunk], rate[chunk], hz, s,
                                   'coast' if unsettled else ''))
            start, unsettled = start + s.window_s, False
    return windows


# --- estimation ---------------------------------------------------------------------------

@lru_cache(maxsize=None)
def _demeaned_lag1(n):
    """(rho grid, expected demeaned lag-1 estimate) for AR(1) windows of n samples."""
    rho = np.linspace(0.0, 0.999, 100)
    centre = np.eye(n) - 1 / n
    lag = np.eye(n, k=1) + np.eye(n, k=-1)
    num = centre @ lag @ centre / 2
    idx = np.arange(n)
    expected = [np.trace(num @ r ** abs(idx[:, None] - idx)) / (n - 1) /
                (np.trace(centre @ r ** abs(idx[:, None] - idx)) / n) for r in rho]
    return rho, np.array(expected)


def _autocorrelation(windows, channels):
    """Lag-1 autocorrelation of ``channels`` from each window's own deviations.

    Demeaning per window keeps a shift of the drift between windows out of the estimate. The
    small-sample bias that leaves is undone by inverting its exact AR(1) expectation.
    """
    num = den = pairs = samples = 0.0
    for k in channels:
        for w in windows:
            cross, square, total, edges, n = w.moments[:, k]
            mu = total / n
            num += cross - mu * (2 * total - edges) + (n - 1) * mu ** 2
            den += square - n * mu ** 2
            pairs, samples = pairs + n - 1, samples + n
    if den <= 0:
        return 0.0
    rho, expected = _demeaned_lag1(int(round(samples / (len(channels) * len(windows)))))
    return float(np.interp(num / pairs * samples / den, expected, rho))


def _scatter_ratio(k, rho, hz, s: Settings):
    """E[sum of squared deviations of k batch means about their mean] over (k - 1) S0 / batch_s.

    Exact for AR(1) noise: it covers the correlation inside a batch (the factor f of spec 1.2)
    and between neighbouring batches, which pull a window's own mean towards each batch.
    """
    if rho <= 0 or k < 2:
        return 1.0
    b, tau = s.batch_s, -1 / hz / np.log(rho)
    e = np.exp(-b / tau)
    var = 1 - tau / b * (1 - e)                                  # f
    cov = [tau / (2 * b) * (1 - e) ** 2 * e ** (d - 1) for d in range(1, k)]
    pairs = sum((k - d) * cov[d - 1] for d in range(1, k))
    return (k * var - (k * var + 2 * pairs) / k) / (k - 1)


def spectral_level(windows, s: Settings, channels=(0, 1), rotate=False):
    """S0 in unit^2*s from the scatter of batch means about their own window mean.

    With ``rotate`` the (E, N) columns become (surge, sway) by each window's compass first, and
    ``channels`` then index those body columns. The lag-1 correlation stays pooled over E and N.
    Returns None without a window of two or more batches.
    """
    rho = _autocorrelation(windows, [0, 1] if rotate else channels)
    # A few long-correlated windows can read past the curve; bound tau by half a batch so S0 stays sane.
    hz = float(np.median([w.hz for w in windows]))
    rho = min(rho, float(np.exp(-1 / hz / (s.batch_s / 2))))
    total = expected = 0.0
    for w in windows:
        if len(w.sub) < 2:
            continue
        dev = w.sub - w.sub.mean(axis=0)
        dev = (_body(dev, w.heading) if rotate else dev)[:, list(channels)]
        total += float((dev ** 2).sum())
        expected += dev.shape[1] * (len(w.sub) - 1) * _scatter_ratio(len(w.sub), rho, w.hz, s)
    # Noise-free input (the sim) would give S0 = 0 and divide every test by zero: floor it.
    return max(s.batch_s * total / expected, s.noise_s0_floor) if expected else None


def _body(world, heading):
    """Rows of (E, N, ...) as (surge, sway left, ...) for a compass heading in rad."""
    forward = world[:, 0] * np.sin(heading) + world[:, 1] * np.cos(heading)
    left = -world[:, 0] * np.cos(heading) + world[:, 1] * np.sin(heading)
    return np.column_stack([forward, left])


def _pool(windows, s0):
    """Duration-weighted drift with overdispersion D (spec 1.2)."""
    t = np.array([w.duration for w in windows])
    m = np.array([w.mean[:2] for w in windows])
    vector = (t[:, None] * m).sum(axis=0) / t.sum()
    df = 2 * len(windows) - 2
    chi2 = float((t * ((m - vector) ** 2).sum(axis=1)).sum() / s0)
    dispersion = chi2 / df if df > 0 else 1.0
    return Drift(vector, float(np.sqrt(s0 / t.sum() * max(1.0, dispersion))), len(windows),
                 dispersion, s0)


def _trend(windows, s0):
    """Weighted slope of the window means against time, and its worst z score."""
    t = np.array([w.t0 + w.duration / 2 for w in windows])
    weight = np.array([w.duration for w in windows]) / s0
    m = np.array([w.mean[:2] for w in windows])
    centred = t - (weight * t).sum() / weight.sum()
    spread = (weight * centred ** 2).sum()
    if spread <= 0:
        return np.zeros(2), 0.0
    slope = (weight[:, None] * centred[:, None] * m).sum(axis=0) / spread
    return slope, float(np.max(np.abs(slope)) * np.sqrt(spread))


def _trend_limit(z, dof):
    """Student-t equivalent of the normal limit ``z`` when S0 is itself estimated from ``dof``.

    Cornish-Fisher expansion, which keeps the false-alarm rate down with few windows.
    """
    return z + (z ** 3 + z) / (4 * dof) + (5 * z ** 5 + 16 * z ** 3 + 3 * z) / (96 * dof ** 2)


def estimate(windows, s: Settings):
    """Drift, trend and change over the usable windows, in time order.

    A window whose mean departs from the earlier pool by more than ``drift_change_chi2`` on two
    consecutive windows is a change: the older windows retire and the pool restarts there.
    """
    usable = sorted((w for w in windows if not w.flag), key=lambda w: w.t0)
    s0 = spectral_level(usable, s)
    if len(usable) < 3 or s0 is None:
        return Estimate('unknown')
    start, state, prev = 0, 'steady', False
    for i in range(3, len(usable)):
        pool = usable[start:i]
        if len(pool) < 3:
            continue
        w, d = usable[i], _pool(pool, s0).vector
        out = w.duration * ((w.mean[:2] - d) ** 2).sum() / s0 > s.drift_change_chi2
        if out and prev:
            start, state, out = i - 1, 'changed', False
        prev = out
    pool = usable[start:]
    if len(pool) < 3:
        return Estimate('changed' if state == 'changed' else 'unknown')
    slope, z = _trend(pool, s0)
    if state == 'steady' and z > _trend_limit(s.drift_trend_z, 2 * len(pool)):
        state = 'trending'
    return Estimate(state, _pool(pool, s0), slope, z, tuple(pool))


def world_drift(windows, s: Settings):
    """The pooled world drift of the usable windows, or None before three of them exist."""
    return estimate(windows, s).drift


def wave_floor(windows, s: Settings):
    """Per channel (yaw, surge, sway) SD of an ``window_s`` window mean, the trust floor.

    The residuals come from the current pool, since windows a change retired sit about another drift.
    Each variance is max(S0 / T_w, between-window scatter), conservative while few windows exist.
    """
    est = estimate(windows, s)
    if est.drift is None:
        return None
    pool = list(est.pool)
    # Window-demeaned scatter ignores drift steps, so S0 uses every usable window, not just the pool.
    usable = [w for w in windows if not w.flag]
    body = np.vstack([_body((w.mean[:2] - est.drift.vector)[None], w.heading) for w in pool])
    yaw = np.array([w.mean[2] for w in pool])
    floor = []
    for res, s0 in ((yaw - yaw.mean(), spectral_level(usable, s, channels=(2,))),
                    (body[:, 0], spectral_level(usable, s, channels=(0,), rotate=True)),
                    (body[:, 1], spectral_level(usable, s, channels=(1,), rotate=True))):
        floor.append(np.sqrt(max((s0 or 0.0) / s.window_s, float((res ** 2).sum() / (len(res) - 1)))))
    return np.array(floor)

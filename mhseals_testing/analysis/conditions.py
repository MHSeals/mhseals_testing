"""Water conditions at neutral: world drift, wave floor and stationarity (conditions spec 1.1 to 1.4).

ROS-free. Inputs are arrays: ``t`` in s, world velocity ``vel`` (E, N) in m/s, compass heading
``heading`` in rad (clockwise from north) and gyro yaw rate ``rate`` in rad/s. Thrust is dead
in the samples used here, so the mean GPS velocity is the boat's drift.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
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
    neutral_gps_hz: float
    neutral_vel_acc_max: float
    neutral_yaw_rate_max: float
    neutral_coverage: float
    neutral_max_gap_s: float
    drift_trend_z: float
    drift_change_chi2: float


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
    flag: str = ''          # '' usable; sparse, spin or coast keeps it out of the pool


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


def _window(t, vel, heading, rate, s: Settings, flag=''):
    """Window statistics for the samples of one chunk."""
    x = np.column_stack([vel, rate])
    duration = float(t[-1] - t[0]) + 1 / s.neutral_gps_hz
    eps = 0.5 / s.neutral_gps_hz                     # half a sample absorbs float error in t
    cuts = int((duration + eps) // s.batch_s)
    parts = [x[(t >= t[0] + i * s.batch_s - eps) & (t < t[0] + (i + 1) * s.batch_s - eps)] for i in range(cuts)]
    mean = x.mean(axis=0)
    if not flag and len(t) < s.neutral_coverage * s.neutral_gps_hz * duration:
        flag = 'sparse'
    elif not flag and abs(mean[2]) >= s.neutral_yaw_rate_max:
        flag = 'spin'
    return Window(
        t0=float(t[0]), duration=duration, mean=mean,
        heading=float(np.arctan2(np.sin(heading).mean(), np.cos(heading).mean())),
        sub=np.array([p.mean(axis=0) for p in parts if len(p)]).reshape(-1, 3),
        moments=np.array([(x[1:] * x[:-1]).sum(axis=0), (x ** 2).sum(axis=0), x.sum(axis=0),
                          x[0] + x[-1], np.full(3, len(x))]), flag=flag)


def build_windows(t, vel, heading, rate, mask, s: Settings, hold=None):
    """Back-to-back windows over each contiguous neutral run.

    ``hold`` is (direction, sigma_sub) of the last hold: leading batches that have not settled
    are dropped, and a first window that still fails is flagged ``coast``.
    """
    t, vel, heading, rate = (np.asarray(a, float) for a in (t, vel, heading, rate))
    idx = np.nonzero(mask)[0]
    runs = np.split(idx, np.nonzero((np.diff(idx) > 1) | (np.diff(t[idx]) > s.neutral_max_gap_s))[0] + 1)
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
            if not len(chunk) or t[chunk[-1]] - start + 1 / s.neutral_gps_hz < s.min_window_s:
                break
            windows.append(_window(t[chunk], vel[chunk], heading[chunk], rate[chunk], s,
                                   'coast' if unsettled else ''))
            start, unsettled = start + s.window_s, False
    return windows

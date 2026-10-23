"""Live tracking score per axis and calibration staleness across sessions (thrust addendum).

ROS-free. The score compares a measured axis speed with the forward model's prediction over a
sliding window; staleness is a robust trend of the stored per-session scores. Thresholds come
from config/tracking.yaml.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml


def load_settings(path=None) -> dict:
    """The tracking.yaml groups ({'score': {...}, 'stale': {...}}), source tree first."""
    source = Path(__file__).resolve().parent.parent.parent / 'config' / 'tracking.yaml'
    if path is None and not source.exists():
        from ament_index_python.packages import get_package_share_directory
        source = Path(get_package_share_directory('mhseals_testing')) / 'config' / 'tracking.yaml'
    return yaml.safe_load(Path(path or source).read_text())


def tracking_score(measured, predicted, command, sigma_wave, settings, ignore=False):
    """Fraction in [0, 1] of the predicted motion the boat followed, or None when gated.

    ``measured``, ``predicted`` and ``command`` are equal-length window samples of one axis.
    Only samples with a command above the deadband, or a prediction above ``active_sigma``
    wave floors, count; ``ignore`` (yaw unreliable, poor conditions) drops the window.
    """
    s = settings['score']
    measured, predicted, command = (np.asarray(x, dtype=float) for x in (measured, predicted, command))
    if ignore or not len(measured):
        return None
    finite = np.isfinite(measured) & np.isfinite(predicted) & np.isfinite(command)
    measured, predicted, command = measured[finite], predicted[finite], command[finite]
    if not len(measured):
        return None
    counted = (np.abs(command) > s['deadband']) | (np.abs(predicted) > s['active_sigma'] * sigma_wave)
    if counted.mean() < s['min_active']:
        return None
    error = np.sqrt(np.mean((measured[counted] - predicted[counted]) ** 2))
    scale = np.sqrt(np.mean(predicted[counted] ** 2)) + s['k'] * sigma_wave
    return float(max(0.0, 1.0 - error / scale)) if scale > 0 else None


def score_colour(score, settings):
    """Health colour of a score; None (gated) has none."""
    s = settings['score']
    if score is None:
        return None
    return 'green' if score >= s['green'] else 'yellow' if score >= s['yellow'] else 'red'


def _weighted_median(values, weights):
    order = np.argsort(values)
    cumulative = np.cumsum(np.asarray(weights)[order])
    return float(np.asarray(values)[order][np.searchsorted(cumulative, cumulative[-1] / 2)])


def theil_sen(x, y, weights):
    """Median pairwise slope, each pair weighted by the product of its weights; None if no pair."""
    pairs = [(i, j) for i in range(len(x)) for j in range(i + 1, len(x)) if x[j] != x[i]]
    if not pairs:
        return None
    slopes = [(y[j] - y[i]) / (x[j] - x[i]) for i, j in pairs]
    return _weighted_median(slopes, [weights[i] * weights[j] for i, j in pairs])


def _falling(x, y, w, s, rng):
    """(slope, whether it is negative beyond its bootstrap interval) of one session series."""
    slope = theil_sen(x, y, w)
    boots = []
    for _ in range(s['bootstrap']):
        pick = rng.integers(0, len(x), len(x))
        if len(set(pick)) < 3:          # a resample with fewer than 3 distinct sessions has no trend
            continue
        value = theil_sen(x[pick], y[pick], w[pick])
        if value is not None:
            boots.append(value)
    if slope is None or not boots:
        return slope, False
    return slope, bool(np.quantile(boots, (1 + s['confidence']) / 2) < 0)


def staleness(sessions, version, settings, seed=0):
    """Judge whether the calibration ``version`` should be recharacterized.

    ``sessions`` are oldest-first dicts {'mean', 'n', 'trust', 'version'} of one axis. Only
    sessions on ``version`` with a finite mean count. Flags when, with at least ``trend_sessions``
    sessions, the weighted Theil-Sen slope is negative beyond its bootstrap interval and still is
    with the newest session dropped (so one bad run cannot make a trend), or when the last
    ``low_runs`` means are all below ``low_score``. Returns {'flag', 'reason', 'slope', 'sessions'}.
    """
    s = settings['stale']
    used = [r for r in sessions if r['version'] == version and r['mean'] is not None
            and np.isfinite(r['mean'])]
    result = dict(flag=False, reason='', slope=None, sessions=len(used))
    if len(used) < s['min_sessions']:
        return result
    x = np.arange(len(used), dtype=float)
    y = np.array([r['mean'] for r in used])
    w = np.array([max(r['n'] * r['trust'], 1e-9) for r in used])
    if len(used) >= s['trend_sessions']:
        result['slope'], falling = _falling(x, y, w, s, np.random.default_rng(seed))
        if falling and _falling(x[:-1], y[:-1], w[:-1], s, np.random.default_rng(seed))[1]:
            result.update(flag=True, reason='falling trend')
            return result
    if len(used) >= s['low_runs'] and all(r['mean'] < s['low_score'] for r in used[-s['low_runs']:]):
        result.update(flag=True, reason='low scores')
    return result

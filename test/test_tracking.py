import numpy as np

from mhseals_testing.analysis.tracking import (
    load_settings, score_colour, staleness, theil_sen, tracking_score)

S = load_settings()
T = np.linspace(0, 10, 50)
V = 0.8 * np.sin(T)


def test_perfect_and_poor_tracking():
    full = np.ones_like(T)
    assert tracking_score(V, V, full, 0.02, S) == 1.0
    assert tracking_score(V * 0.5, V, full, 0.02, S) < 0.6
    assert tracking_score(-V, V, full, 0.02, S) == 0.0


def test_wave_floor_softens_small_errors():
    noisy = V + 0.03 * np.cos(40 * T)
    assert tracking_score(noisy, V, np.ones_like(T), 0.05, S) > tracking_score(noisy, V, np.ones_like(T), 0.0, S)


def test_gates_show_nothing():
    idle = np.zeros_like(T)
    assert tracking_score(V, V, idle, 5.0, S) is None          # predicted motion below the floor
    assert tracking_score(V, V, np.ones_like(T), 0.02, S, ignore=True) is None
    assert tracking_score([], [], [], 0.02, S) is None
    half = np.r_[np.ones(10), np.zeros(40)]
    assert tracking_score(V, V, half, 5.0, S) is None          # under min_active of the window


def test_colours_follow_thresholds():
    assert [score_colour(x, S) for x in (0.9, 0.7, 0.3, None)] == ['green', 'yellow', 'red', None]


def runs(means, version=1):
    return [dict(mean=m, n=20, trust=0.9, version=version) for m in means]


def test_theil_sen_ignores_an_outlier():
    assert theil_sen([0, 1, 2, 3, 4], [1, 2, 3, 40, 5], [1] * 5) == 1.0


def test_steady_scores_never_flag_and_one_bad_run_does_not():
    assert not staleness(runs([0.9, 0.88, 0.91, 0.89]), 1, S)['flag']
    assert not staleness(runs([0.9, 0.91, 0.89, 0.9, 0.3]), 1, S)['flag']


def test_falling_trend_and_two_low_runs_flag():
    falling = staleness(runs([0.95, 0.9, 0.85, 0.8, 0.75, 0.72]), 1, S)
    assert falling['flag'] and falling['reason'] == 'falling trend' and falling['slope'] < 0
    low = staleness(runs([0.9, 0.65, 0.65, 0.65]), 1, S)
    assert low['flag'] and low['reason'] == 'low scores'


def test_other_versions_and_short_history_are_ignored():
    mixed = runs([0.2, 0.2, 0.2], version=1) + runs([0.9, 0.9], version=2)
    assert not staleness(mixed, 2, S)['flag'] and staleness(mixed, 2, S)['sessions'] == 2

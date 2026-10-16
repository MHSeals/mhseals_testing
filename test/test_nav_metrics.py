import math

import numpy as np
import pytest

from mhseals_testing.analysis.nav_metrics import (
    path_length, segment_distances, station_keeping, waypoint_metrics,
)


def test_straight_path_has_zero_cross_track():
    points = [(x, 0.0) for x in np.linspace(0, 10, 50)]
    metrics = waypoint_metrics(points, [(10, 0)], start=(0, 0))
    assert metrics['cross_track_max'] == pytest.approx(0)
    assert metrics['path_length'] == pytest.approx(10)
    assert metrics['final_error'] == pytest.approx(0)


def test_offset_path_reports_the_offset_and_corners_use_the_nearest_leg():
    points = [(x, 0.5) for x in np.linspace(0, 10, 50)]
    metrics = waypoint_metrics(points, [(10, 0)], start=(0, 0))
    assert metrics['cross_track_rms'] == pytest.approx(0.5)
    # At a corner the nearer leg counts, not the infinite line through it.
    assert segment_distances([(11, 1)], [(0, 0), (10, 0), (10, 10)])[0] == pytest.approx(1)
    assert segment_distances([(12, -1)], [(0, 0), (10, 0)])[0] == pytest.approx(math.sqrt(5))
    assert path_length([(0, 0), (3, 4), (3, 0)]) == pytest.approx(9)


def test_station_keeping_recovers_a_steady_drift():
    times = np.linspace(0, 60, 601)
    points = np.column_stack([0.01 * times, -0.02 * times]) + 0.05 * np.sin(times)[:, None]
    metrics = station_keeping(times, points, (0, 0), tolerance=2.0)
    assert metrics['drift_speed'] == pytest.approx(math.hypot(0.01, 0.02), rel=0.05)
    assert metrics['drift_heading'] == pytest.approx(math.atan2(-0.02, 0.01), abs=0.05)
    assert metrics['verdict'] == 'HELD' and metrics['time_inside'] == pytest.approx(1)


def test_station_keeping_leaving_the_circle():
    times = np.arange(0, 10, 1.0)
    points = [(t * 0.5, 0) for t in times]  # out to 4.5 m
    metrics = station_keeping(times, points, (0, 0), tolerance=2.0)
    assert metrics['verdict'] == 'LEFT TOLERANCE' and metrics['max_radius'] == pytest.approx(4.5)
    assert metrics['time_inside'] == pytest.approx(5 / 9)
    assert station_keeping([0, 1], [(0, 0), (1, 1)], (0, 0), 1)['verdict'] == 'INCONCLUSIVE'

"""Conditions checks on a neutral sim session: the estimated drift is the scene's current."""
import json
import math

import numpy as np
import pytest

from mhseals_testing.analysis import conditions as c
from simrig import read_odometry, truth_current, world_arrays

SETTLE_S = 5.0     # the scene current moves the boat before the filters settle


def test_truth_current_points_toward_the_stated_angle():
    e, n = truth_current({'currentSpeed': 0.2, 'currentToward': -math.pi / 2})
    assert (round(e, 6), round(n, 6)) == (0.0, -0.2)


def test_world_arrays_rotate_body_velocity_and_flip_to_compass_heading():
    half = math.sqrt(0.5)      # yaw 90 deg (facing north)
    t, vel, heading, rate = world_arrays([0.0], [[0, 0, half, half]], [[1.0, 0.0, 0.1]])
    assert vel[0] == pytest.approx([0.0, 1.0], abs=1e-9)
    assert heading[0] == pytest.approx(0.0) and rate[0] == 0.1


def test_session_records_the_truth_the_checks_compare_with(session):
    truth = json.loads((session / 'course_truth.json').read_text())
    assert truth['schema'] == 'crane-roboboat-course-truth-v1'
    assert (session / 'bag').is_dir()


def test_hold_still_drift_matches_the_scene_current(request):
    pytest.importorskip('rosbag2_py')   # before the session fixture records one
    session = request.getfixturevalue('session')
    t, vel, heading, rate = read_odometry(session)
    keep = t > t[0] + SETTLE_S
    s = c.load_settings()
    windows = c.build_windows(t[keep], vel[keep], heading[keep], rate[keep], np.ones(keep.sum(), bool), s)
    drift = c.world_drift(windows, s)
    assert drift is not None, 'fewer than three neutral windows: record longer'
    truth = json.loads((session / 'course_truth.json').read_text())
    # speed is rotation invariant; the world-to-map rotation is the map checks' business
    assert abs(np.hypot(*drift.vector) - np.hypot(*truth_current(truth))) <= max(2 * drift.se, 0.02)


def test_frames_when_captured_have_text_and_a_review_svg(session):
    frames = session / 'frames'
    if not frames.is_dir():
        pytest.skip('no --frames capture in this session (boat_trial sim not built yet)')
    assert list(frames.glob('*.txt')) and (frames / 'last.svg').is_file()

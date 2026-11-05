"""Regression from the 11-04 pool session (blastoise, session 20261105_011850): the motion checks must judge what
the boat really did, and every verdict must name the data it used."""
import math

import pytest

from mhseals_testing.procedures.characterize import UnusableResponse, load_settings, response

# /odom/local recorded on the station during run 003-characterize (surge+ at 0.4, 01:43 UTC), 1 s means:
# t, vx, vy, wz, x, y, yaw (deg). Hold 1 ran at about 41-44 s; the procedure's own gate must see it move.
POOL_SURGE = [
    (35, +0.01, -0.00, -0.01, 5.31, 9.27, -27), (36, +0.02, -0.02, -0.01, 5.31, 9.24, -28),
    (37, +0.01, -0.03, +0.00, 5.29, 9.22, -28), (38, -0.00, -0.03, -0.01, 5.27, 9.20, -28),
    (39, -0.01, -0.02, +0.02, 5.27, 9.20, -27), (40, -0.01, +0.01, +0.04, 5.25, 9.21, -25),
    (41, +0.04, +0.06, +0.17, 5.42, 9.28, -15), (42, +0.16, +0.18, +0.25, 5.68, 9.46, -1),
    (43, +0.34, +0.30, +0.28, 5.99, 9.85, +15), (44, +0.39, +0.38, +0.15, 6.17, 10.26, +22),
]


def history(rows):
    return [(t, dict(vx=vx, vy=vy, wz=wz, x=x, y=y, yaw=math.radians(yaw), stamp_age=0.02))
            for t, vx, vy, wz, x, y, yaw in rows]


def test_the_recorded_surge_hold_is_measured_as_moving_and_says_with_what():
    cfg = load_settings()
    evidence = {}
    delta = response(history(POOL_SURGE), (35, 41), (41, 44), cfg['transient_s'], cfg['coast_s'], evidence,
                     cfg['coverage'])
    assert delta[0] > 5 * cfg['min_response']['surge']          # clearly not dead: about +0.3 m/s
    hold = evidence['hold']
    assert hold['n'] == 3 and hold['t0'] == 42 and hold['t1'] == 44
    assert hold['displacement_m'] == pytest.approx(math.hypot(6.17 - 5.68, 10.26 - 9.46), abs=1e-3)
    assert hold['yaw_change_deg'] == pytest.approx(23, abs=0.5) and hold['lag_s']['max'] == 0.02
    assert evidence['settle']['n'] == 5 and evidence['frame'].startswith('drift removed')


def turning_in_a_current(true_sway, current=(0.15, 0.0), turn=math.radians(50), hz=20):
    """Settle 6 s at heading 0 in a steady odom-frame current, then a 3 s hold that turns the boat by ``turn``
    while it sways ``true_sway`` through the water (the pool's waterfall and its curving holds)."""
    rows, t = [], 0.0
    for k in range(int(9 * hz)):
        t = k / hz
        holding = t >= 6
        yaw = turn * min(max(t - 6, 0) / 3, 1)
        c, s = math.cos(yaw), math.sin(yaw)
        e, n = current
        bx, by = c * e + s * n, -s * e + c * n          # the current seen in the body
        by += true_sway if holding else 0.0
        rows.append((t, dict(vx=bx, vy=by, wz=turn / 3 if holding else 0.0, yaw=yaw)))
    return rows


def test_a_turning_boat_in_a_current_is_not_called_backwards():
    """00:39 UTC: "sway responds backwards" while the boat curved in the waterfall current; the body-frame
    difference turned the current into a reversed sway. Removed in the odom frame it is the true response."""
    cfg = load_settings()
    evidence = {}
    delta = response(turning_in_a_current(0.04), (0, 6), (6, 9), cfg['transient_s'], cfg['coast_s'], evidence,
                     cfg['coverage'])
    assert delta[1] == pytest.approx(0.04, abs=1e-3)
    rows = turning_in_a_current(0.04)
    body = [v['vy'] for t, v in rows if 7 <= t <= 9]
    assert sum(body) / len(body) < 0     # what the old body-frame difference saw: a reversed sway


def test_thin_odometry_refuses_instead_of_calling_the_direction_dead():
    """01:35 UTC: surge+ was called dead while the boat moved. A hold the TUI saw too few samples of is unmeasured,
    and the refusal says so with the counts, instead of a verdict on missing data."""
    cfg = load_settings()
    rows = [r for r in history(POOL_SURGE) if not 41 < r[0] < 44]    # the hold's samples never arrived
    with pytest.raises(UnusableResponse, match='too thin to judge the hold'):
        response(rows, (35, 41), (41, 44), cfg['transient_s'], cfg['coast_s'], {}, cfg['coverage'])

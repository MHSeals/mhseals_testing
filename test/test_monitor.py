import json
import math

import pytest

from mhseals_testing.monitor import BoatMonitor, HISTORY_SAMPLES


def test_receipt_rate_quality_and_stamp_freshness():
    clock = [0.0]
    monitor = BoatMonitor(clock=lambda: clock[0])
    assert monitor.snapshot()['sensors'] == {}
    for index in range(10):
        clock[0] = index / 10
        monitor.observe('IMU', stamp_age=.05)
    imu = monitor.snapshot()['sensors']['IMU']
    assert imu['status'] == 'LIVE'
    assert math.isclose(imu['hz'], 10)
    monitor.observe('GPS', issue='no fix', stamp_age=0)
    assert monitor.snapshot()['sensors']['GPS']['status'] == 'WARN'
    monitor.observe('future', stamp_age=-2)
    assert monitor.snapshot()['sensors']['future']['status'] == 'WARN'
    clock[0] += 3
    assert monitor.snapshot()['sensors']['IMU']['status'] == 'STALE'


def test_tf_connectivity_static_age_conflict_cycle_and_recovery():
    clock = [0.0]
    monitor = BoatMonitor(clock=lambda: clock[0])
    monitor.transform('map', 'odom', static=True, authority='global')
    monitor.transform('odom', 'base_link', authority='local')
    assert all(row['status'] == 'LIVE' for row in monitor.snapshot()['tf'])
    monitor.transform('odom', 'base_link', authority='duplicate')
    assert monitor.snapshot()['tf'][1]['status'] == 'WARN'
    clock[0] = 3
    assert monitor.snapshot()['tf'][0]['status'] == 'LIVE'
    assert monitor.snapshot()['tf'][1]['status'] == 'MISSING'
    monitor.transform('odom', 'base_link', authority='local')
    assert monitor.snapshot()['tf'][1]['status'] == 'LIVE'
    monitor.transform('base_link', 'odom', authority='bad_cycle')
    assert monitor.snapshot()['tf'][1]['status'] == 'WARN'


def test_history_windows_and_skips_flagged_samples():
    clock = [0.0]
    monitor = BoatMonitor(clock=lambda: clock[0])
    for tick in range(10):
        clock[0] = tick
        monitor.observe('odometry', {'vx': float(tick)}, issue='bad' if tick == 5 else '')
    assert [t for t, _ in monitor.history('odometry', 3, 6)] == [3, 4, 6]
    assert len(monitor.history('odometry', include_issues=True)) == 10
    assert monitor.history('IMU') == []  # not a history sensor


def test_advisories_are_reported_and_validated():
    monitor = BoatMonitor(clock=lambda: 4)
    monitor.advise('battery', 'WARN', '14.1 V')
    assert monitor.snapshot()['advisories'] == {
        'battery': dict(level='WARN', detail='14.1 V', age=0)}
    with pytest.raises(ValueError):
        monitor.advise('battery', 'BAD')


def test_bad_numeric_data_is_exportable():
    monitor = BoatMonitor(clock=lambda: 1)
    monitor.observe('odometry', {'vx': float('nan')})
    snapshot = monitor.snapshot()
    assert snapshot['sensors']['odometry']['status'] == 'WARN'
    assert monitor.history('odometry') == []
    json.dumps(snapshot, allow_nan=False)


def test_history_and_events_are_bounded():
    monitor = BoatMonitor(clock=lambda: 1)
    for index in range(HISTORY_SAMPLES + 1000):
        monitor.observe('odometry', {'vx': index})
        monitor.event('event')
    assert len(monitor.histories['odometry']) == HISTORY_SAMPLES
    assert len(monitor.events) == 12
    assert len(monitor.sensors['odometry']['times']) == 300


def test_mission_history_retains_the_start_of_a_thirty_minute_run():
    clock = [0.]
    monitor = BoatMonitor(clock=lambda: clock[0], history_sensors=('ground truth',))
    for i in range(50 * 1800):
        clock[0] = i / 50
        monitor.observe('ground truth', {'x': i / 50})
    assert monitor.history('ground truth', 0, 1)[0][0] == 0.


def test_real_gps_gaps_do_not_flicker_stale():
    # tranquility bag boat_20261019_002558: 4.61 Hz with a 1.13 s worst gap in 41 s.
    # Any rate-derived deadline (ts028) must stay above this, so floor it at 1.5 s.
    clock = [0.0]
    monitor = BoatMonitor(clock=lambda: clock[0])
    stale = []
    for gap in [.2] * 20 + [.65, 1.13] + [.2] * 20 + [.7, .8]:
        clock[0] += gap
        monitor.observe('GPS', stamp_age=0)
        # Sample just before the next fix would arrive.
        clock[0] += .01
        stale.append(monitor.snapshot()['sensors']['GPS']['status'] == 'STALE')
        clock[0] -= .01
    assert not any(stale)
    clock[0] += 2.1
    assert monitor.snapshot()['sensors']['GPS']['status'] == 'STALE'

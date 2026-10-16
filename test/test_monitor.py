import json
import math

import pytest

from mhseals_testing.monitor import BoatMonitor


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
    for index in range(13000):
        monitor.observe('odometry', {'vx': index})
        monitor.event('event')
    assert len(monitor.histories['odometry']) == 12000
    assert len(monitor.events) == 12
    assert len(monitor.sensors['odometry']['times']) == 300

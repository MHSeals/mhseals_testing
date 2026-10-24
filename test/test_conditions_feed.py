"""Source contract through actual BoatMonitor snapshots; no ROS or outputs."""
import math

import pytest

from mhseals_testing.harness.conditions_feed import ConditionsFeed
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.profiles import load_profile


def make_feed(name='boat'):
    clock = [10.0]
    monitor = BoatMonitor(clock=lambda: clock[0])
    feed = ConditionsFeed(load_profile(name))
    return clock, monitor, feed


def observe(monitor, gps=None, yaw=0, **kwargs):
    monitor.observe('odometry', dict(yaw=yaw, wz=.01), stamp_age=0)
    monitor.observe('GPS raw', dict(ve=.2, vn=.1, fix=3, vel_acc=.15, **(gps or {})), **kwargs)


def poll(monitor, feed, now, **kwargs):
    return feed.poll(monitor.snapshot(), now, eligible=kwargs.get('eligible', True),
                     commanding=kwargs.get('commanding', False))


def test_gps_receipt_dedup_and_unknown_eligibility():
    clock, monitor, feed = make_feed()
    observe(monitor)
    clock[0] += .2
    sample = poll(monitor, feed, clock[0], eligible=None)
    assert sample.time == 10 and sample.velocity == (.2, .1)
    assert sample.accuracy == .15 and sample.heading == pytest.approx(math.pi / 2)
    assert not sample.eligible
    for _ in range(150):
        clock[0] += .2
        assert poll(monitor, feed, clock[0]) is None
    observe(monitor)
    assert not poll(monitor, feed, clock[0], commanding=None).eligible


@pytest.mark.parametrize('key,value', [('fix', 2), ('ve', math.nan), ('vn', math.inf),
                                      ('vel_acc', 0), ('vel_acc', -1), ('vel_acc', math.nan),
                                      ('vel_acc', math.inf), ('vel_acc', True)])
def test_bad_gps_consumed_once(key, value):
    clock, monitor, feed = make_feed()
    values = dict(ve=.2, vn=.1, fix=3, vel_acc=.15)
    values[key] = value
    monitor.observe('odometry', dict(yaw=0, wz=0))
    monitor.observe('GPS raw', values)
    assert poll(monitor, feed, clock[0]) is None
    monitor.sensors['GPS raw']['values'][key] = .15  # repaired copy is still the same receipt
    assert poll(monitor, feed, clock[0]) is None


@pytest.mark.parametrize('stamp_age', [3, -.5, math.inf, math.nan])
def test_bad_source_stamp(stamp_age):
    clock, monitor, feed = make_feed()
    observe(monitor, stamp_age=stamp_age)
    assert poll(monitor, feed, clock[0]) is None


def test_missing_and_stale_heading_do_not_replay_gps():
    clock, monitor, feed = make_feed()
    monitor.observe('GPS raw', dict(ve=0, vn=0, fix=3, vel_acc=.1))
    assert poll(monitor, feed, clock[0]) is None
    monitor.observe('odometry', dict(yaw=0, wz=0))
    assert poll(monitor, feed, clock[0]) is None
    clock[0] += 3
    monitor.observe('GPS raw', dict(ve=0, vn=0, fix=3, vel_acc=.1))
    assert poll(monitor, feed, clock[0]) is None


def test_sim_uses_selected_messages_own_yaw_and_rate():
    clock, monitor, feed = make_feed('crane_full_stack')
    monitor.observe('odometry', dict(vx=99, vy=99, yaw=0, wz=99))
    monitor.observe('FCU odometry', dict(vx=1, vy=0, yaw=math.pi / 2, wz=.03))
    sample = poll(monitor, feed, clock[0])
    assert sample.velocity == pytest.approx((0, 1))
    assert sample.heading == pytest.approx(0) and sample.rate == .03 and sample.accuracy == 0
    assert sample.eligible


def test_epochs_count_restart_clock_rewind_and_source_switch():
    clock, monitor, feed = make_feed()
    observe(monitor)
    first = poll(monitor, feed, clock[0])
    clock[0] += .2
    observe(monitor)
    snapshot = monitor.snapshot()
    second = poll(monitor, feed, clock[0])
    snapshot['sensors']['GPS raw'].update(count=1, age=1)
    assert feed.poll(snapshot, clock[0], eligible=True, commanding=False) is None
    assert feed.epoch == second.epoch  # a swapped old receipt is not a restart
    clock[0] += 1
    monitor.sensors.clear()
    observe(monitor)
    assert poll(monitor, feed, clock[0]).epoch == first.epoch + 1
    clock[0] = 1
    monitor.sensors.clear()
    observe(monitor)
    assert poll(monitor, feed, clock[0]).epoch == first.epoch + 2
    feed.configure(load_profile('crane_direct'))
    assert feed.epoch == first.epoch + 3


@pytest.mark.parametrize('key,value', [('yaw', math.nan), ('wz', math.inf), ('vx', math.inf),
                                      ('vy', True)])
def test_nonfinite_sim_and_stale_receipts(key, value):
    clock, monitor, feed = make_feed('crane_direct')
    values = dict(vx=0, vy=0, yaw=0, wz=0)
    values[key] = value
    monitor.observe('odometry', values, stamp_age=0)
    assert poll(monitor, feed, clock[0]) is None
    clock[0] += 1
    monitor.observe('odometry', dict(vx=0, vy=0, yaw=0, wz=0), stamp_age=0)
    clock[0] += 3
    assert poll(monitor, feed, clock[0]) is None


def test_source_stamp_and_count_and_high_accuracy_remain_for_window_median():
    clock, monitor, feed = make_feed()
    observe(monitor, stamp_age=.1)
    monitor.sensors['GPS raw']['values']['vel_acc'] = .5
    clock[0] += .2
    sample = poll(monitor, feed, clock[0])
    assert sample.time == 10 and sample.count == 1 and sample.stamp_time == pytest.approx(9.9)
    assert sample.accuracy == .5  # threshold belongs to centered window median, not adapter


def test_cached_snapshot_preserves_receipt_and_stamp_then_expires():
    clock, monitor, feed = make_feed()
    observe(monitor, stamp_age=.1)
    snapshot = monitor.snapshot()
    sample = feed.poll(snapshot, 10.2, eligible=True, commanding=False)
    assert sample.time == 10 and sample.stamp_time == pytest.approx(9.9)
    assert feed.poll(snapshot, 10.4, eligible=True, commanding=False) is None
    delayed = ConditionsFeed(load_profile('boat'))
    assert delayed.poll(snapshot, 13, eligible=True, commanding=False) is None


def test_cached_attitude_expires_relative_to_poll():
    clock, monitor, feed = make_feed()
    monitor.observe('odometry', dict(yaw=0, wz=0), stamp_age=0)
    clock[0] = 11
    monitor.observe('GPS raw', dict(ve=0, vn=0, fix=3, vel_acc=.1), stamp_age=0)
    snapshot = monitor.snapshot()  # both LIVE here; only attitude expires by poll time
    assert feed.poll(snapshot, 12.5, eligible=True, commanding=False) is None


@pytest.mark.parametrize('invalid', [False, True])
def test_equal_count_restart_advances_epoch_once_even_when_invalid(invalid):
    clock, monitor, feed = make_feed()
    observe(monitor)
    old = monitor.snapshot()
    first = poll(monitor, feed, clock[0])
    clock[0] = 11
    monitor.sensors.clear()
    observe(monitor)
    if invalid:
        monitor.sensors['GPS raw']['values']['vel_acc'] = 0
    snapshot = monitor.snapshot()
    sample = feed.poll(snapshot, 11, eligible=True, commanding=False)
    assert (sample is None) == invalid
    assert feed.epoch == first.epoch + 1
    assert feed.poll(snapshot, 11.1, eligible=True, commanding=False) is None
    assert feed.poll(old, 11.1, eligible=True, commanding=False) is None
    assert feed.epoch == first.epoch + 1
    clock[0] = 11.2
    observe(monitor)
    assert poll(monitor, feed, clock[0]).epoch == first.epoch + 1


@pytest.mark.parametrize('metadata', ['received', 'capture'])
def test_missing_receipt_metadata_does_not_fall_back_to_poll_time(metadata):
    clock, monitor, feed = make_feed()
    observe(monitor)
    snapshot = monitor.snapshot()
    if metadata == 'received':
        snapshot['sensors']['GPS raw'].pop('received', None)
    else:
        snapshot['plots'].pop('now')
    assert feed.poll(snapshot, clock[0], eligible=True, commanding=False) is None


def test_delayed_snapshot_expires_message_stamp_before_receipt():
    clock, monitor, feed = make_feed()
    observe(monitor, stamp_age=1)
    snapshot = monitor.snapshot()
    assert feed.poll(snapshot, 11.5, eligible=True, commanding=False) is None

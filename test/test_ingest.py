"""One ingest path: RosIO callbacks and a bare ingest() call give equal snapshots."""

from types import SimpleNamespace as NS

import pytest

from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.profiles import SensorSpec
from mhseals_testing.ros.ingest import ingest
from mhseals_testing.ui.source import LiveSource

T = 100.0
PROFILE = NS(sensors=(SensorSpec('odometry', '/odom', 'odometry'), SensorSpec('imu', '/imu', 'imu')),
             base_frame='base_link', required_tf=(('odom', 'base_link'),), stamp_future=0.25)


def quaternion(w=1.0):
    return NS(x=0.0, y=0.0, z=0.0, w=w)


def odometry():
    vec = NS(x=0.5, y=0.0, z=0.1)
    return NS(header=NS(frame_id='odom', stamp=NS(sec=100, nanosec=0)), child_frame_id='base_link',
              twist=NS(twist=NS(linear=vec, angular=vec)),
              pose=NS(pose=NS(position=NS(x=1.0, y=2.0, z=0.0), orientation=quaternion())))


def imu():
    return NS(header=NS(frame_id='imu', stamp=NS(sec=100, nanosec=0)), orientation=quaternion(),
              orientation_covariance=[0.0] * 9, angular_velocity=NS(x=0.0, y=0.0, z=0.2))


def tf(w=1.0):
    return NS(transforms=[NS(header=NS(frame_id='odom', stamp=NS(sec=99, nanosec=0)),
                             child_frame_id='base_link',
                             transform=NS(rotation=quaternion(w), translation=NS(x=0.0, y=0.0, z=0.0)))])


MESSAGES = [('/odom', 'odometry', odometry()), ('/imu', 'imu', imu()),
            ('/tf', 'tf', tf()), ('/tf', 'tf', tf(w=0.0)),
            ('/drift', 'drift', NS(vector=NS(x=0.1, y=-0.2))),
            ('/events', 'event', {'phase': 'armed', 'pid': 1, 'seq': 3, 'outputs': 'none'})]


def snapshot_of(feed):
    monitor = BoatMonitor(clock=lambda: T)
    feed(monitor)
    return LiveSource(monitor, PROFILE).snapshot()


def test_ingest_reaches_every_snapshot_section():
    snap = snapshot_of(lambda m: [ingest(m, PROFILE, *msg[:2], msg[2], T) for msg in MESSAGES])
    assert set(snap['sensors']) == {'odometry', 'imu'}
    assert snap['drift'] == (0.1, -0.2)
    assert any('armed' in str(e) and 'outputs' in str(e) for e in snap['events'])
    assert any('invalid TF' in str(e) for e in snap['events'])


def test_rosio_callbacks_and_ingest_give_equal_snapshots():
    pytest.importorskip('rclpy')
    from mhseals_testing.ros.node import RosIO
    direct = snapshot_of(lambda m: [ingest(m, PROFILE, t, k, msg, T) for t, k, msg in MESSAGES])

    def via_rosio(monitor):
        io = NS(monitor=monitor, profile=PROFILE, stale_after=2.0, now=lambda: T)
        io._ingest = lambda *a, **k: RosIO._ingest(io, *a, **k)
        for topic, kind, msg in MESSAGES:
            if kind in ('tf',):
                RosIO._tf(io, msg)
            else:
                io._ingest(topic, kind, msg)

    assert snapshot_of(via_rosio) == direct

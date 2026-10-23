"""Feed one received message into the monitor: the single ingest path.

Live ROS callbacks and bag replay both call ``ingest``, so the dashboard
cannot tell where a message came from. Pure: no rclpy import, messages are
read by attribute, ``t`` is the receive time in seconds (``node.now()`` live,
the bag's receive time in replay).
"""

import json

from mhseals_testing.validation import RAW_KINDS, finite, validate, validate_raw

IDENTITY = ('time', 'ros_time', 'phase', 'pid', 'node', 'host', 'seq', 't_ns', 'rec')
KINDS = ('tf', 'tf_static', 'drift', 'event')


def ingest(monitor, profile, topic, kind, msg, t, stale_after=2.0, authority=''):
    """Route ``msg`` (on ``topic``, of ``kind``) into ``monitor``.

    ``kind`` is a sensor kind of the profile (odometry, imu, ...) or one of
    KINDS. ``authority`` names the publisher of a TF message (live only).
    """
    if kind in ('tf', 'tf_static'):
        _tf(monitor, msg, t, kind == 'tf_static', authority)
    elif kind == 'drift':
        monitor.estimate_drift(msg.vector.x, msg.vector.y)
    elif kind == 'event':
        details = {k: v for k, v in msg.items() if k not in IDENTITY}
        monitor.event(msg['phase'] + (' ' + json.dumps(details, sort_keys=True, default=str)
                                      if details else ''))
    else:
        for sensor in profile.sensors:
            if sensor.topic == topic and sensor.kind == kind:
                base = profile.base_frame if sensor.name == 'odometry' else None
                deadline = monitor.deadlines.get(sensor.name, stale_after)  # rate-derived (profiles)
                if kind in RAW_KINDS and isinstance(msg, (bytes, bytearray, memoryview)):
                    # Subscribed raw (live): only the header stamp is read.
                    result = validate_raw(msg, t, deadline)
                else:
                    result = validate(kind, msg, t, base, deadline)
                monitor.observe(sensor.name, result.values, result.issue, result.stamp_age)


def _tf(monitor, message, now, static, authority):
    for transform in message.transforms:
        q, p = transform.transform.rotation, transform.transform.translation
        if (not finite(q.x, q.y, q.z, q.w, p.x, p.y, p.z)
                or abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) > .1):
            monitor.event('invalid TF transform: ' + transform.child_frame_id)
            continue
        stamp = transform.header.stamp
        monitor.transform(transform.header.frame_id, transform.child_frame_id,
                          now - stamp.sec - stamp.nanosec / 1e9, static, authority)

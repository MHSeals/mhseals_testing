"""Per-message sanity checks, independent of ROS so they test with plain objects.

``validate`` turns one sensor message into the values the dashboard shows and
an ``issue`` string when the data can't be trusted (non-finite numbers, wrong
frame, missing fix, bad quaternion, stale or future timestamp).
"""

from dataclasses import dataclass
import math
import struct

# A unit quaternion has squared norm 1; allow slack for float32 publishers.
QUATERNION_TOLERANCE = 0.1
# Clock offsets between boat computers make slightly-future stamps normal.
FUTURE_TOLERANCE = 0.25


@dataclass
class Observation:
    values: dict
    issue: str
    stamp_age: float


def finite(*values):
    return all(math.isfinite(value) for value in values)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def _quaternion_ok(q):
    norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
    return math.isfinite(norm) and abs(norm - 1) <= QUATERNION_TOLERANCE


def _odometry(message, base_frame):
    velocity = message.twist.twist
    position, q = message.pose.pose.position, message.pose.pose.orientation
    values = dict(vx=velocity.linear.x, vy=velocity.linear.y, wz=velocity.angular.z,
                  x=position.x, y=position.y,
                  yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
                  frame=message.header.frame_id, child=message.child_frame_id)
    values['summary'] = f"vx {values['vx']:+.2f} vy {values['vy']:+.2f} wz {values['wz']:+.2f}"
    if not finite(values['vx'], values['vy'], values['wz'], values['x'], values['y']):
        return values, 'non-finite pose/velocity'
    if base_frame and message.child_frame_id != base_frame:
        return values, f'velocity frame is not {base_frame}'
    if not message.header.frame_id:
        return values, 'empty odometry reference frame'
    if not _quaternion_ok(message.pose.pose.orientation):
        return values, 'invalid odometry orientation'
    return values, ''


def _gps(message):
    values = dict(summary=f'fix {message.status.status} | '
                          f'{message.latitude:.6f}, {message.longitude:.6f}')
    covariance = message.position_covariance
    if message.status.status < 0:
        return values, 'GPS has no fix'
    if (not finite(message.latitude, message.longitude)
            or not -90 <= message.latitude <= 90 or not -180 <= message.longitude <= 180):
        return values, 'invalid GPS coordinates'
    if message.position_covariance_type == 0:
        return values, 'GPS covariance unknown'
    if not finite(*covariance) or any(covariance[index] < 0 for index in (0, 4, 8)):
        return values, 'invalid GPS covariance'
    # Diagonal entries are variances (m^2); report the larger horizontal sigma.
    sigma = math.sqrt(max(covariance[0], covariance[4]))
    values['summary'] += f' | max XY-axis sigma {sigma:.2f}m'
    return values, ''


def _imu(message):
    rate = message.angular_velocity
    values = dict(summary=f'wz {rate.z:+.3f} rad/s | {message.header.frame_id}')
    if not _quaternion_ok(message.orientation):
        return values, 'invalid orientation quaternion'
    # REP 145: covariance[0] == -1 means the IMU provides no orientation.
    if message.orientation_covariance[0] < 0:
        return values, 'IMU orientation unavailable'
    if not finite(rate.x, rate.y, rate.z):
        return values, 'invalid IMU angular velocity'
    return values, ''


def _points(message):
    values = dict(summary=f'{message.width * message.height} points | {message.header.frame_id}')
    return values, '' if message.width and message.data else 'empty point cloud'


def _image(message):
    values = dict(summary=message.header.frame_id)
    return values, '' if message.data else 'empty image'


def _detections(message):
    values = dict(count=len(message.detections),
                  summary=f'{len(message.detections)} detections | {message.header.frame_id}')
    return values, ''


def validate(kind, message, now, base_frame=None, stale_after=2.0):
    """Check one message of ``kind`` received at ROS time ``now`` (seconds)."""
    if kind == 'odometry':
        values, issue = _odometry(message, base_frame)
    elif kind == 'gps':
        values, issue = _gps(message)
    elif kind == 'imu':
        values, issue = _imu(message)
    elif kind == 'points':
        values, issue = _points(message)
    elif kind in ('image', 'compressed'):
        values, issue = _image(message)
    elif kind == 'detections':
        values, issue = _detections(message)
    else:
        raise ValueError(f'unknown sensor kind {kind!r}')
    values['sensor_frame'] = message.header.frame_id
    stamp = message.header.stamp
    stamp_age = now - stamp_seconds(stamp)
    if stamp.sec == 0 and stamp.nanosec == 0:
        issue = issue or 'zero message timestamp'
    elif stamp_age < -FUTURE_TOLERANCE or stamp_age > stale_after:
        issue = issue or 'message timestamp outside freshness window'
    return Observation(values, issue, stamp_age)


RAW_KINDS = ('points', 'image', 'compressed')  # heavy messages: only the header is read


def validate_raw(data, now, stale_after=2.0):
    """Like ``validate`` for a serialized CDR message, reading only ``header.stamp``.

    Skips deserializing multi-megabyte clouds and images on the ODROID; the first
    eight payload bytes after the 4-byte encapsulation header are the stamp.
    """
    if len(data) < 12:
        return Observation({}, 'truncated message', None)
    sec, nanosec = struct.unpack_from('<iI' if data[1] & 1 else '>iI', data, 4)
    stamp_age = now - (sec + nanosec * 1e-9)
    issue = ''
    if sec == 0 and nanosec == 0:
        issue = 'zero message timestamp'
    elif stamp_age < -FUTURE_TOLERANCE or stamp_age > stale_after:
        issue = 'message timestamp outside freshness window'
    return Observation({}, issue, stamp_age)

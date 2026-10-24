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
    values = dict(wz=rate.z, summary=f'wz {rate.z:+.3f} rad/s | {message.header.frame_id}')
    if not _quaternion_ok(message.orientation):
        return values, 'invalid orientation quaternion'
    # REP 145: covariance[0] == -1 means the IMU provides no orientation.
    if message.orientation_covariance[0] < 0:
        return values, 'IMU orientation unavailable'
    if not finite(rate.x, rate.y, rate.z):
        return values, 'invalid IMU angular velocity'
    return values, ''


# GPSRAW fix_type: 0-1 none, 2 2D, 3 3D, 4 DGPS, 5 RTK float, 6 RTK fixed. MAVLink marks an
# unknown speed or course with 65535.
GPSRAW_UNKNOWN = 65535
FIX_NAMES = {0: 'none', 1: 'none', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'float', 6: 'fixed'}


def _gpsraw(message):
    """Receiver quality plus the world velocity MAVROS fuses (vel cm/s along cog cdeg)."""
    fix = message.fix_type
    values = dict(fix=fix, sats=message.satellites_visible, h_acc=message.h_acc / 1000,
                  vel_acc=message.vel_acc / 1000, speed=None, cog=None,
                  lat=message.lat / 1e7, lon=message.lon / 1e7, alt=message.alt / 1000)
    if GPSRAW_UNKNOWN not in (message.vel, message.cog):
        values['speed'], values['cog'] = message.vel / 100, message.cog / 100
        heading = math.radians(values['cog'])
        values['ve'], values['vn'] = (values['speed'] * math.sin(heading),
                                      values['speed'] * math.cos(heading))
    values['summary'] = (f"{FIX_NAMES.get(fix, fix)} | {values['sats']} sats | "
                         f"h_acc {values['h_acc']:.2f}m")
    if fix < 3:
        return values, 'GPS raw has no 3D fix'
    return values, ''


def _mag(message):
    field = message.magnetic_field
    values = dict(summary='')
    if not finite(field.x, field.y, field.z):
        return values, 'non-finite magnetic field'
    # The vector stays in tesla for the Status compass row; field_mg converts (1 mG = 1e-7 T).
    values.update(x=field.x, y=field.y, z=field.z)
    values['field_mg'] = math.sqrt(field.x ** 2 + field.y ** 2 + field.z ** 2) * 1e7
    values['summary'] = f"|B| {values['field_mg']:.0f} mG | {message.header.frame_id}"
    return values, '' if values['field_mg'] > 0 else 'magnetic field is zero'


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
    elif kind == 'gpsraw':
        values, issue = _gpsraw(message)
    elif kind == 'mag':
        values, issue = _mag(message)
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

import struct
from types import SimpleNamespace as NS

import pytest

from mhseals_testing.validation import GpsRawInt, validate

NOW = 100.0


def header(frame='odom', stamp=NOW - .05):
    return NS(frame_id=frame, stamp=NS(sec=int(stamp), nanosec=int(round(stamp % 1 * 1e9))))


def quaternion(w=1.0):
    return NS(x=0., y=0., z=0., w=w)


def vector(x=0., y=0., z=0.):
    return NS(x=x, y=y, z=z)


def odometry(vx=.5, child='base_link', w=1.0, **kwargs):
    return NS(header=header(**kwargs), child_frame_id=child,
              twist=NS(twist=NS(linear=vector(vx), angular=vector(z=.1))),
              pose=NS(pose=NS(position=vector(1., 2.), orientation=quaternion(w))))


def gps(status=0, latitude=42.0, covariance_type=2, variance=4.0):
    return NS(header=header('gps'), status=NS(status=status), latitude=latitude, longitude=-71.0,
              position_covariance_type=covariance_type,
              position_covariance=[variance, 0, 0, 0, variance, 0, 0, 0, 9.0])


def imu(w=1.0, orientation_covariance=.01, rate=0.):
    return NS(header=header('imu_link'), orientation=quaternion(w),
              orientation_covariance=[orientation_covariance] + [0] * 8,
              angular_velocity=vector(z=rate))


def test_good_odometry_values():
    result = validate('odometry', odometry(), NOW, base_frame='base_link')
    assert result.issue == ''
    assert result.values['vx'] == .5 and result.values['child'] == 'base_link'
    assert result.stamp_age == pytest.approx(.05)


@pytest.mark.parametrize('message,issue', [
    (odometry(vx=float('nan')), 'non-finite'),
    (odometry(child='odom'), 'velocity frame'),
    (odometry(frame=''), 'empty odometry'),
    (odometry(w=0.), 'orientation'),
    (odometry(stamp=NOW - 5), 'freshness'),
    (odometry(stamp=NOW + 1), 'freshness'),
    (odometry(stamp=0), 'zero message timestamp'),
])
def test_bad_odometry(message, issue):
    assert issue in validate('odometry', message, NOW, base_frame='base_link').issue


def test_gps_reports_sigma_and_rejects_bad_fixes():
    assert 'sigma 2.00m' in validate('gps', gps(), NOW).values['summary']
    assert validate('gps', gps(status=-1), NOW).issue == 'GPS has no fix'
    assert validate('gps', gps(latitude=95), NOW).issue == 'invalid GPS coordinates'
    assert validate('gps', gps(covariance_type=0), NOW).issue == 'GPS covariance unknown'
    assert validate('gps', gps(variance=-1), NOW).issue == 'invalid GPS covariance'


def test_imu_checks_orientation_and_rates():
    assert validate('imu', imu(), NOW).issue == ''
    assert 'quaternion' in validate('imu', imu(w=2.), NOW).issue
    assert 'unavailable' in validate('imu', imu(orientation_covariance=-1), NOW).issue
    assert 'angular velocity' in validate('imu', imu(rate=float('inf')), NOW).issue


def test_points_images_detections_and_unknown_kind():
    cloud = NS(header=header('lidar'), width=0, height=1, data=b'')
    assert validate('points', cloud, NOW).issue == 'empty point cloud'
    assert validate('image', NS(header=header('cam'), data=b'x'), NOW).issue == ''
    detections = NS(header=header('cam'), detections=[1, 2])
    assert validate('detections', detections, NOW).values['count'] == 2
    with pytest.raises(ValueError):
        validate('sonar', cloud, NOW)


def gpsraw(fix=3, vel=150, cog=9000, **kwargs):
    return NS(header=header('gps'), fix_type=fix, satellites_visible=12, h_acc=2100,
              vel_acc=280, vel=vel, cog=cog, lat=430001234, lon=-760000000, alt=100500, **kwargs)


def test_gpsraw_reports_quality_and_world_velocity():
    result = validate('gpsraw', gpsraw(), NOW)
    assert result.issue == ''
    assert result.values['h_acc'] == 2.1 and result.values['vel_acc'] == .28
    assert (result.values['lat'], result.values['lon'], result.values['alt']) == (43.0001234, -76.0, 100.5)
    assert result.values['ve'] == pytest.approx(1.5) and result.values['vn'] == pytest.approx(0, abs=1e-9)


def test_gpsraw_flags_no_fix_and_skips_unknown_velocity():
    assert 'no 3D fix' in validate('gpsraw', gpsraw(fix=2), NOW).issue
    unknown = validate('gpsraw', gpsraw(vel=65535), NOW)
    assert unknown.values['speed'] is None and 've' not in unknown.values


def mavlink(payload, msgid=24):
    """A mavros_msgs/Mavlink frame: the payload as little-endian uint64 words, ``len`` its bytes."""
    words = [int.from_bytes(payload[i:i + 8].ljust(8, b'\0'), 'little') for i in range(0, len(payload), 8)]
    return NS(header=header(''), msgid=msgid, len=len(payload), payload64=words)


def gps_raw_int(fix, h_acc=0):
    """GPS_RAW_INT at the MAVLink spec's wire offsets; MAVLink 2 truncates trailing zero bytes."""
    payload = bytearray(52)
    struct.pack_into('<Qiii', payload, 0, 123456, 430001234, -760000000, 100500)
    struct.pack_into('<HHHH', payload, 20, 90, 150, 150, 9000)     # eph epv vel cog
    payload[28], payload[29] = fix, 14                             # fix_type, satellites_visible
    struct.pack_into('<I', payload, 34, h_acc)
    return bytes(payload).rstrip(b'\0')


@pytest.mark.parametrize('fix, h_acc, word', [(5, 0, 'FLOAT'), (6, 14, 'FIXED')])
def test_gps_raw_int_decodes_from_raw_mavlink_as_gpsraw(fix, h_acc, word):
    from mhseals_testing.ui.feeds.rtk import FIX_WORDS
    raw = GpsRawInt(mavlink(gps_raw_int(fix, h_acc)))
    assert (raw.fix_type, raw.satellites_visible, raw.eph, raw.h_acc) == (fix, 14, 90, h_acc)
    result = validate('gpsraw', raw, NOW)
    assert result.issue == '' and FIX_WORDS[result.values['fix']] == word
    assert (result.values['lat'], result.values['lon'], result.values['speed']) == (43.0001234, -76.0, 1.5)


def test_gps_raw_int_matches_pymavlink():
    mavutil = pytest.importorskip('pymavlink.dialects.v20.common')
    message = mavutil.MAVLink_gps_raw_int_message(1, 6, 430001234, -760000000, 100500, 90, 150, 150, 9000, 14,
                                                  h_acc=14)
    encoder = mavutil.MAVLink(None)
    frame = message.pack(encoder)
    payload = frame[10:10 + frame[1]]                  # v2 header is 10 bytes, byte 1 the payload length
    raw = GpsRawInt(mavlink(payload))
    assert (raw.fix_type, raw.satellites_visible, raw.h_acc, raw.lat) == (6, 14, 14, 430001234)


def mag(x=2e-5, y=0., z=0.):
    return NS(header=header('imu_link'), magnetic_field=vector(x, y, z))


def test_mag_reports_field_in_milligauss():
    assert validate('mag', mag(), NOW).values['field_mg'] == pytest.approx(200)
    assert validate('mag', mag(x=float('nan')), NOW).issue == 'non-finite magnetic field'
    assert validate('mag', mag(x=0.), NOW).issue == 'magnetic field is zero'


def test_mag_undoes_mavros_raw_imu_scaling():
    """MAVROS publishes RAW_IMU's mG times 1000 as 'tesla'; the boat's live -84/-245/-330 mG."""
    values = validate('mag', mag(x=-84000., y=-245000., z=-330000.), NOW).values
    assert values['field_mg'] == pytest.approx(419.5, abs=0.1)
    assert values['x'] == pytest.approx(-84e-7)

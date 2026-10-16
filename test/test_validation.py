from types import SimpleNamespace as NS

import pytest

from mhseals_testing.validation import validate

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

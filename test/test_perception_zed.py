from types import SimpleNamespace as NS

from mhseals_testing.perception import clock, zed_diag


def status(level, message, values, hw='Stereolabs camera: front'):
    return NS(level=level, message=message, hardware_id=hw,
              values=[NS(key=k, value=v) for k, v in values])


JP6 = [
    ('Data Capture', 'Mean Frequency: 14.9 Hz (99.3%)'),
    ('Data Capture', 'Tot. Processing Time: 0.012345 sec (Max. 0.030 sec)'),
    ('Object detection', 'Mean Frequency: 9.8 Hz (65.3%)'),
    ('Object detection', 'Processing Time: 0.150 sec'),
    ('Temperature', 'Left: 45.0 C'),
]
JP5 = [
    ('Capture', 'Mean Frequency: 7.5 Hz (50.0%)'),
    ('Capture', 'Tot. Processing Time: 0.2 sec (Max. 0.5 sec)'),
    ('Object detection', 'Mean Frequency: 7.4 Hz'),
    ('Object detection', 'Processing Time: 0.18 sec'),
]


def test_jp6_duplicate_keys_are_separate_fields():
    parsed = zed_diag.parse(status(0, 'Camera streaming', JP6))
    assert parsed['stages'] == {'camera': {'hz': 14.9, 'proc_ms': 12.345},
                                'sdk': {'hz': 9.8, 'proc_ms': 150.0}}
    assert parsed['inference_ms'] == 150.0
    assert parsed['status'] == {'level': 'ok', 'message': 'Camera streaming'}


def test_jp5_capture_key_and_inference():
    parsed = zed_diag.parse(status(1, 'Slow', JP5))
    assert parsed['stages']['camera'] == {'hz': 7.5, 'proc_ms': 200.0}
    assert parsed['inference_ms'] == 180.0


def test_bad_values_skipped_and_empty_status():
    parsed = zed_diag.parse(status(9, '', [('Object detection', 'Processing Time: nan sec'),
                                           ('Object detection', 'garbage')]))
    assert parsed['stages'] == {} and parsed['inference_ms'] is None
    assert parsed['status']['level'] == 'stale'
    assert zed_diag.is_zed(status(0, '', [])) and not zed_diag.is_zed(status(0, '', [], hw='x'))


def test_clock_offset():
    assert clock.offset(100.25, 100.0) == 0.25
    assert clock.offset(0.0, 100.0) is None and clock.offset(float('inf'), 1.0) is None


def test_level_arrives_as_rclpy_byte():
    """diagnostic_msgs/DiagnosticStatus.level is a ROS ``byte``: live rclpy gives b'\\x00', not 0."""
    assert zed_diag.parse(status(b'\x00', 'Camera grabbing', JP6))['status']['level'] == 'ok'
    assert zed_diag.parse(status(b'\x02', 'Camera error', []))['status']['level'] == 'fail'

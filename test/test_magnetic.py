import math
from types import SimpleNamespace

import pytest

from mhseals_testing.analysis import magnetic
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.ui.panels.status import localization_rows

# The sensors the monitor tests observe by kind, declared with the boat's topics.
SPECS = (SimpleNamespace(name='gpsraw', topic='/mavros/gpsstatus/gps1/raw', kind='gpsraw'),
         SimpleNamespace(name='mag', topic='/mavros/imu/mag', kind='mag'))

# WMM at the site: 470 mG total, 235 horizontal, 405 vertical.
WMM = (235e-7, 0.0, 405e-7)


def test_wmm_ratios():
    assert magnetic.field_mg(*WMM) == pytest.approx(468, abs=3)
    assert magnetic.field_ratio(*WMM) == pytest.approx(1.0, abs=0.01)
    assert magnetic.field_ratio(1330e-7 + 235e-7, 0, 405e-7) > 3  # the bad stored offsets


def test_heading_level_and_tilted():
    assert magnetic.heading((235e-7, 0, 405e-7), 0, 0) == pytest.approx(0)
    assert magnetic.heading((0, -235e-7, 405e-7), 0, 0) == pytest.approx(90)
    # Pitched up 20 degrees facing north: x and z mix, heading stays north.
    p = math.radians(20)
    x, z = 235e-7 * math.cos(p) - 405e-7 * math.sin(p), 235e-7 * math.sin(p) + 405e-7 * math.cos(p)
    assert magnetic.heading((x, 0, z), 0, p) == pytest.approx(0, abs=1e-6)


def test_compass_gyro_zero_when_consistent_and_large_when_not():
    rate = math.radians(3)
    gyro = [(t, rate) for t in range(0, 31)]
    good = [(t, 3.0 * t % 360) for t in range(0, 31)]
    assert magnetic.compass_gyro(good, gyro) == pytest.approx(0, abs=1e-6)
    stuck = [(t, 0.0) for t in range(0, 31)]  # compass ignores the turn: 30 deg over 10 s
    assert magnetic.compass_gyro(stuck, gyro) == pytest.approx(30)


@pytest.mark.parametrize('args,state', [
    ((4.9, 9.9, 0.49, 1.0), 'ok'),
    ((5.1, 0, 0, 1.0), 'unreliable'),
    ((0, 10.1, 0, 1.0), 'unreliable'),
    ((0, 0, 0.5, 1.0), 'unreliable'),
    ((0, 0, 0, 0.69), 'unreliable'),
    ((0, 0, 0, 1.31), 'unreliable'),
    ((None, None, None, None), 'ok'),
])
def test_yaw_unreliable_rule(args, state):
    assert magnetic.yaw_quality(*args)[0] == state


def test_yaw_reason_names_the_cause():
    assert magnetic.yaw_quality(7, None, None, 1.0) == ('unreliable', 'comp–gyro 7°')


def test_localization_rows():
    m = BoatMonitor(clock=lambda: 10)
    assert localization_rows(m.snapshot(), SPECS) == []
    m.observe('gpsraw', dict(fix=3, sats=21, h_acc=2.1), stamp_age=0)
    m.observe('mag', dict(x=1669e-7, y=0, z=0, compass_gyro=7), stamp_age=0)
    rows = {r['name']: r for r in localization_rows(m.snapshot(), SPECS)}
    assert rows['GPS']['state'] == 'OK' and '3D' in rows['GPS']['value'] and '21' in rows['GPS']['value']
    assert rows['Compass']['state'] == 'ERROR' and rows['Compass']['value'] == '1669/470 mG'
    assert rows['Yaw']['state'] == 'WARN' and 'comp–gyro 7' in rows['Yaw']['value']


def test_stale_mag_rows_are_stale():
    now = [10]
    m = BoatMonitor(clock=lambda: now[0])
    m.observe('mag', dict(x=235e-7, y=0, z=405e-7), stamp_age=0)
    now[0] = 100
    rows = {r['name']: r for r in localization_rows(m.snapshot(), SPECS)}
    assert rows['Compass']['state'] == 'STALE' and rows['Yaw']['state'] == 'STALE'


def test_localization_rows_ascii(monkeypatch):
    from mhseals_testing.ui import theme
    monkeypatch.setattr(theme, 'ASCII', True)
    m = BoatMonitor(clock=lambda: 10)
    m.observe('mag', dict(x=1669e-7, y=0, z=0, compass_gyro=70, yaw_sigma=1.0), stamp_age=0)
    for row in localization_rows(m.snapshot(), SPECS):
        assert str(row['value']).isascii(), row
    assert 'sigma' in {r['name']: r for r in localization_rows(m.snapshot(), SPECS)}['Yaw']['value']


def test_rows_found_by_topic_with_the_boat_profile():
    from mhseals_testing.profiles import load_profile
    specs = load_profile('boat').sensors
    m = BoatMonitor(clock=lambda: 10)
    m.observe('GPS raw', dict(fix=3, sats=21, h_acc=2.1), stamp_age=0)
    m.observe('compass', dict(x=235e-7, y=0, z=405e-7), stamp_age=0)
    names = [r['name'] for r in localization_rows(m.snapshot(), specs)]
    assert names == ['GPS', 'Compass', 'Yaw']


def test_rows_found_by_kind_on_other_topics():
    specs = (SimpleNamespace(name='g', topic='/boat1/gps/raw', kind='gpsraw'),
             SimpleNamespace(name='c', topic='/boat1/mag', kind='mag'))
    m = BoatMonitor(clock=lambda: 10)
    m.observe('g', dict(fix=3, sats=9), stamp_age=0)
    m.observe('c', dict(x=235e-7, y=0, z=405e-7), stamp_age=0)
    assert [r['name'] for r in localization_rows(m.snapshot(), specs)] == ['GPS', 'Compass', 'Yaw']


def test_profile_without_the_sensors_has_no_rows():
    m = BoatMonitor(clock=lambda: 10)
    m.observe('GPS raw', dict(fix=3, sats=21, h_acc=2.1), stamp_age=0)
    assert localization_rows(m.snapshot(), ()) == []


def test_validated_mag_message_renders_the_compass_rows():
    """The producer's values, not hand-made ones, carry the vector the Status row reads (FC-5)."""
    from mhseals_testing.validation import validate
    vec = SimpleNamespace(x=WMM[0], y=WMM[1], z=WMM[2])
    stamp = SimpleNamespace(sec=10, nanosec=0)
    header = SimpleNamespace(frame_id='imu', stamp=stamp)
    message = SimpleNamespace(header=header, magnetic_field=vec)
    observation = validate('mag', message, 10.0)
    assert (observation.values['x'], observation.values['z']) == WMM[::2]
    m = BoatMonitor(clock=lambda: 10)
    m.observe('mag', observation.values, stamp_age=0)
    rows = {r['name']: r for r in localization_rows(m.snapshot(), SPECS)}
    assert rows['Compass']['state'] == 'OK' and rows['Compass']['value'] == '468/470 mG'

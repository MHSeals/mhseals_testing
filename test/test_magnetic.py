import math

import pytest

from mhseals_testing.analysis import magnetic
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.ui.dashboard import localization_rows

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
    assert localization_rows(m.snapshot()) == []
    m.observe('gpsraw', dict(fix=3, sats=21, h_acc=2.1), stamp_age=0)
    m.observe('mag', dict(x=1669e-7, y=0, z=0, compass_gyro=7), stamp_age=0)
    rows = {r['name']: r for r in localization_rows(m.snapshot())}
    assert rows['GPS']['state'] == 'OK' and '3D' in rows['GPS']['value'] and '21' in rows['GPS']['value']
    assert rows['Compass']['state'] == 'ERROR' and rows['Compass']['value'] == '1669/470 mG'
    assert rows['Yaw']['state'] == 'WARN' and 'comp–gyro 7' in rows['Yaw']['value']

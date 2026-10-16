import pytest

from mhseals_testing import health
from mhseals_testing.monitor import BoatMonitor


@pytest.mark.parametrize('percentage,level', [(.8, 'OK'), (.2, 'WARN'), (.1, 'FAIL'),
                                              (float('nan'), 'OK')])
def test_battery_levels(percentage, level):
    assert health.battery(15.9, percentage)[0] == level


def test_flight_controller_warns_when_it_could_drive():
    assert health.flight_controller(False, False, '')[0] == 'WARN'
    assert health.flight_controller(True, False, 'MANUAL') == ('OK', 'disarmed, MANUAL')
    assert health.flight_controller(True, True, 'MANUAL')[0] == 'WARN'
    assert health.flight_controller(True, False, 'AUTO')[0] == 'WARN'


def test_clock_reports_worst_live_offset():
    monitor = BoatMonitor(clock=lambda: 0)
    assert health.clock(monitor.snapshot())[0] == 'N/A'
    monitor.observe('IMU', stamp_age=.05)
    monitor.observe('GPS', stamp_age=-.8)
    assert health.clock(monitor.snapshot()) == ('WARN', 'worst GPS -0.80s')


@pytest.mark.parametrize('load,temps,disk,level', [
    (1, [50], 50e9, 'OK'),
    (8, [50], 50e9, 'WARN'),
    (1, [80], 50e9, 'WARN'),
    (1, [90], 50e9, 'FAIL'),
    (1, [], 3e9, 'WARN'),
    (8, [], .5e9, 'FAIL'),
])
def test_compute_levels(load, temps, disk, level):
    assert health.compute(load, 4, temps, disk)[0] == level


def test_read_compute_uses_existing_parent(tmp_path):
    level, detail = health.read_compute(tmp_path / 'bags' / 'not_yet')
    assert level in ('OK', 'WARN', 'FAIL') and 'GB free' in detail


def test_pwm_readback(tmp_path):
    pin = tmp_path / 'pwmchip0' / 'pwm1'
    pin.mkdir(parents=True)
    (pin / 'enable').write_text('1\n')
    (pin / 'period').write_text('20000000')
    (pin / 'duty_cycle').write_text('1500000')
    level, rows = health.pwm_readback([tmp_path / 'pwmchip0', tmp_path / 'missing'], [1, 0])
    assert rows == ['ON 1500us / 50Hz', 'unavailable'] and level == 'N/A'

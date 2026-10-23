import math

import pytest

from mhseals_testing.health import legacy as health
from mhseals_testing.monitor import BoatMonitor


@pytest.mark.parametrize('percentage,level', [(.8, 'OK'), (.2, 'WARN'), (.1, 'FAIL'),
                                              (float('nan'), 'OK')])
def test_battery_levels(percentage, level):
    assert health.battery(15.9, percentage)[0] == level


def test_battery_treats_fcu_unknowns_as_absent():
    assert health.battery(15.2, -0.0099999998) == ('OK', '15.2 V; no charge estimate')
    assert health.battery(65.535, -0.0099999998) == ('N/A', 'no battery data')
    assert health.battery(65.535, .8) == ('OK', 'no voltage; 80%')


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
    monitor.observe('GPS', stamp_age=432123.4)
    assert health.clock(monitor.snapshot()) == ('FAIL', 'worst GPS +432123s')


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
    level, detail, rows = health.pwm_readback(
        [tmp_path / 'pwmchip0', tmp_path / 'missing'], [1, 0])
    assert rows == ['ON 1500us / 50Hz', 'unavailable'] and level == 'WARN'
    assert detail.startswith('1/2 ON?;') and 'disarmed' in detail


@pytest.mark.parametrize('armed,managed,nodes,level,label', [
    (False, False, [], 'OK', 'STOPPED'),
    (True, False, [], 'FAIL', 'NO NODE'),
    (True, True, [], 'FAIL', 'NO NODE'),
    (False, True, [], 'WARN', 'NO NODE'),
    (True, True, ['/thruster_pwm_node'], 'OK', 'RUNNING'),
    (True, True, ['/boat/thruster_pwm_node'], 'OK', 'RUNNING'),
    (False, False, ['/thruster_pwm_node'], 'WARN', 'UNMANAGED'),
    (True, False, ['/thruster_pwm_node'], 'FAIL', 'UNMANAGED'),
    (True, True, ['/thruster_pwm_node'] * 2, 'FAIL', 'DUPLICATE'),
    (False, False, ['/not_thruster_pwm_node'], 'OK', 'STOPPED'),
])
def test_pwm_driver_checks_process_and_ros_endpoint(armed, managed, nodes, level, label):
    actual, detail = health.pwm_driver(nodes, nodes, '/cmd_vel_effort', armed, managed)
    assert actual == level and detail.startswith(label + ';')


def test_driver_on_another_command_topic_is_not_reported_as_stopped():
    nodes = ['/boat/thruster_pwm_node']
    level, detail = health.pwm_driver(nodes, [], '/cmd_vel', False, False)
    assert level == 'WARN' and detail.startswith('NO INPUT;')
    assert health.pwm_driver(nodes, [], '/cmd_vel', True, True)[0] == 'FAIL'
    assert health.pwm_driver(nodes * 2, nodes, '/cmd_vel', True, True)[0] == 'FAIL'


@pytest.fixture
def pwm_chips(tmp_path):
    chips = [tmp_path / f'pwmchip{index}' for index in range(4)]
    for chip in chips:
        pin = chip / 'pwm0'
        pin.mkdir(parents=True)
        (pin / 'enable').write_text('1')
        (pin / 'period').write_text('20000000')
        (pin / 'duty_cycle').write_text('1500000')
    return chips


def test_pwm_pins_report_neutral_pulses_and_actual_duty(pwm_chips):
    level, detail, rows = health.pwm_readback(pwm_chips, [0] * 4, armed=True)
    assert level == 'OK' and detail.startswith('4/4 ON; 0 non-neutral')
    assert rows == ['ON 1500us / 50Hz'] * 4
    (pwm_chips[2] / 'pwm0' / 'duty_cycle').write_text('1600000')
    level, detail, rows = health.pwm_readback(pwm_chips, [0] * 4, armed=True)
    assert level == 'OK' and '1 non-neutral' in detail
    assert rows[2] == 'ON 1600us / 50Hz'


def test_disabled_pins_are_normal_only_when_disarmed(pwm_chips):
    for chip in pwm_chips:
        (chip / 'pwm0' / 'enable').write_text('0')
    assert health.pwm_readback(pwm_chips, [0] * 4)[0] == 'OK'
    level, detail, _ = health.pwm_readback(pwm_chips, [0] * 4, armed=True)
    assert level == 'FAIL' and detail.startswith('0/4 ON;')
    (pwm_chips[0] / 'pwm0' / 'enable').write_text('1')
    assert health.pwm_readback(pwm_chips, [0] * 4)[0] == 'WARN'
    assert health.pwm_readback(pwm_chips, [0] * 4, armed=True)[0] == 'FAIL'


@pytest.mark.parametrize('field,value,reason', [
    ('period', '10000000', 'expected 50Hz'),
    ('period', '0', 'invalid period/duty'),
    ('duty_cycle', '21000000', 'invalid period/duty'),
    ('duty_cycle', '1000000', 'pulse outside'),
    ('duty_cycle', '-1', 'invalid period/duty'),
    ('enable', 'bad', 'unavailable'),
    ('period', 'bad', 'unavailable'),
])
def test_armed_pwm_faults_are_visible(pwm_chips, field, value, reason):
    (pwm_chips[0] / 'pwm0' / field).write_text(value)
    level, detail, _ = health.pwm_readback(pwm_chips, [0] * 4, armed=True)
    assert level == 'FAIL' and reason in detail


def test_missing_and_unexported_pwm_do_not_claim_to_be_active(tmp_path):
    chip = tmp_path / 'pwmchip9'
    assert health.pwm_readback([chip], [0])[0] == 'N/A'
    chip.mkdir()
    level, detail, rows = health.pwm_readback([chip], [0])
    assert level == 'OK' and rows == ['OFF (unexported)']
    assert health.pwm_readback([chip], [0], armed=True)[0] == 'FAIL'
    assert not (chip / 'pwm0').exists()  # status checks never export/enable a pin


def test_battery_ignores_percentage_without_current_and_flags_implausible_voltage():
    """Real boat_20261019_002558 samples: 6 V, 100 %, current 0 while MAVROS warned."""
    assert health.battery(6.0, 1.0, -0.0) == ('WARN', '6.0 V; implausible for 4S (1.5 V/cell)')
    assert health.battery(15.9, 1.0, 0.0) == ('OK', '15.9 V; no charge estimate')
    assert health.battery(15.9, 0.8, 5.0) == ('OK', '15.9 V; 80%')


def test_battery_judges_the_configured_cell_count():
    assert health.battery(8.2, math.nan, 0.0, cells=2) == ('OK', '8.2 V; no charge estimate')
    assert health.battery(12.4, math.nan, 0.0, cells=3)[0] == 'OK'
    assert health.battery(12.4, math.nan, 0.0, cells=4) == ('WARN', '12.4 V; low for 4S')
    assert health.battery(0.0, math.nan, 0.0) == ('WARN', '0.0 V; power brick not connected?')


def test_battery_row_shows_the_voltage_and_profile_sets_cells():
    from mhseals_testing.profiles import load_profile, parse_profile
    from mhseals_testing.ui.dashboard import short
    assert short(health.battery(6.0, 1.0, 0.0)[1]) == '6.0 V'
    assert short(health.battery(15.9, 0.8, 5.0)[1]) == '15.9 V'
    data = {'name': 'x', 'command': {'topic': '/c', 'type': 'twist', 'units': 'effort',
                                     'limit': [1, 1, 1]},
            'odometry': '/o', 'frames': {'odom': 'odom', 'base': 'base'},
            'arming': {'outputs': 'none'}, 'sensors': [], 'battery': {'cells': 3}}
    assert parse_profile(data).battery_cells == 3
    with pytest.raises(ValueError, match='battery.cells'):
        parse_profile({**data, 'battery': {'cells': 0}})

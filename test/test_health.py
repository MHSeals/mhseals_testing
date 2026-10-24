import pytest

from mhseals_testing.health import legacy as health


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
    (chip / 'npwm').write_text('1')
    level, detail, rows = health.pwm_readback([chip], [0])
    assert level == 'OK' and rows == ['OFF (unexported)']
    assert health.pwm_readback([chip], [0], armed=True)[0] == 'FAIL'
    assert not (chip / 'pwm0').exists()  # status checks never export/enable a pin


def test_profile_sets_the_battery_cell_count():
    from mhseals_testing.profiles import parse_profile
    data = {'name': 'x', 'command': {'topic': '/c', 'type': 'twist', 'units': 'effort',
                                     'limit': [1, 1, 1]},
            'odometry': '/o', 'frames': {'odom': 'odom', 'base': 'base'},
            'arming': {'outputs': 'none'}, 'sensors': [], 'battery': {'cells': 3}}
    assert parse_profile(data).battery_cells == 3
    with pytest.raises(ValueError, match='battery.cells'):
        parse_profile({**data, 'battery': {'cells': 0}})


def test_profile_carries_battery_chemistry_and_capacity():
    from mhseals_testing.profiles import parse_profile
    data = {'name': 'x', 'command': {'topic': '/c', 'type': 'twist', 'units': 'effort',
                                     'limit': [1, 1, 1]},
            'odometry': '/o', 'frames': {'odom': 'odom', 'base': 'base'},
            'arming': {'outputs': 'none'}, 'sensors': [],
            'battery': {'chemistry': 'lifepo4', 'capacity_mah': 5000}}
    profile = parse_profile(data)
    assert profile.battery == {'chemistry': 'lifepo4', 'capacity_mah': 5000}
    assert parse_profile({**data, 'battery': None}).battery == {}
    with pytest.raises(ValueError, match='battery.chemistry'):
        parse_profile({**data, 'battery': {'chemistry': 'nimh'}})
    with pytest.raises(ValueError, match='battery.capacity_mah'):
        parse_profile({**data, 'battery': {'capacity_mah': -1}})

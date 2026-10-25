"""Preflight group: each shipped check is OK, fails, and reads unknown (never a pass) without
its fact; the probe's pure fact functions; and --require needing every check OK. No ROS."""

from dataclasses import replace
import json
import math
from types import SimpleNamespace

import pytest

from mhseals_testing.harness import cli
from mhseals_testing.health import probes, registry
from mhseals_testing.health.config import load
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.model import Fact
from mhseals_testing.health.probes import preflight
from mhseals_testing.health.probes.graph import Endpoint, topic_fact_name
from mhseals_testing.profiles import load_profile

PINS = {'pwm_topic': '/pwm', 'pwm_chips': ['a', 'a', 'b', 'b'], 'pwm_channels': [0, 1, 0, 1],
        'mosfet_chip': 'gpiochip0', 'mosfet_line': 13, 'mosfet_active_high': True}
RTCM = topic_fact_name('/mavros/gps_rtk/send_rtcm')

# id -> (facts that pass, facts that fail); an empty fact set must read unknown
CASES = {
    'pre.driver_config': ({'driver.outputs': Fact('match')}, {'driver.outputs': Fact('differs')}),
    'pre.pwm_readback': ({'pwm.readback': Fact('OK', data={'detail': '0/4 ON'})},
                         {'pwm.readback': Fact('FAIL', data={'detail': '2/4 ON'})}),
    'pre.mosfet': ({'driver.mosfet': Fact('on')}, {'driver.mosfet': Fact('off')}),
    'pre.estop': ({'driver.estop': Fact('armed')}, {'driver.estop': Fact('latched')}),
    'pre.calib_dead': ({'calibration.dead': Fact(0)}, {'calibration.dead': Fact(1)}),
    'pre.rc': ({'rc.link': Fact('valid')}, {'rc.link': Fact('invalid')}),
    'pre.mavros_extras': ({RTCM: Fact({'pub': (), 'sub': (Endpoint('/mavros/gps_rtk'),)})},
                          {RTCM: Fact({'pub': (), 'sub': ()})}),
    'pre.gps_fix': ({'gps.fix_type': Fact(6)}, {'gps.fix_type': Fact(1)}),
    'pre.orin_clock': ({'perc.zed_native.clock_offset': Fact(-0.01)},
                                  {'perc.zed_native.clock_offset': Fact(-0.06)}),
    'pre.orin_temp': ({'orin.temp_max': Fact(55.0)}, {'orin.temp_max': Fact(90.0)}),
    'pre.orin_power': ({'orin.power_mode': Fact('MAXN_SUPER')}, {'orin.power_mode': Fact('15W')}),
    'pre.orin_disk': ({'orin.disk_free_gb': Fact(80.0)}, {'orin.disk_free_gb': Fact(2.0)}),
    'pre.watchdog': ({'graph.nodes': Fact(['/costmap_watchdog'])}, {'graph.nodes': Fact(['/x'])}),
    'pre.costmap.zed_native': ({'perc.zed_native.costmap_age': Fact(0.2)},
                               {'perc.zed_native.costmap_age': Fact(3.0)}),
    'pre.tracks.zed_native': ({'perc.zed_native.tracker_hz': Fact(10.0)},
                              {'perc.zed_native.tracker_hz': Fact(0.0)}),
    'pre.recording': ({'bag.active': Fact(True)}, {'bag.active': Fact(False)}),
    'pre.disk': ({'host.disk_free_gb': Fact(100.0)}, {'host.disk_free_gb': Fact(4.0)}),
    'pre.battery': ({'battery.cell_v': Fact(3.9)}, {'battery.cell_v': Fact(3.4)}),
}


@pytest.fixture(scope='module')
def boat():
    registry.discover()
    config = load(user=False, profile=load_profile('boat'))
    return {s.id: s for s in config.specs if s.group == 'Preflight'}, config.probes


def judge(boat, check, facts):
    specs, settings = boat
    spec = replace(specs[check], raise_after=0.0, clear_after=0.0)
    periods = {n: s['period'] for n, s in settings.items() if isinstance(s, dict) and 'period' in s}
    engine = Engine([spec], periods, clock=lambda: 100.0)
    fresh = {name: (100.0, None) for name in periods}
    return engine.tick(facts, fresh).results[check].status


def test_fcu_link_is_the_existing_check_joined_to_the_group_on_the_boat_only(boat):
    registry.discover()
    assert 'pre.fcu' not in boat[0]
    assert load(user=False, profile=load_profile('boat')).joins == {'preflight': ['fcu.link']}
    assert load(user=False, profile=load_profile('crane_full_stack')).joins == {}
    found = {'checks': [{'id': 'fcu.link', 'group': 'FCU', 'status': 'unknown'},
                        {'id': 'pre.a', 'group': 'Preflight', 'status': 'ok'}]}
    assert cli.required(found, ['preflight'], {'preflight': ['fcu.link']})['preflight']['not_ok'] == ['fcu.link']
    assert cli.required(found, ['preflight'])['preflight']['not_ok'] == []


def test_every_boat_preflight_check_is_covered(boat):
    assert set(boat[0]) == set(CASES)


@pytest.mark.parametrize('check', sorted(CASES))
def test_check_ok_fail_and_unknown_never_passes(boat, check):
    good, bad = CASES[check]
    assert judge(boat, check, good) == 'ok'
    assert judge(boat, check, bad) in ('fail', 'warn') and judge(boat, check, bad) != 'ok'
    assert judge(boat, check, {}) == 'unknown'


def test_gps_fix_float_warns_for_precise_tasks(boat):
    assert judge(boat, 'pre.gps_fix', {'gps.fix_type': Fact(5)}) == 'ok'
    assert judge(boat, 'pre.gps_fix', {'gps.fix_type': Fact(3)}) == 'warn'


def test_orin_clock_fails_above_50_ms(boat):
    check = 'pre.orin_clock'
    assert judge(boat, check, {'perc.zed_native.clock_offset': Fact(0.049)}) == 'warn'
    assert judge(boat, check, {'perc.zed_native.clock_offset': Fact(0.051)}) == 'fail'


def test_implausible_fcu_supply_reading_leaves_the_battery_unknown(boat):
    facts = probes.battery_facts(6.5, math.nan, cells=4)    # the FCU supply, not a 4S pack
    assert facts['battery.plausible'].value is False and 'battery.cell_v' not in facts
    assert judge(boat, 'pre.battery', facts) == 'unknown'


def test_crane_profile_keeps_only_the_sim_checks():
    registry.discover()
    ids = {s.id for s in load(user=False, profile=load_profile('crane_full_stack')).specs
           if s.group == 'Preflight'}
    assert ids == {'pre.driver_config', 'pre.calib_dead', 'pre.watchdog', 'pre.costmap.sim',
                   'pre.tracks.sim', 'pre.recording', 'pre.disk'}


def status(**extra):
    return {'session': 's', 'mosfet': 'on', 'outputs': dict(PINS, output_backend='odroid'),
            'error': '', 'pending_restart_keys': [], **extra}


def test_driver_facts_compare_the_pins_and_read_the_mosfet_and_estop():
    facts = preflight.driver_facts(status(), PINS)
    assert facts['driver.outputs'].value == 'match' and facts['driver.mosfet'].value == 'on'
    assert 'driver.estop' not in facts                       # main's driver has no estop block
    moved = preflight.driver_facts(status(), dict(PINS, mosfet_line=12))
    assert moved['driver.outputs'].value == 'differs' and 'mosfet_line' in moved['driver.outputs'].data['reason']
    assert preflight.driver_facts(status(error='bad yaml'), PINS)['driver.outputs'].value == 'error'
    assert preflight.driver_facts(status(pending_restart_keys=['pwm_chips']), PINS)['driver.outputs'].value == 'pending'
    cut = preflight.driver_facts(status(mosfet='off', estop={'enabled': True, 'latched': True,
                                                             'cause': 'rc-stop'}), PINS)
    assert cut['driver.estop'].value == 'latched' and cut['driver.mosfet'].value == 'off'
    assert preflight.driver_facts(status(estop={'enabled': True, 'latched': False}), PINS)[
        'driver.estop'].value == 'armed'
    assert preflight.driver_facts(None, PINS) == {}


MUX = {'mode_ch': 8, 'modes': [0, 0, 0, 4, 0, 15], 'stop_ch': 6, 'rc_scale': [0.5, 0.5, 0.3],
       'rc_max_age': 0.3, 'cmd_timeout': 0.5, 'stick_takeover': 200, 'stick_time': 0.2,
       'stick_release': 0.5, 'ramp': 1.0}


def test_rc_link_uses_the_mux_rule():
    link = preflight.RcLink(MUX)
    assert link.facts(0.0) == {}                             # no frame: unknown
    link.frame([1500] * 8, 1.0)
    assert link.facts(1.1)['rc.link'].value == 'valid'
    assert link.facts(2.0)['rc.link'].value == 'invalid'     # older than rc_max_age
    link.frame([1500, 1500, 0, 65535, 1500, 1500, 1500, 1500], 3.0)
    assert link.facts(3.0)['rc.link'].value == 'invalid'
    link.frame([], 4.0)
    assert 'no receiver' in link.facts(4.0)['rc.link'].data['reason']


def test_bag_and_orin_facts(tmp_path):
    assert preflight.bag_facts({'bag': {'pid': 7}}, lambda pid: True)['bag.active'].value is True
    assert preflight.bag_facts({}, lambda pid: True)['bag.active'].value is False
    (tmp_path / 'zone0').write_text('48500\n')
    (tmp_path / 'status').write_text('pmode:0002 fmode:quiet\n')
    (tmp_path / 'conf').write_text('< POWER_MODEL ID=0 NAME=15W >\n< POWER_MODEL ID=2 NAME=MAXN_SUPER >\n')
    values = preflight.host_values(str(tmp_path / 'zone*'), tmp_path / 'status', tmp_path / 'conf', tmp_path)
    facts = preflight.orin_facts(values, 'jetson')
    assert facts['orin.temp_max'].value == 48.5 and facts['orin.power_mode'].value == 'MAXN_SUPER'
    assert facts['orin.disk_free_gb'].value > 0
    assert preflight.orin_facts(preflight.host_values(str(tmp_path / 'none*'), tmp_path / 'x', tmp_path / 'y',
                                                      tmp_path / 'missing'), 'jetson') == {}


def test_preflight_probe_reads_the_driver_only_while_its_node_runs():
    pytest.importorskip('rclpy')
    from mhseals_testing.ros.preflight import Preflight
    profile = SimpleNamespace(uses_fcu=True, outputs='topic')
    probe = Preflight(profile, dict(PINS, command_mux=MUX), local='blastoise')
    probe.on_status(SimpleNamespace(data=json.dumps(status())))
    assert 'driver.outputs' not in probe.facts(0.0, ['/mavros'])          # latched, driver gone
    assert probe.facts(0.0, ['/thruster_pwm_node'])['driver.outputs'].value == 'match'
    kv = [SimpleNamespace(key='temp_max_c', value='60.0')]
    probe.on_diagnostics(SimpleNamespace(status=[SimpleNamespace(hardware_id='host:jetson', values=kv)]))
    assert probe.facts(probe.host[0], [])['orin.temp_max'].value == 60.0
    assert 'orin.temp_max' not in probe.facts(probe.host[0] + 10, [])     # silent camera host


def report(*rows):
    checks = [{'id': i, 'group': g, 'status': s, 'value': '', 'detail': ''} for i, g, s in rows]
    summary = {st: sum(c['status'] == st for c in checks) for st in ('ok', 'info', 'unknown', 'warn', 'fail')}
    return {'summary': summary, 'gates': {}, 'checks': checks}


@pytest.mark.parametrize('worst, code', [('ok', 0), ('warn', 1), ('unknown', 3), ('fail', 2)])
def test_require_needs_every_check_ok(monkeypatch, capsys, worst, code):
    found = report(('pre.a', 'Preflight', 'ok'), ('pre.b', 'Preflight', worst), ('x', 'Other', 'fail'))
    monkeypatch.setattr(cli, 'run_check', lambda *a, **k: (found, 0))
    monkeypatch.setattr(cli, 'ros_collector', lambda *a: None)
    assert cli.main(['check', '--require', 'preflight', '--json', '-']) == code
    needed = json.loads(capsys.readouterr().out)['required']['preflight']
    assert needed['not_ok'] == ([] if worst == 'ok' else ['pre.b'])

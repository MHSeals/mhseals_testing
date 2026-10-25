"""Host, FCU, battery and snapshot probes: facts from plain arguments, no ROS."""

import math

import pytest

from mhseals_testing.health import probes
from mhseals_testing.monitor import BoatMonitor


def test_battery_ignores_percentage_without_current_and_flags_implausible_voltage():
    """Real boat_20261019_002558 samples: 6 V, 100 %, current 0 while MAVROS warned."""
    assert 'battery.percent' not in probes.battery_facts(15.9, 1.0, 0.0)
    assert probes.battery_facts(15.9, 0.8, 5.0)['battery.percent'].value == pytest.approx(80)
    facts = probes.battery_facts(6.0, 1.0, -0.0)
    assert 'battery.cell_v' not in facts and 'battery.voltage' not in facts
    assert facts['battery.plausible'].label == 'implausible for 4S (1.5 V/cell)'
    assert probes.battery_facts(0.0, math.nan, 0.0)['battery.plausible'].label == 'power brick not connected?'


def test_battery_publishes_pack_voltage_with_the_cell_count():
    facts = probes.battery_facts(8.2, math.nan, 0.0, cells=2)
    assert facts['battery.cell_v'].value == pytest.approx(4.1)
    assert (facts['battery.voltage'].value, facts['battery.voltage'].label) == (8.2, '2S')
    assert probes.battery_facts(65.535, .8, 5.0).keys() == {'battery.percent'}


def test_fcu_facts_say_armed_once():
    facts = probes.fcu_facts(True, True, 'MANUAL')
    assert facts['fcu.armed'].label == '' and facts['fcu.mode'].label == 'ARMED, MANUAL'
    assert probes.fcu_facts(False, False, '').keys() == {'fcu.connected'}


def test_clock_fact_is_the_worst_live_offset():
    monitor = BoatMonitor(clock=lambda: 0)
    assert probes.snapshot_facts(monitor.snapshot()) == {}
    monitor.observe('IMU', stamp_age=.05)
    monitor.observe('GPS', stamp_age=-.8)
    fact = probes.snapshot_facts(monitor.snapshot())['clock.stamp_offset']
    assert (fact.value, fact.label) == (-.8, 'GPS')


def test_host_facts_use_an_existing_parent_and_missing_sensors(tmp_path):
    facts = probes.host_facts(tmp_path / 'not' / 'yet', load=lambda: (2.0, 1, 1), cpus=lambda: 4,
                              zones=str(tmp_path / 'none*'))
    assert facts['host.load'].value == 0.5 and facts['host.load'].label == '2.0/4'
    assert facts['host.disk_free_gb'].value > 0 and 'host.temp_max' not in facts


def test_host_facts_read_the_hottest_zone_and_call_load_once(tmp_path):
    for n, milli in enumerate((41000, 77500, 'bad')):
        (tmp_path / f'zone{n}').write_text(str(milli))
    calls = []
    facts = probes.host_facts(tmp_path, load=lambda: calls.append(1) or (1.0, 1, 1), cpus=lambda: 2,
                              zones=str(tmp_path / 'zone*'))
    assert facts['host.temp_max'].value == 77.5 and len(calls) == 1


def test_hottest_is_shared_by_both_host_readers_and_none_without_a_zone(tmp_path):
    from mhseals_testing.health.probes import preflight
    (tmp_path / 'zone0').write_text('41000')
    (tmp_path / 'zone1').write_text('bad')
    zones = str(tmp_path / 'zone*')
    assert probes.hottest(zones) == 41.0 and probes.hottest(str(tmp_path / 'none*')) is None
    assert preflight.host_values(zones, tmp_path / 'x', tmp_path / 'y', tmp_path)['temp_max_c'] == '41.0'


def test_facts_store_replaces_a_probes_facts_and_keeps_them_on_error():
    store = probes.Facts(clock=lambda: 5.0)
    store.publish('fcu', probes.fcu_facts(True, False, 'MANUAL'))
    store.publish('fcu', {}, error='boom')
    facts, sample = store.read()
    assert 'fcu.mode' in facts and sample['fcu'] == (5.0, 'boom')
    store.publish('fcu', probes.fcu_facts(False, False, ''))
    assert 'fcu.mode' not in store.read()[0]

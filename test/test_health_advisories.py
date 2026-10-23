"""The shipped system and power checks folded into dashboard advisory rows.

The cases moved here from test_health.py: battery, flight controller, clock and compute.
"""

import math
import time

import pytest

from mhseals_testing.health import advisories, probes
from mhseals_testing.health.config import HEALTH_DIR, lint, load
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.model import Fact
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.profiles import load_profile
from mhseals_testing.ui.panels.status import short


CALM = {'host.load': Fact(.1, label='0.4/4'), 'host.disk_free_gb': Fact(50.0)}


class Clock:
    now = 0.0

    def __call__(self):
        return self.now


@pytest.fixture
def health(tmp_path):
    """A Health over the shipped checks with a hand-driven clock; returns (health, clock)."""
    clock = Clock()
    return advisories.Health(None, tmp_path, clock=clock, host=lambda: CALM), clock


def settle(health, clock, snapshot=None, seconds=12):
    """Tick once a second so debounced statuses show."""
    snapshot = snapshot or BoatMonitor(clock=clock).snapshot()
    for _ in range(seconds):
        clock.now += 1
        rows = health.tick(snapshot)
    return rows


def battery_rows(health, clock, *args, **kwargs):
    health.facts.publish('battery', probes.battery_facts(*args, **kwargs))
    return settle(health, clock)


def test_shipped_checks_lint_clean(nav_share):
    config = load(HEALTH_DIR, load_profile('boat'), user=False)  # templates need a profile
    assert {'clock.offset', 'compute.load', 'fcu.mode', 'power.charge'} <= {s.id for s in config.specs}
    assert lint(config.specs, unmatched=config.unmatched) == []


@pytest.mark.parametrize('percentage,level', [(.8, 'OK'), (.2, 'WARN'), (.1, 'FAIL')])
def test_battery_levels(health, percentage, level):
    health, clock = health
    assert battery_rows(health, clock, 15.9, percentage, 5.0)['battery'][0] == level


def test_battery_treats_fcu_unknowns_as_absent(health):
    health, clock = health
    assert battery_rows(health, clock, 15.2, -0.0099999998)['battery'] == ('OK', '15.2V 4S; 3.80V')
    # the FCU reports no voltage but a charge estimate: the cell check has nothing to judge
    assert battery_rows(health, clock, 65.535, .8, 5.0)['battery'] == ('N/A', '80%')


def test_a_boat_without_battery_data_has_no_battery_row(health):
    health, clock = health
    assert 'battery' not in battery_rows(health, clock, 65.535, -0.0099999998)



def test_battery_judges_the_configured_cell_count(health):
    health, clock = health
    assert probes.battery_facts(8.2, math.nan, 0.0, cells=2)['battery.cell_v'].value == pytest.approx(4.1)
    assert battery_rows(health, clock, 12.4, math.nan, 0.0, cells=3)['battery'][0] == 'OK'
    assert battery_rows(health, clock, 12.4, math.nan, 0.0, cells=4)['battery'][0] == 'FAIL'
    assert battery_rows(health, clock, 6.0, 1.0, 0.0)['battery'] == (
        'WARN', 'implausible for 4S (1.5 V/cell)')


def test_battery_row_leads_with_pack_voltage_and_cell_count(health):
    health, clock = health
    rows = battery_rows(health, clock, 15.9, .81, 5.0)
    assert rows['battery'] == ('OK', '15.9V 4S; 3.98V, 81%')
    assert short(rows['battery'][1]) == '15.9V 4S'


def test_flight_controller_warns_when_it_could_drive(health):
    health, clock = health

    def level(*state):
        health.facts.publish('fcu', probes.fcu_facts(*state))
        return settle(health, clock)['flight controller']

    assert level(False, False, '')[0] == 'WARN'
    assert level(True, False, 'MANUAL') == ('OK', 'disarmed, MANUAL')
    assert level(True, True, 'MANUAL') == ('WARN', 'ARMED, MANUAL')
    assert level(True, False, 'AUTO')[0] == 'WARN'


def test_clock_reports_worst_live_offset(health):
    health, clock = health
    monitor = BoatMonitor(clock=lambda: 0)
    assert 'clock' not in settle(health, clock, monitor.snapshot())      # no stamped sensors
    monitor.observe('IMU', stamp_age=.05)
    monitor.observe('GPS', stamp_age=-.8)
    row = settle(health, clock, monitor.snapshot())['clock']
    assert row == ('WARN', 'GPS -0.80s') and short(row[1]) == '-0.80s'
    monitor.observe('GPS', stamp_age=432123.4)
    row = settle(health, clock, monitor.snapshot())['clock']
    assert row[0] == 'FAIL' and short(row[1]) == '+432123s'


@pytest.mark.parametrize('load,temps,disk,level', [
    (1, [50], 50, 'OK'), (8, [50], 50, 'WARN'), (1, [80], 50, 'WARN'),
    (1, [90], 50, 'FAIL'), (1, [], 3, 'WARN'), (8, [], .5, 'FAIL'),
])
def test_compute_levels(health, load, temps, disk, level):
    health, clock = health
    facts = {'host.load': Fact(load / 4, label=f'{load:.1f}/4'),
             'host.disk_free_gb': Fact(disk)}
    if temps:
        facts['host.temp_max'] = Fact(max(temps))
    health.facts.publish('host', facts)
    health._ran['host'] = float('inf')           # keep the test's facts: the real probe is not due
    assert settle(health, clock)['compute'][0] == level



def test_silent_probe_reads_unknown_not_its_last_value(health):
    health, clock = health
    health.facts.publish('battery', probes.battery_facts(15.9, .8, 5.0))
    assert settle(health, clock, seconds=2)['battery'][0] == 'OK'
    assert settle(health, clock, seconds=40)['battery'][0] == 'N/A'      # > 3 periods, then 10 s


def test_a_failing_probe_does_not_stop_the_tick(health):
    health, clock = health
    health.host = lambda: 1 / 0
    settle(health, clock, seconds=3)
    assert 'division' in health.facts.read()[1]['host'][1]


def test_tick_cost_stays_small(tmp_path):
    """Probes plus engine on the App tick: the budget is a few milliseconds."""
    health = advisories.Health(None, tmp_path)
    health.facts.publish('fcu', probes.fcu_facts(True, False, 'MANUAL'))
    health.facts.publish('battery', probes.battery_facts(15.9, .8, 5.0))
    snapshot = BoatMonitor().snapshot()
    start = time.perf_counter()
    for _ in range(50):
        health._ran.clear()                       # force the host probe every time: the worst case
        health.tick(snapshot)
    assert (time.perf_counter() - start) / 50 < 0.02


def test_a_broken_user_file_is_reported_not_fatal(tmp_path):
    (tmp_path / 'bad.yaml').write_text('checks: [{id: x, kind: nope}]')
    health = advisories.Health(None, tmp_path, directory=tmp_path)
    assert 'unknown kind' in health.error and health.tick(BoatMonitor().snapshot()) == {}


def test_battery_row_hidden_until_the_first_plausible_sample(health):
    health, clock = health
    assert 'battery' not in battery_rows(health, clock, 65.535, -0.01)   # FCU "unknown" on both
    assert 'battery' not in battery_rows(health, clock, math.nan, math.nan)
    assert battery_rows(health, clock, 15.9, 0.8, 5.0)['battery'][0] == 'OK'
    # once shown, an empty sample keeps the row and reports it unknown
    assert battery_rows(health, clock, 65.535, -0.01)['battery'][0] == 'N/A'

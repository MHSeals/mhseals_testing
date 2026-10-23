"""The fcu probe, the shipped FCU checks over it, and the anchor colours."""

import math

import pytest

from mhseals_testing.health import anchors
from mhseals_testing.health.config import load
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.probes.fcu import FcuProbe, TESLA_TO_MG
from mhseals_testing.health.model import Fact
from mhseals_testing.health.probes import params as probe


@pytest.fixture
def specs(nav_share):
    return [s for s in load(user=False).specs if s.group == 'FCU']


def report(specs, probe, session=None):
    """Results after a bad reading has persisted past the raise time (the engine debounces)."""
    clock = iter([0.0, 5.0])
    engine = Engine(specs, clock=lambda: next(clock))
    engine.tick(probe.facts(1.0), {}, session)
    return engine.tick(probe.facts(1.0), {}, session).results


def test_probe_publishes_state_battery_gps_mag_and_imu():
    p = FcuProbe()
    p.on_state(True, False, 'MANUAL', 1.0)
    p.on_battery(6.4, 1.0, 1.0)
    p.on_gps_raw(6, 14, 0.4, 1.0)
    p.on_mag(470 / TESLA_TO_MG, 0, 0, 1.0)
    p.on_imu(0, 3, 4, 1.0)
    f = p.facts(1.0)
    assert (f['fcu.connected'].value, f['fcu.mode'].value, f['gps.sats'].value) == (True, 'MANUAL', 14)
    assert f['mag.field_mG'].value == pytest.approx(470) and f['imu.accel_norm'].value == 5.0
    assert f['nav.speed_disagreement'].value == 0.0 and math.isfinite(f['battery.voltage'].value)


def test_speed_disagreement_is_rms_while_moving_and_forgets_after_the_window():
    p = FcuProbe()
    p.on_speeds(0.1, 0.0, 0.0)                 # both slow: not counted
    p.on_speeds(3.8, 0.1, 1.0)
    assert p.facts(2.0)['nav.speed_disagreement'].value == pytest.approx(3.7)
    assert p.facts(20.0)['nav.speed_disagreement'].value == 0.0


def test_mavros_battery_diagnostic_warns_with_its_message(specs):
    p = FcuProbe()
    p.on_diagnostic('mavros: Battery', b'\x01', 'Low voltage', 1.0)
    p.on_diagnostic('mavros: GPS', b'\x02', 'No fix', 1.0)
    results = report(specs, p)
    assert results['battery.diag'].status == 'warn'
    assert p.facts(1.0)['battery.diag_level'].label == 'Low voltage'
    assert 'battery.diag' not in report(specs, FcuProbe())     # no diagnostic seen: skipped


def test_compass_and_speed_checks_colour_their_anchors(specs):
    p = FcuProbe()
    p.on_mag(2663 / TESLA_TO_MG, 0, 0, 1.0)    # the 10-19 reading, 5.7x the expected field
    p.on_speeds(3.8, 0.1, 1.0)
    results = report(specs, p)
    assert results['compass.field'].status == 'warn'          # capped at the check's default severity
    assert anchors.anchor_colour(specs, results, 'rose') == 'yellow'
    assert anchors.anchor_colour(specs, results, 'odometry') == 'yellow'
    healthy = FcuProbe()
    healthy.on_mag(470 / TESLA_TO_MG, 0, 0, 1.0)
    assert anchors.anchor_colour(specs, report(specs, healthy), 'rose') == ''


def test_anchor_colours_follow_the_worst_status():
    class Spec:
        def __init__(self, id, show):
            self.id, self.show = id, show

    class Res:
        def __init__(self, status):
            self.status = status

    specs = [Spec('a', ('rose',)), Spec('b', ('rose',)), Spec('c', ('odometry',))]
    colour = lambda **r: anchors.anchor_colour(specs, {k: Res(v) for k, v in r.items()}, 'rose')
    assert colour(a='ok', b='ok') == '' and colour(a='warn', b='ok') == 'yellow'
    assert colour(a='warn', b='fail') == 'red' and colour(a='unknown', b='ok') == 'dim'
    assert colour(c='fail') == ''


def test_lost_fcu_link_fails_and_default_severity_expect_checks_cap_at_warn(specs):
    p = FcuProbe()
    p.on_state(False, False, 'MANUAL', 1.0)
    results = report(specs, p)
    assert results['fcu.link'].status == 'fail'            # kind says fail, severity fail keeps it
    p.on_state(True, True, 'MANUAL', 1.0)
    assert report(specs, p)['fcu.armed'].status == 'warn'   # default severity caps fail at warn


def test_shipped_fcu_params_target_beats_the_as_found_reference(nav_share, tmp_path):
    """blastoise.param holds the 10-22 as-found 8/0; the inline expect keeps 1/1 the target."""
    reference = tmp_path / 'mhseals_nav' / 'config' / 'fcu' / 'blastoise.param'
    spec = next(s for s in load(user=False).specs if s.id == 'fcu.params')
    names = spec.params.names
    reference.write_text(''.join(f'{n},{8 if n == "EK3_SRC1_YAW" else 0 if n == "COMPASS_USE" else 1}\n'
                                 for n in names))

    def status(yaw, use):
        values = {n: 1 for n in names} | {'EK3_SRC1_YAW': yaw, 'COMPASS_USE': use}
        facts = {probe.fact_name(spec.params.node, n): Fact(v) for n, v in values.items()}
        return Engine([spec]).tick(facts, {'params': (0.0, None)}).results['fcu.params'].status

    assert status(8, 0) == 'fail' and status(1, 1) == 'ok'

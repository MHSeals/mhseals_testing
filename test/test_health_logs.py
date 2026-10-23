"""The logs probe and log_pattern kind: pattern fixtures, windows, suppression, tails."""

import importlib
from types import SimpleNamespace

import pytest

from mhseals_testing.health import registry
from mhseals_testing.health.config import load
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.kinds import log_pattern
from mhseals_testing.health.probes.logs import LogProbe

RTPS_LINE = ('2026-10-18 14:02:11 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7413: '
             'open_and_lock_file failed -> Function open_port_internal')


@pytest.fixture(autouse=True)
def kinds():
    """Register the shipped kinds even if an earlier test imported them into a patched registry."""
    from mhseals_testing.health.kinds import threshold
    for module in (threshold, log_pattern):
        importlib.reload(module) if module.__name__.rsplit('.', 1)[1] not in registry.KINDS else None


@pytest.fixture
def specs():
    return {s.id: s for s in load(user=False).specs if s.kind == 'log_pattern'}


class Clock:
    now = 100.0

    def __call__(self):
        return self.now


def run(specs, ids, lines, clock=None):
    """Probe watching ``ids``, fed ``(node, text)`` lines; returns (probe, engine report)."""
    clock = clock or Clock()
    probe = LogProbe(clock)
    probe.watch([specs[i].params for i in ids])
    for node, text in lines:
        probe.feed(node, text)
    engine = Engine([specs[i] for i in ids], clock=clock)
    report = settle(engine, probe, clock)
    return probe, report


def settle(engine, probe, clock):
    """Tick, wait out the 2 s warn debounce, tick again: the shown report."""
    engine.tick(probe.facts(), {'logs': (clock(), None)})
    clock.now += 2.5
    return engine.tick(probe.facts(), {'logs': (clock(), None)})


def test_shipped_logs_yaml_has_the_rtps_and_noise_checks(specs):
    assert {'log.prearm', 'log.rtps', 'log.noise'} <= set(specs)
    assert specs['log.rtps'].params.nodes == []


def test_rtps_error_line_on_a_child_tail_raises_log_rtps(specs, tmp_path):
    (tmp_path / 'bridge.log').write_text('starting\n' + RTPS_LINE + '\n')
    clock = Clock()
    probe = LogProbe(clock)
    probe.watch([specs['log.rtps'].params])
    probe.tail(tmp_path)
    engine = Engine([specs['log.rtps']], clock=clock)
    result = settle(engine, probe, clock).results['log.rtps']
    assert result.status == 'warn'                      # severity caps the kind's fail
    assert 'RTPS_TRANSPORT_SHM Error' in result.detail


def test_prearm_groups_split_keys_and_fail(specs):
    probe, report = run(specs, ['log.prearm'], [
        ('/mavros', 'PreArm: Compass not healthy'), ('/mavros', 'PreArm: Compass not healthy'),
        ('/mavros', 'PreArm: Battery 1 below minimum')])
    result = report.results['log.prearm']
    assert result.status == 'fail' and result.data['why'] == 'Compass not healthy'
    assert result.data['count'] == 2


def test_unrelated_line_stays_ok(specs):
    _, report = run(specs, ['log.prearm', 'log.rtps'], [('/mavros', 'all fine')])
    assert {r.status for r in report.results.values()} == {'ok'}


def test_min_count_needs_repeats(specs):
    line = ('/ekf', 'Failed to meet update rate')
    _, report = run(specs, ['log.ekf_rate'], [line] * 4)
    assert report.results['log.ekf_rate'].status == 'ok'
    _, report = run(specs, ['log.ekf_rate'], [line] * 5)
    assert report.results['log.ekf_rate'].status == 'warn'


def test_match_expires_after_hold(specs):
    clock = Clock()
    probe, _ = run(specs, ['log.gps_vel_stale'], [('/mavros', 'GPS velocity is stale')], clock)
    clock.now += 31
    assert probe.facts()[next(iter(probe._watches))].value == 0


def test_suppressed_noise_is_counted_and_never_reaches_other_patterns(specs):
    noisy = specs['log.noise']
    greedy = log_pattern.Params(pattern='Publisher')
    clock = Clock()
    probe = LogProbe(clock)
    probe.watch([noisy.params, greedy])
    probe.feed('/x', 'Publisher already registered for node name /a')
    facts = probe.facts()
    assert log_pattern.evaluate(noisy.params, facts).detail == '1 suppressed'
    assert log_pattern.evaluate(greedy, facts).status == 'ok'


def test_node_glob_limits_a_pattern(specs):
    params = log_pattern.Params(pattern='PreArm', nodes=['/mavros*'])
    probe = LogProbe(Clock())
    probe.watch([params])
    probe.on_rosout(SimpleNamespace(level=40, name='mavros.sys', msg='PreArm: x'))
    probe.on_rosout(SimpleNamespace(level=20, name='mavros.sys', msg='PreArm: info is ignored'))
    assert list(probe.facts().values())[0].value == 1


def test_tail_reads_only_new_lines_and_waits_for_a_whole_line(specs, tmp_path):
    params = log_pattern.Params(pattern='boom')
    probe = LogProbe(Clock())
    probe.watch([params])
    log = tmp_path / 'child.log'
    log.write_text('boom\nbo')
    probe.tail(tmp_path)
    probe.tail(tmp_path)
    assert list(probe.facts().values())[0].value == 1
    with log.open('a') as f:
        f.write('om\n')
    probe.tail(tmp_path)
    assert list(probe.facts().values())[0].value == 2


def test_check_without_a_watching_probe_is_unknown(specs):
    assert log_pattern.evaluate(specs['log.rtps'].params, {}).status == 'unknown'

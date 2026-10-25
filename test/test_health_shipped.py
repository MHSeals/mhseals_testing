"""The shipped config/health files load, lint clean and drive the engine."""

import importlib
from dataclasses import replace

import pytest

from mhseals_testing.health import registry
from mhseals_testing.health.config import HEALTH_DIR, lint, load
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.model import Fact
from mhseals_testing.profiles import load_profile


@pytest.fixture(autouse=True)
def kinds():
    """Register the shipped kinds even if an earlier test imported them into a patched registry."""
    from mhseals_testing.health.kinds import state, threshold
    for module, name in ((threshold, 'threshold'), (state, 'state')):
        if name not in registry.KINDS:
            importlib.reload(module)


def test_shipped_health_config_loads_and_lints_clean(nav_share):
    assert HEALTH_DIR.is_dir() and list(HEALTH_DIR.glob('*.yaml'))
    config = load(user=False, profile=load_profile('boat'))
    assert any(s.id == 'cmd.owner' for s in config.specs)
    assert lint(config.specs, unmatched=config.unmatched) == []
    assert config.specs
    from mhseals_testing.procedures import PROCEDURES
    keys = [p.key for p in PROCEDURES]
    assert lint(config.specs, procedures=keys, unmatched=config.unmatched) == []


def test_low_cell_voltage_fails_the_shipped_check():
    spec = next(s for s in load(user=False).specs if s.id == 'power.cell_voltage')
    engine = Engine([spec])
    report = engine.tick({'battery.cell_v': Fact(3.1)}, {'battery': (0, None)})
    assert report.results['power.cell_voltage'].status == 'fail'
    report = Engine([spec]).tick({'battery.cell_v': Fact(3.9)}, {'battery': (0, None)})
    assert report.results['power.cell_voltage'].status == 'ok'


def test_fcu_checks_load_and_lint_clean(nav_share):
    config = load(user=False)
    specs = [s for s in config.specs if s.group == 'FCU']
    assert {'fcu.params', 'compass.field', 'battery.diag'} <= {s.id for s in specs}
    assert lint(specs) == []
    assert config.probes['fcu']['wmm_total_mG'] == 470


def test_rtk_converge_warns_only_after_180_seconds_below_fixed():
    spec = next(s for s in load(user=False).specs if s.id == 'rtk.converge')
    assert spec.show == ('odometry',) and spec.missing == 'skip'
    for seconds, status in ((60, 'ok'), (181, 'warn')):
        report = Engine([spec]).tick({'rtk.seconds_below_fixed': Fact(seconds)}, {})
        assert report.results['rtk.converge'].status == status




def test_poor_water_fails_the_shipped_conditions_check_with_its_reason():
    specs = [s for s in load(user=False).specs if s.id.startswith('conditions.')]
    assert {s.id for s in specs} == {'conditions.poor', 'conditions.marginal'}
    facts = lambda v: {'conditions.verdict': Fact(v, data={'reason': 'drift trust 41% < 50%'})}
    report = Engine(specs).tick(facts('poor'), {})
    assert report.results['conditions.poor'].status == 'fail'
    assert report.results['conditions.poor'].data['reason'] == 'drift trust 41% < 50%'
    assert Engine(specs).tick(facts('marginal'), {}).results['conditions.marginal'].status == 'warn'
    for ok in ('good', 'unknown'):
        results = Engine(specs).tick(facts(ok), {}).results
        assert {r.status for r in results.values()} <= {'ok', 'unknown'}




def perception_ids(profile):
    from mhseals_testing.profiles import load_profile
    config = load(profile=load_profile(profile), user=False)
    assert lint(config.specs, unmatched=config.unmatched) == []
    return {s.id for s in config.specs if s.group == 'Perception'}


def test_each_source_gets_its_own_perception_check_set(nav_share):
    boat, sim = perception_ids('boat'), perception_ids('crane_full_stack')
    assert {'perc.zed_native.rate', 'perc.zed_native.latency', 'perc.zed_native.clock'} <= boat
    assert not any('.sim.' in i or 'legacy_zed' in i for i in boat)
    assert sim == {f'perc.sim.{c}' for c in ('rate', 'latency', 'funnel', 'tracker_rate', 'tracker_latency',
                                             'costmap', 'class_conf', 'heading', 'split')}
    assert perception_ids('crane_direct') == set()


def test_slow_zed_native_rate_warns_and_anchors_the_camera_row():
    from mhseals_testing.profiles import load_profile
    config = load(profile=load_profile('boat'), user=False)
    spec = replace(next(s for s in config.specs if s.id == 'perc.zed_native.rate'), raise_after=0.0)
    assert spec.show == ('sensor:Camera',)
    assert Engine([spec]).tick({'perc.zed_native.hz': Fact(8.0)}, {}).results[spec.id].status == 'warn'
    assert Engine([spec]).tick({'perc.zed_native.hz': Fact(15.0)}, {}).results[spec.id].status == 'ok'


def test_boat_profile_has_no_camera_or_detections_row():
    from mhseals_testing.profiles import load_profile
    names = {s.name for s in load_profile('boat').sensors}
    assert 'camera' not in names and 'detections' not in names and 'tracks' in names


def graph(names, observations=(), tracks=()):
    from mhseals_testing.health.probes.graph import topic_fact_name
    return {'graph.nodes': Fact(list(names)),
            topic_fact_name('/perception/observations'): Fact({'pub': list(observations),
                                                               'sub': []}),
            topic_fact_name('/perception/tracks'): Fact({'pub': list(tracks), 'sub': []})}


def test_camera_liveness_needs_the_camera_hosts_wrapper_backend_and_tracker(nav_share):
    from types import SimpleNamespace
    from mhseals_testing.profiles import load_profile
    ids = ('node.camera', 'pub.observations', 'pub.tracks')
    shipped = load(profile=load_profile('boat'), user=False).specs
    specs = [replace(s, raise_after=0.0) for s in shipped if s.id in ids]
    assert {s.id for s in specs} == set(ids)
    backend = SimpleNamespace(node='/zed_backend', qos={}, type='')
    tracker = SimpleNamespace(node='/object_tracker', qos={}, type='')

    def status(*args):
        results = Engine(specs).tick(graph(*args), {'graph': (0, None)}).results
        return tuple(results[i].status for i in ids)
    perception = ['/zed_backend', '/object_tracker']
    assert status(['/front/zed_node', *perception], [backend], [tracker]) == ('ok',) * 3
    assert status(perception, [backend], [tracker]) == ('warn', 'ok', 'ok')
    assert status(['/front/zed_node'], [backend, backend], []) == ('ok', 'warn', 'warn')
    sim = {s.id for s in load(profile=load_profile('crane_full_stack'), user=False).specs}
    assert not set(ids) & sim


def test_negative_zed_native_clock_offset_is_judged_by_magnitude():
    from mhseals_testing.profiles import load_profile
    config = load(profile=load_profile('boat'), user=False)
    spec = replace(next(s for s in config.specs if s.id == 'perc.zed_native.clock'), raise_after=0.0)

    def status(offset):
        return Engine([spec]).tick({'perc.zed_native.clock_offset': Fact(offset)}, {}).results[spec.id].status
    assert [status(v) for v in (0.01, -0.05, -0.1, 0.1)] == ['ok', 'warn', 'fail', 'fail']


def test_zed_native_latency_bands_follow_the_orin_decision():
    from mhseals_testing.profiles import load_profile
    config = load(profile=load_profile('boat'), user=False)
    spec = replace(next(s for s in config.specs if s.id == 'perc.zed_native.latency'), raise_after=0.0)

    def status(v):
        return Engine([spec]).tick({'perc.zed_native.latency': Fact(v)}, {}).results[spec.id].status
    assert [status(v) for v in (0.1, 0.3, 0.6)] == ['ok', 'warn', 'fail']

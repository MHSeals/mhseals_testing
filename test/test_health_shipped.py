"""The shipped config/health files load, lint clean and drive the engine."""

import importlib

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

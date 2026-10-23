"""The shipped config/health files load, lint clean and drive the engine."""

import importlib

import pytest

from mhseals_testing.health import registry
from mhseals_testing.health.config import HEALTH_DIR, lint, load
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.model import Fact


@pytest.fixture(autouse=True)
def kinds():
    """Register the shipped kinds even if an earlier test imported them into a patched registry."""
    from mhseals_testing.health.kinds import threshold
    if 'threshold' not in registry.KINDS:
        importlib.reload(threshold)


def test_shipped_health_config_loads_and_lints_clean():
    assert HEALTH_DIR.is_dir() and list(HEALTH_DIR.glob('*.yaml'))
    config = load(user=False)
    assert config.specs
    assert lint(config.specs, unmatched=config.unmatched) == []


def test_low_cell_voltage_fails_the_shipped_check():
    spec = next(s for s in load(user=False).specs if s.id == 'power.cell_voltage')
    engine = Engine([spec])
    report = engine.tick({'battery.cell_v': Fact(3.1)}, {})
    assert report.results['power.cell_voltage'].status == 'fail'
    report = Engine([spec]).tick({'battery.cell_v': Fact(3.9)}, {})
    assert report.results['power.cell_voltage'].status == 'ok'

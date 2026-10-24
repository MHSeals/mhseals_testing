"""Shared inventory/provider boundary; no runtime or mutation adapters."""
from types import SimpleNamespace

import pytest

from mhseals_testing import catalog_inspect as inspect
from test_catalog import bundle


def test_inventory_unknown_and_explicit_compiler_evidence():
    bundles = {'mhseals_hardware': bundle('mhseals_hardware', 'hardware.x')}
    view = inspect.inspection(bundles, [])
    assert view['selection']['status'] == 'unknown'
    assert not view['rows'][0]['writable'] and not view['complete']
    selected = inspect.inspection(bundles, [], compiled={
        'hardware.x': {'value': 7, 'origins': [{'source': 'operator.yaml', 'line': 2}]}})
    assert selected['selection']['values']['hardware.x']['value'] == 7
    assert selected['selection']['status'] == 'compiled'
    assert selected['rows'] == view['rows']
    assert inspect.inspection(bundles, [], compiled={'wrong': {}})['selection']['status'] == 'unavailable'


@pytest.mark.parametrize('failure,status', [
    (ModuleNotFoundError("absent", name='astro_run_config'), 'missing-optional'),
    (ModuleNotFoundError("dependency", name='required_dependency'), 'missing-dependency'),
    (FileNotFoundError('catalog.yaml'), 'missing-resource'),
    (ValueError('bad resource'), 'invalid-installed'),
])
def test_optional_absence_differs_from_broken_installed(monkeypatch, failure, status):
    def provider(name):
        if name == 'astro_run_config':
            raise failure
        owner = name.split('.')[0]
        return SimpleNamespace(load_catalog=lambda: bundle(owner, owner + '.key'))
    monkeypatch.setattr(inspect, 'import_module', provider)
    view = inspect.installed()
    assert len(view['rows']) == 2
    assert view['packages'][-1]['status'] == status
    assert not view['complete']  # optional absence is explicit in coverage, never silently complete


def test_installed_collision_visible_and_surviving_owner_retained(monkeypatch):
    def provider(name):
        owner = name.split('.')[0]
        if name == 'astro_run_config':
            raise ModuleNotFoundError(name=name)
        return SimpleNamespace(load_catalog=lambda: bundle(owner, 'shared'))
    monkeypatch.setattr(inspect, 'import_module', provider)
    view = inspect.installed()
    assert len(view['rows']) == 1
    assert view['packages'][1]['status'] == 'invalid-installed'
    assert 'collision' in view['packages'][1]['detail']

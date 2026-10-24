"""Explicit installed/archived domain resources; no ROS import or runtime claim."""
from importlib.resources import files
from pathlib import Path
import yaml

from mhseals_hardware.catalog import load_catalog
from mhseals_nav.catalog import navigation_catalog
from mhseals_testing.catalog import assemble
from mhseals_testing.catalog_coverage import reconcile
import mhseals_nav
import pytest


def test_actual_domain_bundles_and_changed_default():
    config = Path(mhseals_nav.__file__).parents[1] / 'config'
    raw = (config / 'nav2_params.yaml').read_bytes()
    policy = (config / 'catalog.yaml').read_bytes()
    nav = navigation_catalog(raw, resource='config/nav2_params.yaml', policy_raw=policy)
    hardware = load_catalog()
    view = assemble({'mhseals_nav': nav, 'mhseals_hardware': hardware})
    assert not view.missing
    assert len(view.rows) == len(nav.rows) + len(hardware.rows)
    assert all(not row.writable and row.blockers for row in view.rows)
    assert all(row.source for row in view.rows)
    defaults = files('mhseals_hardware').joinpath('config/default.yaml').read_bytes()
    sid = nav.descriptor['default_sources'][0]['id']
    resources = {('mhseals_hardware', 'hardware-default'): ('4d672165', defaults),
                 ('mhseals_nav', sid): ('82b833ebdf6d10ee4607a9dbeb5b0adc2e85e75f', raw)}
    coverage = reconcile(view, resources, {})
    assert any(row.disposition == 'untraced' for row in coverage)
    assert all(row.blockers for row in coverage)
    assert any('missing-consumers' in row.blockers for row in coverage)
    assert sum(row.disposition == 'descriptor-backed' for row in coverage) == len(view.rows)
    changed = yaml.safe_load(raw)
    changed['controller_server']['ros__parameters']['controller_frequency'] = 20.0
    changed_raw = yaml.safe_dump(changed).encode()
    updated = navigation_catalog(changed_raw, resource='config/nav2_params.yaml', policy_raw=policy)
    key = 'navigation/controller_server/ros__parameters/controller_frequency'
    assert updated.field(key).default == 20.0
    assert nav.field(key).default != updated.field(key).default
    resources[('mhseals_nav', sid)] = ('changed', changed_raw)
    with pytest.raises(ValueError, match='changed descriptor resource'):
        reconcile(view, resources, {})

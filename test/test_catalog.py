from dataclasses import FrozenInstanceError

import pytest
from mhseals_config.catalog import DescriptorBundle
from mhseals_testing.catalog import assemble


def bundle(owner, key='x', aliases=()):
    return DescriptorBundle.read(dict(schema_version=1, owner_package=owner,
        revision='test', consumer_versions={}, default_sources=[],
        leaves=[dict(canonical_key=key, aliases=list(aliases))]), {}, {}, {})


def test_assembly_is_immutable_inventory():
    original = bundle('a', aliases=('alias',))
    view = assemble({'a': original}, required=('a', 'b'))
    assert view.field('alias') is view.field('x')
    assert view.field('x').leaf is original.rows[0]
    assert view.blockers == ('missing-required-bundle:b',)
    assert not view.field('x').writable
    with pytest.raises(TypeError):
        view.identities['new'] = view.rows[0]
    with pytest.raises(FrozenInstanceError):
        view.rows[0].owner = 'other'
    with pytest.raises(ValueError):
        view.field('unknown')


def test_owner_and_cross_package_alias_validation():
    with pytest.raises(ValueError, match='owner'):
        assemble({'a': bundle('b')}, required=('a',))
    with pytest.raises(ValueError, match='undeclared'):
        assemble({'a': bundle('a')}, required=('b',))
    with pytest.raises(ValueError, match='collision'):
        assemble({'a': bundle('a', 'x', ('shared',)),
                  'b': bundle('b', 'shared')}, required=('a', 'b'))


def test_bounded_coverage_preserves_empty_maps_and_named_families():
    from mhseals_testing.catalog_coverage import reconcile, markdown
    view = assemble({'a': bundle('a')}, required=('a',))
    rows = reconcile(view, {('a', 'default'): ('current-sha', b'x: 2\nempty: {}\n')},
                     {('a', 'audit.family'): 'current-sha'})
    assert {r.identity for r in rows} == {'default:/x', 'default:/empty', 'audit.family'}
    assert all(r.disposition == 'untraced' for r in rows)
    assert 'audit.family' in markdown(rows)
    with pytest.raises(ValueError, match='outside'):
        reconcile(view, {}, {}, traces={('a', 'missing'): ('x',)})
    with pytest.raises(ValueError, match='owner'):
        reconcile(view, {}, {('b', 'family'): 'sha'}, traces={('b', 'family'): ('x',)})

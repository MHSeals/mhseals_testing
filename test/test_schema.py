"""Every package's schema.yaml loads strictly, and the Config catalog and CLI read them."""
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from mhseals_testing import schema
from mhseals_testing.harness import cli

OWN = Path(__file__).resolve().parents[1]


@pytest.fixture
def shares(tmp_path, monkeypatch):
    """This package's real share plus the hardware and nav ones named by MHSEALS_HARDWARE/MHSEALS_NAV."""
    roots = {'mhseals_testing': OWN}
    for package, env in (('mhseals_hardware', 'MHSEALS_HARDWARE'), ('mhseals_nav', 'MHSEALS_NAV')):
        if os.environ.get(env):
            roots[package] = Path(os.environ[env])
    monkeypatch.setattr(schema, 'share_dir', lambda package: roots.get(package, tmp_path / 'absent' / package))
    return roots


def test_own_schema_keys_exist_in_the_shipped_yaml(shares):
    keys = schema.load('mhseals_testing')
    assert keys
    for key in keys:
        name, *path = key.split('.')
        node = yaml.safe_load((OWN / 'config' / f'{name}.yaml').read_text())
        for part in path:
            node = node[part]
        entry = keys[key]
        kind = {'float': (int, float), 'int': int, 'bool': bool, 'str': str, 'list': list}[entry['type']]
        assert isinstance(node, kind), key


@pytest.mark.parametrize('package', ['mhseals_hardware', 'mhseals_nav'])
def test_sibling_schemas_load_through_the_same_reader(shares, package):
    if package not in shares:
        pytest.skip(f'set MHSEALS_{package.split("_")[1].upper()} to a {package} tree')
    assert schema.load(package)


@pytest.mark.parametrize('entry', [
    {'type': 'double', 'tier': 'live', 'doc': 'x'},
    {'type': 'float', 'tier': 'sometimes', 'doc': 'x'},
    {'type': 'float', 'tier': 'live'},
    {'type': 'str', 'tier': 'live', 'doc': 'x', 'range': [0, 1]},
    {'type': 'list', 'tier': 'live', 'doc': 'x', 'range': [0]},
    {'type': 'float', 'tier': 'live', 'doc': 'x', 'range': [2, 1]},
    {'type': 'float', 'tier': 'live', 'doc': 'x', 'extra': 1},
])
def test_bad_entries_are_refused_with_the_key(tmp_path, monkeypatch, entry):
    (tmp_path / 'p' / 'config').mkdir(parents=True)
    (tmp_path / 'p' / 'config' / 'schema.yaml').write_text(yaml.safe_dump({'a.b': entry}))
    monkeypatch.setattr(schema, 'share_dir', lambda package: tmp_path / package)
    with pytest.raises(ValueError, match=r'a\.b'):
        schema.load('p')


def test_duplicate_keys_are_refused_with_the_line(tmp_path, monkeypatch):
    (tmp_path / 'p' / 'config').mkdir(parents=True)
    (tmp_path / 'p' / 'config' / 'schema.yaml').write_text('a: {type: int, tier: live, doc: x}\n'
                                                          'a: {type: int, tier: live, doc: y}\n')
    monkeypatch.setattr(schema, 'share_dir', lambda package: tmp_path / package)
    with pytest.raises(ValueError, match=r'schema\.yaml, line 2'):
        schema.load('p')


def test_cli_lists_keys_and_reports_a_missing_package(shares, capsys, tmp_path):
    code = cli.main(['config', 'catalog', '--json', '-'])
    view = json.loads(capsys.readouterr().out)
    assert any(row['package'] == 'mhseals_testing' and row['key'] == 'characterize.coast_s'
               for row in view['keys'])
    assert code == (2 if view['missing'] else 0)
    assert ('mhseals_hardware' in view['missing']) == ('mhseals_hardware' not in shares)


def test_popup_walks_every_key_and_opens_details(shares):
    from mhseals_testing.ui.popups import catalog
    found, missing = schema.rows()
    seen, opened = [], []

    def popup(modal, handle, refresh=None):
        if not modal['title'].startswith('Catalog'):
            opened.append(modal['title'])
            handle('down')
            assert modal['scroll'] == 1
            assert handle('escape') is True
            return
        for _ in range(len(found) + len(missing)):
            seen.append(modal['rows'][0][0])
            handle('enter')
            handle('down')
        assert handle('escape') is True
    catalog.open_catalog(SimpleNamespace(popup=popup))
    assert {f'{package}: {key}' for package, key, _ in found} <= set(seen)
    assert len(opened) == len(seen)

"""Every package's schema.yaml loads strictly, and the Config catalog and CLI read them."""
import io
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest
from rich.console import Console
import yaml

from mhseals_testing import schema
from mhseals_testing.harness import cli
from mhseals_testing.health.config import share_dir
from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import layout, popups
from mhseals_testing.ui.panels.status import status_rows
from mhseals_testing.ui.popups import catalog
from test_app import make_app

OWN = Path(__file__).resolve().parents[1]


@pytest.fixture
def shares(tmp_path, monkeypatch):
    """This package's source tree plus the hardware and nav ones named by MHSEALS_HARDWARE/NAV."""
    roots = {'mhseals_testing': OWN}
    for package, env in (('mhseals_hardware', 'MHSEALS_HARDWARE'), ('mhseals_nav', 'MHSEALS_NAV')):
        if os.environ.get(env):
            roots[package] = Path(os.environ[env])
    monkeypatch.setattr(schema, 'share_dir',
                        lambda package: roots.get(package, tmp_path / 'absent' / package))
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
        kind = {'float': (int, float), 'int': int, 'bool': bool, 'str': str,
                'list': list}[entry['type']]
        assert isinstance(node, kind), key


@pytest.mark.parametrize('package', ['mhseals_hardware', 'mhseals_nav'])
def test_sibling_schemas_load_through_the_same_reader(shares, package):
    if package not in shares:
        pytest.skip(f'set MHSEALS_{package.split("_")[1].upper()} to a {package} tree')
    assert schema.load(package)


@pytest.mark.parametrize('entry', [
    {'type': 'double', 'units': '', 'tier': 'live', 'doc': 'x'},
    {'type': 'float', 'units': '', 'tier': 'sometimes', 'doc': 'x'},
    {'type': 'float', 'units': 'm', 'tier': 'live'},
    {'type': 'float', 'tier': 'live', 'doc': 'x'},
    {'type': 'str', 'units': '', 'tier': 'live', 'doc': 'x', 'range': [0, 1]},
    {'type': 'list', 'units': '', 'tier': 'live', 'doc': 'x', 'range': [0]},
    {'type': 'float', 'units': '', 'tier': 'live', 'doc': 'x', 'range': [2, 1]},
    {'type': 'float', 'units': '', 'tier': 'live', 'doc': 'x', 'extra': 1},
])
def test_bad_entries_are_refused_with_the_key(tmp_path, monkeypatch, entry):
    (tmp_path / 'p' / 'config').mkdir(parents=True)
    (tmp_path / 'p' / 'config' / 'schema.yaml').write_text(yaml.safe_dump({'a.b': entry}))
    monkeypatch.setattr(schema, 'share_dir', lambda package: tmp_path / package)
    with pytest.raises(ValueError, match=r'a\.b'):
        schema.load('p')


def test_duplicate_keys_are_refused_with_the_line(tmp_path, monkeypatch):
    (tmp_path / 'p' / 'config').mkdir(parents=True)
    (tmp_path / 'p' / 'config' / 'schema.yaml').write_text(
        'a: {type: int, units: "", tier: live, doc: x}\n'
        'a: {type: int, units: "", tier: live, doc: y}\n')
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


def test_syntax_errors_name_the_file(tmp_path, monkeypatch):
    (tmp_path / 'p' / 'config').mkdir(parents=True)
    (tmp_path / 'p' / 'config' / 'schema.yaml').write_text('a: [1,\n')
    monkeypatch.setattr(schema, 'share_dir', lambda package: tmp_path / package)
    with pytest.raises(yaml.YAMLError, match=r'schema\.yaml'):
        schema.load('p')


def test_installed_schemas_load_from_the_install_share():
    """Container gate: the real share_dir resolves all three installed schema files."""
    try:
        from ament_index_python.packages import get_package_share_directory as installed
        shares = {package: Path(installed(package)) for package in schema.PACKAGES}
    except (ImportError, LookupError, AssertionError):   # AssertionError: test stubs
        pytest.skip('ament cannot resolve every package: not an installed workspace')
    assert OWN not in shares['mhseals_testing'].parents
    found, missing = schema.rows()
    assert not missing
    assert {package for package, _, _ in found} == set(schema.PACKAGES)
    for package, share in shares.items():
        assert share_dir(package) == share
        assert (share / 'config' / 'schema.yaml').is_file()
    own = {key for package, key, _ in found if package == 'mhseals_testing'}
    own_file = shares['mhseals_testing'] / 'config' / 'schema.yaml'
    assert own == set(yaml.safe_load(own_file.read_text()))


def test_real_config_row_catalog_and_narrow_navigation(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path)
    app.watch_config()
    names = [row['name'] for row in status_rows(app.snapshot(), load_profile('boat').sensors)]
    app.cursors['status'] = names.index('Config')

    def forbidden(*args, **kwargs):
        raise AssertionError('read-only catalog called writer/adapter')
    monkeypatch.setattr(app.watcher, 'reload', forbidden)
    monkeypatch.setattr(app.history, 'changes', forbidden)
    entry = dict(type='float', units='m', tier='live', doc='d')
    found = [(package, 'leaf', entry) for package in schema.PACKAGES]
    monkeypatch.setattr(schema, 'rows', lambda: (found, {'mhseals_nav': 'no schema'}))
    seen = []

    def popup(modal, handle, refresh=None):
        seen.append(modal['title'])
        if modal['title'] == 'Config':
            assert any(label.strip() == 'Catalog' for label, _ in modal['rows'])
            for _ in range(len(app.watcher.watches)):
                handle('down')
            handle('enter')
        elif modal['title'] == 'Catalog (read only)':
            for _ in range(2 + len(found)):
                output = io.StringIO()
                Console(file=output, width=36, height=12).print(layout.popup(modal, 36, 12)[0])
                assert output.getvalue()
                seen.append(modal['rows'][0][0])
                handle('enter')
                handle('down')
        else:
            handle('down')
            assert modal['scroll'] == 1
            assert handle('escape') is True
        assert handle('escape') is True
    app.popup = popup
    app.on_status('enter')
    assert seen[:2] == ['Config', 'Catalog (read only)']
    assert {f'{package}: leaf' for package, _, _ in found} <= set(seen)
    assert 'mhseals_nav' in seen


def test_catalog_uses_existing_popup_emergency_passthrough(monkeypatch):
    monkeypatch.setattr(schema, 'rows', lambda: ([], {}))
    calls = []
    keys = iter(['space', 'E', 'b', 'enter', 'space', 'E', 'escape', 'escape'])
    app = SimpleNamespace(modal=None, refresh=lambda: None, tick=lambda: None,
                          keys=SimpleNamespace(read=lambda **kw: next(keys)),
                          operator_stop=lambda: calls.append('stop'),
                          revoke_agent=lambda: calls.append('revoke'))
    app.popup = lambda modal, handle: popups.popup(app, modal, handle)
    catalog.open_catalog(app)
    assert calls == ['stop', 'revoke', 'stop', 'revoke']
    assert app.modal is None

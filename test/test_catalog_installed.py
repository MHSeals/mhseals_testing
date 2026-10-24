"""Run with explicitly installed packages; never infer installation from checkouts."""
from importlib import metadata
import os
from pathlib import Path
import sys

import pytest

from mhseals_testing import catalog_inspect


@pytest.fixture
def installed_root():
    root = os.environ.get('MHSEALS_CATALOG_TEST_INSTALL')
    if not root:
        pytest.skip('set MHSEALS_CATALOG_TEST_INSTALL to the private installed prefix')
    root = Path(root)
    import mhseals_testing, mhseals_hardware, mhseals_nav
    for module in (mhseals_testing, mhseals_hardware, mhseals_nav):
        assert root in Path(module.__file__).resolve().parents
    return root


def test_actual_console_entry_installed_union_no_effects(installed_root, monkeypatch, capsys):
    from mhseals_testing.harness import cli, config_cli, processes, apply
    from mhseals_testing.history import adapters, History
    def forbidden(*args, **kwargs):
        pytest.fail('catalog entered operational path')
    for obj, name in ((cli, 'host_collector'), (cli, 'load_profile'),
                      (cli.health_config, 'load'), (config_cli, 'main'),
                      (processes, 'ProcessManager'), (apply, 'Staging')):
        monkeypatch.setattr(obj, name, forbidden)
    for name in ('record', 'observe', 'label'):
        monkeypatch.setattr(History, name, forbidden)
    # All history adapters are outside this path; forbid their dispatch entry.
    for name in ('read', 'observe_file', 'record_file', 'observe_all'):
        monkeypatch.setattr(adapters, name, forbidden)
    entry = next(item for item in metadata.distribution('mhseals_testing').entry_points
                 if item.group == 'console_scripts' and item.name == 'boat_trial')
    assert entry.value == 'mhseals_testing.harness.cli:main'
    assert entry.load()(['config', 'catalog', '--json', '-'], collect=forbidden) == 0
    import json
    view = json.loads(capsys.readouterr().out)
    assert view == catalog_inspect.installed()
    assert view['selection']['status'] == 'unknown'
    keys = [row['key'] for row in view['rows']]
    assert any(key.startswith('navigation/') for key in keys)
    assert 'mavros.intervals' in keys and 'lidar.model' in keys
    row = next(row for row in view['rows'] if row['key'] == 'hardware.controller.kp')
    assert row['source']['sha256'] == row['descriptor']['default_ref']['source_hash']
    assert row['descriptor']['value']['constraint_ref']
    assert row['blockers'] and not row['writable']


def test_nondefault_compiled_origins_are_separate_from_inventory(installed_root, tmp_path):
    from mhseals_hardware.startup_config import compile_inputs
    from mhseals_hardware.catalog import load_catalog
    selected = tmp_path / 'selected-operator.yaml'
    selected.write_text('manual_yaw_scale: 0.37\n')
    document = compile_inputs(operator=selected)
    key = 'hardware.manual_yaw_scale'
    evidence = {key: {'value': document['effective']['manual_yaw_scale'],
                      'origins': document['origins']['manual_yaw_scale']}}
    view = catalog_inspect.installed(compiled=evidence)
    assert view['selection']['status'] == 'compiled'
    assert view['selection']['values'][key]['value'] == .37
    assert view['selection']['values'][key]['origins'][-1]['source'] == str(selected)
    package_default = next(row.default for row in load_catalog().rows
                           if row.descriptor['canonical_key'] == key)
    assert next(row['default'] for row in view['rows'] if row['key'] == key) == package_default


@pytest.mark.parametrize('filename', ['mavros_setup.yaml', 'odometry_startup.yaml',
                                      'ekf_local.yaml', 'ekf_global.yaml', 'navsat_transform.yaml'])
def test_corrupt_installed_family_visible_without_hardware_loss(
        installed_root, monkeypatch, tmp_path, filename):
    import shutil
    from mhseals_nav import owner_catalog
    before = catalog_inspect.installed()
    unaffected = [row for row in before['rows'] if row['owner'] != 'mhseals_nav']
    assert any(row['owner'] == 'mhseals_hardware' for row in unaffected)
    share = tmp_path / 'share'
    shutil.copytree(installed_root / 'share/mhseals_nav', share)
    (share / 'config' / filename).write_text('malformed: [')
    monkeypatch.setattr(owner_catalog, 'package_share', lambda name: share)
    view = catalog_inspect.installed()
    nav = next(item for item in view['packages'] if item['owner'] == 'mhseals_nav')
    assert nav['status'] == 'invalid-installed'
    assert {item['name'] for item in nav['families']} == {'nav2', 'mavros', 'lidar', 'odometry', 'localization'}
    assert view['rows'] == unaffected  # hardware and any valid optional Astro remain intact
    assert [p for p in view['packages'] if p['owner'] != 'mhseals_nav'] == [
        p for p in before['packages'] if p['owner'] != 'mhseals_nav']
    assert not view['complete']
    (share / 'config/lidar.yaml').unlink()
    (share / 'config' / filename).write_bytes(
        (installed_root / 'share/mhseals_nav/config' / filename).read_bytes())
    view = catalog_inspect.installed()
    assert next(item for item in view['packages'] if item['owner'] == 'mhseals_nav')[
        'status'] == 'missing-resource'


def test_real_optional_astro_present_missing_and_corrupt(installed_root, monkeypatch, tmp_path):
    import shutil
    prefix = os.environ.get('MHSEALS_ASTRO_CATALOG_PREFIX')
    if not prefix:
        pytest.skip('explicit optional Astro installed fixture required')
    monkeypatch.syspath_prepend(prefix)
    monkeypatch.setattr(sys, 'prefix', str(prefix))
    view = catalog_inspect.installed()
    astro = next(item for item in view['packages'] if item['owner'] == 'astro-run-config')
    assert astro['status'] == 'available'
    assert any(row['key'] == 'astro.run.ros.domain_id' for row in view['rows'])
    missing = tmp_path / 'prefix'
    monkeypatch.setattr(sys, 'prefix', str(missing))
    view = catalog_inspect.installed()
    assert view['packages'][-1]['status'] == 'missing-resource'
    shutil.copytree(Path(prefix) / 'share', missing / 'share')
    (missing / 'share/astro-run-config/run-defaults.yaml').write_text('invalid: [')
    view = catalog_inspect.installed()
    assert view['packages'][-1]['status'] == 'invalid-installed'


def test_long_navigation_details_reach_final_fields_at_narrow_width(installed_root):
    import io
    from types import SimpleNamespace
    from mhseals_testing.ui.popups import catalog
    import json
    from rich.console import Console
    from mhseals_testing.ui import layout

    key = 'navigation/controller_server/ros__parameters/FollowPath/rotate_to_heading_angular_vel'
    row = next(row for row in catalog.installed()['rows'] if row['key'] == key)
    source_lines = len(json.dumps(row, indent=2).splitlines())

    def popup(modal, handle):
        # Simulate repeated Down through the real layout: offsets count wrapped lines.
        for _ in range(source_lines * 5):
            handle('down')
        output = io.StringIO()
        panel, width = layout.popup(modal, 36, 12)
        Console(file=output, width=width, height=12, color_system=None).print(panel)
        assert modal['scroll'] > source_lines
        final = output.getvalue()
        assert 'writable' in final and 'false' in final
        assert 'numeric_bounds' in final
        bottom = modal['scroll']
        handle('down')
        layout.popup(modal, 36, 12)
        assert modal['scroll'] == bottom
        for _ in range(bottom):
            handle('up')
            panel, width = layout.popup(modal, 36, 12)
            Console(file=output, width=width, color_system=None).print(panel)
            if 'blockers' in output.getvalue():
                break
        assert 'blockers' in output.getvalue()
        assert handle('escape') is True

    catalog.details(SimpleNamespace(popup=popup), key, row)


def test_installed_startup_catalog_families_guides_and_narrow_detail(installed_root):
    import io
    from types import SimpleNamespace
    from rich.console import Console
    from mhseals_testing.ui import layout
    from mhseals_testing.ui.popups import catalog
    from mhseals_nav.owner_catalog import load_catalog
    from mhseals_hardware.catalog import load_catalog as hardware_catalog
    from mhseals_nav.lidar_configuration import package_share

    view = catalog_inspect.installed()
    nav = next(item for item in view['packages'] if item['owner'] == 'mhseals_nav')
    assert {family['name'] for family in nav['families']} == {
        'nav2', 'mavros', 'lidar', 'odometry', 'localization'}
    expected = load_catalog()
    assert len([row for row in view['rows'] if row['owner'] == 'mhseals_nav']) == len(expected.rows)
    assert len([row for row in view['rows'] if row['owner'] == 'mhseals_hardware']) == len(hardware_catalog().rows)
    key = 'localization.ekf_local.odom0_config'
    row = next(row for row in view['rows'] if row['key'] == key)
    assert row['numeric_bounds'] == 'unknown; see constraint_ref' and not row['writable']
    guide = row['constraint_guide']
    path = package_share(guide['package']) / guide['resource']
    text = path.read_text()
    assert key in text and 'exactly 15 actual booleans' in text
    assert row['descriptor']['value']['constraint_ref'] == 'startup.rule.' + key
    seen = []

    def popup(modal, handle):
        import json
        assert modal['title'] == key
        seen.append(modal['title'])
        for _ in range(len(json.dumps(row).splitlines()) + 2000):
            handle('down')
        output = io.StringIO()
        panel, width = layout.popup(modal, 36, 12)
        Console(file=output, width=width, color_system=None).print(panel)
        # The final guide link remains reachable after wrapped descriptor content.
        rendered = ''.join(output.getvalue().replace('│', '').split())
        assert 'startup-catalog-constraints.md' in rendered
        assert modal['scroll'] > len(modal['rows'])
        assert handle('escape') is True

    catalog.details(SimpleNamespace(popup=popup), key, row)
    assert seen == [key]

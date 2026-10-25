"""Shared test fixtures."""

import sys
from types import ModuleType

import pytest

from nav_paths import find_nav


@pytest.fixture
def nav_root():
    """Root of the mhseals_nav tree to compare against, or skip."""
    root = find_nav()
    if root is None:
        pytest.skip('mhseals_nav not found: set MHSEALS_NAV')
    return root


@pytest.fixture(autouse=True)
def data_root(tmp_path, monkeypatch):
    """Keep tests off the developer's real data root (state.json, sessions)."""
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path))
    return tmp_path


@pytest.fixture(autouse=True)
def no_tmux(monkeypatch):
    """Render goldens must not depend on whether the suite runs inside tmux."""
    monkeypatch.delenv('TMUX', raising=False)


@pytest.fixture(autouse=True)
def isolated_user_config(tmp_path_factory, monkeypatch):
    """Keep the persisted settings file and the mux's saved holds of the real user out of every test."""
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path_factory.mktemp('xdg')))
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path_factory.mktemp('run')))


@pytest.fixture(autouse=True)
def one_machine_registry(tmp_path_factory, monkeypatch):
    """Every role on this machine, with every capability, so boat profiles resolve here."""
    from mhseals_testing import devices
    root = tmp_path_factory.mktemp('devices')
    deployment = root / 'devices.yaml'
    deployment.write_text('hosts: {local: {capabilities: [thrusters, mavros, gpu, zed, lidar]}}\n')
    package = devices.layer_paths()[0]
    monkeypatch.setattr(devices, 'layer_paths',
                        lambda: (package, deployment, root / 'none.yaml', root / 'none.env'))
    monkeypatch.setattr(devices, 'local_addresses', lambda: set())


@pytest.fixture
def nav_share(monkeypatch, tmp_path):
    """A stand-in mhseals_nav share holding the FCU parameter reference."""
    from mhseals_testing.health import config
    path = tmp_path / 'mhseals_nav' / 'config' / 'fcu'
    path.mkdir(parents=True)
    (path / 'blastoise.param').write_text('EK3_SRC1_YAW,1\nCOMPASS_USE,1\n')
    monkeypatch.setattr(config, 'share_dir', lambda package: tmp_path / package)


@pytest.fixture(autouse=True)
def nav2_user_file(tmp_path, monkeypatch):
    """Keep tests off the real ~/.config/mhseals/nav2.yaml."""
    path = tmp_path / 'user' / 'nav2.yaml'
    monkeypatch.setenv('MHSEALS_NAV2', str(path))
    return path




def pytest_addoption(parser):
    parser.addoption('--update-goldens', action='store_true', help='rewrite the golden text renders')


@pytest.fixture(autouse=True)
def source_nav_share(monkeypatch):
    """Give the host-only factory harness its exact archived package authority."""
    nav_root = find_nav()
    if nav_root is None:
        return  # Do not skip unrelated tests when no optional nav tree is present.
    try:
        import ament_index_python.packages  # noqa: F401
    except ModuleNotFoundError as error:
        if error.name not in ('ament_index_python', 'ament_index_python.packages'):
            raise
        # Keep the real resolver and strict YAML validation. Only the absent
        # source-host package index is replaced; installed-wheel runs use ament.
        package = ModuleType('ament_index_python')
        packages = ModuleType('ament_index_python.packages')

        class PackageNotFoundError(KeyError):
            """As ament's: any other package is simply not installed here."""

        def share_directory(name):
            if name != 'mhseals_nav':
                raise PackageNotFoundError(f"package '{name}' not found")
            return str(nav_root)

        packages.PackageNotFoundError = PackageNotFoundError
        packages.get_package_share_directory = share_directory
        package.packages = packages
        monkeypatch.setitem(sys.modules, 'ament_index_python', package)
        monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)

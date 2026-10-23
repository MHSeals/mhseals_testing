"""Shared test fixtures."""

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
    """Keep the persisted settings file of the real user out of every test."""
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path_factory.mktemp('xdg')))


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

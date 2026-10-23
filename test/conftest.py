"""Shared test helpers."""

import os
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def find_nav(env=None, parent=None):
    """Locate mhseals_nav: $MHSEALS_NAV, a sibling checkout (plain or integ-), then the ament share dir."""
    env = os.environ if env is None else env
    parent = parent or ROOT.parent
    candidates = [Path(env['MHSEALS_NAV'])] if env.get('MHSEALS_NAV') else []
    candidates += [parent / 'mhseals_nav', parent / 'integ-mhseals_nav']
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory('mhseals_nav')))
    except Exception:
        pass
    return next((c for c in candidates if c.is_dir()), None)


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

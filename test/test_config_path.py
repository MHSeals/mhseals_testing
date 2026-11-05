"""health.config.config_path: the one lookup for shipped files (source checkout, else the installed share)."""
from pathlib import Path
import sys
from types import ModuleType

import pytest

from mhseals_testing.health import config

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def share(tmp_path, monkeypatch):
    """An installed share directory ament resolves for every package."""
    root = tmp_path / 'share'
    module = ModuleType('ament_index_python.packages')
    module.get_package_share_directory = lambda package: str(root / package)
    monkeypatch.setitem(sys.modules, 'ament_index_python', ModuleType('ament_index_python'))
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', module)
    return root


def test_source_checkout_first(share):
    assert config.config_path('config', 'input.yaml') == ROOT / 'config' / 'input.yaml'


def test_share_when_the_source_lacks_the_file(share, tmp_path, monkeypatch):
    monkeypatch.setattr(config, 'SOURCE', tmp_path / 'nowhere')
    assert config.config_path('config', 'input.yaml') == share / 'mhseals_testing' / 'config' / 'input.yaml'
    assert config.config_path('config', 'x.yaml', package='mhseals_nav') == share / 'mhseals_nav' / 'config' / 'x.yaml'


def test_source_path_outside_a_ros_install(tmp_path, monkeypatch):
    monkeypatch.setitem(sys.modules, 'ament_index_python', None)    # import fails
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', None)
    monkeypatch.setattr(config, 'SOURCE', tmp_path / 'ws' / 'mhseals_testing')
    assert config.config_path('config', 'a.yaml') == tmp_path / 'ws' / 'mhseals_testing' / 'config' / 'a.yaml'
    assert config.config_path('config', package='mhseals_nav') == tmp_path / 'ws' / 'mhseals_nav' / 'config'
    with pytest.raises(LookupError):
        config.config_path('package.xml', installed=True)


def test_installed_skips_the_source(share):
    assert config.config_path('config', 'input.yaml', installed=True) == share / 'mhseals_testing' / 'config' / 'input.yaml'
    assert config.share_dir('mhseals_nav') == share / 'mhseals_nav'


def test_only_config_path_asks_ament():
    """No module rolls its own share lookup again (atlas #33)."""
    hits = [str(p.relative_to(ROOT)) for p in (ROOT / 'mhseals_testing').rglob('*.py')
            if 'get_package_share_directory' in p.read_text()]
    assert hits == ['mhseals_testing/health/config.py']

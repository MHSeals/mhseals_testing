"""The lasting nav2.yaml: where it lives, atomic save, and that the launch merge accepts it."""

import sys

import pytest
import yaml

from mhseals_testing import nav2_limits


def test_path_follows_override_then_xdg(monkeypatch, tmp_path):
    monkeypatch.delenv('MHSEALS_NAV2')
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    assert nav2_limits.path() == tmp_path / 'mhseals/nav2.yaml'


def test_save_load_round_trip_and_replace(tmp_path):
    target = tmp_path / 'a' / 'nav2.yaml'
    assert nav2_limits.load(target) == {}
    nav2_limits.save({'velocity_smoother': {'ros__parameters': {'max_velocity': [1.0, 0.0, 0.5]}}}, target)
    nav2_limits.save({'velocity_smoother': {'ros__parameters': {'max_velocity': [0.8, 0.0, 0.4]}}}, target)
    assert nav2_limits.load(target)['velocity_smoother']['ros__parameters']['max_velocity'] == [0.8, 0.0, 0.4]
    assert [p.name for p in target.parent.iterdir()] == ['nav2.yaml']
    with pytest.raises(TypeError):
        nav2_limits.save([1], target)


def test_launch_argument_names_the_file(nav2_user_file):
    assert nav2_limits.launch_argument() == f'limits_file:={nav2_user_file}'


def test_merge_limits_applies_the_user_file(nav_root, tmp_path):
    sys.path.insert(0, str(nav_root))
    try:
        navigation_config = pytest.importorskip('mhseals_nav.navigation_config')
    finally:
        sys.path.remove(str(nav_root))
    base = yaml.safe_load((nav_root / 'config' / 'nav2_params.yaml').read_text())
    follow = lambda tree: tree['controller_server']['ros__parameters']['FollowPath']
    overlay = {'controller_server': {'ros__parameters': {'FollowPath': {'vx_max': 0.25}}}}
    assert follow(base)['vx_max'] != 0.25
    target = nav2_limits.save(overlay, tmp_path / 'nav2.yaml')
    ceilings = navigation_config.load_ceilings(nav_root / 'config' / 'nav2_ceilings.yaml')
    merged = navigation_config.merge_limits(base, navigation_config.load_overlay(target), ceilings)
    assert follow(merged)['vx_max'] == 0.25
    assert navigation_config.load_overlay(tmp_path / 'missing.yaml') == {}

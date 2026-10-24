"""The lifecycle check must list exactly the selected navigation servers."""

import ast
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parent / 'mhseals_testing'


def assigned_list(path, name):
    """Read a literal assignment without importing ROS dependencies."""
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Assign) and any(getattr(t, 'id', None) == name for t in node.targets):
            return ast.literal_eval(node.value)


def factory(nav_root, share):
    """Execute the exact pinned launch factory with inert launch dependencies."""
    import sys
    import tempfile
    import types
    import os
    from unittest.mock import patch

    source = HERE / 'fixtures/navigation.launch.py'
    function = next(node for node in ast.parse(source.read_text()).body
                    if isinstance(node, ast.FunctionDef) and node.name == 'navigation_nodes')
    share.mkdir()
    import shutil
    shutil.copytree(nav_root / 'mhseals_nav/behavior_trees', share / 'behavior_trees')
    (share / 'launch').mkdir()
    shutil.copyfile(source, share / 'launch/navigation.launch.py')
    actions = types.ModuleType('launch.actions')
    actions.RegisterEventHandler = lambda *a, **kw: kw
    events = types.ModuleType('launch.event_handlers')
    events.OnShutdown = lambda **kw: kw
    namespace = {'os': os, 'tempfile': tempfile, 'yaml': yaml,
                 'LaunchConfiguration': lambda key: SimpleNamespace(perform=lambda ctx: ctx[key]),
                 'FindPackageShare': lambda name: SimpleNamespace(perform=lambda ctx: str(share)),
                 'Node': lambda **kw: kw, 'OpaqueFunction': lambda **kw: kw}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)

    def run(context):
        with patch.dict(sys.modules, {'launch.actions': actions, 'launch.event_handlers': events}):
            return namespace['navigation_nodes'](context)
    return run


def test_fixture_matches_installed_launch_file(nav_root):
    # Byte-copy provenance: integrated mhseals_nav fbebfbc83148da71b08d420e0ab21def6883e852.
    installed = nav_root / 'launch/navigation.launch.py'
    assert (HERE / 'fixtures/navigation.launch.py').read_bytes() == installed.read_bytes()


@pytest.mark.parametrize('overlay', ['absent', 'expanded'])
def test_selected_servers_and_overlay_handling(nav_root, tmp_path, monkeypatch, overlay):
    monkeypatch.setenv('HOME', str(tmp_path))
    limits = tmp_path / 'limits.yaml'
    if overlay == 'expanded':
        limits.write_text('controller_server:\n  ros__parameters:\n    FollowPath:\n      vx_max: 0.3\n')
    nodes = factory(nav_root, tmp_path / 'share')({
        'params_file': str(nav_root / 'config/nav2_params.yaml'),
        'limits_file': '~/limits.yaml', 'config_root': str(tmp_path / 'snapshots'),
        'config_scope': 'servers', 'config_candidate': '', 'sim': 'true', 'use_lidar': 'true'})
    launched = [node['executable'] for node in nodes if 'package' in node
                and node['package'] != 'nav2_lifecycle_manager']
    assert list(assigned_list(PACKAGE / 'ros/nav2.py', 'SERVERS')) == launched
    assert 'smoother_server' not in launched
    offered = yaml.safe_load(Path(nodes[0]['parameters'][0]).read_text())
    lifecycle = offered['lifecycle_manager_navigation']['ros__parameters']
    assert lifecycle['node_names'] == launched
    base = yaml.safe_load((nav_root / 'config/nav2_params.yaml').read_text())
    expected = (0.3 if overlay == 'expanded' else
                base['controller_server']['ros__parameters']['FollowPath']['vx_max'])
    assert offered['controller_server']['ros__parameters']['FollowPath']['vx_max'] == expected

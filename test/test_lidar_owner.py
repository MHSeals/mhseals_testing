"""Actual boat/TUI component propagation into inert paired lidar construction."""
import ast
from dataclasses import replace
from hashlib import sha256
from types import SimpleNamespace

import pytest
import yaml

from mhseals_testing.harness.components import components
from mhseals_testing.profiles import load_profile
from mhseals_testing.ui.app import stack_overlay
from test_mavros_owner import context_for, launch_factory


@pytest.fixture
def calibration_share(nav_root, tmp_path):
    from mhseals_nav.lidar_configuration import CORRECTIONS, validate_calibration
    package = tmp_path / 'pointcloud'
    path = package / 'params/VLP16db.yaml'
    path.parent.mkdir(parents=True)
    # Recorded inert VLP16db schema: sixteen unique rows, finite correction fields.
    raw = yaml.safe_dump({'num_lasers': 16, 'distance_resolution': .002,
                          'lasers': [{**dict.fromkeys(sorted(CORRECTIONS), 0.), 'laser_id': i}
                                     for i in range(16)]})
    validate_calibration(raw)
    path.write_text(raw)
    assert sha256(raw.encode()).hexdigest() == (
        '13e205f6edcff31d4f51489581372531aa29648233f89e6b1d3cd83a430d39da')
    return lambda name: nav_root if name == 'mhseals_nav' else package


def sensor_argv(profile_args=None, overlay_args=None, route='sensors'):
    boat = load_profile('boat')
    specs = {key: dict(value) for key, value in boat.components.items()}
    specs['sensors']['args'] = {**specs['sensors']['args'], **(profile_args or {})}
    specs['sensors']['launch'] = ['mhseals_nav', f'{route}.launch.py']
    boat = replace(boat, components=specs)
    overlay = stack_overlay(boat, SimpleNamespace(fcu_url='serial:///inert-fcu'))
    assert overlay == {'odometry': {'fcu_url': 'serial:///inert-fcu'}}
    overlay['sensors'] = overlay_args or {}
    return next(item.argv for item in components(boat, overlay, selection=None)
                if item.key == 'sensors')


def sensor_factory(nav_root, share):
    from mhseals_nav.lidar_configuration import resolve_lidar
    ns = launch_factory(nav_root, 'sensors')
    ns['resolve_lidar'] = lambda *args, **kw: resolve_lidar(*args, share=share, **kw)
    # Bind actual transport constants as well as factory bodies from shipped source.
    source = nav_root / 'launch/sensors.launch.py'
    assignments = [node for node in ast.parse(source.read_text()).body
                   if isinstance(node, ast.Assign)]
    exec(compile(ast.Module(body=assignments, type_ignores=[]), '<sensor constants>', 'exec'), ns)
    return ns


def sensor_context(ns, nav_root, argv, route):
    context = context_for(ns, argv)
    if route == 'robot':
        robot = launch_factory(nav_root, 'robot')
        include = next(entity for entity in robot['generate_launch_description']()
                       if hasattr(entity, 'launch_arguments'))
        robot_context = context_for(robot, argv)
        context.update({key: value.perform(robot_context) if hasattr(value, 'perform') else value
                        for key, value in include.launch_arguments})
    return context


def selected_file(tmp_path, name, patch):
    path = tmp_path / name
    path.write_text(yaml.safe_dump({'lidar': patch}))
    return str(path)


@pytest.mark.parametrize('route,ip', [('sensors', '192.168.0.201'), ('robot', '192.168.1.201')])
def test_shipped_boat_lidar_defaults_deployment_and_selected_aliases(
        nav_root, tmp_path, calibration_share, route, ip):
    ns = sensor_factory(nav_root, calibration_share)
    argv = sensor_argv(route=route)
    assert not any(arg.startswith(('velodyne_', 'lidar_')) for arg in argv)
    defaults = ns['sensor_nodes'](sensor_context(ns, nav_root, argv, route))
    driver, transform = [node.parameters[0] for node in defaults[:2]]
    assert driver == {'model': 'VLP16', 'frame_id': 'lidar_link', 'rpm': 600.,
                      'device_ip': ip, 'port': 2368}
    assert transform == {'model': 'VLP16', 'frame_id': 'lidar_link', 'calibration': str(
        calibration_share('velodyne_pointcloud') / 'params/VLP16db.yaml')}
    first = selected_file(tmp_path, 'profile.yaml', {'rpm': 420, 'endpoint': {'ip': '10.1.2.3'}})
    custom_calibration = tmp_path / 'custom-calibration.yaml'
    custom_calibration.write_bytes((calibration_share('velodyne_pointcloud') /
                                   'params/VLP16db.yaml').read_bytes())
    second = selected_file(tmp_path, 'overlay.yaml', {
        'model': 'VLP16', 'frame': 'lidar_link', 'rpm': 450,
        'calibration': {'file': custom_calibration.name},
        'endpoint': {'ip': '10.2.3.4', 'port': 9876}})
    for overlay, expected_ip, expected_port in (({}, '10.1.2.3', 2368),
            ({'lidar_params_file': second}, '10.2.3.4', 9876),
            ({'lidar_params_file': second, 'velodyne_ip': '192.168.0.201',
              'velodyne_port': '5432'}, '192.168.0.201', 5432)):
        argv = sensor_argv({'lidar_params_file': first}, overlay, route)
        path = overlay.get('lidar_params_file', first)
        assert f'lidar_params_file:={path}' in argv
        nodes = ns['sensor_nodes'](sensor_context(ns, nav_root, argv, route))
        driver, transform = [node.parameters[0] for node in nodes[:2]]
        assert driver['device_ip'] == expected_ip and driver['port'] == expected_port
        assert driver['rpm'] == (450. if overlay else 420.)
        assert driver['model'] == transform['model'] == 'VLP16'
        assert driver['frame_id'] == transform['frame_id'] == 'lidar_link'
        if overlay:
            assert transform['calibration'] == str(custom_calibration)
        assert 'calibration' not in driver and 'rpm' not in transform
        assert nodes[0].additional_env == nodes[1].additional_env == ns['large_data_env']
        assert nodes[1].remappings == [('velodyne_points', '/points')]
        # Compare full-robot include with equivalent direct sensor context.
        if route == 'robot':
            direct_args = {'lidar_params_file': first, 'lidar_required_frame': 'lidar_link',
                           'lidar_deployment_file': str(nav_root / 'config/lidar_robot.yaml')}
            direct_argv = sensor_argv(direct_args, overlay)
            direct = ns['sensor_nodes'](sensor_context(ns, nav_root, direct_argv, 'sensors'))
            assert [node.parameters for node in direct[:2]] == [
                node.parameters for node in nodes[:2]]


def test_tui_sensor_local_frame_and_full_robot_refusal(nav_root, tmp_path, calibration_share):
    path = selected_file(tmp_path, 'local.yaml', {'frame': 'sensor_local'})
    ns = sensor_factory(nav_root, calibration_share)
    argv = sensor_argv({'lidar_params_file': path})
    nodes = ns['sensor_nodes'](sensor_context(ns, nav_root, argv, 'sensors'))
    assert all(node.parameters[0]['frame_id'] == 'sensor_local' for node in nodes[:2])
    argv = sensor_argv({'lidar_params_file': path}, route='robot')
    context = sensor_context(ns, nav_root, argv, 'robot')
    ns['Node'] = lambda **kw: pytest.fail('constructed incompatible lidar pair')
    with pytest.raises(ValueError, match='URDF'):
        ns['sensor_nodes'](context)


@pytest.mark.parametrize('route', ['sensors', 'robot'])
@pytest.mark.parametrize('kind', ['missing', 'invalid'])
def test_tui_bad_selected_lidar_refuses_before_node(
        nav_root, tmp_path, calibration_share, route, kind):
    path = tmp_path / 'selected.yaml'
    if kind == 'invalid':
        path.write_text('lidar: {rpm: 0}')
    ns = sensor_factory(nav_root, calibration_share)
    argv = sensor_argv(overlay_args={'lidar_params_file': str(path)}, route=route)
    context = sensor_context(ns, nav_root, argv, route)
    ns['Node'] = lambda **kw: pytest.fail('constructed invalid lidar pair')
    with pytest.raises((ValueError, OSError)):
        ns['sensor_nodes'](context)


@pytest.mark.parametrize('route', ['sensors', 'robot'])
@pytest.mark.parametrize('patch,packages', [({'sim': 'true'}, ['ros_tcp_endpoint']),
                                         ({'sensors_ignore': ' camera, lidar '}, [])])
def test_tui_sim_or_ignored_lidar_never_constructs_physical_pair(
        nav_root, calibration_share, route, patch, packages):
    ns = sensor_factory(nav_root, calibration_share)
    ns['resolve_lidar'] = lambda *a, **kw: pytest.fail('resolved unused physical lidar')
    argv = sensor_argv(overlay_args=dict(patch, lidar_params_file='/missing'), route=route)
    nodes = ns['sensor_nodes'](sensor_context(ns, nav_root, argv, route))
    assert [node.package for node in nodes] == packages


@pytest.mark.parametrize('route', ['sensors', 'robot'])
def test_tui_deployment_leaf_validation_cannot_be_masked(
        nav_root, tmp_path, calibration_share, route):
    deployment = selected_file(tmp_path, 'deployment.yaml', {'endpoint': {'port': 6543}})
    selected = selected_file(tmp_path, 'selected.yaml', {'rpm': 420})
    ns = sensor_factory(nav_root, calibration_share)
    argv = sensor_argv({'lidar_deployment_file': deployment},
                       {'lidar_params_file': selected}, route)
    context = sensor_context(ns, nav_root, argv, route)
    nodes = ns['sensor_nodes'](context)
    assert nodes[0].parameters[0]['port'] == 6543
    assert nodes[0].parameters[0]['rpm'] == 420.
    selected_file(tmp_path, 'deployment.yaml', {'endpoint': {'port': 0}})
    selected_file(tmp_path, 'selected.yaml', {'endpoint': {'port': 7654}})
    ns['Node'] = lambda **kw: pytest.fail('constructed pair from invalid masked deployment')
    with pytest.raises(ValueError):
        ns['sensor_nodes'](context)

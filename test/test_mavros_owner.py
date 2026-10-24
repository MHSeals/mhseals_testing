"""Actual boat/TUI/component propagation into inert navigation launch factories."""
import ast
from dataclasses import replace
from types import SimpleNamespace

import pytest
import yaml

from mhseals_testing.harness.components import components
from mhseals_testing.profiles import load_profile
from mhseals_testing.ui.app import stack_overlay


def launch_factory(nav_root, name):
    """Execute shipped factory bodies with recording constructors, never ROS."""
    from mhseals_nav.navigation_config import persistent_limits_path
    from mhseals_nav.startup_snapshot import snapshot_root, snapshot_scope

    class Configuration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context[self.name]

    def record(*args, **kwargs):
        return SimpleNamespace(args=args, **kwargs)

    source = nav_root / 'launch' / f'{name}.launch.py'
    functions = [node for node in ast.parse(source.read_text()).body
                 if isinstance(node, ast.FunctionDef)]
    ns = {key: record for key in (
        'Node', 'DeclareLaunchArgument', 'GroupAction', 'IncludeLaunchDescription',
        'OpaqueFunction', 'PythonLaunchDescriptionSource', 'IfCondition',
        'UnlessCondition', 'ParameterValue', 'TimerAction')}
    ns.update(LaunchConfiguration=Configuration, LaunchDescription=lambda entities: entities,
              PathJoinSubstitution=lambda parts: '/'.join(map(str, parts)),
              FindPackageShare=lambda package: str(nav_root),
              persistent_limits_path=persistent_limits_path,
              snapshot_root=snapshot_root, snapshot_scope=snapshot_scope)
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), ns)
    return ns


def defaults(entities):
    return {entity.args[0]: entity.default_value for entity in entities
            if hasattr(entity, 'default_value')}


def boat_argv(profile_args, overlay_args, launch='odom.launch.py'):
    """Start with the shipped boat, then use actual UI and component merges."""
    boat = load_profile('boat')
    specs = {key: dict(value) for key, value in boat.components.items()}
    specs['odometry']['args'] = {**specs['odometry']['args'], **profile_args}
    specs['odometry']['launch'] = ['mhseals_nav', launch]
    boat = replace(boat, components=specs)
    overlay = stack_overlay(boat, SimpleNamespace(fcu_url='serial:///inert-fcu'))
    assert overlay == {'odometry': {'fcu_url': 'serial:///inert-fcu'}}
    overlay['odometry'].update(overlay_args)
    return next(item.argv for item in components(boat, overlay, selection=None)
                if item.key == 'odometry')


def selected_file(tmp_path, name, entries):
    path = tmp_path / name
    path.write_text(yaml.safe_dump({'mavros_setup': {
        'ros__parameters': {'message_rates': entries}}}))
    return str(path)


def context_for(ns, argv):
    declared = defaults(ns['generate_launch_description']())
    declared.update(arg.split(':=', 1) for arg in argv[4:])
    return declared


@pytest.mark.parametrize('explicit', [None, '20.0', '13'])
def test_shipped_boat_tui_selected_rates_match_direct_and_robot(nav_root, tmp_path, explicit):
    from mhseals_nav.mavros_configuration import ALIASES
    first = selected_file(tmp_path, 'profile.yaml', ['33:6', '30:7', '31:8',
                          '24:9', '27:11', '1:12', '193:14', '65:15'])
    second = selected_file(tmp_path, 'overlay.yaml', ['33:16', '30:17', '31:18',
                           '24:19', '27:21', '1:22', '193:23', '65:24'])
    odom, robot = launch_factory(nav_root, 'odom'), launch_factory(nav_root, 'robot')
    for ns in (odom, robot):
        assert all(defaults(ns['generate_launch_description']())[key] == '' for key in ALIASES)
    # Shipped profile and stack_overlay do not inject any old rate defaults.
    shipped = boat_argv({}, {})
    assert not any(arg.split(':=')[0] in ALIASES for arg in shipped)
    profile_args = {'mavros_setup_params_file': first}
    overlay_args = {'mavros_setup_params_file': second}
    if explicit is not None:
        profile_args['mavros_imu_rate'] = '9'
        overlay_args['mavros_imu_rate'] = explicit
    for overlays, path, expected in (({}, first, ('11', '12', '14')),
                                     (overlay_args, second, ('21', '22', '23'))):
        direct_argv = boat_argv(profile_args, overlays)
        robot_argv = boat_argv(profile_args, overlays, 'robot.launch.py')
        assert f'mavros_setup_params_file:={path}' in direct_argv
        assert f'mavros_setup_params_file:={path}' in robot_argv
        direct = odom['create_mavros_setup_node'](context_for(odom, direct_argv))[0]
        entities = robot['generate_launch_description']()
        timer = next(entity for entity in entities if getattr(entity, 'period', None) == 2.0)
        forwarded = {key: value.perform(context_for(robot, robot_argv))
                     for key, value in timer.actions[0].launch_arguments}
        included = odom['create_mavros_setup_node'](
            dict(defaults(odom['generate_launch_description']()), **forwarded))[0]
        assert direct.parameters == included.parameters
        assert direct.parameters[0] == path
        rates = dict(entry.split(':') for entry in direct.parameters[1]['message_rates'])
        assert tuple(rates[key] for key in ('27', '1', '193')) == expected
        imu = (explicit if overlays else '9') if explicit is not None else None
        assert rates['30'] == (f'{float(imu):g}' if imu else ('17' if overlays else '7'))
        assert rates['31'] == (f'{float(imu):g}' if imu else ('18' if overlays else '8'))
        if explicit is not None:
            assert f'mavros_imu_rate:={imu}' in direct_argv
        else:
            assert not any(arg.split(':=')[0] in ALIASES for arg in direct_argv)


@pytest.mark.parametrize('invalid', ['missing', 'malformed'])
@pytest.mark.parametrize('route', ['odom', 'robot'])
def test_tui_invalid_selected_setup_refuses_before_node(nav_root, tmp_path, invalid, route):
    path = tmp_path / 'selected.yaml'
    if invalid == 'malformed':
        path.write_text('mavros_setup: []')
    ns = launch_factory(nav_root, 'odom')
    argv = boat_argv({}, {'mavros_setup_params_file': str(path)}, f'{route}.launch.py')
    context = context_for(ns, argv)
    if route == 'robot':
        robot = launch_factory(nav_root, 'robot')
        timer = next(entity for entity in robot['generate_launch_description']()
                     if getattr(entity, 'period', None) == 2.0)
        context.update({key: value.perform(context_for(robot, argv))
                        for key, value in timer.actions[0].launch_arguments})
    ns['Node'] = lambda **kwargs: pytest.fail('Node constructed before selected-file validation')
    with pytest.raises((OSError, ValueError)):
        ns['create_mavros_setup_node'](context)

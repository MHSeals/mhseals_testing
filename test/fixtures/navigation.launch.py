"""Explicit boat server set: no implicit docking, route server or SLAM."""
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from mhseals_nav.startup_snapshot import snapshot_root, snapshot_scope
import yaml


def navigation_nodes(context):
    from mhseals_nav.startup_snapshot import select_snapshot
    from mhseals_nav.startup_projection import resource_paths, SERVERS
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnShutdown
    selection = select_snapshot(
        LaunchConfiguration('config_root').perform(context),
        LaunchConfiguration('config_scope').perform(context),
        LaunchConfiguration('params_file').perform(context),
        LaunchConfiguration('limits_file').perform(context),
        sim=LaunchConfiguration('sim').perform(context),
        use_lidar=LaunchConfiguration('use_lidar').perform(context),
        config_candidate=LaunchConfiguration('config_candidate').perform(context) or None,
        resources=resource_paths(FindPackageShare('mhseals_nav').perform(context)))
    handle = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    with handle:
        yaml.safe_dump(selection['parameters'], handle)
    source = handle.name
    cleanup = RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
        function=lambda _: os.unlink(source) if os.path.exists(source) else None)]))
    servers = [(package, name) for package, name in SERVERS
               if name in selection['parameters']]
    nodes = []
    for package, executable in servers:
        remaps = []
        if executable in ('controller_server', 'behavior_server'):
            remaps = [('cmd_vel', 'cmd_vel_nav')]
            if executable == 'controller_server':
                # RViz uses /local_plan for the controller's active path;
                # Nav2 calls the same stream /received_global_plan.
                remaps.append(('received_global_plan', 'local_plan'))
        elif executable == 'velocity_smoother':
            remaps = [('cmd_vel', 'cmd_vel_nav'),
                      ('cmd_vel_smoothed', 'cmd_vel')]
        nodes.append(Node(
            package=package, executable=executable, name=executable,
            parameters=[source],
            remappings=remaps, output='screen'))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[source]))
    return ([cleanup] if cleanup else []) + nodes


def generate_launch_description():
    share = FindPackageShare('mhseals_nav')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [share, 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('config_candidate', default_value=''),
        DeclareLaunchArgument('config_root', default_value=snapshot_root()),
        DeclareLaunchArgument('config_scope', default_value=snapshot_scope(),
                             description='Reuse for an exact partial restart'),
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('limits_file', default_value='',
                             description='Overlay (measured limits, ~/.config/mhseals/nav2.yaml) merged with params_file'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        LogInfo(msg='LIDAR DISABLED: no range-obstacle avoidance; visual supervision required.',
                condition=UnlessCondition(LaunchConfiguration('use_lidar'))),
        OpaqueFunction(function=navigation_nodes),
    ])

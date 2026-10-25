"""Explicit boat server set: no implicit docking, route server or SLAM."""
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def navigation_nodes(context):
    from mhseals_nav.startup_projection import compile_navigation, SERVERS
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnShutdown
    parameters = compile_navigation(
        LaunchConfiguration('params_file').perform(context),
        LaunchConfiguration('limits_file').perform(context),
        sim=LaunchConfiguration('sim').perform(context),
        use_lidar=LaunchConfiguration('use_lidar').perform(context),
        share=FindPackageShare('mhseals_nav').perform(context))
    handle = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    with handle:
        yaml.safe_dump(parameters, handle)
    source = handle.name
    cleanup = RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
        function=lambda _: os.unlink(source) if os.path.exists(source) else None)]))
    servers = [(package, name) for package, name in SERVERS
               if name in parameters]
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
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('limits_file', default_value='',
                             description='Overlay (measured limits, ~/.config/mhseals/nav2.yaml) merged with params_file'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        LogInfo(msg='LIDAR DISABLED: no range-obstacle avoidance; visual supervision required.',
                condition=UnlessCondition(LaunchConfiguration('use_lidar'))),
        DeclareLaunchArgument('command_mux', default_value='false',
                              description='A command_mux arbitrates /cmd_vel; start the '
                                          'stale-costmap watchdog that asks it to hold'),
        OpaqueFunction(function=navigation_nodes),
        # Hold-only: without a mux nothing would act on its requests.
        Node(package='mhseals_nav', executable='costmap_watchdog', name='costmap_watchdog',
             output='screen', condition=IfCondition(LaunchConfiguration('command_mux')),
             parameters=[PathJoinSubstitution([share, 'config', 'costmap_watchdog.yaml']),
                         {'use_sim_time': ParameterValue(LaunchConfiguration('sim'),
                                                         value_type=bool)}]),
    ])

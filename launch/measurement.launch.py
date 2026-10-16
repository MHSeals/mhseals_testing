"""Localization (and optionally local sensor drivers) for a test session.

Includes mhseals_nav's odom.launch.py. With fcu:=false (simulation) no MAVROS
runs: the EKFs consume /odom/mavros, /imu/raw and /gps/fix straight from the
simulator. boat_test --start-stack passes the profile's ``bringup`` arguments.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav = FindPackageShare('mhseals_nav')
    arguments = [
        ('sim', 'false', 'Use simulation time (requires /clock)'),
        ('fcu', 'true', 'Start MAVROS; false when a simulator publishes the FCU topics'),
        ('fcu_url', '', 'MAVROS FCU URL; empty uses the odom.launch.py default'),
        ('start_sensors', 'false', 'Also start local sensor drivers'),
        ('sensors_ignore', 'camera', 'Drivers to skip; the camera normally runs on the Jetson'),
    ]
    config = {name: LaunchConfiguration(name) for name, _, _ in arguments}
    odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav, 'launch', 'odom.launch.py'])),
        launch_arguments={key: config[key] for key in ('sim', 'fcu', 'fcu_url')}.items())
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav, 'launch', 'sensors.launch.py'])),
        condition=IfCondition(config['start_sensors']),
        launch_arguments={key: config[key] for key in ('sim', 'sensors_ignore')}.items())
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=description)
         for name, default, description in arguments] + [odom, sensors])

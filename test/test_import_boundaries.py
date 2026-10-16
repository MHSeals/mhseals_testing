"""The pure core must import without ROS so it runs (and is tested) anywhere."""

import ast
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parent.parent / 'mhseals_testing'
ROS_MODULES = {'rclpy', 'rosbag2_py', 'launch', 'launch_ros', 'tf2_ros', 'geometry_msgs',
               'nav_msgs', 'sensor_msgs', 'std_msgs', 'tf2_msgs', 'vision_msgs', 'nav2_msgs'}
# Only ros/ talks to ROS. characterization.read_rosbag imports lazily inside the function.
PURE = [path for path in PACKAGE.rglob('*.py') if 'ros' not in path.relative_to(PACKAGE).parts]


def top_level_imports(path):
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Import):
            yield from (alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            yield node.module.split('.')[0]


@pytest.mark.parametrize('path', PURE, ids=lambda path: str(path.relative_to(PACKAGE)))
def test_core_never_imports_ros_at_module_level(path):
    assert not ROS_MODULES & set(top_level_imports(path))

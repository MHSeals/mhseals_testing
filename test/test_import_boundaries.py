"""Every package module compiles (a bad paste fails here, not at boat_test start), and the pure core imports
without ROS so it runs (and is tested) anywhere. One test per check, naming every offender."""

import ast
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent.parent / 'mhseals_testing'
MODULES = sorted(PACKAGE.rglob('*.py'))
ROS_MODULES = {'rclpy', 'rosbag2_py', 'launch', 'launch_ros', 'tf2_ros', 'geometry_msgs',
               'nav_msgs', 'sensor_msgs', 'std_msgs', 'tf2_msgs', 'vision_msgs', 'nav2_msgs'}
# Only ros/ and a feature package's ros_*.py talk to ROS. characterization.read_rosbag imports lazily inside
# the function.
PURE = [path for path in MODULES
        if 'ros' not in path.relative_to(PACKAGE).parts and not path.name.startswith('ros_')]


def top_level_imports(path):
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Import):
            yield from (alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            yield node.module.split('.')[0]


def test_every_module_compiles():
    broken = []
    for path in MODULES:
        try:
            compile(path.read_text(), str(path), 'exec')
        except SyntaxError as error:
            broken.append(f'{path.relative_to(PACKAGE)}: {error}')
    assert len(MODULES) > 100 and not broken, broken


def test_core_never_imports_ros_at_module_level():
    offenders = {str(path.relative_to(PACKAGE)): sorted(ROS_MODULES & set(top_level_imports(path)))
                 for path in PURE}
    assert len(PURE) > 100 and not {k: v for k, v in offenders.items() if v}

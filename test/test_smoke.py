"""The smoke tier's own checks: every module imports, the shipped config loads, help.yaml matches the parsers.

``tools/smoke`` runs every test marked ``smoke`` (this file, the help keys, the arm link, the agent
verbs and live C) in under a minute; it gates a landing. The full suite runs in CI and never blocks.
"""

import argparse
import importlib
import pkgutil
import shlex
from pathlib import Path

import pytest
import yaml

import mhseals_testing

pytestmark = pytest.mark.smoke
ROOT = Path(__file__).resolve().parents[1]
# Modules that need a ROS install (rclpy and message packages): they import on the boat, not here.
ROS = {'rclpy', 'rcl_interfaces', 'std_msgs', 'std_srvs', 'geometry_msgs', 'nav_msgs', 'sensor_msgs', 'mavros_msgs',
       'nav2_msgs', 'action_msgs', 'builtin_interfaces', 'tf2_ros', 'rosbag2_py', 'ament_index_python', 'cv_bridge',
       'visualization_msgs', 'diagnostic_msgs', 'lifecycle_msgs', 'rosidl_runtime_py', 'message_filters',
       'tf2_msgs', 'rosgraph_msgs', 'unique_identifier_msgs', 'rclpy_message_converter', 'launch', 'launch_ros'}


def test_every_module_imports():
    broken = []
    for info in pkgutil.walk_packages(mhseals_testing.__path__, 'mhseals_testing.'):
        try:
            importlib.import_module(info.name)
        except ModuleNotFoundError as error:
            if (error.name or '').split('.')[0] not in ROS:
                broken.append(f'{info.name}: {error}')
        except Exception as error:      # noqa: BLE001 - every broken module is listed, not the first
            broken.append(f'{info.name}: {type(error).__name__}: {error}')
    assert not broken, '\n'.join(broken)


@pytest.mark.parametrize('path', sorted((ROOT / 'config').rglob('*.yaml')), ids=lambda p: str(p.relative_to(ROOT)))
def test_shipped_yaml_parses(path):
    yaml.safe_load(path.read_text())


def test_shipped_config_validates():
    from mhseals_testing.harness import agent_config, settings
    from mhseals_testing.procedures import characterize
    from mhseals_testing.ui import keymap
    agent_config.load()
    settings.Settings()
    characterize.load_settings()
    _, warning = keymap.load(user=None)
    assert not warning


def commands():
    return yaml.safe_load((ROOT / 'help.yaml').read_text())['commands']


def parser_for(run):
    """(parser, words before the options) for a help.yaml ``run`` line of this package, or None."""
    from mhseals_testing.harness.cli import build_parser as trial
    from mhseals_testing.ui.app import build_parser as console
    words = shlex.split(run)[3:]
    if words[0] == 'boat_test':
        return console(), words[1:]
    if words[0] == 'boat_trial':
        return trial(), words[1:]
    return None


@pytest.mark.parametrize('command', [c for c in commands() if parser_for(c['run'])], ids=lambda c: c['name'])
def test_help_yaml_options_and_examples_parse(command):
    parser, head = parser_for(command['run'])
    options = {s for action in parser._actions for s in action.option_strings}
    for word in head:       # a boat_trial verb (config apply): its own subparser's options
        sub = next((a.choices[word] for a in parser._actions
                    if isinstance(a, argparse._SubParsersAction) and word in a.choices), None)
        if sub is None:
            break
        parser = sub
        options |= {s for action in sub._actions for s in action.option_strings}
    for param in command.get('params', []):
        if param['name'].startswith('-'):
            assert param['name'] in options, f"{command['name']}: help.yaml documents {param['name']}"
    for example in command.get('examples', []):
        parser_for(example)[0].parse_args(shlex.split(example)[4:])   # after ros2 run PACKAGE PROGRAM

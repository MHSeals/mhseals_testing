"""Exercise the actual ProcessManager call seams with inert process doubles."""
from dataclasses import replace

import pytest
import yaml

from mhseals_testing.configuration import bootstrap
from mhseals_testing.harness.components import Stack, components
from mhseals_testing.profiles import load_profile
from mhseals_testing.ros.arming import Arming, driver_commands
from test_arming import Processes, Ros
from test_components import FakeProcesses
from test_configuration_union import preparations


def reference(tmp_path):
    root = tmp_path / 'store'
    return bootstrap(root, 'session', preparations(root, 'session', tmp_path))


def test_same_reference_in_both_argv_routes_mux_excluded(tmp_path, monkeypatch):
    import shutil
    import sys
    from pathlib import Path
    from types import ModuleType, SimpleNamespace
    from mhseals_testing.configuration import navigation
    from mhseals_testing.harness import session
    from mhseals_testing.startup_owner import prepare_session

    source = Path(navigation.__file__).parents[1]
    share = tmp_path / 'nav-share'
    for name in ('config', 'launch'):
        shutil.copytree(source / name, share / name)
    shutil.copytree(source / 'mhseals_nav/behavior_trees', share / 'behavior_trees')
    packages = ModuleType('ament_index_python.packages')
    packages.get_package_share_directory = lambda name: str(share)
    monkeypatch.setitem(sys.modules, 'ament_index_python', ModuleType('ament_index_python'))
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    monkeypatch.setenv('MHSEALS_CONFIG_ROOT', str(tmp_path / 'store'))
    profile = load_profile('crane_full_stack').with_velocity_control()
    created = session.create('crane_full_stack', directory=tmp_path / 'session')
    selected = prepare_session(profile, SimpleNamespace(config=None),
                               created.directory, 'own', {})
    nav = next(c for c in components(profile, selection=selected) if c.key == 'navigation')
    commands = dict(driver_commands(profile, tmp_path / 'snapshot.yaml', selected))
    for key, value in selected.arguments().items():
        assert nav.argv.count(f'{key}:={value}') == 1
        assert commands['hardware'].count(f'{key}:={value}') == 1
        assert commands['velocity'].count(f'{key}:={value}') == 1
        assert not any(key in arg for arg in commands['mux'])
    assert 'use_sim_time:=true' in commands['velocity']
    assert 'output_backend:=topic' in commands['hardware']
    mismatched = replace(profile, odometry='/different/odom')
    with pytest.raises(ValueError, match='topology differs'):
        driver_commands(mismatched, tmp_path / 'snapshot.yaml', selected)
    with pytest.raises(ValueError, match='topology differs'):
        prepare_session(mismatched, SimpleNamespace(config=None),
                        created.directory, 'adopt', {})


def test_stack_validates_actual_start_and_restart(tmp_path):
    selected = reference(tmp_path)
    profile = load_profile('crane_full_stack')
    processes = FakeProcesses()
    items = [replace(c, needs=()) for c in components(profile, selection=selected)]
    stack = Stack(items, processes, lambda: 1., selected)
    stack.start('navigation', [])
    stack.stop('navigation', [])
    next((tmp_path / 'store').rglob('hardware-baseline.json')).write_text('{}')
    with pytest.raises(ValueError):
        stack.start('navigation', [])
    assert len(processes.started) == 1


def test_arming_uses_captured_effective_and_validates_before_start(tmp_path):
    selected = reference(tmp_path)
    profile = load_profile('crane_full_stack')
    ros = Ros()
    processes = Processes(ros)
    snapshot = tmp_path / 'snapshot.yaml'
    arm = Arming(profile, ros, processes, {'invalid': 'mutable draft'}, snapshot,
                 selection=selected)
    arm.begin()
    recorded = yaml.safe_load(snapshot.read_text())
    assert recorded['manual_yaw_scale'] == 0.8
    assert recorded['output_backend'] == 'topic'
    assert processes.started == ['hardware', 'mux']
    arm.disarm()
    next((tmp_path / 'store').rglob('nav2-baseline.json')).write_text('{}')
    before = snapshot.read_bytes()
    with pytest.raises(ValueError):
        arm.begin()
    assert snapshot.read_bytes() == before
    assert processes.started == ['hardware', 'mux']


def test_backend_mismatch_refuses_before_snapshot_and_process(tmp_path):
    selected = reference(tmp_path)  # topic backend selected
    ros = Ros()
    processes = Processes(ros)
    snapshot = tmp_path / 'snapshot.yaml'
    arm = Arming(load_profile('boat'), ros, processes, {}, snapshot, selection=selected)
    with pytest.raises(RuntimeError, match='output backend'):
        arm.begin()
    assert not snapshot.exists() and not processes.started


def test_main_parser_can_defer_mutable_authoring_inputs(tmp_path):
    from mhseals_testing.ui.app import parse_args
    path = tmp_path / 'corrupt.yaml'
    path.write_text('this is: [broken')
    args, _ = parse_args(['--config', str(path)], defer_hardware=True)
    assert args.hardware_config is None


def test_actual_ui_owner_refuses_missing_selection_before_ros_init(tmp_path, monkeypatch):
    import sys
    from types import ModuleType, SimpleNamespace
    from mhseals_testing.harness import session
    from mhseals_testing.ui import app
    created = session.create('boat', directory=tmp_path / 'session')
    selection = dict(root=str(tmp_path / 'store'), scope='absent', candidate='a' * 64,
                     domains=['hardware', 'navigation'])
    session.persist_startup_selection(created.directory, selection)
    calls = []
    modules = {
        'rclpy': {'init': lambda **kw: calls.append('ROS init')},
        'rclpy.signals': {'SignalHandlerOptions': SimpleNamespace(NO=0)},
        'mhseals_testing.ros.health': {'subscribe_fcu': None},
        'mhseals_testing.ros.perception': {'subscribe_detections': None},
        'mhseals_testing.ros.mission': {'TaskWatcher': None},
        'mhseals_testing.ros.nav2': {'Nav2': None},
        'mhseals_testing.ros.node': {'RosIO': None, 'node_name': lambda: 'test'},
        'mhseals_testing.ros.plant': {'Plant': None},
    }
    for name, fields in modules.items():
        module = ModuleType(name)
        module.__dict__.update(fields)
        monkeypatch.setitem(sys.modules, name, module)
    args = SimpleNamespace(review=None, settings=None, fcu_url='explicit', config='/unread/broken.yaml', save_config=False)
    monkeypatch.setattr(app, 'parse_args', lambda *a, **kw: (args, load_profile('boat')))
    monkeypatch.setattr(app.settings, 'save', lambda *a: None)
    monkeypatch.setattr(app, 'Registry', lambda: object())
    monkeypatch.setattr(app, 'decide', lambda *a: SimpleNamespace(
        mode='adopt', session=str(created.directory)))
    with pytest.raises(ValueError, match='selection missing'):
        app.main([])
    assert calls == []
    assert not (tmp_path / 'store').exists()

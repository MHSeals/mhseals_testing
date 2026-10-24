"""Actual App profile argv reaches the pinned owner's pure launch factory, without ROS."""
import ast
from hashlib import sha256
from pathlib import Path
import sys
from types import ModuleType

import pytest
import yaml

from mhseals_testing.harness.components import Stack, components
from test_components import FakeProcesses
from mhseals_testing.ui.app import parse_args, stack_overlay


@pytest.mark.parametrize('profile_name, context', [('boat', 'included'),
                                                ('crane_full_stack', 'standalone')])
def test_app_selected_yaml_reaches_owner_factory_and_fresh_edits(tmp_path, monkeypatch,
                                                               profile_name, context):
    import mhseals_nav
    from mhseals_nav.odometry_configuration import from_launch
    from mhseals_nav.localization_configuration import compile_localization, simulation_clock

    share = Path(mhseals_nav.__file__).resolve().parent.parent
    source = share / 'launch/ekf.launch.py'
    if not source.exists():
        pytest.skip('owner launch resources unavailable in this package fixture')
    substitutions = ModuleType('launch.substitutions')
    class LaunchConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, invocation):
            return invocation.get(self.name, '')
    substitutions.LaunchConfiguration = LaunchConfiguration
    monkeypatch.setitem(sys.modules, 'launch.substitutions', substitutions)
    packages = ModuleType('ament_index_python.packages')
    packages.get_package_share_directory = lambda name: str(share)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    operator, deployment = tmp_path / 'operator.yaml', tmp_path / 'deployment.yaml'
    deployment.write_text('mavros_velocity_stddev: 2\n')
    data = yaml.safe_load((Path(__file__).resolve().parents[1] /
                           f'config/profiles/{profile_name}.yaml').read_text())
    data['components']['odometry']['args'].update(
        odometry_startup_file=str(operator), odometry_deployment_file=str(deployment),
        ekf_local_config_file=str(tmp_path / 'selected-local.yaml'),
        ekf_global_config_file=str(tmp_path / 'selected-global.yaml'))
    base_paths = {}
    for node in ('ekf_local', 'ekf_global', 'navsat_transform'):
        filename = {'ekf_local': 'selected-local.yaml',
                    'ekf_global': 'selected-global.yaml'}.get(node, 'selected-navsat.yaml')
        path = tmp_path / filename
        document = yaml.safe_load((share / f'config/{node}.yaml').read_text())
        document['/**']['ros__parameters']['frequency'] = {
            'ekf_local': 17., 'ekf_global': 23., 'navsat_transform': 11.}[node]
        path.write_text(yaml.safe_dump(document))
        base_paths[node] = path
    data['components']['odometry']['args']['navsat_transform_config_file'] = str(
        base_paths['navsat_transform'])
    profile_file = tmp_path / 'profile.yaml'
    profile_file.write_text(yaml.safe_dump(data))
    tree = ast.parse(source.read_text())
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                    and node.name == 'ekf_nodes')
    for delay in (2.25, 3.75):
        operator.write_text(f'ekf_delay_{context}: {delay}\n')
        args, profile = parse_args(['--profile', str(profile_file)])
        children = {item.key: item for item in components(profile, stack_overlay(profile, args))}
        # Exercise full --start-stack ordering and a partial EKF start with odometry external.
        processes = FakeProcesses()
        stack = Stack(tuple(children.values()), processes, lambda: 0.0)
        for key in stack.start_order():
            stack.start(key, ())
        assert ('ekf', children['ekf'].argv) in processes.started
        partial = FakeProcesses()
        Stack(tuple(children.values()), partial, lambda: 0.0).start(
            'ekf', children['odometry'].nodes)
        assert partial.started == [('ekf', children['ekf'].argv)]
        invocations = [{name: value for name, value in
                        (arg.split(':=', 1) for arg in children[key].argv[4:])}
                       for key in ('odometry', 'ekf')]
        selected = [from_launch(invocation) for invocation in invocations]
        assert all(item.delay == delay and item.values['mavros_velocity_stddev'] == 2.0
                   for item in selected)
        assert all(type(item.adapter_parameters['velocity_stddev']) is float
                   for item in selected)
        assert selected[0].inputs == selected[1].inputs
        assert selected[0].inputs[-1]['sha256'] == sha256(operator.read_bytes()).hexdigest()
        calls = []
        def collect(**kwargs):
            calls.append(kwargs)
            return kwargs
        namespace = dict(from_launch=from_launch, LaunchConfiguration=LaunchConfiguration,
                         Node=collect, TimerAction=collect,
                         compile_localization=compile_localization,
                         simulation_clock=simulation_clock)
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
        for started in (processes.started, partial.started):
            argv = next(argv for key, argv in started if key == 'ekf')
            invocation = dict(arg.split(':=', 1) for arg in argv[4:])
            invocation.setdefault('ekf_overlay_file', '')
            calls.clear()
            namespace['ekf_nodes'](invocation)
            assert next(call['period'] for call in calls if 'period' in call) == delay
            nodes = {call['name']: call for call in calls if 'name' in call}
            assert nodes['ekf_local']['parameters'][0]['frequency'] == 17.
            assert nodes['ekf_global']['parameters'][0]['frequency'] == 23.
            assert nodes['navsat_transform']['parameters'][0]['frequency'] == 11.
            assert all(node['parameters'][0]['use_sim_time'] is profile.use_sim_time
                       for node in nodes.values())
            compiled = compile_localization(base_paths, '', profile.use_sim_time)
            for entry in compiled.inputs:
                if entry['sha256'] is not None:
                    assert entry['sha256'] == sha256(Path(entry['path']).read_bytes()).hexdigest()
            old = base_paths['ekf_global'].read_text()
            base_paths['ekf_global'].write_text(old.replace('23.0', '29.0'))
            assert nodes['ekf_global']['parameters'][0]['frequency'] == 23.
            base_paths['ekf_global'].write_text(old)

"""Endpoint planning preserves explicit selections without device discovery."""
from dataclasses import replace
import sys
from types import ModuleType

from mhseals_testing.harness import components as module
from mhseals_testing.harness.components import components
from mhseals_testing.ui.app import parse_args, stack_overlay
import pytest


@pytest.fixture
def resolver(monkeypatch):
    calls = []
    stub = ModuleType('mhseals_nav.endpoint_configuration')
    stub.resolve_endpoints = lambda **kw: calls.append(kw)
    monkeypatch.setitem(sys.modules, 'mhseals_nav.endpoint_configuration', stub)
    return calls


def arguments(children):
    return {item.key: dict(arg.split(':=', 1) for arg in item.argv[4:])
            for item in children if item.argv and item.key in ('odometry', 'sensors')}


def test_omitted_cli_inherits_profile_alias_and_tui_context(resolver):
    args, profile = parse_args([])
    assert stack_overlay(profile, args) == {}
    specs = {key: {**spec, 'args': dict(spec.get('args', {}))}
             for key, spec in profile.components.items()}
    specs['sensors']['args']['fcu_url'] = 'serial:///profile:57600'
    selected = arguments(components(replace(profile, components=specs)))
    assert all(row['fcu_url'] == 'serial:///profile:57600' and
               row['endpoint_context'] == 'tui' for row in selected.values())
    assert resolver[-1]['aliases'] == {'fcu_url': 'serial:///profile:57600'}


def test_actual_cli_wins_across_participants(resolver):
    args, profile = parse_args(['--fcu-url', 'tcp://localhost:5762'])
    selected = arguments(components(profile, stack_overlay(profile, args)))
    assert all(row['fcu_url'] == args.fcu_url for row in selected.values())


@pytest.mark.parametrize('name,value', [
    ('endpoint_startup_file', '/operator.yaml'),
    ('endpoint_deployment_file', '/deployment.yaml'),
    ('endpoint_context', 'standalone'), ('gcs_url', ''),
    ('ros_ip', '0.0.0.0'), ('ros_port', 7777),
])
def test_shared_endpoint_selection_and_explicit_empty_gcs(resolver, name, value):
    _, profile = parse_args([])
    selected = arguments(components(profile, {'sensors': {name: value}}))
    assert all(row[name] == str(value) for row in selected.values())
    assert resolver[-1].get('aliases', {}).get(name, value) == value


@pytest.mark.parametrize('name', ['endpoint_startup_file', 'endpoint_deployment_file',
                                  'endpoint_context', 'fcu_url', 'gcs_url', 'ros_ip', 'ros_port'])
def test_conflicts_precede_constructors(resolver, monkeypatch, name):
    _, profile = parse_args([])
    constructors = []
    monkeypatch.setattr(module, 'Component', lambda *a: constructors.append(a))
    with pytest.raises(ValueError, match='conflicting endpoint selection'):
        components(profile, {'odometry': {name: 'first'}, 'sensors': {name: 'second'}})
    assert constructors == [] and resolver == []


def test_direct_unity_requires_no_endpoint_import(monkeypatch):
    args, profile = parse_args(['--profile', 'crane_direct', '--fcu-url', 'unused'])
    monkeypatch.setitem(sys.modules, 'mhseals_nav.endpoint_configuration', None)
    assert stack_overlay(profile, args) == {} and components(profile) == ()


def test_resolver_refusal_precedes_component_construction(resolver, monkeypatch):
    _, profile = parse_args([])
    constructors = []
    monkeypatch.setattr(module, 'Component', lambda *a: constructors.append(a))

    def refuse(**kwargs):
        raise ValueError('invalid selected endpoint layer')

    monkeypatch.setattr(sys.modules['mhseals_nav.endpoint_configuration'],
                        'resolve_endpoints', refuse)
    with pytest.raises(ValueError, match='invalid selected endpoint layer'):
        components(profile, {'sensors': {'endpoint_startup_file': '/missing'}})
    assert constructors == []


def test_inherit_aliases_do_not_hide_shared_explicit_gcs_disable(resolver):
    _, profile = parse_args([])
    selected = arguments(components(profile, {
        'odometry': {'gcs_url': '', 'fcu_url': ''},
        'sensors': {'gcs_url': '__inherit__', 'ros_ip': '', 'ros_port': ''}}))
    assert all(row['gcs_url'] == '' for row in selected.values())
    assert resolver[-1]['aliases'] == {'gcs_url': ''}


@pytest.mark.parametrize('profile_name', ['boat', 'crane_full_stack'])
@pytest.mark.parametrize('explicit', [False, True])
@pytest.mark.parametrize('route', ['odom', 'robot'])
def test_app_yaml_argv_reaches_actual_endpoint_factories(
        nav_root, tmp_path, profile_name, explicit, route):
    from hashlib import sha256
    from test_mavros_owner import context_for, launch_factory
    from mhseals_nav.endpoint_configuration import resolve_endpoints
    import yaml

    deployment = tmp_path / 'deployment.yaml'
    operator = tmp_path / 'operator.yaml'
    deployment.write_text('fcu_url: serial:///inert-deployment:57600\nunity: {port: 7001}\n')
    cli = ['--profile', profile_name]
    if explicit:
        cli += ['--fcu-url', 'serial:///inert-cli:57600']
    args, profile = parse_args(cli)
    specs = {key: {**spec, 'args': dict(spec.get('args', {}))}
             for key, spec in profile.components.items()}
    specs['odometry']['launch'] = ['mhseals_nav', route + '.launch.py']
    specs['sensors']['args'].update(
        endpoint_deployment_file=str(deployment), endpoint_startup_file=str(operator), gcs_url='')
    profile = replace(profile, components=specs)
    old = None
    for port in (7002, 7003):
        operator.write_text(yaml.safe_dump({'fcu_url': 'tcp://localhost:5763',
                                           'unity': {'ip': '0.0.0.0', 'port': port}}))
        children = {item.key: item for item in components(profile, stack_overlay(profile, args))}
        odom = launch_factory(nav_root, 'odom')
        invocation = context_for(odom, children['odometry'].argv)
        if route == 'robot':
            robot = launch_factory(nav_root, 'robot')
            parent = context_for(robot, children['odometry'].argv)
            assert robot['endpoint_preflight'](parent) == []
            include = next(entity for entity in robot['generate_launch_description']()
                           if hasattr(entity, 'launch_arguments') and
                           'urdf_file' in dict(entity.launch_arguments))
            invocation.update({key: value.perform(parent) if hasattr(value, 'perform') else value
                               for key, value in include.launch_arguments})
        node = odom['create_mavros_node'](invocation)[0]
        params = node.parameters[1]
        expected = 'serial:///inert-cli:57600' if explicit and profile.uses_fcu else (
            'tcp://localhost:5763')
        assert params == {'fcu_url': expected, 'gcs_url': '',
                          'use_sim_time': profile.use_sim_time}
        sensors = launch_factory(nav_root, 'sensors')
        invocation = context_for(sensors, children['sensors'].argv)
        invocation['sim'] = 'true'  # Exercise only the inert Unity endpoint consumer.
        unity = sensors['sensor_nodes'](invocation)[0].parameters[0]
        assert unity == {'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': port}
        assert type(unity['ROS_TCP_PORT']) is int
        if old is not None:
            assert old['ROS_TCP_PORT'] == 7002
        old = unity
        selected = resolve_endpoints(deployment_file=str(deployment), operator_file=str(operator),
                                     context='tui')
        assert selected.inputs[-1]['sha256'] == sha256(operator.read_bytes()).hexdigest()
        assert selected.inputs[-2]['sha256'] == sha256(deployment.read_bytes()).hexdigest()


@pytest.mark.parametrize('kind', ['missing', 'masked-invalid', 'alias'])
def test_real_endpoint_refusal_precedes_components(tmp_path, monkeypatch, kind):
    _, profile = parse_args([])
    operator = tmp_path / 'operator.yaml'
    deployment = tmp_path / 'deployment.yaml'
    operator.write_text('unity: {port: 7777}\n')
    deployment.write_text('unity: {port: 0}\n')
    patch = {'endpoint_startup_file': str(operator)}
    if kind == 'missing':
        patch['endpoint_startup_file'] = str(tmp_path / 'missing')
    elif kind == 'masked-invalid':
        patch['endpoint_deployment_file'] = str(deployment)
    else:
        patch['fcu_url'] = 'udp://127.0.0.1:14550'
    constructors = []
    monkeypatch.setattr(module, 'Component', lambda *a: constructors.append(a))
    with pytest.raises((ValueError, OSError)):
        components(profile, {'sensors': patch})
    assert constructors == []

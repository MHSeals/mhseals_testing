"""Managed full and partial starts refuse unsupported profile graphs before spawning."""
import ast
from dataclasses import replace
from pathlib import Path

from mhseals_testing.harness import components as module
from mhseals_testing.harness.components import components, Stack
from mhseals_testing.profiles import load_profile
import pytest
from test_components import FakeProcesses
import yaml

ROOT = Path(__file__).parents[1]


@pytest.mark.parametrize('name', ['boat', 'crane_full_stack'])
def test_shipped_full_and_partial_starts_keep_clock_and_selectors(name):
    """Both management routes retain one clock and the same selected files."""
    profile = load_profile(name)
    overlay = {'odometry': {'ekf_overlay_file': '/selected/localization.yaml',
                            'ekf_local_config_file': '/selected/local.yaml'}}
    children = components(profile, overlay)
    expected = 'true' if profile.use_sim_time else 'false'
    localized = [item for item in children if item.key in ('odometry', 'ekf')]
    for item in localized:
        assert item.arguments['sim'] == expected
        assert item.arguments['ekf_overlay_file'] == '/selected/localization.yaml'
        assert item.arguments['ekf_local_config_file'] == '/selected/local.yaml'
    processes = FakeProcesses()
    stack = Stack(children, processes, lambda: 0.)
    for key in stack.start_order():
        stack.start(key, ())
    assert any(key == 'ekf' for key, _ in processes.started)
    partial = FakeProcesses()
    odometry = next(item for item in children if item.key == 'odometry')
    Stack(children, partial, lambda: 0.).start('ekf', odometry.nodes)
    assert partial.started == [('ekf', next(item.argv for item in children if item.key == 'ekf'))]


@pytest.mark.parametrize('field,value', [
    ('map_frame', 'renamed'), ('odom_frame', 'renamed'), ('base_frame', 'renamed'),
    ('odometry', '/different/odom'), ('use_sim_time', 'false'), ('use_sim_time', 0),
])
@pytest.mark.parametrize('partial', [False, True])
def test_invalid_profile_precedes_component_and_process_construction(monkeypatch, field, value,
                                                                     partial):
    """Invalid custom graph leaves never create even the first child specification."""
    profile = replace(load_profile('boat'), **{field: value})
    constructors, processes = [], FakeProcesses()
    monkeypatch.setattr(module, 'Component', lambda *args: constructors.append(args))
    with pytest.raises(ValueError, match='managed localization'):
        stack = Stack(components(profile), processes, lambda: 0.)
        if partial:
            stack.start('ekf', ('robot_state_publisher',))
        else:
            for key in stack.start_order():
                stack.start(key, ())
    assert constructors == []
    assert processes.started == []


@pytest.mark.parametrize('clock', ['true', 'False', '', 0, None])
def test_overlay_clock_update_rejected_before_components(monkeypatch, clock):
    """A component argument update cannot contradict or weaken the profile clock."""
    constructors = []
    monkeypatch.setattr(module, 'Component', lambda *args: constructors.append(args))
    with pytest.raises(ValueError, match='sim must match'):
        components(load_profile('boat'), {'ekf': {'sim': clock}})
    assert constructors == []


def test_missing_component_clock_inherits_profile_boolean():
    """Managed child clocks are explicit even when the device profile omits the selector."""
    profile = load_profile('crane_full_stack')
    me, parts = profile.placed
    bare = {name: replace(part, defaults={k: v for k, v in part.defaults.items() if k != 'sim'})
            for name, part in parts.items()}
    children = components(profile, None, (me, bare))
    assert all(item.arguments['sim'] == 'true' and '"sim":"true"' in item.argv[-1]
               for item in children if item.key in ('odometry', 'ekf'))


@pytest.mark.parametrize('clock', ['false', 0, None])
def test_loader_does_not_coerce_managed_clock(tmp_path, clock):
    """Reject malformed raw YAML clock values before bool coercion can hide them."""
    data = yaml.safe_load((ROOT / 'config/profiles/boat.yaml').read_text())
    data['use_sim_time'] = clock
    selected = tmp_path / 'profile.yaml'
    selected.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='use_sim_time'):
        load_profile(selected)


def test_direct_unity_bypasses_managed_graph_contract():
    """Unity's supported direct graph needs no managed localization components."""
    profile = load_profile('crane_direct')
    assert profile.odometry != '/odom/local'
    assert components(profile) == ()


def test_app_preflights_components_before_process_manager():
    """The actual application entry evaluates the boundary before keeper creation."""
    source = (ROOT / 'mhseals_testing/ui/app.py').read_text()
    tree = ast.parse(source)
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                    and node.name == 'session_main')
    calls = [node for node in ast.walk(function) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Name)]
    first_components = min(node.lineno for node in calls if node.func.id == 'components')
    first_manager = min(node.lineno for node in calls if node.func.id == 'ProcessManager')
    assert first_components < first_manager

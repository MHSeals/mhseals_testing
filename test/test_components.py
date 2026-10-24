"""Stack components: parsing, state and the start and stop rules."""

from pathlib import Path

import pytest

from mhseals_testing.harness.components import (
    ComponentError, Stack, components, parse_components, state)
from mhseals_testing.profiles import load_profile


class FakeProcesses:
    log_directory = Path('/logs')

    def __init__(self):
        self.live, self.codes, self.stopped, self.started = set(), {}, [], []
        self.stopped_at = {}

    def running(self, name):
        return name in self.live

    def returncode(self, name):
        return self.codes.get(name)

    def start(self, name, argv):
        self.live.add(name)
        self.started.append((name, tuple(argv)))

    def stop(self, name):
        self.live.discard(name)
        self.stopped.append(name)
        self.stopped_at[name] = 100.0


class Profile:
    components = {
        'odometry': {'launch': ['mhseals_nav', 'odom.launch.py'], 'args': {'sim': 'false'},
                     'nodes': ['ekf_local'], 'start': True},
        'navigation': {'launch': ['mhseals_nav', 'navigation.launch.py'], 'nodes': ['bt_navigator'],
                       'needs': ['odometry']},
        'camera': {'remote': 'squirtle', 'nodes': ['zed_detections_converter']},
    }


def make():
    processes, clock = FakeProcesses(), [0.0]
    stack = Stack(components(Profile, {'odometry': {'fcu_url': 'x'}}), processes, lambda: clock[0])
    return stack, processes, clock


def test_argv_and_shipped_profiles():
    odometry = components(Profile, {'odometry': {'fcu_url': 'x'}})[0]
    assert odometry.argv == ('ros2', 'launch', 'mhseals_nav', 'odom.launch.py', 'sim:=false', 'fcu_url:=x')
    boat = {item.key: item for item in components(load_profile('boat'))}
    # Same graph as the Orin: no rosbridge args, or the converter drops to websocket mode.
    assert not any('rosbridge' in arg or 'ws://' in arg for arg in boat['objects'].argv)
    assert boat['camera'].remote == 'squirtle' and not boat['camera'].argv
    sim = load_profile('crane_full_stack')
    assert 'sim:=true' in components(sim)[0].argv and 'fcu:=false' in components(sim)[0].argv
    assert load_profile('crane_direct').components is None


def test_navigation_gets_limits_file(monkeypatch, tmp_path):
    monkeypatch.setenv('HOME', str(tmp_path))
    from mhseals_testing import nav2_limits
    for name in ('boat', 'crane_full_stack'):
        items = {item.key: item for item in components(load_profile(name))}
        flagged = [item for item in items.values() if item.limits]
        assert flagged
        for item in flagged:
            assert item.argv.count(nav2_limits.launch_argument()) == 1
        assert not any(a.startswith('limits_file:=') for i in items.values() if not i.limits for a in i.argv)


def test_state_ours_starting_exited_external():
    stack, processes, clock = make()
    assert stack.state('odometry', set()) == ('off', '')
    stack.start('odometry', set())
    assert stack.state('odometry', set()) == ('starting', '')
    assert stack.state('odometry', {'/ekf_local'})[0] == 'ours'
    clock[0] = 21
    assert stack.state('odometry', set()) == ('starting', 'WARN: no ekf_local')
    processes.live.clear()
    processes.codes['odometry'] = 3
    assert stack.state('odometry', set())[1].startswith('exit 3 ')
    del processes.codes['odometry']
    assert stack.state('odometry', {'ekf_local'})[0] == 'external'


def test_start_needs_prompt_and_order():
    stack, processes, _ = make()
    with pytest.raises(ComponentError, match='Start Odometry first') as error:
        stack.start('navigation', set())
    assert error.value.keys == ('odometry',) and not processes.started
    assert stack.start_order() == ['odometry']
    stack.start('odometry', set())
    stack.start('navigation', set())
    with pytest.raises(ComponentError, match='already running'):
        stack.start('navigation', set())


def test_external_is_never_stopped():
    stack, processes, _ = make()
    with pytest.raises(ComponentError, match='running outside this session'):
        stack.stop('odometry', {'ekf_local'})
    with pytest.raises(ComponentError, match='squirtle'):
        stack.start('camera', set())
    with pytest.raises(ComponentError, match='outside'):
        stack.stop('camera', {'zed_detections_converter'})
    assert processes.stopped == []


def test_stop_ours_stops_dependants_first():
    stack, processes, _ = make()
    stack.start('odometry', set())
    stack.start('navigation', {'ekf_local'})
    assert stack.stop('odometry', {'ekf_local'}) == ['navigation', 'odometry']
    assert processes.stopped == ['navigation', 'odometry']


def test_remote_state_without_process():
    assert state(components(Profile)[2], set(), FakeProcesses(), 0)[0] == 'off'


def test_parse_components_accepts_none():
    assert parse_components(None) == {}


def test_parse_components_rejects_needs_cycle():
    data = {'a': {'remote': 'x', 'needs': ['b']}, 'b': {'remote': 'x', 'needs': ['a']}}
    with pytest.raises(ValueError, match=r'components\.a\.needs: cycle'):
        parse_components(data)


def test_lingering_names_of_a_stopped_child_are_not_external_nor_duplicates():
    from mhseals_testing.health.probes.graph import collapse
    stack, processes, clock = make()
    stack.start('odometry', set())
    stack.stop('odometry', {'ekf_local'})
    clock[0] = 100.0 + 10
    assert stack.state('odometry', {'/ekf_local'}) == ('off', '')       # not 'external'
    assert stack.ghosts() == {'ekf_local'}
    assert collapse(['/ekf_local', '/ekf_local', '/x', '/x'], stack.ghosts()) == ['/ekf_local', '/x', '/x']
    clock[0] = 100.0 + 30                                               # the lease has long expired
    assert stack.state('odometry', {'/ekf_local'})[0] == 'external' and not stack.ghosts()


@pytest.mark.parametrize('profile_name, context', [('boat', 'included'),
                                                ('crane_full_stack', 'standalone')])
def test_odometry_yaml_selectors_reach_both_actual_children(profile_name, context):
    profile = load_profile(profile_name)
    chosen = {'odometry_startup_file': '/operator/odometry.yaml',
              'odometry_deployment_file': '/deployment/odometry.yaml',
              'navsat_transform_config_file': '/deployment/navsat.yaml',
              'ekf_local_config_file': '/deployment/local.yaml',
              'ekf_global_config_file': '/deployment/global.yaml',
              'ekf_overlay_file': '/operator/ekf.yaml'}
    items = {item.key: item for item in components(profile, {'odometry': chosen})}
    for key in ('odometry', 'ekf'):
        argv = items[key].argv
        assert f'ekf_startup_context:={context}' in argv
        assert not any(arg.startswith('ekf_delay:=') for arg in argv)
        for name, value in chosen.items():
            assert f'{name}:={value}' in argv


def test_explicit_odometry_alias_survives_and_mismatches_refuse():
    profile = load_profile('boat')
    items = components(profile, {'ekf': {'ekf_delay': '2.5'}})
    assert all('ekf_delay:=2.5' in item.argv for item in items if item.key in ('ekf', 'odometry'))
    with pytest.raises(ValueError, match='conflicting odometry selection'):
        components(profile, {'odometry': {'odometry_startup_file': '/a.yaml'},
                             'ekf': {'odometry_startup_file': '/b.yaml'}})


@pytest.mark.parametrize('selector', ['ekf_local_config_file', 'ekf_global_config_file'])
def test_conflicting_localization_files_refuse_during_planning(selector):
    with pytest.raises(ValueError, match=f'conflicting odometry selection: {selector}'):
        components(load_profile('boat'), {'odometry': {selector: '/selected/a.yaml'},
                                          'ekf': {selector: '/selected/b.yaml'}})

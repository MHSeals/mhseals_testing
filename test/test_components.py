"""Stack components: device parts, parsing, state and the start and stop rules."""

from dataclasses import replace
import json
from pathlib import Path

import pytest

from mhseals_testing.devices import Host, Part
from mhseals_testing.harness.components import (
    ComponentError, Stack, components, parse_components)
from mhseals_testing.profiles import load_profile

ME = Host('local', address='127.0.0.1')
JETSON = Host('blastoise-jetson', roles=('camera_host',), address='192.168.0.153',
              user='roboboat', container='roboboat_dev')


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


class FakeUnits:
    """harness.remote.Units as the Stack sees it: status per part, start and stop requests."""

    def __init__(self):
        self.records, self.errors, self.calls = {}, {}, []

    def status(self, item):
        return self.records.get(item.part), self.errors.get(item.part)

    def start(self, item, commander=False):
        self.calls.append(('start', item.part, commander))
        self.records[item.part] = {'state': 'running'}

    def stop(self, item):
        self.calls.append(('stop', item.part))
        self.records[item.part] = {'state': 'off'}

    def take_notices(self):
        return []

    def held_by_host(self, adopted=None):
        return {}


def part(name, launch, host=ME, profile='odroid', nodes=(), **spec):
    return Part(name, profile, None, host, {'launch': launch, 'nodes': list(nodes), **spec},
                {'sim': 'false'})


PARTS = {
    'odometry': part('odometry', 'odom.launch.py', nodes=['ekf_local'], args={'fcu': 'true'}),
    'navigation': part('navigation', 'navigation.launch.py', nodes=['bt_navigator'],
                       kind='commander', args={'command_mux': 'true'}),
    'camera': part('camera', 'camera.launch.py', JETSON, 'jetson', ['zed_node']),
    'objects': part('objects', 'objects.launch.py', nodes=['object_tracker']),
    'sensors': part('sensors', 'sensors.launch.py'),
    'tracker': part('tracker', 'objects.launch.py', JETSON, 'jetson', ['object_tracker'],
                    kind='commander'),
}


class Profile:
    map_frame, odom_frame, base_frame = 'map', 'odom', 'base_link'
    odometry, use_sim_time = '/odom/local', False
    components = {
        'odometry': {'start': True},
        'navigation': {'needs': ['odometry'], 'limits': True},
        'camera': {},
    }


SIM_PARTS = {name: replace(found, defaults={'sim': 'true'}) for name, found in PARTS.items()}


def items(profile=Profile, overlay=None):
    return components(profile, overlay, (ME, SIM_PARTS if profile.use_sim_time else PARTS))


def make(units=None):
    processes, clock = FakeProcesses(), [0.0]
    stack = Stack(items(Profile, {'odometry': {'fcu_url': 'serial:///inert-fcu'}}),
                  processes, lambda: clock[0], units)
    return stack, processes, clock


def overrides(argv):
    found = [arg.split(':=', 1)[1] for arg in argv if arg.startswith('overrides:=')]
    return json.loads(found[0]) if found else {}


def test_argv_runs_one_part_through_device_launch_with_session_overrides(monkeypatch, tmp_path):
    monkeypatch.setenv('HOME', str(tmp_path))
    from mhseals_testing import nav2_limits
    odometry, navigation, camera = items(Profile, {'odometry': {'fcu_url': 'serial:///inert-fcu'}})
    assert odometry.argv[:6] == ('ros2', 'launch', 'mhseals_nav', 'device.launch.py',
                                 'device:=odroid', 'only:=odometry')
    # Only what the session adds travels; the device profile's own args stay in the profile.
    assert overrides(odometry.argv) == {'parts': {'odometry': {'args': {
        'fcu_url': 'serial:///inert-fcu', 'endpoint_context': 'tui'}}}}
    assert odometry.arguments['fcu'] == 'true' and odometry.arguments['sim'] == 'false'
    assert overrides(navigation.argv)['parts']['navigation']['args'] == {
        'limits_file': str(nav2_limits.path())}
    assert camera.remote == 'blastoise-jetson' and camera.unit == 'mhseals-camera'
    assert camera.argv == ('ros2', 'launch', 'mhseals_nav', 'device.launch.py', 'device:=jetson',
                           'only:=camera')
    assert odometry.host is None and navigation.kind == 'commander'


def test_shipped_profiles_resolve_through_the_device_profiles():
    boat = {item.key: item for item in components(load_profile('boat'))}
    assert boat['objects'].nodes == ('zed_backend', 'object_tracker', 'costmap')
    assert boat['camera'].nodes == ('zed_node',) and boat['camera'].profile == 'jetson'
    assert boat['navigation'].kind == 'commander' and boat['odometry'].arguments['fcu'] == 'true'
    for name in ('boat', 'crane_full_stack'):
        navigation = next(i for i in components(load_profile(name)) if i.key == 'navigation')
        assert (navigation.arguments.get('command_mux') == 'true') == load_profile(name).muxed
    sim = {item.key: item for item in components(load_profile('crane_full_stack'))}
    assert all(item.profile == 'sim' and item.host is None for item in sim.values())
    assert sim['odometry'].arguments['sim'] == 'true' and sim['odometry'].arguments['fcu'] == 'false'
    assert load_profile('crane_full_stack').uses_fcu is False and load_profile('boat').uses_fcu
    assert load_profile('crane_direct').components is None


def test_boat_session_start_brings_up_only_sensing():
    started = [item for item in components(load_profile('boat')) if item.start]
    assert {item.key for item in started} <= {'odometry', 'ekf', 'sensors', 'watchdogs',
                                              'operator_heartbeat'}
    assert 'watchdogs' in {item.key for item in started}     # hold-only, up before any autonomy
    # Every role is on this machine here, so it is an operator host: the heartbeat comes up too.
    assert 'operator_heartbeat' in {item.key for item in started}
    for item in started:
        assert not item.limits and item.kind == 'sensing'


def test_the_operator_heartbeat_starts_with_the_session_only_on_its_own_computer():
    """link_watchdog (require_operator) holds until the heartbeat runs: an operator TUI starts it."""
    beat = part('operator_heartbeat', None, nodes=['operator_heartbeat'])
    laptop = Host('laptop', roles=('operator',), address='192.168.0.20')

    class Operator(Profile):
        components = dict(Profile.components, operator_heartbeat={'start': 'here'})
    here = Stack(components(Operator, None, (ME, dict(PARTS, operator_heartbeat=beat))),
                 FakeProcesses(), lambda: 0.0, FakeUnits())
    assert here.autostart(set(), [], sim=False) == []
    assert ('operator_heartbeat', here.items['operator_heartbeat'].argv) in here.processes.started
    units = FakeUnits()
    boat = Stack(components(Operator, None, (ME, dict(PARTS, operator_heartbeat=replace(beat, host=laptop)))),
                 FakeProcesses(), lambda: 0.0, units)
    boat.autostart(set(), [], sim=False)       # a TUI on the boat leaves the laptop's part alone
    assert not boat.items['operator_heartbeat'].start and units.calls == []
    with pytest.raises(ValueError, match='start: true, false or here'):
        parse_components({'a': {'start': 'there'}})


def test_a_commander_marked_start_is_a_load_error_off_the_simulator():
    class Marked(Profile):
        components = dict(Profile.components, navigation={'needs': ['odometry'], 'start': True})
    with pytest.raises(ValueError, match='navigation can command motion'):
        items(Marked)


def test_profile_keys_and_part_names():
    with pytest.raises(ValueError, match='launch, args, nodes and host come from the device'):
        parse_components({'a': {'launch': ['mhseals_nav', 'odom.launch.py']}})
    with pytest.raises(ValueError, match='same part'):
        parse_components({'a': {'part': 'x'}, 'b': {'part': 'x'}})
    assert parse_components({'a': None}) == {'a': {}}
    assert parse_components(None) == {}


def test_parse_components_rejects_needs_cycle():
    data = {'a': {'needs': ['b']}, 'b': {'needs': ['a']}}
    with pytest.raises(ValueError, match=r'components\.a\.needs: cycle'):
        parse_components(data)


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


class Tracked(Profile):
    components = dict(Profile.components, objects={'needs': ['navigation'], 'start': True})
    use_sim_time = True


def test_autostart_never_moves_the_real_boat_even_through_needs():
    processes = FakeProcesses()
    stack = Stack(items(Tracked), processes, lambda: 0.0)
    assert stack.start_order() == ['odometry', 'navigation', 'objects']   # navigation pulled in
    lines = stack.autostart(set(), [], sim=False)
    assert [name for name, _ in processes.started] == ['odometry']
    assert lines[0].startswith('refused to auto-start Navigation')
    assert lines[1].startswith('skipped Objects: Navigation')
    stack = Stack(items(Tracked), FakeProcesses(), lambda: 0.0)
    assert stack.autostart(set(), [], sim=True) == []                     # the simulator may
    assert [name for name, _ in stack.processes.started] == ['odometry', 'navigation', 'objects']


def test_autostart_survives_a_failed_start_and_attaches_to_what_runs():
    processes = FakeProcesses()

    def start(name, argv):
        if name == 'navigation':
            raise OSError('ros2: not found')
        processes.live.add(name)
        processes.started.append((name, tuple(argv)))
    processes.start = start
    stack = Stack(items(Tracked), processes, lambda: 0.0)
    lines = stack.autostart({'/ekf_local'}, ['/ekf_local'], sim=True)   # odometry already up
    assert lines == ['attached to Odometry (external)',
                     'could not start Navigation: ros2: not found',
                     'skipped Objects: Navigation did not start']
    assert processes.started == []


def test_autostart_leaves_a_nodeless_component_off_beside_a_running_stack():
    class Endpoint(Profile):
        components = dict(Profile.components, sensors={'start': True})
    processes = FakeProcesses()
    stack = Stack(items(Endpoint), processes, lambda: 0.0)
    lines = stack.autostart({'/ekf_local'}, ['/ekf_local'], sim=True)
    assert lines[1].startswith('left Sensors off') and processes.started == []
    stack.autostart(set(), [], sim=True)                    # nothing up yet: start it
    assert [name for name, _ in processes.started] == ['odometry', 'sensors']


def test_autostart_reports_a_foreign_odometry_publisher_and_starts_nothing():
    processes = FakeProcesses()
    stack = Stack(items(Tracked), processes, lambda: 0.0)
    lines = stack.autostart(set(), ['/rosbag2_player'], sim=True)
    assert 'rosbag2_player' in lines[0] and processes.started == []


def test_external_is_never_stopped():
    stack, processes, _ = make()
    with pytest.raises(ComponentError, match='running outside this session'):
        stack.stop('odometry', {'ekf_local'})
    with pytest.raises(ComponentError, match='blastoise-jetson; start it there'):
        stack.start('camera', set())                        # no remote control in this stack
    with pytest.raises(ComponentError, match='blastoise-jetson; stop it there'):
        stack.stop('camera', {'/front/zed_node'})
    assert processes.stopped == []


def test_stop_ours_stops_dependants_first():
    stack, processes, _ = make()
    stack.start('odometry', set())
    stack.start('navigation', {'ekf_local'})
    assert stack.stop('odometry', {'ekf_local'}) == ['navigation', 'odometry']
    assert processes.stopped == ['navigation', 'odometry']


def test_remote_parts_start_and_stop_as_units_on_their_host():
    units = FakeUnits()
    stack, processes, _ = make(units)
    assert stack.state('camera', set()) == ('unknown', 'unit status on blastoise-jetson not read yet')
    units.records['camera'] = {'state': 'off'}
    assert stack.state('camera', set()) == ('off', '')
    stack.start('camera', set())
    assert units.calls == [('start', 'camera', False)] and not processes.started
    assert stack.state('camera', set()) == ('starting', 'unit on blastoise-jetson')
    assert stack.state('camera', {'/front/zed_node'}) == ('ours', 'unit on blastoise-jetson')
    assert stack.stop('camera', {'/front/zed_node'}) == ['camera']
    assert units.calls[-1] == ('stop', 'camera')
    units.records['camera'] = {'state': 'exited', 'code': 2, 'log': '/d/units/x.log'}
    assert stack.state('camera', set()) == ('exited', 'unit exited 2 on blastoise-jetson: /d/units/x.log')


def test_unreachable_is_not_off_and_refuses_start_but_stop_still_tries():
    units = FakeUnits()
    stack, _, _ = make(units)
    units.errors['camera'] = 'ssh: timeout'
    assert stack.state('camera', set()) == ('unreachable', 'ssh: timeout')
    with pytest.raises(ComponentError, match='unreachable \\(ssh: timeout\\)'):
        stack.start('camera', set())
    assert stack.stop('camera', set()) == ['camera']      # the state is unknown: stop is sent anyway
    assert units.calls == [('stop', 'camera')]


def test_remote_nodes_up_while_ssh_fails_or_before_the_first_read_stay_stoppable():
    units = FakeUnits()
    stack, _, _ = make(units)
    up = {'/front/zed_node'}
    # Before the first status call: maybe ours, so not external; Start waits, Stop goes out.
    assert stack.state('camera', up)[0] == 'unknown'
    with pytest.raises(ComponentError, match='not read yet'):
        stack.start('camera', up)
    assert stack.stop('camera', up) == ['camera']
    # ssh down with the nodes still in the graph: unreachable, never external, and Stop is sent.
    units.records.pop('camera')
    units.errors['camera'] = 'ssh: timeout'
    assert stack.state('camera', up) == ('unreachable', 'ssh: timeout')
    assert stack.stop('camera', up) == ['camera']
    # Read as off with the nodes up: someone else's; a Stop still goes out (harmless) and starts none.
    units.errors.clear()
    units.records['camera'] = {'state': 'off'}
    assert stack.state('camera', up)[0] == 'external'
    assert stack.stop('camera', up) == ['camera']
    assert units.calls == [('stop', 'camera')] * 3


def test_a_remote_commander_needs_the_operators_confirmed_go_ahead():
    class Remote(Profile):
        components = {'tracker': {}}
    units = FakeUnits()
    stack = Stack(items(Remote), FakeProcesses(), lambda: 0.0, units)
    with pytest.raises(ComponentError, match='can command motion'):
        stack.start('tracker', set())
    assert units.calls == []
    stack.start('tracker', set(), commander=True)
    assert units.calls == [('start', 'tracker', True)]
    assert stack.stop('tracker', set()) == ['tracker']    # stop has no kind checks


def test_a_disabled_part_says_where_and_never_starts():
    disabled = dict(PARTS, camera=part('camera', 'camera.launch.py', ME, 'sim', ['zed_node'],
                                       enabled=False))
    stack = Stack(components(Profile, None, (ME, disabled)), FakeProcesses(), lambda: 0.0)
    assert stack.state('camera', set()) == ('disabled', 'disabled on sim')
    with pytest.raises(ComponentError, match='disabled on sim'):
        stack.start('camera', set())


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
    found = {item.key: item for item in components(profile, {'odometry': chosen})}
    for key in ('odometry', 'ekf'):
        arguments = found[key].arguments
        assert arguments['ekf_startup_context'] == context
        assert 'ekf_delay' not in arguments
        for name, value in chosen.items():
            assert arguments[name] == value
            assert overrides(found[key].argv)['parts'][found[key].part]['args'][name] == value


def test_explicit_odometry_alias_survives_and_mismatches_refuse():
    profile = load_profile('boat')
    found = components(profile, {'ekf': {'ekf_delay': '2.5'}})
    assert all(item.arguments['ekf_delay'] == '2.5' for item in found if item.key in ('ekf', 'odometry'))
    with pytest.raises(ValueError, match='conflicting odometry selection'):
        components(profile, {'odometry': {'odometry_startup_file': '/a.yaml'},
                             'ekf': {'odometry_startup_file': '/b.yaml'}})


@pytest.mark.parametrize('selector', ['ekf_local_config_file', 'ekf_global_config_file'])
def test_conflicting_localization_files_refuse_during_planning(selector):
    with pytest.raises(ValueError, match=f'conflicting odometry selection: {selector}'):
        components(load_profile('boat'), {'odometry': {selector: '/selected/a.yaml'},
                                          'ekf': {selector: '/selected/b.yaml'}})

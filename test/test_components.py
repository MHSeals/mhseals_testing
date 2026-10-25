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
        self.now, self.failing = 0.0, {}       # host -> (why, since, last read)

    def clock(self):
        return self.now

    def status(self, item):
        return self.records.get(item.part), self.errors.get(item.part)

    def trouble(self):
        return dict(self.failing)

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
    # The watchdogs run inside odometry's launch on the boat (nav's with:): a read-only row.
    watchdogs = next(item for item in components(load_profile('boat')) if item.key == 'watchdogs')
    assert watchdogs.carried_by == 'Odometry' and watchdogs not in started
    # MAVROS and the EKFs are the boat's own: a TUI that misses them in discovery must not start a second set.
    assert not {'odometry', 'ekf'} & {item.key for item in started}
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
    with pytest.raises(ValueError, match='launch, args and host come from the device'):
        parse_components({'a': {'launch': ['mhseals_nav', 'odom.launch.py']}})
    with pytest.raises(ValueError, match='a.nodes: list of node names'):
        parse_components({'a': {'nodes': 'lidar_relay_publish'}})
    with pytest.raises(ValueError, match='same part'):
        parse_components({'a': {'part': 'x'}, 'b': {'part': 'x'}})
    assert parse_components({'a': None}) == {'a': {}}
    assert parse_components(None) == {}


def test_parse_components_rejects_needs_cycle():
    data = {'a': {'needs': ['b']}, 'b': {'needs': ['a']}}
    with pytest.raises(ValueError, match=r'components\.a\.needs: cycle'):
        parse_components(data)


def test_state_running_starting_exited_whoever_started_it():
    stack, processes, clock = make()
    assert stack.state('odometry', set()) == ('off', '')
    stack.start('odometry', set())
    assert stack.state('odometry', set()) == ('starting', 'started by this TUI')
    assert stack.state('odometry', {'/ekf_local'}) == ('running', 'started by this TUI')
    clock[0] = 21
    assert stack.state('odometry', set()) == ('running', 'started by this TUI')   # our child runs
    processes.live.clear()
    processes.codes['odometry'] = 3
    assert stack.state('odometry', set())[1].startswith('exit 3 ')
    del processes.codes['odometry']
    # Someone else's: running, never 'external'; who started it is only the detail.
    assert stack.state('odometry', {'ekf_local'}) == ('running', 'in the graph, not started by this TUI')


def test_this_computers_process_table_decides_a_local_part_not_the_graph():
    table = {'mhseals-odometry': {'running': True, 'pid': 42, 'how': 'launch'}}
    processes, clock = FakeProcesses(), [0.0]
    stack = Stack(items(Profile), processes, lambda: clock[0], scan=lambda specs: dict(table))
    assert stack.state('odometry', set()) == ('running', 'pid 42, not started by this TUI')
    with pytest.raises(ComponentError, match='Odometry runs outside the TUI'):
        stack.stop('odometry', set())          # a shell's process is never stopped from here
    with pytest.raises(ComponentError, match='already running'):
        stack.start('odometry', set())
    assert processes.stopped == [] and processes.started == []
    table.clear()                 # read again after SCAN_S: gone from the process table
    clock[0] = Stack.SCAN_S
    assert stack.state('odometry', {'/ekf_local'})[0] == 'off'     # a lingering name is not a part


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


class Watched(Profile):
    components = {'odometry': {}, 'watchdogs': {'title': 'Safety watchdogs'}}


WATCHED_PARTS = dict(
    PARTS, odometry=replace(PARTS['odometry'], spec=dict(PARTS['odometry'].spec,
                                                         **{'with': ['watchdogs']})),
    watchdogs=part('watchdogs', 'watchdogs.launch.py',
                   nodes=['link_watchdog', 'fence_watchdog', 'fcu_watchdog']))
DOGS = {'/link_watchdog', '/fence_watchdog', '/fcu_watchdog'}


def test_the_watchdogs_odometry_brings_are_a_read_only_row():
    processes = FakeProcesses()
    stack = Stack(components(Watched, None, (ME, WATCHED_PARTS)), processes, lambda: 0.0)
    assert stack.items['watchdogs'].carried_by == 'Odometry'
    assert stack.state('watchdogs', set()) == ('off', 'starts with Odometry')
    with pytest.raises(ComponentError, match='start with Odometry'):
        stack.start('watchdogs', set())
    stack.start('odometry', set())                     # they come up inside its launch
    assert stack.state('watchdogs', DOGS) == ('external', 'with Odometry')
    with pytest.raises(ComponentError, match='run with Odometry'):
        stack.stop('watchdogs', DOGS)
    assert stack.stop('odometry', DOGS) == ['odometry']   # which takes them along
    assert [name for name, _ in processes.started] == ['odometry']


def test_a_watchdogs_row_carried_by_odometry_may_not_start_with_the_session():
    class Started(Profile):
        components = {'odometry': {}, 'watchdogs': {'start': True}}
    with pytest.raises(ValueError, match='watchdogs.start: watchdogs starts with Odometry'):
        components(Started, None, (ME, WATCHED_PARTS))


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
    assert lines == ['attached to Odometry (running)',
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


def test_a_boat_tui_never_auto_starts_a_part_the_boat_runs_while_discovery_is_slow():
    """10-25: a laptop TUI missed the boat's running stack and started a second MAVROS on the FCU link."""
    boat = Host('blastoise', roles=('odroid',), address='192.168.0.10')
    laptop = Host('tranquility', roles=('operator',), address='192.168.0.2')

    class Started(Profile):   # even with odometry and the EKFs marked start again
        components = dict(load_profile('boat').components, odometry={'start': True},
                          ekf={'title': 'EKFs', 'needs': ['odometry'], 'start': True})
    for profile in (load_profile('boat'), Started):
        _, parts = load_profile('boat').placed
        placed = (laptop, {name: replace(found, host=laptop if name == 'operator_heartbeat' else boat)
                           for name, found in parts.items()})
        units, processes = FakeUnits(), FakeProcesses()
        stack = Stack(components(profile, None, placed), processes, lambda: 0.0, units)
        assert stack.start_order()
        for record in (None, {'state': 'off'}):     # unit status not read yet, or no unit: detached
            units.records = {item.part: record for item in stack.items.values()}
            lines = stack.autostart(set(), [], sim=False)          # nothing discovered yet
            assert units.calls == []                               # nothing started on the boat
            assert [name for name, _ in processes.started] == ['operator_heartbeat']
            processes.started.clear()
            processes.live.clear()
            assert 'Safety watchdogs' not in ' | '.join(lines)    # they come with odometry
            if profile is Started:
                assert 'left Odometry to blastoise' in ' | '.join(lines)
                assert 'skipped EKFs: Odometry did not start' in lines


def test_autostart_reports_a_foreign_odometry_publisher_and_starts_nothing():
    processes = FakeProcesses()
    stack = Stack(items(Tracked), processes, lambda: 0.0)
    lines = stack.autostart(set(), ['/rosbag2_player'], sim=True)
    assert 'rosbag2_player' in lines[0] and processes.started == []


def test_a_part_no_tui_started_is_never_stopped():
    stack, processes, _ = make()
    with pytest.raises(ComponentError, match='runs outside the TUI'):
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
    stack, processes, clock = make(units)
    assert stack.state('camera', set()) == ('unknown', 'blastoise-jetson not read yet')
    with pytest.raises(ComponentError, match='not read yet; try again shortly'):
        stack.start('camera', set())          # it may run there already: never a blind start
    units.records['camera'] = {'state': 'off', 'running': False}
    assert stack.state('camera', set()) == ('off', '')
    stack.start('camera', set())
    assert units.calls == [('start', 'camera', False)] and not processes.started
    assert stack.state('camera', set()) == ('starting', 'unit on blastoise-jetson')
    assert stack.state('camera', {'/front/zed_node'}) == ('running', 'unit on blastoise-jetson')
    clock[0] = 21
    assert stack.state('camera', set()) == ('running', 'unit on blastoise-jetson')
    assert stack.stop('camera', {'/front/zed_node'}) == ['camera']
    assert units.calls[-1] == ('stop', 'camera')
    units.records['camera'] = {'state': 'exited', 'code': 2, 'log': '/d/units/x.log'}
    assert stack.state('camera', set()) == ('exited', 'unit exited 2 on blastoise-jetson: /d/units/x.log')


def test_a_discovery_dropout_never_changes_a_reported_part():
    units = FakeUnits()
    stack, _, _ = make(units)
    units.records['camera'] = {'state': 'off', 'running': True, 'pid': 9, 'how': 'launch'}
    seen = stack.state('camera', {'/front/zed_node'})
    assert seen == ('running', 'pid 9 on blastoise-jetson, not started by a TUI')
    assert stack.state('camera', set()) == seen          # the nodes left the graph: still running
    units.records['camera'] = {'state': 'off', 'running': False}
    assert stack.state('camera', {'/front/zed_node'}) == ('off', '')   # nor do lingering names start it


def test_an_ssh_blip_keeps_the_last_report_and_stop_still_tries():
    units = FakeUnits()
    stack, _, _ = make(units)
    units.records['camera'] = {'unit': 'mhseals-camera', 'state': 'running', 'how': 'unit',
                               'running': True, 'by': {'host': 'tranquility'}}
    before = stack.state('camera', set())
    assert before == ('running', 'unit on blastoise-jetson, started from tranquility')
    units.errors['camera'] = 'ssh: timeout'               # Units keeps the record through the blip
    assert stack.state('camera', set()) == before
    assert stack.stop('camera', set()) == ['camera'] and units.calls == [('stop', 'camera')]


def test_a_remote_part_no_tui_started_is_never_stopped_and_never_started_twice():
    units = FakeUnits()
    stack, _, _ = make(units)
    units.records['camera'] = {'state': 'off', 'running': True, 'pid': 9, 'how': 'nodes'}
    with pytest.raises(ComponentError, match='runs outside the TUI \\(pid 9 on blastoise-jetson'):
        stack.stop('camera', set())
    with pytest.raises(ComponentError, match='already running'):
        stack.start('camera', set())
    assert units.calls == []


def test_remote_nodes_up_before_the_first_read_are_running_and_stay_stoppable():
    units = FakeUnits()
    stack, _, _ = make(units)
    up = {'/front/zed_node'}
    # Before the first report the graph is positive proof (never 'external'); Stop goes out.
    assert stack.state('camera', up) == ('running', 'on blastoise-jetson (in the graph; '
                                                    'blastoise-jetson not read yet)')
    with pytest.raises(ComponentError, match='already running'):
        stack.start('camera', up)
    assert stack.stop('camera', up) == ['camera']
    assert units.calls == [('stop', 'camera')]


def test_a_remote_commander_needs_the_operators_confirmed_go_ahead():
    class Remote(Profile):
        components = {'tracker': {}}
    units = FakeUnits()
    units.records['tracker'] = {'state': 'off', 'running': False}
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


def test_lingering_names_of_a_stopped_child_are_not_running_nor_duplicates():
    from mhseals_testing.health.probes.graph import collapse
    stack, processes, clock = make()
    stack.start('odometry', set())
    stack.stop('odometry', {'ekf_local'})
    clock[0] = 100.0 + 10
    assert stack.state('odometry', {'/ekf_local'}) == ('off', '')
    assert stack.ghosts() == {'ekf_local'}
    assert collapse(['/ekf_local', '/ekf_local', '/x', '/x'], stack.ghosts()) == ['/ekf_local', '/x', '/x']
    clock[0] = 100.0 + 70                                               # the 60 s lease has expired
    assert stack.state('odometry', {'/ekf_local'})[0] == 'running' and not stack.ghosts()


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

"""boat_agent start / stop COMPONENT: the desk starts an allowed stack part through the Status row's own path
(ui/parts.py -> Stack.start, the processes and units stubbed), under the claim, and the row names the agent."""

import pytest

from mhseals_testing import agent_hook as hook
from mhseals_testing.hook import hosts
from mhseals_testing.agent import channel as ch
from mhseals_testing.agent import config as agent_config
from mhseals_testing.harness.components import Stack
from mhseals_testing.ui import parts
from test_agent_cli import Desk, agent_id, call, lines  # noqa: F401 (agent_id: fixture)
from test_agent_desk import ask, claim, fence_topic, final, rig  # noqa: F401 (fixtures)
from test_components import ME, WATCHED_PARTS, FakeProcesses, FakeUnits, Profile, items
from mhseals_testing.harness.components import components

BOAT_AGENT = hook.DEFAULTS['boat_agent']


class Boatish(Profile):
    components = {
        'odometry': {'title': 'Odom stack'},
        'navigation': {'needs': ['odometry'], 'limits': True},
        'camera': {},
        'objects': {},
    }


@pytest.fixture
def desk(rig):
    rig.app.stack = Stack(items(Boatish), FakeProcesses(), lambda: rig.clock[0], FakeUnits())
    rig.app.ros.nodes = lambda: []
    rig.app.is_armed = False
    return rig


def test_shipped_allowlist_is_the_odometry_stack_ekfs_sensors_and_perception():
    assert agent_config.load()['start'] == ['odometry', 'ekf', 'sensors', 'camera', 'objects']


def test_an_allowed_start_goes_through_the_status_rows_start_path_and_names_the_agent(desk):
    c = claim(desk)
    reply = final(ask(desk, 'start', c, component='odometry')['id'])
    assert reply['ok'] and reply['result'] == {'component': 'odometry', 'action': 'start', 'components': ['odometry']}
    stack = desk.app.stack
    assert [name for name, _ in stack.processes.started] == ['odometry']        # Stack.start's own spawn
    assert stack.by == {'odometry': 'claude-3f2a'}
    assert desk.app.notice == 'Agent claude-3f2a: starting Odom stack on this computer'
    assert ('component', {'action': 'start', 'component': 'odometry', 'host': 'this computer',
                          'by': 'agent', 'who': 'claude-3f2a'}) in desk.app.events
    row = {r['key']: r for r in parts.feed(desk.app)}['odometry']
    assert row['value'].endswith('by agent claude-3f2a') and 'started by agent claude-3f2a' in row['detail']
    # a remote part goes out as its unit, still not as a commander
    stack.units.records['camera'] = {'state': 'off'}
    assert final(ask(desk, 'start', c, component='camera')['id'])['ok']
    assert stack.units.calls == [('start', 'camera', False)]


def test_an_agent_stop_stops_its_part_and_clears_the_attribution(desk):
    c = claim(desk)
    final(ask(desk, 'start', c, component='odometry')['id'])
    reply = final(ask(desk, 'stop_part', c, component='odometry')['id'])
    assert reply['ok'] and reply['result']['components'] == ['odometry']
    assert desk.app.stack.processes.stopped == ['odometry'] and desk.app.stack.by == {}
    assert desk.app.notice == 'Agent claude-3f2a: stopping Odom stack'


def test_refusals_name_the_reason_and_start_nothing(desk):
    stack = desk.app.stack
    req = ch.request('start', 'claude-3f2a', {'component': 'odometry'})          # unclaimed
    desk.desk.tick()
    reply = final(req['id'])
    assert reply['error'] == 'unknown_claim' and 'needs your claim' in reply['hint']
    c = claim(desk)
    reply = final(ask(desk, 'start', c, who='other-1', component='odometry')['id'])
    assert reply['error'] == 'not_holder'
    reply = final(ask(desk, 'start', c, component='thrusters')['id'])
    assert reply['error'] == 'unknown_component' and 'odometry, navigation, camera, objects' in reply['hint']
    reply = final(ask(desk, 'start', c, component='navigation')['id'])             # commander, not listed
    assert reply['error'] == 'not_allowed' and 'agent.yaml start: lists odometry' in reply['hint']
    assert ch.exit_code(reply) == 2 and desk.desk.summary()['attention'].endswith('start refused')   # the panel
    assert stack.processes.started == [] and stack.units.calls == []
    desk.app.settings.agents = 'block'
    assert final(ask(desk, 'start', c, component='odometry')['id'])['error'] == 'blocked'


def test_a_commander_or_driver_is_refused_even_when_agent_yaml_lists_it(desk, monkeypatch):
    real = desk.desk.config.get
    monkeypatch.setattr(desk.desk.config, 'get', lambda: {**real(), 'start': ['navigation', 'odometry']})
    c = claim(desk)
    assert final(ask(desk, 'start', c, component='navigation')['id'])['error'] == 'not_allowed'
    assert desk.app.stack.processes.started == []


def test_stop_is_refused_while_armed_and_when_it_would_stop_a_part_not_the_agents(desk):
    c = claim(desk)
    final(ask(desk, 'start', c, component='odometry')['id'])
    desk.app.stack.processes.live.add('navigation')            # the operator's Nav2 needs odometry
    reply = final(ask(desk, 'stop_part', c, component='odometry')['id'])
    assert reply['error'] == 'not_allowed' and 'also stops Navigation' in reply['hint']
    desk.app.stack.processes.live.discard('navigation')
    desk.app.is_armed = True
    reply = final(ask(desk, 'stop_part', c, component='odometry')['id'])
    assert reply['error'] == 'refused' and reply['hint'].startswith('armed')
    assert desk.app.stack.processes.stopped == []


def test_a_component_the_stack_refuses_comes_back_with_its_reason(desk):
    c = claim(desk)
    final(ask(desk, 'start', c, component='odometry')['id'])
    desk.app.ros.nodes = lambda: ['/ekf_local']
    reply = final(ask(desk, 'start', c, component='odometry')['id'])
    assert reply['error'] == 'refused' and 'already running' in reply['hint']


def test_the_operators_start_is_not_attributed(desk):
    desk.app.stack.by['odometry'] = 'old-agent'
    parts.start(desk.app, 'odometry')
    assert desk.app.stack.by == {} and desk.app.notice == 'Starting Odom stack on this computer'


# --- CLI ------------------------------------------------------------------------------------

class Watched(Profile):
    components = {'odometry': {'title': 'Odom stack'}, 'watchdogs': {'title': 'Safety watchdogs'}}


def test_stop_is_refused_while_the_part_carries_the_watchdogs(desk):
    """On the boat odometry's launch brings the watchdogs (with:): stopping it would silently stop them."""
    stack = Stack(components(Watched, None, (ME, WATCHED_PARTS)), FakeProcesses(), lambda: desk.clock[0], FakeUnits())
    desk.app.stack = stack
    assert stack.items['watchdogs'].carried_by == 'Odom stack'
    c = claim(desk)
    assert final(ask(desk, 'start', c, component='odometry')['id'])['ok']
    reply = final(ask(desk, 'stop_part', c, component='odometry')['id'])
    assert reply['error'] == 'not_allowed' and 'also stops Safety watchdogs' in reply['hint']
    assert stack.processes.stopped == [] and stack.by == {'odometry': 'claude-3f2a'}


def test_cli_start_and_stop_component_send_their_ops_and_plain_stop_stays_the_boat_stop():
    desk = Desk(lambda req: [dict(state='done', result={})])
    assert call(['start', 'odometry', '--claim', 'c4'], desk)[0] == 0
    assert call(['stop', 'ekf', '--claim', 'c4'], desk)[0] == 0
    assert [(r['op'], r['args'], r['claim']) for r in lines('req')] == [
        ('start', {'component': 'odometry'}, 'c4'), ('stop_part', {'component': 'ekf'}, 'c4')]
    code, out = call(['stop', 'ekf'], desk)
    assert code == 2 and out[-1]['error'] == 'bad_args' and 'needs --claim' in out[-1]['hint']
    with pytest.raises(SystemExit):
        call(['start', 'odometry'], desk)                         # --claim is required


# --- hook -----------------------------------------------------------------------------------

@pytest.mark.parametrize('command, verb', [
    ('ssh blastoise-odroid ros2 launch mhseals_nav device.launch.py device:=odometry', 'start COMPONENT --claim ID'),
    ('ssh blastoise-odroid ros2 launch mhseals_nav odom.launch.py', 'start COMPONENT --claim ID'),   # one part's
    ('ssh blastoise-jetson "ros2 launch mhseals_nav camera.launch.py"', 'start COMPONENT --claim ID'),
    ('ssh blastoise-odroid systemctl --user start mhseals-odometry', 'start COMPONENT --claim ID'),
    ('ssh blastoise-jetson systemctl --user stop mhseals-camera', 'stop COMPONENT --claim ID'),
])
def test_the_hook_routes_a_boat_component_start_or_stop_to_the_verb(command, verb):
    reason = hook.deny_reason(*hook.denied(command, hook.DEFAULTS), who='claude-3f2a', config=hook.DEFAULTS)
    assert f'{BOAT_AGENT} {verb} --who claude-3f2a (claim first' in reason


def test_a_local_dev_launch_is_not_blocked_and_the_playbook_names_the_verb(monkeypatch):
    monkeypatch.setattr(hosts, 'hostname', lambda: 'devbox')
    assert hook.denied('ros2 launch mhseals_nav device.launch.py device:=odometry', hook.DEFAULTS) is None
    assert hook.denied('ros2 launch mhseals_nav odom.launch.py', hook.DEFAULTS) is None
    # an mhseals_hardware launch starts the driver: the arming rule blocks it on every host
    assert 'arming' in hook.deny_reason(*hook.denied('ros2 launch --debug mhseals_hardware sensors.launch.py',
                                                     hook.DEFAULTS), who='claude-3f2a', config=hook.DEFAULTS)
    text = hook.start_text('claude-3f2a', config=hook.DEFAULTS)
    assert '`start COMPONENT --claim ID`' in text and '`arm --claim ID' in text


class Needy(Profile):
    components = {
        'odometry': {'title': 'Odom stack'},
        'ekf': {'title': 'EKFs', 'needs': ['odometry']},
        'mapping': {'needs': ['odometry']},
        'objects': {'needs': ['mapping']},
    }


def test_an_agent_start_brings_its_allowed_needs_and_refuses_one_not_the_agents(desk):
    from test_components import PARTS, part
    placed = dict(PARTS, ekf=part('ekf', 'ekf.launch.py', nodes=['ekf_global']),
                  mapping=part('mapping', 'mapping.launch.py', nodes=['rtabmap']))
    desk.app.stack = stack = Stack(components(Needy, None, (ME, placed)), FakeProcesses(),
                                   lambda: desk.clock[0], FakeUnits())
    c = claim(desk)
    reply = final(ask(desk, 'start', c, component='ekf')['id'])
    assert reply['ok'] and reply['result']['components'] == ['odometry', 'ekf']
    assert [name for name, _ in stack.processes.started] == ['odometry']   # EKFs wait for it to run
    assert desk.app.notice == 'Agent claude-3f2a: starting EKFs on this computer after Odom stack'
    reply = final(ask(desk, 'start', c, component='objects')['id'])        # needs Mapping, not an agent's
    assert reply['error'] == 'not_allowed' and 'also starts Mapping' in reply['hint']
    assert [name for name, _ in stack.processes.started] == ['odometry']

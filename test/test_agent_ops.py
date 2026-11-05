"""boat_agent arm and reload through the desk (ui/feeds/agent_ops.py), on test_agent_desk's fakes."""

import io
from types import SimpleNamespace

import pytest

from mhseals_testing.agent import channel as ch, cli as agent_cli
from mhseals_testing.harness.cli import build_parser
from mhseals_testing.harness.watch import Change

from test_agent_desk import claim, final, rig  # noqa: F401 - the rig fixture


@pytest.fixture
def boat(rig, monkeypatch, tmp_path):  # noqa: F811
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path / 'xdg'))
    app = rig.app
    app.is_armed = False
    app.ros.driver_status = {'estop': {'enabled': True, 'latched': False}}

    def agent_arm(who, checks, sim=True):
        app.calls.append(('agent_arm', who, sim))
        app.is_armed = True
        return ''
    app.agent_arm = agent_arm
    reloaded = []
    app.watcher = SimpleNamespace(watches={'hardware': 1, 'settings': 2},
                                  reload=lambda name: reloaded.append(name) or Change(name, True))
    app.announce = lambda change: change
    rig.reloaded = reloaded
    return rig


def send(rig, op, args, claim_id=None, who='claude-3f2a'):
    req = ch.request(op, who, args, claim=claim_id)
    rig.desk.tick()
    return final(req['id'])


def test_arm_counts_down_then_arms_and_logs_the_agent(boat):
    c = claim(boat)
    boat.app.answers = [[]]                     # nobody presses a key: the countdown runs out
    reply = send(boat, 'arm', {'countdown': 0.05}, c)
    assert reply['ok'] and boat.app.armed and ('agent_arm', 'claude-3f2a', False) in boat.app.calls
    states = [f['state'] for p, f in boat.app.events if p == 'agent_arm']
    assert states == ['countdown', 'armed']
    assert boat.desk.recent[-1]['label'] == 'arm' and boat.desk.recent[-1]['ok']


def test_the_operator_cancels_the_arm_countdown(boat):
    c = claim(boat)
    boat.app.answers = [['escape']]
    reply = send(boat, 'arm', {'countdown': 5}, c)
    assert not reply['ok'] and reply['error'] == 'cancelled' and not boat.app.armed
    assert 'cancelled' in boat.app.notice


def test_space_during_the_countdown_cancels_the_arm(boat):
    c = claim(boat)
    boat.app.answers = [['space']]
    reply = send(boat, 'arm', {}, c)
    assert reply['error'] == 'cancelled' and not boat.app.armed
    assert not [call for call in boat.app.calls if call[0] == 'agent_arm']


def test_the_e_stop_and_a_missing_claim_refuse_the_arm(boat):
    boat.app.ros.driver_status = {'estop': {'latched': True, 'cause': 'rc'}}
    c = claim(boat)
    reply = send(boat, 'arm', {'countdown': 0}, c)
    assert reply['error'] == 'refused' and 'e-stop' in reply['hint'] and not boat.app.armed
    assert send(boat, 'arm', {'countdown': 0})['error'] == 'unknown_claim'


def test_an_agent_may_lengthen_the_arm_countdown_but_never_shorten_it(boat):
    c = claim(boat)
    boat.app.answers = [[]]
    reply = send(boat, 'arm', {'countdown': 0}, c)
    started = [f for p, f in boat.app.events if p == 'agent_arm' and f['state'] == 'countdown']
    assert reply['ok'] and started[0]['countdown'] == 5.0     # agent.yaml arm.countdown


def test_reload_config_rereads_the_watched_files(boat):
    assert send(boat, 'reload', {'config': 'hardware'})['ok'] and boat.reloaded == ['hardware']
    assert send(boat, 'reload', {'config': 'nope'})['error'] == 'bad_args'
    assert send(boat, 'reload', {'config': ''})['result']['files'][-1]['name'] == 'settings'


def test_the_cli_parses_the_new_verbs():
    parser = build_parser()
    args = parser.parse_args(['arm', '--claim', 'c1', '--countdown', '3', '--who', 'x'])
    assert args.command == 'arm' and args.countdown == 3.0
    assert parser.parse_args(['reload', 'config', 'hardware', '--who', 'x']).name == 'hardware'
    assert {'arm', 'reload', 'set'} <= set(agent_cli.VERBS)


@pytest.mark.parametrize('words, verb, says', [(['reload', 'odometry'], 'reload', '--claim')])
def test_cli_reload_checks_their_words_before_sending(words, verb, says):
    out = io.StringIO()
    agent = agent_cli.Agent(build_parser().parse_args([*words, '--who', 'x']), registry=SimpleNamespace(),
                            out=out, settings=agent_cli.DEFAULTS)
    assert getattr(agent, verb)() == ch.EXIT_CODES['bad_args'] and says in out.getvalue()


pytestmark = pytest.mark.smoke   # tools/smoke


def test_an_agent_stop_during_the_countdown_cancels_the_arm(boat):
    """Sim rehearsal 11-04: boat_agent stop during the banner stopped the boat, then the countdown armed it."""
    c = claim(boat)
    boat.app.answers = [[lambda: ch.request('stop', 'claude-3f2a', {})]]
    reply = send(boat, 'arm', {'countdown': 3}, c)
    assert not reply['ok'] and reply['error'] == 'cancelled' and 'stopped by agent claude-3f2a' in reply['hint']
    assert not boat.app.armed and not [call for call in boat.app.calls if call[0] == 'agent_arm']


def test_a_run_this_tui_can_never_do_is_refused_before_any_ask(boat):
    """Sim rehearsal 11-04: run f in a velocity TUI ended ok/done with no result; it is refused, naming the fix."""
    from test_agent_desk import ask
    c = claim(boat)
    boat.app.answers = []                                   # no confirm popup may open
    reply = final(ask(boat, 'run', c, key='f')['id'])
    assert not reply['ok'] and reply['error'] == 'refused'
    assert 'boat_agent set command_mode=effort' in reply['hint'] and 'no restart' in reply['hint']
    assert not boat.app.armed and not [call for call in boat.app.calls if call[0] == 'agent_arm']


def test_the_arm_countdown_default_is_agent_configs_one_constant():
    from mhseals_testing.agent import config as agent_config
    shipped = agent_config.load()
    arm = {k: v for k, v in shipped.items() if k != 'arm'}
    assert agent_config.validate(arm)['arm'] == {'countdown': agent_config.ARM_COUNTDOWN}

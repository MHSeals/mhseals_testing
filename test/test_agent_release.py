"""boat_agent release HOLD --why, --why on acting verbs, note --doing, the quiet idle lapse (owner decision 11-01)."""

import io
import json
from types import SimpleNamespace

import pytest

from mhseals_testing.agent import channel as ch, cli as agent_cli, grants as agent_grants
from mhseals_testing.harness.cli import build_parser
from mhseals_testing.ros.authority import Arbiter

from test_agent_desk import claim, final, rig  # noqa: F401 - the rig fixture

NOW = 1_800_000_000.0


def test_holds_are_lease_watchdog_node_operator_or_estop():
    assert agent_grants.kind({'cause': 'lease', 'by': 'tui'}) == 'lease'
    assert agent_grants.kind({'cause': 'watchdog', 'by': 'tui'}) == 'watchdog'
    assert agent_grants.kind({'cause': 'geofence', 'by': 'fence_watchdog'}) == 'node'
    for hold in ({'cause': 'keys', 'by': 'tui'}, {'cause': 'agent_stop', 'by': 'agent x'}, {'cause': 'revoke'},
                 {'cause': 'keys', 'by': 'view x'}, {'cause': 'odd', 'by': 'tui'}):
        assert agent_grants.kind(hold) == 'operator', hold
    assert agent_grants.kind({'cause': 'estop', 'by': 'thruster_pwm_node'}) == 'estop'


def test_a_grant_names_the_agent_the_holds_and_may_end(tmp_path):
    path = tmp_path / 'agent_grants.yaml'
    assert agent_grants.load(path) == ([], '')
    path.write_text("grants:\n  - {who: claude-1, holds: [keys], until: '2030-01-01 00:00', why: bug}\n"
                    "  - {who: '*', holds: ['*'], until: '2020-01-01 00:00'}\n")
    grants, error = agent_grants.load(path)
    assert not error and len(grants) == 2
    assert agent_grants.granted(grants, 'claude-1', 'keys', NOW)['why'] == 'bug'
    assert agent_grants.granted(grants, 'claude-2', 'keys', NOW) is None         # the wildcard grant ended
    assert agent_grants.granted(grants, 'claude-1', 'revoke', NOW) is None
    path.write_text('grants: {who: x}\n')
    assert agent_grants.load(path)[0] == [] and agent_grants.load(path)[1]


def verdict(holds, name='all', grants=(), cleared=(True, '')):
    return agent_grants.verdict(holds, name, 'claude-1', list(grants), lambda hold: cleared, NOW)


def test_every_hold_must_be_the_agents_to_release_now():
    lease, keys = {'cause': 'lease', 'by': 'tui'}, {'cause': 'keys', 'by': 'tui'}
    fence = {'cause': 'geofence', 'by': 'fence_watchdog'}
    assert verdict([lease]) is None and verdict([lease], 'lease') is None
    assert verdict([fence], cleared=(False, 'outside'))[0] == 'refused'
    assert verdict([fence]) is None
    assert verdict([lease, keys])[0] == 'not_allowed' and 'keys' in verdict([lease, keys])[1]
    grant = {'who': 'claude-1', 'holds': ['keys'], 'until': None, 'why': ''}
    assert verdict([lease, keys], grants=[grant]) is None
    assert verdict([lease], 'estop')[0] == 'not_allowed' and 'never' in verdict([lease], 'estop')[1]
    assert verdict([{'cause': 'estop', 'by': 'x'}], grants=[dict(grant, holds=['*'])])[0] == 'not_allowed'
    assert verdict([], 'lease')[0] == 'refused' and verdict([lease], 'keys')[0] == 'bad_args'


@pytest.fixture
def held(rig, monkeypatch, tmp_path):  # noqa: F811
    monkeypatch.setenv('HOME', str(tmp_path))
    app, released = rig.app, []
    app.plant.release_hold = lambda by='tui': released.append(by) or True
    app.release_refusal = lambda: ''
    app.watchdog_reasons = lambda: ('', '')
    app.ros.watch_beats = lambda nodes: None
    app.ros.beats = lambda: {'/fence_watchdog': rig.monotonic}
    app.ros.beat_status = lambda: {'/fence_watchdog': rig.fence}
    app.ros.nodes = lambda: ['/fence_watchdog']
    rig.released, rig.fence, rig.grants = released, {'ok': False, 'reason': 'outside'}, tmp_path / 'grants.yaml'
    import time
    rig.monotonic = time.monotonic()
    rig.desk.config.get = (lambda get: lambda: dict(get(), grants=str(rig.grants)))(rig.desk.config.get)

    def holds(*causes):
        rig.app.arbiter.on_mux({'input': 'hold', 'holds': [{'cause': c, 'by': b} for c, b in causes]})
    rig.holds = holds
    return rig


def release(rig, hold, why='the lease lapsed while I was away', who='claude-3f2a'):
    req = ch.request('release_hold', who, {'hold': hold}, why=why)
    rig.desk.tick()
    return final(req['id'])


def test_a_lease_hold_is_released_at_once_and_names_the_agent(held):
    held.holds(('lease', 'tui'))
    reply = release(held, 'lease')
    assert reply['ok'] and reply['result']['released'] == ['lease'] and held.released == ['agent claude-3f2a']
    assert held.desk.summary()['attention'] == 'claude-3f2a released a hold'
    assert held.desk.recent[-1]['why'] == 'the lease lapsed while I was away'
    assert ('agent_release', {'who': 'claude-3f2a', 'holds': ['lease'], 'why': 'the lease lapsed while I was away'}) \
        in held.app.events


def test_a_watchdog_hold_waits_for_its_fault_to_clear(held):
    held.holds(('geofence', 'fence_watchdog'))
    reply = release(held, 'geofence')
    assert reply['error'] == 'refused' and 'outside' in reply['hint'] and not held.released
    held.fence = {'ok': True, 'reason': ''}
    assert release(held, 'geofence')['ok'] and held.released


def test_an_operator_hold_needs_a_grant_and_the_e_stop_never_goes(held):
    held.holds(('keys', 'tui'))
    reply = release(held, 'keys')
    assert reply['error'] == 'not_allowed' and 'grant' in reply['hint'] and not held.released
    held.grants.write_text('grants:\n  - {who: claude-3f2a, holds: [keys], why: known bug}\n')
    assert release(held, 'keys')['ok'] and held.released
    assert release(held, 'estop')['error'] == 'not_allowed'


def test_disarmed_a_stale_mux_releases_from_its_saved_hold_record(held, tmp_path):
    from mhseals_hardware import hold_store
    path = tmp_path / 'command_mux_holds.json'
    held.app.args.hardware_config = {'command_mux': {'state_file': str(path)}}
    held.app.release_refusal = lambda: 'Release refused: mux state is stale or unknown'
    held.app.is_armed = False
    held.clock[0] += 60                                   # the mux stopped at disarm: its state went stale
    fence = {'cause': 'geofence', 'by': 'fence_watchdog', 'since': 5.0, 'token': None}
    keys = {'cause': 'keys', 'by': 'tui', 'since': 6.0, 'token': None}
    hold_store.save(path, {'geofence': fence})
    reply = release(held, 'geofence')                     # the fault must still have cleared
    assert reply['error'] == 'refused' and 'outside' in reply['hint'] and hold_store.load(path) == {'geofence': fence}
    held.fence = {'ok': True, 'reason': ''}
    reply = release(held, 'geofence')
    assert reply['ok'] and reply['result']['released'] == ['geofence'] and hold_store.load(path) == {}
    assert not held.released                              # no mux to publish to: the record is the release
    hold_store.save(path, {'geofence': fence, 'keys': keys})
    reply = release(held, 'geofence')                     # an operator hold still needs a grant
    assert reply['error'] == 'not_allowed' and 'keys' in reply['hint'] and len(hold_store.load(path)) == 2
    held.app.arming = SimpleNamespace(remote=object(), config_path=None)   # a remote boat's record is not here
    hold_store.save(path, {'geofence': fence})
    assert release(held, 'geofence')['error'] == 'refused' and hold_store.load(path) == {'geofence': fence}
    held.app.is_armed = True                              # armed, a stale mux still refuses
    held.holds(('lease', 'tui'))
    held.clock[0] += 60
    assert release(held, 'lease')['hint'] == 'Release refused: mux state is stale or unknown' and not held.released


def test_a_release_needs_why_and_refuses_while_agents_are_blocked(held):
    held.holds(('lease', 'tui'))
    assert release(held, 'lease', why='')['error'] == 'bad_args'
    held.app.settings.agents = 'block'
    assert release(held, 'lease')['error'] == 'blocked' and not held.released


# --- the CLI ---------------------------------------------------------------------------------------------------

def cli(argv, monkeypatch, owner=True):
    monkeypatch.setenv('BOAT_AGENT', 'claude-test')
    args = build_parser().parse_args(argv)
    out = io.StringIO()
    registry = SimpleNamespace(owner=lambda: {'pid': 1} if owner else None, alive=lambda pid: True,
                               state=lambda: {})
    agent = agent_cli.Agent(args, registry=registry, out=out, settings=agent_cli.DEFAULTS,
                            sleep=lambda s: None)
    sent = []
    agent.send = lambda op, args=None, claim=None, dry=False, wait=None: sent.append((op, args, claim)) or 0
    args.who, hint = agent_cli.who_of(args)
    if not hint and agent_cli.needs_why(args):
        return 2, sent, out
    return getattr(agent, args.command)(), sent, out


@pytest.mark.parametrize('argv', [['claim'], ['run', 'square', '--claim', 'c1'], ['set', 'x=1', '--claim', 'c1'],
                                  ['fcu', 'set', 'SR0_EXTRA1', '5', '--claim', 'c1'], ['release', 'lease']])
def test_acting_verbs_need_why(argv, monkeypatch):
    code, sent, _ = cli(argv, monkeypatch)
    assert code == 2 and not sent
    code, sent, _ = cli(argv + ['--why', 'tuning'], monkeypatch)
    assert code == 0 and sent


@pytest.mark.parametrize('argv', [['stop'], ['status'], ['fcu', 'get', 'SR0_EXTRA1'], ['note', 'x'],
                                  ['ask', 'ok?']])
def test_reads_notes_and_stop_need_no_why(argv, monkeypatch):
    args = build_parser().parse_args(argv)
    assert not agent_cli.needs_why(args)


def test_release_with_a_hold_releases_it_and_without_ends_the_claim(monkeypatch):
    code, sent, _ = cli(['release', 'lease', '--why', 'x'], monkeypatch)
    assert code == 0 and sent == [('release_hold', {'hold': 'lease'}, None)]
    code, sent, _ = cli(['release', '--claim', 'c1', '--why', 'done'], monkeypatch)
    assert sent == [('release', None, 'c1')]
    code, sent, out = cli(['release', '--why', 'x'], monkeypatch)
    assert code == 2 and not sent and 'HOLD' in out.getvalue()


def test_the_why_rides_on_the_request(tmp_path, monkeypatch):
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path))
    req = ch.request('claim', 'claude-1', {'reason': 'r'}, why='tuning the controller')
    assert req['why'] == 'tuning the controller' and req['args'] == {'reason': 'r'}
    assert 'why' not in ch.request('status', 'claude-1')


def test_note_doing_sets_the_doing_line(tmp_path, monkeypatch):
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path))
    code, _, out = cli(['note', '--doing', 'tuning yaw'], monkeypatch)
    line = json.loads(out.getvalue())
    assert code == 0 and line['phase'] == 'doing' and line['cmd'] == 'tuning yaw'


def test_fcu_get_without_a_tui_reads_the_newest_snapshot(tmp_path, monkeypatch):
    store = tmp_path / 'store'
    store.mkdir()
    (store / 'fcu_blastoise_20261101-100000.param').write_text('# x\nSR0_EXTRA1,10\nSCHED_LOOP_RATE,200\n')
    monkeypatch.setattr(agent_cli, 'DEFAULTS', dict(agent_cli.DEFAULTS, fcu={'store': str(store)}))
    code, sent, out = cli(['fcu', 'get', 'SR0_EXTRA1'], monkeypatch, owner=False)
    doc = json.loads(out.getvalue())
    assert code == 0 and not sent and doc['source'] == 'snapshot' and doc['params'] == {'SR0_EXTRA1': 10.0}


# --- the idle lease --------------------------------------------------------------------------------------------

@pytest.mark.parametrize('quiet, hold', [(True, False), (False, True)])
def test_an_idle_lapse_while_disarmed_and_neutral_leaves_no_hold(quiet, hold):
    clock, holds = [100.0], []
    config = SimpleNamespace(get=lambda: {'lease': {'idle_s': 600, 'motion_s': 10, 'max_claim_s': 3600}})
    arbiter = Arbiter(lambda record: None, lambda phase, **f: None, tui_owner=True, clock=lambda: clock[0],
                      hold=lambda cause, by: holds.append(cause), config=config, quiet=lambda: quiet)
    ack = arbiter.handle({'op': 'claim', 'who': 'claude-1', 'request': 'r', 'lease_s': 600})
    clock[0] += 601
    arbiter.tick()
    assert (holds == ['lease']) == hold
    assert arbiter.ended[ack['id']] == ('lapsed' if quiet else 'expired')
    assert ch.exit_code({'ok': False, 'error': 'lapsed'}) == 3


def test_set_a_live_nav2_key_goes_through_nav2_live(rig):  # noqa: F811
    applied = []

    class Live:
        claim = None

        def apply(self, values):
            applied.append((values, self.claim))
            ok = values.get('controller_server.FollowPath.max_vel_x') != 9.0
            return {'status': 'verified' if ok else 'refused', 'rows': [], 'reasons': [] if ok else ['over the cap']}

        def readback(self, values):
            return {k: 1.2 for k in values}
    rig.app.live_nav2 = Live
    rig.app.live_nav2 = lambda live=Live(): live
    c = claim(rig)
    req = ch.request('set', 'claude-3f2a', {'settings': {'nav2.controller_server.FollowPath.max_vel_x': '1.2'}},
                     claim=c, why='overshoot at 1.5')
    rig.desk.tick()
    reply = final(req['id'])
    assert reply['ok'] and applied == [({'controller_server.FollowPath.max_vel_x': 1.2}, c)]
    assert reply['result']['nav2']['rows'] == [{'key': 'controller_server.FollowPath.max_vel_x', 'requested': 1.2,
                                                'readback': 1.2, 'unchanged': True}]   # read back, even unchanged
    req = ch.request('set', 'claude-3f2a', {'settings': {'nav2.controller_server.FollowPath.max_vel_x': '9'}},
                     claim=c, why='x')
    rig.desk.tick()
    reply = final(req['id'])
    assert reply['error'] == 'refused' and 'over the cap' in reply['hint']


# --- the agent panel ---------------------------------------------------------------------------------------------

def test_the_panel_shows_doing_why_and_the_header_one_attention_indicator(rig):  # noqa: F811
    from mhseals_testing.ui import chrome
    from mhseals_testing.ui.panels import agent_log
    ch.note('claude-3f2a', 'doing', 'tuning yaw on the circle course')
    req = ch.request('claim', 'claude-3f2a', {'reason': 'r'}, why='yaw overshoots 20 degrees')
    rig.desk.tick()
    summary = rig.desk.summary()
    assert summary['doing']['claude-3f2a']['text'] == 'tuning yaw on the circle course'
    whats = [row['what'] for row in agent_log.fold(summary)]
    assert 'doing: tuning yaw on the circle course' in whats
    assert any('claim' in w and 'yaw overshoots 20 degrees' in w for w in whats), whats
    assert not [n for n in summary['notes'] if n['phase'] == 'doing']          # never a shell row
    assert final(req['id'])['ok']
    item = chrome.working_item({'agent_desk': dict(summary, attention='claude-3f2a asks')})
    assert 'agent wants attention' in item.plain


pytestmark = pytest.mark.smoke   # tools/smoke

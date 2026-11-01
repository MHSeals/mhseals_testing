"""The operator-declared boat state (harness/operator_state.py): normal, manual, battery swap, power cycle.

Agents pause (changing verbs exit 7), leases suspend instead of lapsing (no lease hold), a battery swap or power cycle
survives the reboot as ``restarted_after``, and boat_agent names the last state it saw when the boat is unreachable.
"""

import io
import json
import subprocess
from types import MethodType

from mhseals_testing.harness import agent_channel as ch, agent_relay, operator_state as op
from mhseals_testing.ui import chrome, status_file
from mhseals_testing.ui.app import App as RealApp

from test_agent_cli import Reg, call, agent_id  # noqa: F401 (agent_id: fixture)
from test_agent_desk import ask, claim, fence_topic, final, replies, rig  # noqa: F401 (fixtures)

T0 = 1_700_000_000 * 10 ** 9


def operator(rig):
    """The real App's operator methods on the desk rig's fake app."""
    for name in ('set_operator_state', 'operator_sync', 'operator_start'):
        setattr(rig.app, name, MethodType(getattr(RealApp, name), rig.app))
    return rig.app


# --- the record ---------------------------------------------------------------------------------

def test_a_declared_state_saves_reads_back_and_says_since_and_eta():
    record = op.save(op.change('battery_swap', eta_s=600, reason='pack 2', by='operator', now_ns=T0))
    loaded = op.load()
    assert loaded['state'] == 'battery_swap' and loaded['eta_ns'] == T0 + 600 * 10 ** 9 and op.paused(loaded)
    assert op.text(record) == f'operator: battery swap since {op.clock(T0)} (ETA {op.clock(T0 + 600e9)}) — pack 2'
    assert op.summary(loaded)['text'] == op.text(record) and op.current()['state'] == 'battery_swap'
    assert op.text(op.change('normal', eta_s=600)) == 'operator: normal' and not op.change('normal', eta_s=9)['eta_ns']


def test_no_file_or_a_broken_one_is_normal(tmp_path):
    assert op.load()['state'] == 'normal' and not op.paused(op.current())
    op.path().parent.mkdir(parents=True, exist_ok=True)
    op.path().write_text('{"state": "on fire"}')
    assert op.load()['state'] == 'normal'


def test_a_swap_from_an_earlier_boot_is_the_restart_marker_for_one_session():
    op.save(op.change('power_cycle', by='operator', now_ns=T0))
    record = op.start(boot='new-boot')
    assert record['state'] == 'normal' and record['restarted_after']['state'] == 'power_cycle'
    assert record['restarted_after']['since_ns'] == T0 and 'restarted after power cycle' in op.text(record)
    assert op.load()['restarted_after']['state'] == 'power_cycle'          # status reads it from the file
    assert op.start(boot='new-boot')['restarted_after'] is None            # the next TUI start: gone
    assert op.change('manual', prev=record)['restarted_after'] is None


def test_a_swap_on_this_boot_stays_and_manual_ends_with_its_tui():
    op.save(op.change('battery_swap', now_ns=T0))
    boot = op.load()['boot']
    assert op.start(boot=boot)['state'] == 'battery_swap'                   # a TUI restart, the boat stayed up
    op.save(op.change('manual'))
    assert op.start(boot=boot)['state'] == 'normal' and op.load()['restarted_after'] is None


# --- the desk: verbs pause, leases suspend, never a lease hold -----------------------------------

def test_changing_verbs_exit_7_with_since_and_eta_and_reads_keep_working(rig):
    app = operator(rig)
    c = claim(rig)
    app.set_operator_state('battery_swap', eta_s=600)
    for verb, args in (('run', {'key': 'square'}), ('goal', {'x': 1.0, 'y': 2.0}), ('start', {'component': 'ekf'}),
                       ('fcu_set', {'changes': {'SR0_EXTRA1': 50}}), ('claim', {'reason': 'x'})):
        reply = final(ask(rig, verb, c, **args)['id'])
        assert reply['error'] == 'paused' and ch.exit_code(reply) == 7, verb
        assert reply['hint'].startswith('operator: battery swap since ') and '(ETA ' in reply['hint']
        assert 'paused' in reply['hint'] and reply['result']['operator']['state'] == 'battery_swap'
    req = ch.request('mark', 'claude-3f2a', {'note': 'waiting'})
    rig.desk.tick()
    assert final(req['id'])['ok']                                           # not a changing verb
    assert ('operator_state', 'battery_swap') in [(p, f.get('state')) for p, f in app.events]
    assert status_file._operator(app, rig.desk)['state'] == 'battery_swap'
    req = ch.request('release', 'claude-3f2a', claim=c)
    rig.desk.tick()
    assert final(req['id'])['ok']                                           # release always works


def test_no_lease_hold_while_manual_and_a_kept_claim_resumes_on_normal(rig):
    app = operator(rig)
    c = claim(rig)
    app.set_operator_state('manual', reason='manual control (m)')
    for _ in range(5):                                                      # 5 x 100 s: 4 idle leases
        rig.clock[0] += 100
        app.arbiter.tick()
        ch.renew('claude-3f2a', c, None)                                    # follow --claim keeps it
        rig.desk.tick()
    assert not [call for call in app.calls if call[0] == 'hold'] and app.arbiter.table.get(c)['state'] == 'active'
    assert final(ask(rig, 'run', c, key='square')['id'])['error'] == 'paused'
    app.set_operator_state('normal')
    assert app.arbiter.table.get(c)['state'] == 'active' and app.arbiter.paused is None
    rig.clock[0] += 100                                                     # the resumed lease is fresh
    app.arbiter.tick()
    assert app.arbiter.table.get(c) is not None and not [x for x in app.calls if x[0] == 'hold']
    rig.app.settings.agent_motion = 'allow'
    assert final(ask(rig, 'run', c, key='square')['id'])['ok']


def test_a_claim_not_kept_alive_while_paused_ends_cleanly_with_no_hold(rig):
    app = operator(rig)
    c = claim(rig)
    app.set_operator_state('power_cycle')
    rig.clock[0] += 400
    app.arbiter.tick()
    assert app.arbiter.table.get(c) is not None                            # suspended, not lapsed
    app.set_operator_state('normal')
    assert app.arbiter.table.get(c) is None and not [x for x in app.calls if x[0] == 'hold']
    assert 'ended: not heard from while paused (no hold)' in app.notice
    reply = final(ask(rig, 'run', c, key='square')['id'])
    assert reply['error'] == 'paused_end' and ch.exit_code(reply) == 3 and 'no hold was made' in reply['hint']


def test_an_agent_run_in_flight_ends_paused_when_the_operator_declares_a_swap(rig):
    app = operator(rig)
    c = claim(rig)
    app.settings.agent_motion = 'allow'
    app.on_step = {1: lambda: app.set_operator_state('battery_swap')}
    reply = final(ask(rig, 'run', c, key='square')['id'])
    assert reply['error'] == 'paused' and ch.exit_code(reply) == 7 and 'battery swap' in reply['hint']
    assert ('hold', 'lease') not in app.calls and app.arbiter.table.get(c)['state'] == 'active'


def test_an_unknown_claim_after_a_restart_says_why(rig):
    op.save(dict(op.change('normal'), restarted_after={'state': 'battery_swap', 'since_ns': T0}))
    reply = final(ask(rig, 'run', 'c41', key='square')['id'])
    assert reply['error'] == 'unknown_claim' and 'not in this TUI session' in reply['hint']
    assert 'restarted after battery swap' in reply['hint'] and 'FCU params and RTK' in reply['hint']


def test_a_power_cycle_from_an_earlier_boot_is_journaled_by_the_new_tui(rig, monkeypatch):
    app = operator(rig)
    op.save(op.change('power_cycle', now_ns=T0))
    monkeypatch.setattr(op, 'boot_id', lambda source=None: 'after-reboot')
    app.operator_start()
    assert app.notice.startswith(f'Restarted after power cycle at {op.clock(T0)}')
    assert [f for p, f in app.events if p == 'operator_restart'][0]['restarted_after']['state'] == 'power_cycle'
    assert status_file._operator(app, rig.desk)['restarted_after']['state'] == 'power_cycle'


# --- boat_agent: no TUI, unreachable ------------------------------------------------------------

def test_no_tui_names_the_last_operator_state(agent_id):
    op.save(op.change('battery_swap', eta_s=300, now_ns=T0))
    code, out = call(['claim', '--reason', 'x'], registry=Reg(pid=None))
    assert code == 4 and out[0]['error'] == 'no_owner'
    assert out[0]['hint'].startswith('TUI not running: last operator: battery swap since')
    assert out[0]['result']['operator']['state'] == 'battery_swap'
    code, out = call(['status'], registry=Reg(pid=None))
    assert out[0]['tui'] == 'not running' and out[0]['operator']['state'] == 'battery_swap'


class Proc:
    def __init__(self, out, err, code):
        self.stdout, self.stderr, self.code = io.StringIO(out), io.StringIO(err), code

    def wait(self):
        return self.code


def test_the_relay_caches_the_last_state_and_says_unreachable_with_it(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path / 'cache'))
    sent = []

    def popen(argv, **kw):
        sent.append(argv)
        return Proc(json.dumps({'ok': False, 'error': 'paused', 'result': {
            'operator': op.summary(op.change('battery_swap', eta_s=600, now_ns=T0))}}) + '\n', '', 7)
    out = io.StringIO()
    assert agent_relay.run('blastoise-odroid', ['ask', 'two words'], out=out, popen=popen) == 7
    assert json.loads(out.getvalue())['error'] == 'paused'                # streamed through unchanged
    assert sent[0][-2] == 'blastoise-odroid' and sent[0][-1].startswith('BOAT_AGENT_TARGET=local ~/astro_dock/')
    assert sent[0][-1].endswith("ask 'two words'")
    out = io.StringIO()
    code = agent_relay.run('blastoise-odroid', ['status'], out=out,
                           popen=lambda argv, **kw: Proc('', 'ssh: connect to host: No route to host\n', 255))
    reply = json.loads(out.getvalue())
    assert code == 4 and reply['error'] == 'unreachable' and ch.exit_code(reply) == 4
    assert reply['hint'].startswith('TUI unreachable on blastoise-odroid (ssh: connect to host: No route to host): '
                                    'last operator state battery swap since')
    assert 'expected down' in reply['hint'] and reply['operator']['state'] == 'battery_swap'


def test_unreachable_with_nothing_cached_says_so(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path / 'cache'))
    reply = agent_relay.unreachable('blastoise-odroid', 'timeout')
    assert 'no operator state seen yet' in reply['hint'] and 'link is down' in reply['hint']


def test_tools_boat_agent_relays_only_to_another_host(tmp_path):
    script = (status_file.__file__.rsplit('/mhseals_testing/', 1)[0]) + '/tools/boat_agent'
    fake = tmp_path / 'bin'
    fake.mkdir()
    for name in ('ssh', 'docker'):
        (fake / name).write_text(f'#!/bin/sh\necho {name} "$@"\n')
        (fake / name).chmod(0o755)
    env = {'PATH': f'{fake}:/usr/bin:/bin', 'HOME': str(tmp_path), 'XDG_CACHE_HOME': str(tmp_path)}
    local = subprocess.run([script, 'status'], env=env, capture_output=True, text=True).stdout
    assert local.startswith('docker exec -i roboboat_dev')
    remote = subprocess.run([script, 'status'], env=dict(env, BOAT_AGENT_TARGET='blastoise'),
                            capture_output=True, text=True).stdout
    assert remote.startswith('ssh -o ConnectTimeout=8 -o BatchMode=yes blastoise-odroid BOAT_AGENT_TARGET=local')


# --- what the operator sees ---------------------------------------------------------------------

def test_the_header_shows_the_state_chip_and_the_banner_outranks_the_agent_banner():
    view = {'authority': ('hold', 'HOLD'), 'armed': False, 'operator': op.change('battery_swap', eta_s=600)}
    assert 'BATTERY SWAP' in ''.join(t.plain for t in chrome.chips(view))
    assert 'BATTERY SWAP' not in ''.join(t.plain for t in chrome.chips(dict(view, operator=op.normal())))
    line = chrome.operator_banner(view['operator'], 100).plain
    assert 'battery swap since' in line and 'agents paused, leases suspended' in line and line.rstrip().endswith('S state')
    assert len(line) == 100


def test_exit_codes_for_pause_and_reach():
    assert [ch.exit_code({'ok': False, 'error': e}) for e in ('paused', 'paused_end', 'unreachable')] == [7, 3, 4]

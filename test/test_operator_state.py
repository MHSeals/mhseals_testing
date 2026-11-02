"""The operator-declared boat state (harness/operator_state.py): normal, manual, battery swap, power cycle.

Agents pause (changing verbs exit 7), leases suspend instead of lapsing (no lease hold), a battery swap or power cycle
survives the reboot as ``restarted_after``, and boat_agent names the last state it saw when the boat is unreachable.
"""

import io
import json
import subprocess
import os
import time
from types import MethodType, SimpleNamespace

from mhseals_testing.harness import operator_state as op
from mhseals_testing.agent import channel as ch, relay as agent_relay
from mhseals_testing.ros.authority import Arbiter
from mhseals_testing.ui import authority_chip, chrome, layout, status_file
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
    assert op.text(record) == f'operator: battery swap since {op.clock(T0)} {time.strftime("%Z")} (ETA {op.clock(T0 + 600e9)}) — pack 2'
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
    rig.clock[0] += app.arbiter.table.idle_s + 100                         # past the idle lease
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
    assert sent[0][-3:-1] == ['--', 'blastoise-odroid']
    assert sent[0][-1].startswith('BOAT_AGENT_TARGET=local ~/astro_dock/')
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
    assert remote.startswith('ssh -o ConnectTimeout=8 -o BatchMode=yes -- blastoise-odroid BOAT_AGENT_TARGET=local')


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


# --- review fixes -------------------------------------------------------------------------------

def test_a_run_queued_before_the_pause_is_refused_paused_when_it_would_start(rig):
    app = operator(rig)
    c = claim(rig)
    app.settings.agent_motion = 'allow'
    app.modal = 'Operator state'                                            # the S menu is open: the run queues
    req = ch.request('run', 'claude-3f2a', {'key': 'square'}, claim=c)
    rig.desk.tick()
    assert [r['state'] for r in replies(req['id'])] == ['queued']
    app.set_operator_state('manual')                                        # the operator picks Manual
    app.modal = None
    rig.desk.tick()
    reply = final(req['id'])
    assert reply['error'] == 'paused' and ch.exit_code(reply) == 7 and reply['result']['operator']['state'] == 'manual'
    assert [r['state'] for r in replies(req['id'])] == ['queued', 'done'] and rig.desk.act is None


def test_a_claim_over_the_claim_topic_is_refused_while_paused():
    arbiter = Arbiter(lambda record: None, lambda *a, **k: None, tui_owner=True)
    arbiter.pause(op.change('battery_swap', eta_s=600))
    ack = arbiter.handle({'op': 'claim', 'request': 'r1', 'who': 'claude-3f2a'})
    assert not ack['ok'] and ack['error'] == 'paused' and 'battery swap' in ack['hint'] and not arbiter.table.claims
    arbiter.resume()
    assert arbiter.handle({'op': 'claim', 'request': 'r2', 'who': 'claude-3f2a'})['ok']


def test_the_operator_banner_names_a_held_claim_and_stopped_still_wins():
    claims = [{'id': 'c7', 'who': 'claude-3f2a', 'owner': 'agent', 'state': 'active', 'since_ns': T0,
               'seen_ns': T0, 'lease_s': 120}]
    agent = authority_chip.banner({'claims': claims}, T0 / 1e9)
    line = chrome.operator_banner(op.change('manual'), 120, agent).plain
    assert 'claude-3f2a · c7 suspended' in line and 'manual since' in line and line.rstrip().endswith('S state')
    assert len(line) == 120
    import test_dashboard
    profile = test_dashboard.load_profile('boat')
    snapshot = test_dashboard.monitor().snapshot(profile.required_tf)

    def row(**fields):
        view = test_dashboard.view(profile, armed=False, operator=op.change('manual'), agent=agent, **fields)
        return test_dashboard.text(layout.header(snapshot, view, 120), 120, 4).splitlines()[1]
    assert 'claude-3f2a · c7 suspended' in row(authority=('hold', 'HOLD'))
    stopped = row(authority=('stop', 'STOP'))
    assert 'STOPPED' in stopped and 'claude-3f2a' in stopped and 'suspended' not in stopped


def test_the_relay_takes_only_a_host_name(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path / 'cache'))
    for bad in ('-oProxyCommand=sh', '../x', 'a b', 'host;id', '', 'h\u00f6st'):
        out = io.StringIO()
        assert agent_relay.run(bad, ['status'], out=out, popen=lambda *a, **k: 1 / 0) == 2, bad
        assert json.loads(out.getvalue())['error'] == 'bad_args'
    assert agent_relay.HOST.fullmatch('roboboat@blastoise-odroid.local') and agent_relay.HOST.fullmatch('10.0.0.5')


def test_a_corrupt_cache_answers_unreachable_with_no_last_state(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path / 'cache'))
    path = agent_relay.cache_path('blastoise-odroid')
    path.parent.mkdir(parents=True)
    good = op.summary(op.change('battery_swap', now_ns=T0))
    for doc in ('{', '[1]', {'operator': 'x', 'seen_ns': T0}, {'operator': dict(good, since_ns='11:30'), 'seen_ns': T0},
                {'operator': good, 'seen_ns': 'now'}, {'operator': dict(good, since_ns=10 ** 30), 'seen_ns': T0},
                {'operator': dict(good, state='normal', restarted_after='x'), 'seen_ns': T0},
                {'operator': dict(good, reason=['x']), 'seen_ns': T0}):
        path.write_text(doc if isinstance(doc, str) else json.dumps(doc))
        reply = agent_relay.unreachable('blastoise-odroid', 'timeout')
        assert reply['error'] == 'unreachable' and reply['operator'] is None and reply['seen_ns'] is None, doc
        assert 'no operator state seen yet' in reply['hint']
    path.write_text(json.dumps({'operator': good, 'seen_ns': T0}))
    assert agent_relay.unreachable('blastoise-odroid', 'timeout')['operator']['state'] == 'battery_swap'


def test_two_saves_in_one_clock_tick_are_both_seen():
    op.save(op.change('manual', now_ns=T0))
    assert op.current()['state'] == 'manual'
    stamp = os.stat(op.path()).st_mtime_ns
    op.save(op.change('normal', now_ns=T0))
    os.utime(op.path(), ns=(stamp, stamp))                                  # the same tick
    assert op.current()['state'] == 'normal'


def test_status_names_a_part_start_under_way_while_paused():
    items = {'ekf': SimpleNamespace(title='EKF'), 'sensors': SimpleNamespace(title='Sensors')}
    stack = SimpleNamespace(items=items, state=lambda key, nodes: ('starting' if key == 'ekf' else 'running', ''))
    app = SimpleNamespace(stack=stack, ros=SimpleNamespace(nodes=lambda: []))
    op.save(op.change('manual'))
    found = status_file._operator(app, None)
    assert found['starting_while_paused'] == ['ekf'] and found['text'].endswith('starting while paused: EKF')
    op.save(op.change('normal'))
    assert 'starting_while_paused' not in status_file._operator(app, None)

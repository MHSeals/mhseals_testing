"""boat_trial agent verbs (spec agent-control section 3): exit codes, streams, leases, no ROS.

A fake desk answers through the injected sleep, so each test plays the TUI's side of the channel.
"""

import io
import json

import pytest

from mhseals_testing import data
from mhseals_testing.harness import agent_channel as ch
from mhseals_testing.harness import agent_cli, cli
from mhseals_testing.harness.registry import runtime_dir

SETTINGS = {'request_s': 5.0, 'ask_s': 1.0, 'lease': {'idle_s': 3.0, 'motion_s': 0.3}}


class Reg:
    def __init__(self, pid=77, session=None):
        self.pid, self.session, self.dead = pid, session, False

    def owner(self):
        return {'pid': self.pid} if self.pid else None

    def alive(self, pid):
        return not self.dead

    def state(self):
        return {'owner': self.owner(), 'session': str(self.session) if self.session else None}


class Desk:
    """Plays the owner TUI: ``script(req)`` returns the reply dicts for each new request."""

    def __init__(self, script=None, every=None):
        self.script, self.every = script or (lambda req: []), every
        self.tail, self.t, self.queue = ch.Tail(), 0.0, []

    def clock(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds
        for record in self.tail.read(max_lines=10 ** 4)[0]:
            if record['kind'] == 'req':
                self.queue += [(record, r) for r in self.script(record)]
        if self.queue:
            req, reply = self.queue.pop(0)
            if callable(reply):
                reply()
            else:
                ch.reply(req['id'], 'tui', **reply)
        if self.every:
            self.every(self)


def call(argv, desk=None, registry=None, **seams):
    args = cli.build_parser().parse_args(argv)
    out = io.StringIO()
    desk = desk or Desk()
    code = agent_cli.main(args, registry=registry or Reg(), out=out, clock=desk.clock,
                          sleep=desk.sleep, settings=SETTINGS, **seams)
    return code, [json.loads(x) for x in out.getvalue().splitlines()]


def lines(kind=None):
    return [r for r in data.read_jsonl(ch.channel_path()) if kind in (None, r['kind'])]


@pytest.mark.parametrize('error,code', [
    ('', 0), ('busy', 2), ('blocked', 5), ('motion_observe', 2), ('no_mux', 2),
    ('revoked', 3), ('overridden: keys', 3), ('expired', 3), ('cancelled: handover', 3),
    ('no_owner', 4), ('timeout', 6)])
def test_exit_codes(error, code):
    assert ch.exit_code({'ok': not error, 'error': error}) == code


@pytest.mark.parametrize('argv', [['claim', '--who', 'a'], ['release', '--claim', 'c1'],
                                  ['run', 's', '--claim', 'c1'], ['goal', '1', '2', '--claim', 'c1'],
                                  ['ask', 'lines off?']])
def test_owner_verbs_exit_4_headless_and_journal_their_own_reply(argv):
    code, out = call(argv, registry=Reg(pid=None))
    assert code == 4 and out[0]['error'] == 'no_owner' and out[0]['hint']
    req, reply = lines()
    assert req['kind'] == 'req' and req['op'] == argv[0] and reply['re'] == req['id']


def test_claim_returns_the_desk_reply():
    desk = Desk(lambda req: [dict(state='done', result={'id': 'c4'})])
    code, out = call(['claim', '--who', 'claude-3f2a', '--reason', 'tune'], desk)
    assert code == 0 and out[-1]['result'] == {'id': 'c4'}
    [req] = lines('req')
    assert req['who'] == 'claude-3f2a' and req['args'] == {'reason': 'tune'} and req['claim'] is None


def test_run_streams_progress_and_renews_the_motion_lease():
    steps = [dict(state=s) for s in ('queued', 'started', 'progress', 'progress', 'progress')]
    desk = Desk(lambda req: steps + [dict(state='done', result={'run_file': 'r.json'})])
    code, out = call(['run', 'square', '--claim', 'c4', '--who', 'a'], desk)
    assert code == 0 and [r['state'] for r in out] == ['queued', 'started', *['progress'] * 3, 'done']
    [req] = lines('req')
    assert req['args'] == {'key': 'square'} and req['claim'] == 'c4' and req['dry'] is False
    renews = lines('renew')
    assert renews and all(r['claim'] == 'c4' and r['act'] == req['id'] for r in renews)


def test_run_mission_sends_the_task_it_names():
    desk = Desk(lambda req: [dict(state='done', result={})])
    call(['run', 'mission', '--task', 'nav_channel', '--claim', 'c4', '--dry-run'], desk)
    [req] = lines('req')
    assert req['args'] == {'key': 'mission', 'task': 'nav_channel'}


def test_dry_run_sends_dry_and_does_not_renew():
    desk = Desk(lambda req: [dict(state='done', result={'blockers': []})])
    code, _ = call(['goal', '3', '-4.5', '0.5', '--claim', 'c4', '--dry-run'], desk)
    [req] = lines('req')
    assert code == 0 and req['dry'] and req['args'] == {'x': 3.0, 'y': -4.5, 'yaw': 0.5}
    assert not lines('renew')


@pytest.mark.parametrize('xy', [['nan', '0'], ['0', 'inf'], ['1', '2', 'nan']])
def test_a_non_finite_goal_is_bad_args_and_sends_nothing(xy):
    code, out = call(['goal', *xy, '--claim', 'c4'], Desk(lambda req: []))
    assert code == 2 and out == [{'ok': False, 'error': 'bad_args', 'hint': 'goal needs finite x, y (and yaw)'}]
    assert not lines('req')


def test_an_override_mid_run_exits_3_with_the_hint():
    desk = Desk(lambda req: [dict(state='started'),
                             dict(state='done', ok=False, error='overridden: keys',
                                  hint='operator must U and re-arm')])
    code, out = call(['run', 'square', '--claim', 'c4'], desk)
    assert code == 3 and out[-1]['hint'] == 'operator must U and re-arm'


def test_the_tui_dying_mid_act_exits_4():
    registry = Reg()
    desk = Desk(lambda req: [dict(state='started'), lambda: setattr(registry, 'dead', True)])
    code, out = call(['run', 'square', '--claim', 'c4'], desk, registry)
    assert code == 4 and out[-1]['error'] == 'no_owner'


def test_no_reply_past_the_deadline_times_out(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    data.write_json(runtime_dir() / 'status.json', {'pid': 77}, 'mhseals.status/1')   # the desk is up
    args = cli.build_parser().parse_args(['claim'])
    stream, desk = io.StringIO(), Desk()
    code = agent_cli.main(args, registry=Reg(), out=stream, clock=desk.clock, sleep=desk.sleep,
                          settings=dict(SETTINGS, request_s=-3.0))
    assert code == 6 and json.loads(stream.getvalue())['error'] == 'timeout'


@pytest.mark.parametrize('argv', [['run', 'square', '--claim', 'c4'], ['goal', '1', '2', '--claim', 'c4']])
def test_an_owner_without_a_desk_exits_4_not_6(argv, monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    data.write_json(runtime_dir() / 'status.json', {'pid': 12}, 'mhseals.status/1')   # an earlier TUI's
    args = cli.build_parser().parse_args(argv)
    stream, desk = io.StringIO(), Desk()
    code = agent_cli.main(args, registry=Reg(), out=stream, clock=desk.clock, sleep=desk.sleep,
                          settings=dict(SETTINGS, request_s=-3.0))
    reply = json.loads(stream.getvalue().splitlines()[-1])
    assert code == 4 and reply['error'] == 'no_owner' and 'desk' in reply['hint']


def test_a_rotated_channel_times_out():
    def rotate():
        ch.rotate(False, limit=0)
    code, out = call(['run', 'square', '--claim', 'c4'], Desk(lambda req: [dict(state='started'), rotate]))
    assert code == 6 and 'rotated' in out[-1]['hint']


def test_ask_times_out_after_ask_s():
    code, out = call(['ask', 'dock lines off?', '--options', 'yes,no'],
                     Desk(lambda req: [dict(state='queued')]))
    [req] = lines('req')
    assert code == 6 and req['args'] == {'question': 'dock lines off?', 'options': ['yes', 'no']}


def test_stop_falls_back_to_the_mux_hold_when_the_tui_is_silent():
    held = []
    code, out = call(['stop', '--who', 'a'], backup=lambda who: held.append(who) or True)
    assert code == 0 and held == ['a'] and out[-1]['result']['via'] == 'ros'


def test_stop_answered_by_the_tui_does_not_use_ros():
    code, _ = call(['stop'], Desk(lambda req: [dict(state='done')]),
                   backup=lambda who: pytest.fail('backup used'))
    assert code == 0


def test_stop_headless_without_a_mux_exits_4():
    code, out = call(['stop'], registry=Reg(pid=None), backup=lambda who: False)
    assert code == 4 and out[-1]['error'] == 'no_owner'


def test_note_and_headless_mark(data_root):
    session = data_root / 'sessions' / '20261025_120000_b_boat'
    session.mkdir(parents=True)
    data.append_jsonl(session / 'events.jsonl', {'phase': 'marker', 'id': 3, 't_ns': 1})
    assert call(['note', 'checking odom', '--who', 'a'])[0] == 0
    assert lines('note')[0]['cmd'] == 'checking odom'
    code, out = call(['mark', 'buoy missed'], registry=Reg(pid=None))
    marker = data.read_jsonl(session / 'events.jsonl')[-1]
    assert code == 0 and marker['kind'] == 'agent' and marker['id'] == 4 and marker['rec'] is False


def test_status_reads_the_status_file_and_filters_sections(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    import time
    data.write_json(runtime_dir() / 'status.json', {'t_ns': time.time_ns(), 'owner': {'pid': 77},
                    'authority': {'owner': 'agent'}, 'health': {'fail': 0}}, 'mhseals.status/1')
    code, [doc] = call(['status', '--section', 'authority'])
    assert code == 0 and doc['authority'] == {'owner': 'agent'} and 'health' not in doc
    assert doc['stale'] is False
    code, [doc] = call(['status'], registry=Reg(pid=None))
    assert doc['owner'] is None and doc['stale'] and 'host.disk_free_gb' in doc['host_facts']


def test_follow_until_an_event_renews_the_idle_lease(data_root):
    session = data_root / 'sessions' / 's1'
    session.mkdir(parents=True)
    data.append_jsonl(session / 'events.jsonl', {'phase': 'old'})

    def journal(desk):
        if desk.t > 0.5:
            data.append_jsonl(session / 'events.jsonl', {'phase': 'run_end', 'run': 'r2'})
    code, out = call(['follow', '--until', 'run_end:run=r2', '--claim', 'c4'],
                     Desk(every=journal), Reg(session=session))
    assert code == 0 and out[-1]['phase'] == 'run_end' and 'old' not in [e['phase'] for e in out]
    assert lines('renew')[0]['claim'] == 'c4'


def test_follow_times_out_and_owner_none_ends_when_the_tui_quits(data_root):
    assert call(['follow', '--timeout', '1'])[0] == 6
    registry = Reg()

    def quit(desk):
        registry.dead = desk.t > 0.3
    assert call(['follow', '--until', 'owner:none'], Desk(every=quit), registry)[0] == 0


def test_config_apply_own_claim_passes_only_the_claim_fence():
    from test_nav2_live import World, make
    world = World()
    world.record.update(owner='agent', claims=[{'id': 'c4', 'who': 'a', 'state': 'active'}])

    def fences(claim=None):
        return ' '.join(make(world, claim=claim).fences(world.owner_record))
    assert 'claim held' in fences() and 'claim held' in fences('c9')
    assert fences('c4') == ''
    world.record.update(input='manual', owner='keyboard')
    assert 'claim held' in fences('c4')
    world.record.update(input='auto', owner='agent', procedure={'name': 'square'})
    assert fences('c4') == 'procedure running: square'
    world.record.update(procedure=None)
    world.goal_list = ['navigate_to_pose:ab12']
    assert 'a goal is active' in fences('c4')


def test_cli_registers_every_verb():
    parser = cli.build_parser()
    for verb in agent_cli.VERBS:
        assert parser.parse_args([verb, *{'release': ['--claim', 'c'], 'run': ['k', '--claim', 'c'],
                                          'goal': ['1', '2', '--claim', 'c'], 'note': ['t'],
                                          'ask': ['q']}.get(verb, [])]).command == verb

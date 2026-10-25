"""The agent desk end to end on fakes: every takeover case of spec agent-control section 4.2.

The CLI side is the channel (harness/agent_channel.py); the TUI side is a fake app with the real
arbiter, the real AuthorityFeed sources, the real Goals and the real desk. A fake procedure loops
like the runner: each step checks the runner blockers, and a pause ticks the feeds.
"""

import copy
import json
import time
from types import SimpleNamespace

import pytest

from mhseals_testing.harness import agent_channel as ch, runs
from mhseals_testing.harness.settings import Settings, load as load_settings, save as save_settings
from mhseals_testing.runner import Aborted, interlock_abort
from mhseals_testing.ui import agent_acts, status_file
from mhseals_testing.ui.feeds import agent as agent_feed
from mhseals_testing.ui.feeds.authority import AuthorityFeed


class Registry:
    def __init__(self):
        self.data = {}

    def read(self):
        return copy.deepcopy(self.data)

    def update(self, change):
        change(self.data)
        self.data = json.loads(json.dumps(self.data))


class Goal:
    def __init__(self):
        self.done, self.status, self.cancelled = False, 'RUNNING', False

    def cancel(self, timeout=3.0):
        self.cancelled, self.done, self.status = True, True, 'CANCELED'


class Nav2:
    def __init__(self):
        self.goal, self.costmap_time = None, time.monotonic()

    def lifecycle(self, timeout=2.0):
        return dict.fromkeys(('bt_navigator', 'controller_server', 'planner_server', 'velocity_smoother'),
                             'active')

    def pose(self):
        return (0.0, 0.0, 0.0)

    def fetch_costmap(self):
        info = SimpleNamespace(width=100, height=100, resolution=0.5,
                               origin=SimpleNamespace(position=SimpleNamespace(x=-25.0, y=-25.0)))
        return SimpleNamespace(info=info, data=[0] * 10000)

    def plan(self, goal, timeout=2.0):
        return [(0.0, 0.0), (goal[0], goal[1])]

    def navigate(self, goals, timeout=10.0):
        self.goal = Goal()
        return self.goal


class Plant:
    def __init__(self, app):
        self.app = app

    def neutral(self):
        self.app.calls.append('neutral')

    def hold(self, cause, by='tui'):
        self.app.calls.append(('hold', cause))
        self.app.set_mux('hold', cause)


class App:
    """The App surface the desk, its acts and the authority feed read."""

    def __init__(self, tmp_path, clock):
        self.now = clock
        self.calls, self.events, self.keys = [], [], []
        self.watching = self.review = None
        self.args = SimpleNamespace(review=None, bench_override=False)
        self.feeds, self.notice, self.modal, self.running, self.manual = [], '', None, None, False
        self.detaching, self.activity, self.run_id = False, 'results', None
        self._runner_cancelled = self._runner_paused = False
        self.settings = Settings()
        self.profile = SimpleNamespace(name='sim', muxed=True, use_sim_time=False, command_units='velocity',
                                       command_topic='/cmd_vel', command_type='twist')
        self.paths = SimpleNamespace(runs=tmp_path / 'runs', journal=tmp_path / 'events.jsonl', directory=tmp_path)
        self.paths.runs.mkdir()
        self.is_armed, self.fault, self.mux = True, '', {'input': 'auto'}
        self.arming = SimpleNamespace(fault=lambda: self.fault)
        self.plant = Plant(self)
        self.recorder = SimpleNamespace(mode='lean')
        self.source = SimpleNamespace(live=True)
        self.monitor = SimpleNamespace(snapshot=lambda: {'sensors': {'odometry': {'status': 'LIVE', 'age': 0.1}}})
        self.facts = {'gps.fix_type': SimpleNamespace(value=6)}
        self.ros = SimpleNamespace(emit=self.emit, nodes=lambda: ['/bt_navigator', '/planner_server'])
        self._watcher, self.engine, self.nav = None, None, Nav2()
        self.steps, self.on_step, self.arm_answer, self.answers = 3, {}, True, []

    armed = property(lambda self: self.is_armed)

    def emit(self, phase, **fields):
        self.events.append((phase, fields))

    def nav2(self):
        return self.nav

    def set_mux(self, input, cause=None):
        self.mux = {'input': input, 'cause': cause}
        self.arbiter.on_mux({**self.mux, 't_ns': int(self.now() * 1e9)})

    def override_cause(self):
        return '' if self.mux['input'] == 'auto' else self.mux['input']

    def operator_stop(self):
        self.plant.hold('keys')
        self.stop('stopped by operator')

    def stop(self, reason, by='operator'):
        self.calls.append(('stop', reason))
        if self._runner_paused:
            self._runner_cancelled = True
        self.is_armed = False

    def revoke_agent(self):
        holder = self.arbiter.table.holder()
        self.arbiter.revoke(holder['id'])

    def arm_panel(self):
        self.calls.append('arm_panel')
        self.is_armed = self.arm_answer

    def availability(self, procedure):
        return '' if self.armed or 'armed' not in procedure.needs else 'outputs are not armed'

    def arm_blockers(self):
        return []

    def motion_blockers(self):
        return []

    def snapshot(self):
        return {'results': []}

    def popup(self, modal, handle, refresh=None):
        """Feed the scripted answers; each key is preceded by a tick, as the real popup does."""
        self.modal = modal
        try:
            for answer in self.answers.pop(0) if self.answers else ['enter']:
                self.tick()
                if refresh:
                    refresh()
                if callable(answer):
                    answer()
                    continue
                result = handle(answer)
                if result is not None:
                    return result
            while True:                       # no more keys: wait for the deadline or a cause
                self.now_advance(1.0)
                self.tick()
                refresh()
        finally:
            self.modal = None

    def now_advance(self, seconds):
        self.clock[0] += seconds

    def tick(self):
        for feed in self.feeds:
            feed.tick(self, None, self.now())

    def run_procedure(self, procedure):
        """The runner, reduced: blockers each step; a pause ticks the feeds and honours the cancel."""
        self.running, self.run_id = procedure.title, runs.next_run_id(self.paths.runs, procedure.title)
        aborted = None
        try:
            for step in range(self.steps):
                self.on_step.get(step, lambda: None)()
                if self._runner_paused:
                    self.tick()
                    if self._runner_cancelled:
                        raise Aborted('motion cancelled by operator')
                reasons = agent_feed.blockers(self)
                if reasons:
                    raise interlock_abort(reasons)
                self.clock[0] += 1.0
        except Aborted as error:
            aborted = str(error)
            raise
        finally:
            self.running = None
            runs.write_run(self.paths.runs, {'procedure': procedure.title, 'run_id': self.run_id,
                                             'results': [{'verdict': 'PASS'}]},
                           status=runs.ABORTED if aborted else None)
        self.notice = f'{procedure.title} complete'


@pytest.fixture
def rig(tmp_path):
    clock = [1000.0]
    app = App(tmp_path, lambda: clock[0])
    app.clock = clock
    registry = Registry()
    feed = AuthorityFeed(app, app.ros, clock=lambda: clock[0], registry=registry)
    app.arbiter = feed.arbiter
    app.feeds.append(feed)
    app.set_mux('auto')
    desk = agent_feed.start(app, app.ros, registry=registry, clock=lambda: clock[0],
                            status=lambda a, d: None)
    return SimpleNamespace(app=app, desk=desk, clock=clock, registry=registry)


def replies(req_id):
    lines = [json.loads(line) for line in ch.channel_path().read_text().splitlines()]
    return [r for r in lines if r['kind'] == 'reply' and r['re'] == req_id]


def final(req_id):
    done = [r for r in replies(req_id) if r['state'] == 'done']
    assert len(done) == 1, replies(req_id)
    return done[0]


def claim(rig, who='claude-3f2a'):
    req = ch.request('claim', who, {'reason': 'test'})
    rig.desk.tick()
    reply = final(req['id'])
    assert reply['ok'], reply
    return reply['result']['claim']


def ask(rig, op, claim_id, who='claude-3f2a', dry=False, **args):
    req = ch.request(op, who, args, claim=claim_id, dry=dry)
    rig.desk.tick()                       # read: queued (or answered)
    if not dry and any(r['state'] == 'queued' for r in replies(req['id'])):
        rig.desk.tick()                   # start it
    return req


# --- claims, leases, refusals ----------------------------------------------------------------

def test_a_claim_gets_the_idle_lease_and_a_run_goes_through_with_confirm(rig):
    c = claim(rig)
    assert rig.app.arbiter.table.get(c)['lease_s'] == 120
    assert rig.app.activity == 'agent'                     # agent_view follow
    req = ask(rig, 'run', c, key='square')
    reply = final(req['id'])
    assert reply['ok'] and reply['result']['status'] == runs.MEASURED
    assert reply['result']['run_file'].endswith('001-waypoint-square.json')
    states = [r['state'] for r in replies(req['id'])]
    assert states[:3] == ['queued', 'asking', 'started'] and states[-1] == 'done'
    phases = [p for p, _ in rig.app.events]
    assert 'agent_req' in phases and 'agent_act' in phases


def test_motion_refusals_name_the_human_action(rig):
    c = claim(rig)
    rig.app.settings.agent_motion = 'observe'
    reply = final(ask(rig, 'run', c, key='square')['id'])
    assert reply['error'] == 'motion_observe' and ch.exit_code(reply) == 2
    rig.app.settings.agent_motion = 'allow'
    reply = final(ask(rig, 'run', c, key='manual')['id'])
    assert reply['error'] == 'not_allowed'
    reply = final(ask(rig, 'run', c, who='other', key='square')['id'])
    assert reply['error'] == 'not_holder'
    rig.app.profile.muxed = False
    reply = final(ask(rig, 'run', c, key='square')['id'])
    assert reply['error'] == 'no_mux'


def test_agent_motion_resets_to_confirm_at_every_start():
    settings = Settings(agent_motion='allow', agent_view='off')
    assert save_settings(settings)
    loaded = load_settings()
    assert loaded.agent_motion == 'confirm' and loaded.agent_view == 'off'


def test_no_verb_arms_a_run_waits_for_the_human_arm_panel(rig):
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    rig.app.is_armed, rig.app.arm_answer = False, False
    req = ask(rig, 'run', c, key='square')
    reply = final(req['id'])
    assert reply['error'] == 'not_armed' and 'arm_panel' in rig.app.calls
    assert 'waiting_arm' in [r['state'] for r in replies(req['id'])]
    rig.app.arm_answer = True
    assert final(ask(rig, 'run', c, key='square')['id'])['ok']


def test_the_confirm_popup_refuses_on_esc_and_on_timeout(rig):
    c = claim(rig)
    rig.app.answers = [['escape']]
    reply = final(ask(rig, 'run', c, key='square')['id'])
    assert reply['error'] == 'refused'
    rig.app.answers = [[]]
    reply = final(ask(rig, 'run', c, key='square')['id'])
    assert reply['error'] == 'refused' and 'timeout' in reply['hint']


def test_a_dry_run_moves_nothing(rig):
    c = claim(rig)
    reply = final(ask(rig, 'run', c, dry=True, key='square')['id'])
    assert reply['ok'] and reply['result']['asks'] == {'arm': False, 'confirm': True}
    reply = final(ask(rig, 'goal', c, dry=True, x=5, y=0)['id'])
    assert reply['ok'] and reply['result']['gate'] is None and reply['result']['path_m'] == 5.0
    assert rig.app.nav.goal is None and not rig.app.running and rig.app.goals.active is None


# --- the channel contract (spec section 2) ----------------------------------------------------

def test_a_stale_request_is_never_executed(rig):
    c = claim(rig)
    req = ch.request('run', 'claude-3f2a', {'key': 'square'}, claim=c, request_s=-1)
    rig.desk.tick()
    assert final(req['id'])['error'] == 'expired' and rig.app.running is None


def test_a_restart_resumes_the_position_and_skips_answered_requests(rig):
    c = claim(rig)
    answered = ch.request('mark', 'claude-3f2a')
    ch.reply(answered['id'], 'tui')                  # answered, but the position was not persisted
    pending = ch.request('mark', 'claude-3f2a')
    again = agent_feed.AgentDesk(rig.app, rig.app.ros, registry=rig.registry, clock=lambda: rig.clock[0])
    again.tick()
    again.tick()
    assert len(replies(answered['id'])) == 1 and final(pending['id'])['ok']
    assert rig.registry.data['agent']['offset'] == ch.channel_path().stat().st_size
    third = agent_feed.AgentDesk(rig.app, rig.app.ros, registry=rig.registry, clock=lambda: rig.clock[0])
    third.tick()
    assert len(replies(pending['id'])) == 1 and c


def test_a_busy_ui_leaves_the_act_queued_until_its_deadline(rig):
    c = claim(rig)
    rig.app.modal = {'kind': 'message'}
    req = ch.request('run', 'claude-3f2a', {'key': 'square'}, claim=c, request_s=0.05)
    rig.desk.tick()
    time.sleep(0.06)
    rig.desk.tick()
    assert final(req['id'])['error'] == 'busy_ui' and rig.app.running is None


def test_a_flood_keeps_the_tick_short(rig):
    ch.channel_path().parent.mkdir(parents=True, exist_ok=True)
    with open(ch.channel_path(), 'a') as handle:      # one write: 10 k fsyncs would only slow the test
        handle.writelines(json.dumps(ch.line('note', 'flood', phase='text', cmd=str(i))) + '\n'
                          for i in range(10_000))
    started = time.perf_counter()
    rig.desk.tick()
    assert time.perf_counter() - started < 0.02
    assert len(rig.desk.notes) == 64


def test_status_file_is_written_as_json(rig, tmp_path):
    claim(rig)
    rig.app.monitor.snapshot = lambda: {'sensors': {'odometry': {'status': 'LIVE', 'age': float('inf')}}}
    target = tmp_path / 'status.json'
    status_file.write(rig.app, rig.desk, target)
    status = json.loads(target.read_text())
    assert status['schema'] == status_file.SCHEMA and status['session']['armed'] is True
    assert status['odometry']['age'] is None and status['agent']['act'] is None
    assert status['health']['gates']['square'] is None


def test_procedure_names_and_keys_resolve():
    found, name = agent_acts.procedure('w')
    assert name == 'square' and found.title == 'Waypoint square'
    assert agent_acts.procedure('nope') == (None, None)
    with pytest.raises(ValueError):
        agent_acts.goal_args({'x': 'nan', 'y': 0})


def test_a_request_queued_behind_a_goal_expires_at_its_deadline(rig):
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    ask(rig, 'goal', c, x=5, y=0)
    req = ch.request('run', 'claude-3f2a', {'key': 'square'}, claim=c, request_s=0.05)
    ch.renew('claude-3f2a', c, rig.desk.act['id'])
    rig.desk.tick()
    time.sleep(0.06)
    rig.desk.tick()
    assert final(req['id'])['error'] == 'busy_ui' and rig.app.running is None

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

    def fetch_costmap(self, timeout=2.0):
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
    rig.app.answers = [['enter']]                          # Refuse is focused: Enter alone never allows
    assert final(ask(rig, 'run', c, key='square')['id'])['error'] == 'refused'
    rig.app.answers = [['left', 'enter']]
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
    lease = rig.desk.lease()
    rig.desk.lease = lambda: dict(lease, motion_s=100)        # a CLI that keeps renewing
    rig.app.answers = [[]]
    reply = final(ask(rig, 'run', c, key='square')['id'])
    assert reply['error'] == 'refused' and 'timeout' in reply['hint']


def test_a_lapsed_motion_lease_in_the_confirm_popup_expires_the_claim(rig):
    c = claim(rig)
    rig.app.answers = [[]]                                    # nobody answers; the CLI died: no renewals
    reply = final(ask(rig, 'run', c, key='square')['id'])
    assert reply['error'] == 'expired' and rig.app.arbiter.ended[c] == 'expired'
    assert ('hold', 'lease') in rig.app.calls and rig.app.running is None


def test_a_dry_run_moves_nothing(rig):
    c = claim(rig)
    reply = final(ask(rig, 'run', c, dry=True, key='square')['id'])
    assert reply['ok'] and reply['result']['asks'] == {'arm': False, 'confirm': True}
    reply = final(ask(rig, 'goal', c, dry=True, x=5, y=0)['id'])
    assert reply['ok'] and reply['result']['gate'] is None and reply['result']['path_m'] == 5.0
    assert rig.app.nav.goal is None and not rig.app.running and rig.app.goals.active is None


def test_a_dry_run_while_an_act_runs_is_busy_and_asks_nav2_nothing(rig):
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    ask(rig, 'goal', c, x=5, y=0)
    asked = []
    rig.app.nav.lifecycle = lambda timeout=2.0: asked.append(timeout) or {}
    reply = final(ask(rig, 'goal', c, dry=True, x=3, y=0)['id'])
    assert reply['error'] == 'busy' and ch.exit_code(reply) == 2 and asked == []


def test_dry_runs_ask_nav2_with_short_waits_and_reuse_its_lifecycle(rig):
    c = claim(rig)
    asked, lifecycle = [], rig.app.nav.lifecycle
    rig.app.nav.lifecycle = lambda timeout=2.0: asked.append(timeout) or lifecycle()
    for _ in range(2):
        assert final(ask(rig, 'goal', c, dry=True, x=5, y=0)['id'])['result']['gate'] is None
    assert asked == [agent_feed.LIFECYCLE_TIMEOUT_S]


def test_run_blockers_answer_only_renew_stop_and_release(rig):
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    sent, asked, lifecycle = {}, [], rig.app.nav.lifecycle
    rig.app.nav.lifecycle = lambda timeout=2.0: asked.append(rig.app.running) or lifecycle()

    def write():
        sent['mark'] = ch.request('mark', 'claude-3f2a', {'note': 'gust'})
        sent['dry'] = ch.request('goal', 'claude-3f2a', {'x': 5, 'y': 0}, claim=c, dry=True)
        sent['renew'] = ch.renew('claude-3f2a', c, rig.desk.act['id'])

    def look():
        sent['during'] = replies(sent['mark']['id']) + replies(sent['dry']['id']) + asked
    rig.app.on_step = {0: write, 2: look}
    assert final(ask(rig, 'run', c, key='square')['id'])['ok']
    assert sent['during'] == []                              # deferred, not answered inside the runner
    rig.desk.tick()
    assert final(sent['mark']['id'])['ok'] and final(sent['dry']['id'])['ok'] and asked == [None]


def test_a_slow_nav2_never_stalls_the_keys(rig):
    """The goal's Nav2 steps run beside a popup that keeps reading keys: Space lands mid-send."""
    from mhseals_testing.ui import popups
    import threading
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    nav, sending, release = rig.app.nav, threading.Event(), threading.Event()
    fast, log = nav.navigate, []

    def slow(goals, timeout=10.0):
        sending.set()
        release.wait(5)
        return fast(goals, timeout)

    class Keys:
        def read(self, timeout=0.1):
            time.sleep(0.01)
            if sending.is_set() and not log:
                log.append('space')
                return 'space'
            if log == ['space'] and ('stop', 'stopped by operator') in rig.app.calls:
                log.append('stopped while Nav2 was sending' if not release.is_set() else 'late')
                release.set()
            return None
    nav.navigate, rig.app.keys, rig.app.refresh = slow, Keys(), lambda force=False: None
    rig.app.popup = lambda modal, handle, refresh=None: popups.popup(rig.app, modal, handle, refresh)
    reply = final(ask(rig, 'goal', c, x=5, y=0)['id'])
    assert log == ['space', 'stopped while Nav2 was sending']
    assert reply['error'] == 'overridden: keys' and nav.goal.cancelled and rig.app.goals.active is None


def test_ctrl_c_during_an_agent_run_replies_handover(rig):
    def interrupt():
        raise KeyboardInterrupt
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    rig.app.on_step = {1: interrupt}
    req = ch.request('run', 'claude-3f2a', {'key': 'square'}, claim=c)
    with pytest.raises(KeyboardInterrupt):
        rig.desk.tick()
    reply = final(req['id'])
    assert reply['error'] == 'handover' and ch.exit_code(reply) == 3 and rig.desk.act is None


# --- the takeover table (spec 4.2) ------------------------------------------------------------

def run_with(rig, step, action):
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    rig.app.on_step = {step: action}
    req = ask(rig, 'run', c, key='square')
    return c, final(req['id'])


def test_space_cancels_an_agent_run_and_it_never_resumes(rig):
    def space():
        rig.app.operator_stop()
        raise Aborted('motion cancelled by operator')
    c, reply = run_with(rig, 1, space)
    assert reply['error'] == 'overridden: keys' and ch.exit_code(reply) == 3
    assert reply['hint'] == 'operator must U and re-arm'
    assert rig.app.arbiter.table.get(c)['state'] == 'active'          # suspended, not dropped
    rig.desk.tick()
    assert rig.app.running is None and rig.desk.act is None          # nothing restarts it


def test_manual_mode_cancels_the_agent_run(rig):
    _, reply = run_with(rig, 1, lambda: setattr(rig.app, 'manual', True))
    assert reply['error'] == 'overridden: keyboard'


@pytest.mark.parametrize('input, cause', [('rc', 'rc'), ('manual', 'keyboard')])
def test_an_rc_or_keyboard_mux_takeover_pauses_then_aborts_the_agent_run(rig, input, cause):
    def takeover():
        rig.app.set_mux(input)
        rig.app._runner_paused = True                # the runner pauses on the override
    _, reply = run_with(rig, 1, takeover)
    assert reply['error'] == f'overridden: {cause}'
    assert rig.app._runner_cancelled                 # set by desk.tick inside the pause


def test_e_revokes_cancels_the_run_and_the_next_call_says_revoked(rig):
    c, reply = run_with(rig, 1, lambda: rig.app.revoke_agent())
    assert reply['error'] == 'revoked' and ch.exit_code(reply) == 3
    assert ('hold', 'revoke') in rig.app.calls
    again = final(ask(rig, 'run', c, key='square')['id'])
    assert again['error'] == 'revoked'


def test_a_physical_fault_cancels_the_agent_run(rig):
    _, reply = run_with(rig, 1, lambda: setattr(rig.app, 'fault', 'driver fault'))
    assert reply['error'] == 'overridden: fault'


def test_a_lapsed_motion_lease_stops_exactly_like_e(rig):
    rig.app.steps = 30                    # the fake run takes 30 s; the CLI never renews
    c, reply = run_with(rig, 0, lambda: None)
    assert reply['error'] == 'expired' and ch.exit_code(reply) == 3
    assert rig.app.calls[:2] == ['neutral', ('hold', 'lease')]
    assert rig.app.arbiter.table.get(c) is None and rig.app.arbiter.ended[c] == 'expired'


def test_renewals_from_the_acts_cli_keep_the_motion_lease(rig):
    rig.app.steps = 30
    holder = {}

    def renew():
        ch.renew('claude-3f2a', holder['c'], rig.desk.act['id'])
    rig.app.on_step = {s: renew for s in range(0, 30, 3)}
    holder['c'] = c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    assert final(ask(rig, 'run', c, key='square')['id'])['ok']


def test_an_idle_lease_lapse_expires_the_claim(rig):
    c = claim(rig)
    rig.clock[0] += 121
    rig.app.arbiter.tick()
    assert ('hold', 'lease') in rig.app.calls
    assert final(ask(rig, 'run', c, key='square')['id'])['error'] == 'expired'


def test_notes_by_the_holder_renew_the_idle_lease(rig):
    c = claim(rig)
    for _ in range(3):
        rig.clock[0] += 100
        ch.note('claude-3f2a', 'post', 'ros2 topic hz /odom', tool_id='t1', exit=0, dur_s=2.1)
        rig.desk.tick()
        rig.app.arbiter.tick()
    assert rig.app.arbiter.table.get(c)
    assert [p for p, _ in rig.app.events].count('agent_note') == 3, (rig.desk.errors, list(rig.desk.notes))


def test_release_by_another_who_is_refused(rig):
    c = claim(rig)
    req = ch.request('release', 'intruder', claim=c)
    rig.desk.tick()
    assert final(req['id'])['error'] == 'not_holder' and rig.app.arbiter.table.get(c)


def test_agents_block_refuses_new_claims_with_exit_5(rig):
    rig.app.arbiter.allow_agents = False
    req = ch.request('claim', 'claude-3f2a')
    rig.desk.tick()
    assert ch.exit_code(final(req['id'])) == 5


def test_a_goal_is_cancelled_by_e_through_the_authority_sources(rig):
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    req = ask(rig, 'goal', c, x=5, y=0)
    assert rig.app.goals.active['claim'] == c and [r['state'] for r in replies(req['id'])][-1] == 'started'
    rig.app.revoke_agent()                           # E: the arbiter cancels app.goals' goal (G3)
    assert rig.app.nav.goal.cancelled and rig.app.goals.active is None
    rig.desk.tick()
    assert final(req['id'])['error'] == 'revoked'


@pytest.mark.parametrize('takeover', ['space', 'rc', 'lease'])
def test_a_goal_is_cancelled_by_any_override(rig, takeover):
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    req = ask(rig, 'goal', c, x=5, y=0)
    if takeover == 'space':
        rig.app.operator_stop()
    elif takeover == 'rc':
        rig.app.set_mux('rc')
    else:
        rig.clock[0] += 11
    rig.desk.tick()
    assert rig.app.nav.goal.cancelled and ch.exit_code(final(req['id'])) == 3


def test_a_goal_that_arrives_succeeds(rig):
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    req = ask(rig, 'goal', c, x=5, y=0)
    rig.app.nav.goal.done, rig.app.nav.goal.status = True, 'SUCCEEDED'
    ch.renew('claude-3f2a', c, req['id'])
    rig.desk.tick()
    reply = final(req['id'])
    assert reply['ok'] and reply['result']['status'] == 'SUCCEEDED' and rig.desk.act is None


def test_handover_cancels_the_act_but_keeps_the_claim(rig):
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    req = ask(rig, 'goal', c, x=5, y=0)
    queued = ch.request('ask', 'claude-3f2a', {'question': 'Dock lines off?'})
    rig.desk.tick()
    rig.app.detaching = True
    rig.desk.close()
    rig.app.arbiter.close(keep_claims=True)
    assert final(req['id'])['error'] == 'handover' and final(queued['id'])['error'] == 'handover'
    assert rig.app.nav.goal.cancelled and rig.registry.data['authority']['claims'][0]['id'] == c


def test_stop_needs_no_claim_and_cancels_the_act(rig):
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    goal = ask(rig, 'goal', c, x=5, y=0)
    req = ch.request('stop', 'other-agent')
    rig.desk.tick()
    assert final(req['id'])['ok'] and ('hold', 'keys') in rig.app.calls and not rig.app.armed
    assert final(goal['id'])['result']['cause'] == 'stop'


def test_the_desk_popup_focuses_refuse_and_counts_down_in_its_body():
    from types import SimpleNamespace
    from mhseals_testing.ui import agent_acts
    now, seen = [0.0], []

    def popup(modal, handle, check):
        now[0] = 4.0
        check()
        seen.append((modal['options'][modal['selected']], modal['countdown']))
        return handle('enter')
    app = SimpleNamespace(popup=popup)
    assert agent_acts.choose(app, 'Run square?', ('Allow', 'Refuse'), 10, lambda: None,
                             clock=lambda: now[0]) == ('Refuse', None)
    assert seen == [('Refuse', (6.0, 10))]


def test_ask_returns_the_humans_choice_and_mark_journals(rig):
    rig.app.answers = [['enter'], ['up', 'enter']]         # focus starts on the agent's safe 'no'
    req = ask(rig, 'ask', None, question='Dock lines off?', options=['yes', 'no'])
    assert final(req['id'])['result'] == {'choice': 'no'}
    req = ask(rig, 'ask', None, question='Dock lines off?', options=['yes', 'no'])
    assert final(req['id'])['result'] == {'choice': 'yes'}
    req = ch.request('mark', 'claude-3f2a', {'note': 'gust'})
    rig.desk.tick()
    assert final(req['id'])['ok']
    assert ('marker', {'kind': 'agent', 'id': 1, 'who': 'claude-3f2a', 'note': 'gust',
                       't_ns': rig.app.events[-2][1]['t_ns']}) in rig.app.events


# --- the channel contract (spec section 2) ----------------------------------------------------

def test_a_stale_request_is_never_executed(rig):
    c = claim(rig)
    req = ch.request('run', 'claude-3f2a', {'key': 'square'}, claim=c, request_s=-1)
    rig.desk.tick()
    assert final(req['id'])['error'] == 'expired' and rig.app.running is None


def test_a_deadline_is_clamped_to_its_stamp_plus_request_s(rig):
    late = dict(ch.line('req', 'claude-3f2a', id='f' * 32, op='mark', args={}, claim=None, dry=False,
                        deadline_ns=time.time_ns() + int(3600e9)), t_ns=time.time_ns() - int(10e9))
    ch.append(late)
    rig.desk.tick()
    assert final(late['id'])['error'] == 'expired'


def test_a_motion_request_from_before_a_takeover_is_never_replayed(rig):
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    old = ch.request('run', 'claude-3f2a', {'key': 'square'}, claim=c)
    time.sleep(0.001)
    again = agent_feed.AgentDesk(rig.app, rig.app.ros, registry=rig.registry, clock=lambda: rig.clock[0])
    again.tick()
    reply = final(old['id'])
    assert reply['error'] == 'expired' and 'before' in reply['hint'] and rig.app.running is None


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


def test_recent_keeps_finished_and_refused_acts_after_act_clears(rig):
    c = claim(rig)
    rig.app.settings.agent_motion = 'observe'
    ask(rig, 'run', c, key='square')
    view = rig.desk.summary()
    assert view['act'] is None
    last = view['recent'][-1]
    assert (last['op'], last['ok'], last['error']) == ('run', False, 'motion_observe') and last['hint']
    assert {'id', 'who', 'label', 't_ns'} <= set(last)
    rig.app.settings.agent_motion = 'allow'
    ask(rig, 'goal', c, x=5, y=0)
    assert rig.desk.summary()['act']['motion_s'] == 10

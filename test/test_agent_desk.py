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

from mhseals_testing.harness import agent_channel as ch, goals as goals_module, runs
from mhseals_testing.harness.settings import Settings, load as load_settings, save as save_settings
from mhseals_testing.runner import Aborted, interlock_abort
from mhseals_testing.ui import agent_acts, status_file
from mhseals_testing.ui.feeds import agent as agent_feed
from mhseals_testing.ui.feeds.authority import AuthorityFeed

# fence_watchdog's message: a map box around the start, as the sim's
FENCE = {'valid': True, 'frame': 'map', 'points': [-40.0, -40.0, 40.0, -40.0, 40.0, 40.0, -40.0, 40.0],
         'margin_m': 5.0, 'outward_angle_deg': 60.0, 'outward_speed_mps': 0.1,
         'reason': '', 'error': '', 'stamp_ns': 1}


@pytest.fixture(autouse=True)
def fence_topic(monkeypatch):
    monkeypatch.setattr(goals_module, 'fence_config', lambda: ('/fence_watchdog/fence', 3.0))


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
    """No costmap until a fetch, like ros.nav2.Nav2 (whose first lease drops the old grid)."""

    def __init__(self):
        self.goal, self.costmap_time, self.costmap_delay = None, None, 0.0
        self.fence = FENCE

    def fence_state(self, timeout=1.0):
        return self.fence, 0.5

    def lifecycle(self, timeout=2.0):
        return dict.fromkeys(('bt_navigator', 'controller_server', 'planner_server', 'velocity_smoother'),
                             'active')

    def pose(self):
        return (0.0, 0.0, 0.0)

    def fetch_costmap(self, timeout=2.0):
        self.costmap_time = time.monotonic() - self.costmap_delay
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
                                       command_topic='/cmd_vel', command_type='twist', uses_fcu=True,
                                       sensors=(SimpleNamespace(name='GPS', kind='gps'),))
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
        self.agent_arm_why = ''

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

    def operator_stop(self, agent=None):
        self.stopped_by = f'agent {agent}' if agent else 'operator'
        self.plant.hold('keys')
        self.stop(f'stopped by {self.stopped_by}')

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

    def agent_arm(self, who, checks):
        self.calls.append(('agent_arm', who, tuple(checks)))
        self.is_armed = self.arm_answer and not self.agent_arm_why
        return self.agent_arm_why

    def disarm(self, reason='disarmed', by='operator'):
        self.calls.append(('disarm', reason, by))
        self.is_armed = False

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
    assert reply['ok'], (reply.get('error'), reply.get('hint'), reply.get('result'))
    return reply['result']['claim']


def ask(rig, op, claim_id, who='claude-3f2a', dry=False, **args):
    req = ch.request(op, who, args, claim=claim_id, dry=dry)
    rig.desk.tick()                       # read: queued (or answered)
    if not dry and any(r['state'] == 'queued' for r in replies(req['id'])):
        rig.desk.tick()                   # start it
    if dry and rig.desk.dry is not None:  # a goal dry run ends on its own thread; the next tick answers
        rig.desk.dry[1].join(5.0)
        rig.desk.tick()
    return req


# --- claims, leases, refusals ----------------------------------------------------------------

def test_a_claim_gets_the_idle_lease_and_a_run_goes_through_with_confirm(rig):
    c = claim(rig)
    assert rig.app.arbiter.table.get(c)['lease_s'] == 120
    assert rig.app.activity == 'agent'                     # Activity always follows a claim
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


def test_run_mission_names_its_task_or_says_the_operator_picks(rig):
    """C27: --task reaches the mission (no task menu); without it the reply says who picks."""
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    seen = []
    rig.app.on_step = {0: lambda: seen.append(getattr(rig.app, 'agent_task', None))}
    req = ask(rig, 'run', c, key='mission', task='nav_channel')
    started = next(r for r in replies(req['id']) if r['state'] == 'started')
    assert started['result'] == {'task': 'nav_channel'} and seen == ['nav_channel']
    assert final(req['id'])['ok'] and rig.app.agent_task is None
    assert rig.desk.recent[-1]['label'] == 'run mission nav_channel'
    req = ask(rig, 'run', c, key='mission')
    assert next(r for r in replies(req['id']) if r['state'] == 'started')['result'] == {
        'task': 'the operator picks it'}
    reply = final(ask(rig, 'run', c, key='mission', task='nope')['id'])
    assert reply['error'] == 'bad_args' and 'nav_channel' in reply['hint']
    assert final(ask(rig, 'run', c, key='square', task='nav_channel')['id'])['error'] == 'bad_args'


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
    settings = Settings(agent_motion='allow')
    assert save_settings(settings)
    loaded = load_settings()
    assert loaded.agent_motion == 'confirm' and not hasattr(loaded, 'agent_view')
    from mhseals_testing.harness.settings import settings_path
    path = settings_path()
    path.write_text(path.read_text() + 'agent_view: off\n')   # an older file: read past, even strictly
    assert load_settings(strict=True).agent_motion == 'confirm'


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


@pytest.mark.parametrize('released', [True, False])
def test_a_hold_the_mux_restores_on_arming_waits_for_the_human_release(rig, released):
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    rig.app.is_armed = False

    def arm_panel():                         # the mux comes up with the saved keys hold
        rig.app.is_armed = True
        rig.app.set_mux('hold', 'keys')
        rig.app.tick()
        if released:                         # U's confirm inside the arm panel
            rig.app.set_mux('auto')
    rig.app.arm_panel = arm_panel
    reply = final(ask(rig, 'run', c, key='square')['id'])
    assert reply['ok'] if released else reply['error'] == 'overridden: keys'


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
    assert reply['hint'] == 'operator must re-arm and release the hold (U)'
    assert rig.app.arbiter.table.get(c)['state'] == 'active'          # suspended, not dropped
    rig.desk.tick()
    assert rig.app.running is None and rig.desk.act is None          # nothing restarts it


def test_a_hold_the_stopped_mux_saved_suspends_the_claim_while_disarmed(rig):
    """M2: after Space the mux is down but its keys hold is saved; the next run waits for the human."""
    from mhseals_hardware import hold_store
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    rig.app.is_armed = False
    hold_store.save(hold_store.default_path(), {'keys': {'cause': 'keys', 'by': 'tui', 'since': 1.0, 'token': None}})
    reply = final(ask(rig, 'run', c, key='square')['id'])
    assert reply['error'] == 'overridden' and 'U' in reply['hint'] and 'arm_panel' not in rig.app.calls
    hold_store.default_path().unlink()                                # the human released it
    assert final(ask(rig, 'run', c, key='square')['id'])['ok']


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


def test_another_agents_stop_needs_no_claim_and_names_the_stopper(rig):
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    goal = ask(rig, 'goal', c, x=5, y=0)
    req = ch.request('stop', 'other-agent')
    rig.desk.tick()
    assert final(req['id'])['ok'] and ('hold', 'keys') in rig.app.calls and not rig.app.armed
    assert final(goal['id'])['result']['cause'] == 'overridden: agent other-agent'


def test_an_agents_own_stop_reads_stop(rig):
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    goal = ask(rig, 'goal', c, x=5, y=0)
    req = ch.request('stop', 'claude-3f2a')
    rig.desk.tick()
    assert final(req['id'])['ok'] and not rig.app.armed
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


def test_a_dry_run_flood_answers_one_per_tick_and_never_holds_the_loop(rig):
    c = claim(rig)
    reqs = [ch.request('goal', 'claude-3f2a', {'x': 5, 'y': 0}, claim=c, dry=True) for _ in range(5)]
    planned, plan = [], rig.app.nav.plan
    rig.app.nav.plan = lambda *a, **k: planned.append(1) or plan(*a, **k)
    mark = ch.request('mark', 'claude-3f2a', {'note': 'behind the flood'})
    rig.desk.tick()
    assert replies(mark['id'])                              # a request behind the flood is answered at once
    rig.desk.dry[1].join(5.0)
    assert len(planned) == 1 and not any(replies(r['id']) for r in reqs)   # answered on a later tick
    for _ in range(5):
        rig.desk.tick()
        if rig.desk.dry is not None:
            rig.desk.dry[1].join(5.0)
    rig.desk.tick()
    assert len(planned) == 5 and all(final(r['id'])['ok'] for r in reqs)


def test_a_slow_getcostmap_never_holds_the_tick(rig):
    """GetCostmap can take seconds: the goal dry run waits on its own thread, Space never waits."""
    c = claim(rig)
    gate = __import__('threading').Event()
    fetch = rig.app.nav.fetch_costmap
    rig.app.nav.fetch_costmap = lambda timeout=2.0: gate.wait(5.0) and fetch(timeout)
    req = ch.request('goal', 'claude-3f2a', {'x': 5, 'y': 0}, claim=c, dry=True)
    began = time.monotonic()
    rig.desk.tick()
    rig.desk.tick()
    assert time.monotonic() - began < 0.5 and not replies(req['id'])
    stop = ch.request('stop', 'claude-3f2a', {})
    rig.desk.tick()
    assert final(stop['id'])['ok'] and time.monotonic() - began < 0.5   # stop answered while Nav2 waits
    gate.set()
    rig.desk.dry[1].join(5.0)
    rig.desk.tick()
    assert final(req['id'])['ok']


def test_procedure_names_and_keys_resolve():
    found, name = agent_acts.procedure('w')
    assert name == 'square' and found.title == 'Waypoint square'
    assert agent_acts.procedure('nope') == (None, None)
    with pytest.raises(ValueError):
        agent_acts.goal_args({'x': 'nan', 'y': 0})
    assert agent_acts.goal_args({'x': 1, 'y': 2, 'yaw': None}) == (1.0, 2.0, 0.0)


@pytest.mark.parametrize('fcu, status, row, fix', [
    (False, 0, 'LIVE', 3), (False, 0, 'WARN', 3), (False, 2, 'LIVE', 5), (False, -1, 'LIVE', 0),
    (False, 0, 'STALE', 0), (True, 0, 'LIVE', 6)])
def test_the_sim_goal_fix_comes_from_gpsraw_with_an_fcu_else_from_the_navsat_status(rig, fcu, status, row, fix):
    snapshot = {'sensors': {'odometry': {'status': 'LIVE', 'age': 0.1},
                            'GPS': {'status': row, 'values': {'status': status}}}}
    rig.app.monitor.snapshot, rig.app.profile.uses_fcu = lambda: snapshot, fcu
    rig.app.profile.use_sim_time = True
    assert agent_acts.goal_facts(rig.app).fix == fix


@pytest.mark.parametrize('fcu', [False, True])
@pytest.mark.parametrize('gpsraw, fix', [(None, 0), (0, 0), (2, 2), (3, 3)])
def test_a_boat_goal_needs_the_mavros_fix_even_without_an_fcu_and_with_a_navsat_fix(rig, fcu, gpsraw, fix):
    snapshot = {'sensors': {'odometry': {'status': 'LIVE', 'age': 0.1},
                            'GPS': {'status': 'LIVE', 'values': {'status': 2}}}}
    rig.app.monitor.snapshot, rig.app.profile.uses_fcu = lambda: snapshot, fcu
    rig.app.profile.use_sim_time = False
    rig.app.facts = {} if gpsraw is None else {'gps.fix_type': SimpleNamespace(value=gpsraw)}
    assert agent_acts.goal_facts(rig.app).fix == fix
    if fix < 3:
        assert 'below 3D' in agent_acts.goals_module.gate(agent_acts.goal_facts(rig.app))


@pytest.mark.parametrize('dry', [True, False])
def test_a_huge_goal_is_bad_args_and_never_crashes_the_tui(rig, dry):
    c = claim(rig)
    reply = final(ask(rig, 'goal', c, dry=dry, x=1e308, y=0)['id'])
    assert not reply['ok'] and reply['error'] == 'bad_args' and 'within' in reply['hint']
    assert rig.app.goals.active is None and rig.desk.act is None
    reply = final(ask(rig, 'goal', c, dry=True, x=5, y=0)['id'])   # the desk still answers
    assert reply['ok']


def test_a_math_error_in_an_act_refuses_only_that_act(rig, monkeypatch):
    c = claim(rig)
    monkeypatch.setattr(agent_acts, 'goal_facts', lambda *a, **k: 1 / 0)
    reply = final(ask(rig, 'goal', c, x=5, y=0)['id'])
    assert reply['error'] == 'refused' and rig.desk.act is None and not rig.desk.busy


def test_a_goal_the_gates_refuse_never_asks_allow(rig):
    c = claim(rig)
    rig.app.facts = {'gps.fix_type': SimpleNamespace(value=0)}
    reply = final(ask(rig, 'goal', c, x=5, y=0)['id'])
    assert reply['error'] == 'refused' and 'below 3D' in reply['hint']
    assert 'asking' not in [r['state'] for r in replies(reply['re'])]


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


@pytest.mark.parametrize('fence, x, reason', [
    (FENCE, 5, None),
    (dict(FENCE, points=[-8.0, -8.0, 4.0, -8.0, 4.0, 8.0, -8.0, 8.0]), 5, 'outside the fence'),
    (dict(FENCE, points=[-8.0, -8.0, 8.0, -8.0, 8.0, 8.0, -8.0, 8.0]), 5, 'fence margin'),
    (None, 5, 'no fence from fence_watchdog'),
    (dict(FENCE, valid=False, points=None, error='no valid fence configured'), 5, 'no valid fence'),
    (dict(FENCE, frame='latlon', points=[42.0, -71.0, 42.0, -70.99, 42.01, -70.99]), 5, 'no toLL'),
])
def test_a_goal_past_the_fence_or_its_margin_is_refused_before_allow(rig, fence, x, reason):
    """C9: the dry run and the act refuse it up front; no Allow is asked, nothing moves."""
    c = claim(rig)
    rig.app.nav.fence = fence
    reply = final(ask(rig, 'goal', c, dry=True, x=x, y=0)['id'])
    assert reply['result']['gate'] == reason or reason in reply['result']['gate']
    if reason is None:
        return
    reply = final(ask(rig, 'goal', c, x=x, y=0)['id'])
    assert reply['error'] == 'refused' and reason in reply['hint']
    assert 'asking' not in [r['state'] for r in replies(reply['re'])]
    assert rig.app.nav.goal is None and rig.app.goals.active is None


def test_manual_takeover_cancels_the_agent_goal_in_nav2_and_the_desk_keeps_answering(rig):
    """C3: m cancels the goal at once (rc 3, no resume); in manual the desk answers without Nav2."""
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    req = ask(rig, 'goal', c, x=5, y=0)
    rig.desk.take_over()                                     # app.run_procedure(manual) calls it first
    assert rig.app.nav.goal.cancelled and rig.app.goals.active is None and rig.desk.act is None
    reply = final(req['id'])
    assert reply['error'] == 'overridden: keyboard' and ch.exit_code(reply) == 3 and 'U' in reply['hint']
    rig.app.manual, rig.app.running = True, 'Manual control'
    written = []
    rig.desk.status = lambda app, desk: written.append(desk.summary())
    rig.clock[0] += agent_feed.STATUS_S
    dry = ask(rig, 'goal', c, dry=True, x=5, y=0)
    goal = ask(rig, 'goal', c, x=5, y=0)
    assert final(dry['id'])['error'] == final(goal['id'])['error'] == 'overridden'
    assert written and rig.app.nav.goal.cancelled            # status.json keeps updating; nothing resumed


def test_space_and_fence_holds_say_what_the_operator_does(rig):
    """C1/C11: the hint names U, not 'operator left manual'; a fence hold says how to get out."""
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    req = ask(rig, 'goal', c, x=5, y=0)
    rig.app.set_mux('hold', 'geofence')
    rig.desk.tick()
    reply = final(req['id'])
    assert reply['error'] == 'overridden: geofence' and 'inward' in reply['hint'] and '(U)' in reply['hint']
    assert '(U)' in agent_feed.HINTS['keyboard'] and 'left manual' not in agent_feed.HINTS['keyboard']


def test_goal_freshness_is_judged_on_the_grid_fetched_for_it(rig):
    c = claim(rig)
    rig.app.nav.costmap_delay = 60.0     # the fetch returns a grid received a minute ago
    reply = final(ask(rig, 'goal', c, dry=True, x=5, y=0)['id'])
    assert reply['result']['gate'] == 'global costmap is stale'
    rig.app.settings.agent_motion = 'allow'
    reply = final(ask(rig, 'goal', c, x=5, y=0)['id'])
    assert reply['error'] == 'refused' and reply['hint'] == 'global costmap is stale'
    assert rig.app.nav.goal is None and rig.app.goals.active is None


def test_dry_run_helpers_name_an_off_course_sim_boat_a_boat_yaml_and_the_operator_popup(tmp_path):
    import json
    from pathlib import Path
    from types import SimpleNamespace
    from mhseals_testing.ui import agent_acts
    from mhseals_testing.ui.feeds.agent import operator_popup
    truth = tmp_path / 'course_truth.json'
    truth.write_text(json.dumps({'elements': [{'label': 'red_pole_buoy', 'x': 0.0, 'y': 0.0}]}))
    history = {'ground truth': [(0.0, {'x': 0.0, 'y': -90.0})]}
    app = SimpleNamespace(
        profile=SimpleNamespace(course_truth=str(truth), ground_truth='/crane/odom', outputs='topic',
                               use_sim_time=True),
        monitor=SimpleNamespace(history=lambda name: history.get(name, [])),
        config_file=Path.home() / '.config/mhseals/hardware.yaml')
    assert agent_acts.mission_course(app).startswith('the boat is 90 m from')
    assert 'hardware_crane.yaml' in agent_acts.hardware_advisory(app)
    app.config_file = Path('/ws/install/mhseals_testing/share/mhseals_testing/config/sim/hardware_crane.yaml')
    assert agent_acts.hardware_advisory(app) == ''
    app.profile.outputs, app.config_file = 'odroid', Path.home() / '.config/mhseals/hardware.yaml'
    assert agent_acts.hardware_advisory(app) == ''                  # the boat's own YAML on the boat
    assert operator_popup(SimpleNamespace(modal=None)) is None
    assert operator_popup(SimpleNamespace(modal={'kind': 'message', 'title': 'Ready',
                                                 'message': 'Clear the area and press Enter... '})) == \
        'Ready: Clear the area and press Enter... '


# --- sim-only auto-arm (run --sim-arm, harness/sim_arm.py) -----------------------------------

@pytest.fixture
def sim(rig, monkeypatch):
    """A disarmed rig whose sim checks pass unless a test sets ``facts``."""
    from mhseals_testing.harness import sim_arm
    from test_sim_arm import passing
    from dataclasses import replace
    from mhseals_hardware import hold_store
    facts = passing(rig.app.paths.directory)
    state = {'facts': replace(facts, env=dict(facts.env, MHSEALS_SIM_ARM='1')),
             'nodes': ['/ros_tcp_endpoint', '/thruster_pwm_node', '/command_mux']}   # what arming started
    monkeypatch.setenv('MHSEALS_SIM_ARM', '1')       # the sim container's switch (agent.yaml ships it off)
    monkeypatch.setattr(sim_arm, 'gather', lambda app: replace(
        state['facts'], holds=tuple(sorted(hold_store.load(state['facts'].hold_file)))))
    monkeypatch.setattr(sim_arm, 'graph_facts', lambda ros, truth: (
        ('/ros_tcp_endpoint',), 12.5, ('/ros_tcp_endpoint',), tuple(state['nodes'])))
    rig.app.profile.ground_truth = '/crane/odom'
    rig.app.settings.agent_motion = 'allow'
    rig.app.is_armed = False
    rig.state = state
    return rig


def armed_by_agent(app):
    return [c for c in app.calls if isinstance(c, tuple) and c[0] == 'agent_arm']


def test_sim_arm_arms_a_verified_sim_runs_and_disarms_after(sim):
    from mhseals_testing.harness import sim_arm
    c = claim(sim)
    seen = []
    sim.app.on_step = {0: lambda: seen.append(sim.app.armed)}
    req = ask(sim, 'run', c, key='square', sim_arm=True)
    assert final(req['id'])['ok'] and seen == [True]
    assert armed_by_agent(sim.app) == [('agent_arm', 'claude-3f2a', sim_arm.CHECKS)]
    assert 'arm_panel' not in sim.app.calls and not sim.app.armed
    assert ('disarm', 'sim auto-arm: run ended', 'agent') in sim.app.calls
    assert 'sim_arming' in [r['state'] for r in replies(req['id'])]
    logged = next(f for p, f in sim.app.events if p == 'sim_arm')
    assert logged['who'] == 'claude-3f2a' and logged['claim'] == c and logged['ok']
    assert logged['passed'] == list(sim_arm.CHECKS)


def test_a_failed_sim_check_refuses_without_arming(sim):
    from dataclasses import replace
    c = claim(sim)
    sim.state['facts'] = replace(sim.state['facts'], nodes=('/mavros',))
    reply = final(ask(sim, 'run', c, key='square', sim_arm=True)['id'])
    assert reply['error'] == 'refused' and ch.exit_code(reply) == 2
    assert reply['hint'].startswith('sim-arm refused: no_hardware') and '/mavros' in reply['hint']
    assert not armed_by_agent(sim.app) and 'arm_panel' not in sim.app.calls and not sim.app.armed
    assert {'check': 'no_hardware', 'ok': False, 'detail': 'real hardware nodes: /mavros'} \
        in reply['result']['sim_arm']


def test_unreadable_sim_facts_refuse(sim, monkeypatch):
    from mhseals_testing.harness import sim_arm

    def broken(app):
        raise OSError('no graph')
    monkeypatch.setattr(sim_arm, 'gather', broken)
    reply = final(ask(sim, 'run', claim(sim), key='square', sim_arm=True)['id'])
    assert reply['error'] == 'refused' and 'facts' in reply['hint'] and not armed_by_agent(sim.app)


def test_sim_arm_off_in_agent_yaml_is_not_allowed(sim, monkeypatch):
    monkeypatch.delenv('MHSEALS_SIM_ARM')
    config = sim.desk.config.get()
    sim.desk.config = SimpleNamespace(get=lambda: dict(config, sim_arm=dict(config['sim_arm'], enabled=False)),
                                      error=None)
    reply = final(ask(sim, 'run', claim(sim), key='square', sim_arm=True)['id'])
    assert reply['error'] == 'not_allowed' and 'sim_arm.enabled' in reply['hint'] and not armed_by_agent(sim.app)


def test_a_hold_the_mux_restores_undoes_the_sim_arm_and_is_never_released(sim):
    sim.app.agent_arm_why = 'the mux holds (costmap_stale); only the operator releases it (U)'
    reply = final(ask(sim, 'run', claim(sim), key='square', sim_arm=True)['id'])
    assert reply['error'] == 'refused' and 'costmap_stale' in reply['hint'] and not sim.app.armed
    assert sim.app.running is None


def test_a_saved_hold_refuses_the_sim_arm_before_arming(sim):
    from mhseals_hardware import hold_store
    hold_store.save(hold_store.default_path(),
                    {'costmap_stale': {'cause': 'costmap_stale', 'by': 'costmap_watchdog', 'since': 1.0, 'token': None}})
    reply = final(ask(sim, 'run', claim(sim), key='square', sim_arm=True)['id'])
    assert reply['error'] == 'overridden' and 'costmap' in reply['hint'] and not armed_by_agent(sim.app)


def test_the_stop_file_ends_a_sim_armed_run_and_disarms(sim):
    from mhseals_testing.harness import sim_arm
    sim.app.on_step = {1: lambda: sim_arm.stop_path().parent.mkdir(parents=True, exist_ok=True)
                       or sim_arm.stop_path().touch()}
    reply = final(ask(sim, 'run', claim(sim), key='square', sim_arm=True)['id'])
    assert reply['error'] == 'cancelled' and reply['result']['cause'] == 'stop file'
    assert not sim.app.armed and ('stop', 'interlock: agent: stop file') in sim.app.calls   # the runner's abort


def test_a_sim_armed_run_past_max_s_is_disarmed(sim):
    config = sim.desk.config.get()
    sim.desk.config = SimpleNamespace(get=lambda: dict(config, sim_arm=dict(config['sim_arm'], max_s=1.5)),
                                      error=None)
    reply = final(ask(sim, 'run', claim(sim), key='square', sim_arm=True)['id'])
    assert reply['result']['cause'] == 'sim-arm timeout' and not sim.app.armed


def test_a_lapsed_lease_aborts_a_sim_armed_run_and_disarms(sim):
    c = claim(sim)
    sim.app.on_step = {1: lambda: sim.clock.__setitem__(0, sim.clock[0] + 60)}   # the CLI stopped renewing
    reply = final(ask(sim, 'run', c, key='square', sim_arm=True)['id'])
    assert reply['error'] == 'expired' and ('hold', 'lease') in sim.app.calls and not sim.app.armed


def test_space_during_a_sim_armed_run_stops_it(sim):
    def space():
        sim.app.operator_stop()
        raise Aborted('motion cancelled by operator')
    sim.app.on_step = {1: space}
    reply = final(ask(sim, 'run', claim(sim), key='square', sim_arm=True)['id'])
    assert reply['error'] == 'overridden: keys' and not sim.app.armed


def test_a_sim_arm_dry_run_lists_the_checks_and_arms_nothing(sim):
    from dataclasses import replace
    c = claim(sim)
    reply = final(ask(sim, 'run', c, dry=True, key='square', sim_arm=True)['id'])
    assert reply['ok'] and all(row['ok'] for row in reply['result']['sim_arm'])
    sim.state['facts'] = replace(sim.state['facts'], env=dict(sim.state['facts'].env, ROS_DOMAIN_ID='42'))
    reply = final(ask(sim, 'run', c, dry=True, key='square', sim_arm=True)['id'])
    assert reply['hint'].startswith('sim-arm refused: domain') and not armed_by_agent(sim.app)


def test_without_sim_arm_a_run_still_waits_for_the_human_arm_panel(sim):
    sim.app.arm_answer = False
    reply = final(ask(sim, 'run', claim(sim), key='square')['id'])
    assert reply['error'] == 'not_armed' and 'arm_panel' in sim.app.calls and not armed_by_agent(sim.app)


def test_agent_yaml_ships_sim_arm_off_and_only_the_sim_env_turns_it_on(sim, monkeypatch):
    from mhseals_testing.harness import sim_arm
    from mhseals_testing.harness import agent_config
    assert agent_config.load()['sim_arm']['enabled'] is False
    monkeypatch.delenv('MHSEALS_SIM_ARM')
    reply = final(ask(sim, 'run', claim(sim), key='square', sim_arm=True)['id'])
    assert reply['error'] == 'not_allowed' and 'MHSEALS_SIM_ARM=1' in reply['hint'] and not armed_by_agent(sim.app)
    assert not sim_arm.enabled({'enabled': False}, {'MHSEALS_SIM_ARM': 'true'})
    assert sim_arm.enabled({'enabled': False}, {'MHSEALS_SIM_ARM': '1'}) and sim_arm.enabled({'enabled': True}, {})


@pytest.mark.parametrize('error', [ValueError('bad'), KeyError('mux'), AttributeError('plant')])
def test_any_error_while_sim_arming_disarms_and_refuses(sim, error):
    def arm(who, checks):
        sim.app.is_armed = True                     # the driver came up, then something broke
        raise error
    sim.app.agent_arm = arm
    reply = final(ask(sim, 'run', claim(sim), key='square', sim_arm=True)['id'])
    assert reply['error'] == 'refused' and 'arming failed' in reply['hint'] and not sim.app.armed
    assert any(c[0] == 'disarm' and 'arming failed' in c[1] and c[2] == 'agent'
               for c in sim.app.calls if isinstance(c, tuple)) and sim.app.running is None


def test_a_ctrl_c_while_sim_arming_disarms_and_propagates(sim):
    def arm(who, checks):
        sim.app.is_armed = True
        raise KeyboardInterrupt
    sim.app.agent_arm = arm
    act = {'who': 'claude-3f2a', 'claim': 'c', 'req': {'id': 'r'}, 'cause': None}
    sim.desk.set_state = lambda act, state: None
    with pytest.raises(KeyboardInterrupt):
        sim.desk.arm_sim(act)
    assert not sim.app.armed and act['arming'] is False


@pytest.mark.parametrize('joins, why', [(['/mavros'], 'real hardware nodes: /mavros'),
                                        (['/zed_node'], 'real hardware nodes: /zed_node'),
                                        (['/thruster_pwm_node'], 'real hardware nodes: thruster_pwm_node')])
def test_the_periodic_recheck_disarms_when_the_graph_stops_proving_a_sim(sim, joins, why):
    from mhseals_testing.harness import sim_arm
    sim.app.steps = 8                                # past RECHECK_S, inside the motion lease
    sim.app.on_step = {2: lambda: sim.state['nodes'].extend(joins)}
    reply = final(ask(sim, 'run', claim(sim), key='square', sim_arm=True)['id'])
    assert reply['error'] == 'cancelled' and reply['result']['cause'] == f'sim-arm recheck: {why}'
    assert not sim.app.armed and ('stop', f'interlock: agent: sim-arm recheck: {why}') in sim.app.calls   # disarmed
    logged = next(f for p, f in sim.app.events if p == 'sim_arm_recheck')
    assert logged['ok'] is False and logged['refusal'] == why and sim_arm.RECHECK_S == 5.0


def test_the_recheck_keeps_a_sim_whose_graph_holds_and_ignores_its_own_driver(sim):
    sim.app.steps = 8                                # past RECHECK_S, inside the motion lease
    reply = final(ask(sim, 'run', claim(sim), key='square', sim_arm=True)['id'])
    assert reply['ok'], (reply.get('error'), reply.get('hint'), reply.get('result'))
    assert not [e for e in sim.app.events if e[0] == 'sim_arm_recheck']


def test_the_recheck_disarms_when_the_clock_goes(sim, monkeypatch):
    from mhseals_testing.harness import sim_arm
    sim.app.steps = 8                                # past RECHECK_S, inside the motion lease
    sim.app.on_step = {2: lambda: monkeypatch.setattr(sim_arm, 'graph_facts', lambda ros, truth: ((), 0.0, (), ()))}
    reply = final(ask(sim, 'run', claim(sim), key='square', sim_arm=True)['id'])
    assert reply['result']['cause'] == 'sim-arm recheck: no /clock publisher' and not sim.app.armed


HOLD = {'costmap_stale': {'cause': 'costmap_stale', 'by': 'costmap_watchdog', 'since': 1.0, 'token': None}}


def saved_hold(sim, age_s):
    import os
    from mhseals_hardware import hold_store
    path = sim.state['facts'].hold_file
    hold_store.save(path, HOLD)
    stamp = sim.desk.started_ns / 1e9 - age_s
    os.utime(path, (stamp, stamp))
    return path


def test_a_fresh_sim_session_clears_a_previous_sessions_holds_and_logs_it(sim):
    from mhseals_hardware import hold_store
    path = saved_hold(sim, 3600)                     # left by the session before this TUI started
    reply = final(ask(sim, 'run', claim(sim), key='square', sim_arm=True)['id'])
    assert reply['ok'] and hold_store.load(path) == {} and armed_by_agent(sim.app)
    logged = next(f for p, f in sim.app.events if p == 'sim_arm_stale_holds')
    assert logged['cleared'] == ['costmap_stale'] and logged['hold_file'] == path


@pytest.mark.parametrize('case', ['raised now', 'armed before', 'not a sim'])
def test_holds_of_this_session_or_outside_a_verified_sim_are_never_cleared(sim, case):
    from dataclasses import replace
    from mhseals_hardware import hold_store
    path = saved_hold(sim, -5 if case == 'raised now' else 3600)
    if case == 'armed before':
        sim.app.ever_armed = True                    # the operator armed this session: its holds are its own
    if case == 'not a sim':
        sim.state['facts'] = replace(sim.state['facts'], env=dict(sim.state['facts'].env, ROS_DOMAIN_ID='42'))
    reply = final(ask(sim, 'run', claim(sim), key='square', sim_arm=True)['id'])
    assert reply['error'] == 'refused' and hold_store.load(path) == HOLD and not armed_by_agent(sim.app)
    assert not [e for e in sim.app.events if e[0] == 'sim_arm_stale_holds']


def test_a_sim_armed_run_confirms_its_ready_prompt_as_the_agent(sim, monkeypatch):
    from mhseals_testing.ui import popups
    from mhseals_testing.ui.app import App as RealApp
    asked, answers = [], []
    monkeypatch.setattr(popups, 'prompt', lambda app, message, title='Ready': asked.append(message) or '')
    sim.app.on_step = {0: lambda: answers.append(RealApp.prompt(sim.app, 'Clear the area and press Enter... '))}
    assert final(ask(sim, 'run', claim(sim), key='square', sim_arm=True)['id'])['ok']
    assert answers == [''] and asked == [] and sim.app.agent_ready is None
    logged = next(f for p, f in sim.app.events if p == 'ready')
    assert logged['by'] == 'agent' and logged['who'] == 'claude-3f2a' and logged['sim_arm'] is True


def test_runs_without_sim_arm_keep_the_human_ready(sim, monkeypatch):
    from mhseals_testing.ui import popups
    from mhseals_testing.ui.app import App as RealApp
    asked = []
    monkeypatch.setattr(popups, 'prompt', lambda app, message, title='Ready': asked.append(message) or '')
    sim.app.is_armed = True                          # the operator armed: a plain agent run
    sim.app.on_step = {0: lambda: RealApp.prompt(sim.app, 'Clear the area and press Enter... ')}
    assert final(ask(sim, 'run', claim(sim), key='square')['id'])['ok']
    assert asked == ['Clear the area and press Enter... '] and not [e for e in sim.app.events if e[0] == 'ready']


def test_another_agents_stop_names_it_to_the_interrupted_agent_and_the_operator(rig):
    def other_stops():
        rig.desk.do_stop(ch.request('stop', 'codex-77aa', {}))
        raise Aborted('motion cancelled by operator')
    c, reply = run_with(rig, 1, other_stops)
    assert reply['error'] == 'overridden: agent codex-77aa' and ch.exit_code(reply) == 3
    assert 'agent codex-77aa stopped the boat' in reply['hint'] and '(U)' in reply['hint']
    assert ('stop', 'stopped by agent codex-77aa') in rig.app.calls
    later = final(ask(rig, 'run', c, key='square')['id'])     # the hold stays the other agent's
    assert later['error'] == 'overridden' and 'agent codex-77aa stopped the boat' in later['hint']


def test_an_agents_own_stop_and_the_operators_space_are_named_as_such(rig):
    def own():
        rig.desk.do_stop(ch.request('stop', 'claude-3f2a', {}))
        raise Aborted('motion cancelled')
    _, reply = run_with(rig, 1, own)
    assert 'codex' not in reply['hint'] and ('stop', 'stopped by agent claude-3f2a') in rig.app.calls
    rig.app.operator_stop()
    assert rig.app.stopped_by == 'operator' and ('stop', 'stopped by operator') in rig.app.calls


def test_a_claim_by_an_id_that_already_holds_one_is_refused_as_a_duplicate(rig):
    c = claim(rig)
    req = ch.request('claim', 'claude-3f2a', {'reason': 'second agent, same id'})
    rig.desk.tick()
    reply = final(req['id'])
    assert reply['error'] == 'duplicate_who' and ch.exit_code(reply) == 2
    assert 'already holds a claim' in reply['hint'] and 'own --who' in reply['hint'] and c not in reply['hint']
    assert '--claim' not in reply['hint']
    req = ch.request('claim', 'codex-77aa', {})
    rig.desk.tick()
    busy = final(req['id'])
    assert busy['error'] == 'busy' and 'claude-3f2a' in busy['hint'] and c not in busy['hint']   # who, not the id


# --- operator visibility: refusals, lapsed leases, settings switched mid-act ------------------

def test_a_refused_claim_or_run_is_a_footer_notice_naming_the_agent(rig):
    rig.app.arbiter.allow_agents = False
    ch.request('claim', 'codex-1')
    rig.desk.tick()
    assert rig.app.notice == 'Agent codex-1 claim refused: blocked (operator must set Agents to allow)'
    rig.app.arbiter.allow_agents = True
    c = claim(rig)
    rig.app.settings.agent_motion = 'observe'
    ask(rig, 'run', c, key='square')
    assert rig.app.notice.startswith(f'Agent claude-3f2a {c} run refused: motion_observe (')


def test_a_dry_run_refusal_stays_in_the_agent_log(rig):
    c = claim(rig)
    rig.app.notice = 'before'
    rig.app.settings.agent_motion = 'observe'
    ask(rig, 'run', c, dry=True, key='square')
    assert rig.app.notice == 'before'


def test_an_idle_lease_lapse_is_a_notice(rig):
    c = claim(rig)
    rig.clock[0] += 121
    rig.app.arbiter.tick()
    rig.desk.tick()
    assert rig.app.notice.startswith(f'Agent claude-3f2a {c} lease expired: stopped')
    rig.app.notice = ''
    rig.desk.tick()
    assert rig.app.notice == ''                       # told once


def test_a_lapsed_motion_lease_is_a_notice(rig):
    rig.app.steps = 30
    c, reply = run_with(rig, 0, lambda: None)
    assert reply['error'] == 'expired' and f'{c} lease expired' in rig.app.notice


@pytest.mark.parametrize('switch, cause', [
    (lambda app: agent_feed.step_setting(app, 'agent_motion', 'right'), 'motion observe'),   # allow -> observe
    (lambda app: (setattr(app.settings, 'agents', 'block'), app.desk.setting_changed()), 'agents block')])
def test_switching_a_control_row_mid_run_stops_it_as_e_does(rig, switch, cause):
    c, reply = run_with(rig, 1, lambda: switch(rig.app))
    assert reply['error'] == f'overridden: {cause}' and ch.exit_code(reply) == 3
    assert reply['hint'].startswith('operator set Agent')
    assert rig.app.calls[:2] == ['neutral', ('hold', cause.replace(' ', '_'))]   # neutral, then the hold
    assert f'{c} stopped: {cause}' in rig.app.notice
    assert rig.app.arbiter.table.get(c) is not None                                # the claim stays
    rig.desk.tick()
    assert rig.desk.act is None and rig.app.running is None                        # never resumes
    rig.app.revoke_agent()                                                         # E still wins
    assert rig.app.arbiter.table.get(c)['state'] == 'revoked' and ('hold', 'revoke') in rig.app.calls


def test_switching_observe_mid_goal_cancels_it_in_nav2(rig):
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    req = ask(rig, 'goal', c, x=5, y=0)
    agent_feed.step_setting(rig.app, 'agent_motion', 'right')
    assert rig.app.nav.goal.cancelled and rig.app.goals.active is None
    assert final(req['id'])['error'] == 'overridden: motion observe'
    assert ('hold', 'motion_observe') in rig.app.calls


def test_agents_block_refuses_a_run_under_a_live_claim(rig):
    c = claim(rig)
    rig.app.settings.agent_motion = 'allow'
    rig.app.settings.agents = 'block'
    assert final(ask(rig, 'run', c, key='square')['id'])['error'] == 'agents_block'


def test_the_status_rows_switches_reach_the_desk():
    from mhseals_testing.ui.app import App as BoatApp
    seen = []
    app = SimpleNamespace(settings=SimpleNamespace(agents='allow'), arbiter=None, notice='',
                          desk=SimpleNamespace(setting_changed=lambda: seen.append('desk')))
    BoatApp.toggle_agents(app)
    assert app.settings.agents == 'block' and seen == ['desk'] and app.notice == 'Agents block'

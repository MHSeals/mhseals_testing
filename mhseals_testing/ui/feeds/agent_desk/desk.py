"""The agent desk: the session owner reads the agent channel and executes agent acts (spec agent-control 2-4).

Only the owner of a live session has a desk (``start`` returns None for a guest or a review), so only
one process ever acts on a request. Each tick reads at most 64 channel lines, answers claim,
release, stop and mark at once, queues run, goal and ask, and starts the next queued act when no
popup is open and nothing runs. An act runs through the same code as a key (ui/agent_acts.py).

Safety, in one place:

* A human override (Space, manual, RC, a hold, a fault), ``E``, a lapsed lease or a release cancels
  the act at once and nothing resumes it. A run aborts through the runner's interlock path
  (``blockers``, neutral first); while it is paused for an override, the next tick sets
  ``app.cancel_run(cause)``. A goal is cancelled. A desk popup, and any popup of its run (``run_act``), closes.
* The motion lease: the act's blocking CLI renews every ``motion_s``/3; a lapse expires the claim
  through the arbiter, which stops exactly as ``E`` does.
* Only ``release HOLD --why`` (ui/feeds/agent_ops.py, agent/grants.py) releases a hold: a lease or
  watchdog hold once its fault cleared, an operator hold with a grant, never the e-stop. ``arm`` arms after a countdown banner the
  operator cancels; the e-stop refuses it. ``run`` of an armed procedure raises the human arm panel, except
  ``run --sim-arm``: it arms (``app.agent_arm``) only when every harness/sim_arm.py check proves a
  simulator, and disarms when the run ends, aborts, passes ``sim_arm.max_s``, loses its lease or meets the
  stop file.
* A request past its deadline is answered ``expired``, never executed; a request already answered
  (a crash between act and persist) is skipped. The read position persists in ``registry.agent``.

This module is the glue (the feed's tick, the summary, close); each part lives beside it in agent_desk/:
channel.py reads and answers, queue.py holds the acts and their order, lease.py the claim, the motion lease and
override cancel, popups.py the confirm and the ask, verbs.py and settings.py the immediate verbs, common.py the
constants. ui/feeds/agent.py re-exports it.
"""

from collections import deque
import time

from mhseals_testing.agent import channel as agent_channel, config as agent_config
from mhseals_testing.harness import background, control
from mhseals_testing.harness.goals import Goals
from mhseals_testing.ui import agent_acts
from mhseals_testing.ui.feeds.agent_desk.channel import ChannelMixin
from mhseals_testing.ui.feeds.agent_desk.common import ATTENTION_S, DRY_CLOSE_S, NOTES, NOTICED
from mhseals_testing.ui.feeds.agent_desk.lease import LeaseMixin
from mhseals_testing.ui.feeds.agent_desk.popups import PopupsMixin
from mhseals_testing.ui.feeds.agent_desk.queue import QueueMixin
from mhseals_testing.ui.feeds.agent_desk.settings import SettingsMixin
from mhseals_testing.ui.feeds.agent_desk.verbs import VerbsMixin

class AgentDesk(PopupsMixin, ChannelMixin, LeaseMixin, VerbsMixin, SettingsMixin, QueueMixin):
    """The desk; ``clock`` is the wall clock in seconds (deadlines are ``time_ns`` stamps)."""

    def __init__(self, app, ros, *, registry=None, config=None, clock=time.time, path=None, status=None):
        self.app, self.ros, self.registry, self.clock = app, ros, registry, clock
        self.goals = Goals(agent_acts.Nav2Goals(app), lambda phase, record: app.emit(phase, **record), time.time_ns)
        self.config = config or agent_config.Watch()
        saved = (registry.read() or {}).get('agent') if registry else None
        self.tail = agent_channel.Tail(path, **(saved or {}))
        if saved:
            self.answered = agent_channel.replied(saved, path)
        else:
            self.tail.seek_end()          # first start: nothing older is ours to answer
            self.answered = set()
        self.path = self.tail.path
        self.started_ns = time.time_ns()    # motion requests stamped before this are a replay
        self.queue, self.act, self.busy, self.deferred = [], None, False, []
        self.stops, self.stopped_by = 0, None   # agent stops so far and the last one's id (cancel an arm countdown)
        self.lifecycle_states, self.lifecycle_at = {}, None
        self.notes = deque(maxlen=NOTES)
        self.notices = deque(maxlen=5)
        self.recent = deque(maxlen=20)      # finished or refused acts, newest last
        self.errors, self.saved = set(), None
        self.status, self.status_at = status, None
        self.active_claim = None
        self.claim_who, self.expired_seen = {}, set()   # agent claim id -> who; lapsed leases already told
        self.dry = None                     # (req, thread, box) of the goal dry run in flight
        self.background = background.Reader(lambda: self.config.get().get('background', ''))
        self.fcu = None                     # harness.agent_fcu.AgentFcu, made on the first fcu op
        self.doing = {}                     # who -> {text, t_ns}: each agent's latest note --doing
        self.working = None                 # the op whose handler runs now: other reqs wait, then expire if late
        self.watched, self.watch_at = set(), float('-inf')   # hold watchdogs whose ~/status the desk subscribed to

    # --- the feed --------------------------------------------------------------------------

    def tick(self, app=None, snapshot=None, now=None):
        self.answer_dry()
        self.poll()
        self.watch_holds()
        if self.act is not None:
            self.watch()
        self.note_expiry()
        if self.queue:
            self.start_next()
        self.follow()
        self.persist()
        self.write_status()

    def view(self):
        return {'agent_desk': self.summary()}

    def summary(self):
        """What the banner, activity view and status.json show of the desk."""
        act = self.act
        lease = None
        if act and act.get('started'):
            lease = max(0.0, self.lease()['motion_s'] - (self.clock() - act['renewed']))
        return {'act': None if act is None else {
                    'id': act['id'], 'op': act['op'], 'who': act['who'], 'claim': act['claim'],
                    'label': act['label'], 'state': act['state'], 'elapsed_s': self.clock() - act['at'],
                    'motion_lease_s': lease, 'motion_s': self.lease()['motion_s'], 'cause': act['cause'],
                    'why': str(act['req'].get('why') or '')},
                'queue': [{'id': r['id'], 'op': r['op'], 'who': r['who'], 'why': str(r.get('why') or '')}
                          for r in self.queue],
                'recent': list(self.recent), 'notes': list(self.notes)[-20:], 'config_error': self.config.error,
                'doing': dict(self.doing), 'attention': self.attention(), 'background': self.background.get()}

    def attention(self):
        """Why an agent wants the operator now ('' when not): an ask waiting, or a refusal or a hold release in
        the last ATTENTION_S. The header shows one indicator; the details stay in the agent panel."""
        if self.act is not None and self.act['op'] == 'ask':
            return f"{self.act['who']} asks"
        now_ns = time.time_ns()
        for item in reversed(self.recent):
            if now_ns - item['t_ns'] > ATTENTION_S * 1e9:
                break
            if item['op'] == 'release_hold' and item['ok']:
                return f"{item['who']} released a hold"
            if not item['ok'] and item['op'] in NOTICED and not ended(item['error']):
                return f"{item['who']} {item['op']} refused"
        return ''

    def blockers(self):
        """Extra runner blockers while an agent run is in flight: polls the channel, enforces the lease.

        Called on every runner step, so it answers only renew, stop and release; anything else
        waits in ``deferred`` for the next tick and never stalls the key loop.
        """
        if self.act is None or self.act['op'] != 'run':
            return []
        self.poll(urgent=True)
        cause = self.watch()
        return [f'agent: {cause}'] if cause else []

        # a refusal stays in the agent panel; the header's one "agent wants attention" (``attention``) points there

    # --- around the acts -------------------------------------------------------------------

    def closed_hint(self):
        taker = getattr(self.app, 'handed_by', None)
        return (f'control taken over by {control.name(taker)}; the claim is revoked, claim again only if asked'
                if taker else 'TUI closed')

    def close(self):
        """Cancel whatever runs: a takeover never inherits an act (S3). A human takeover's hints name the taker."""
        closed = self.closed_hint()
        if self.act is not None:
            self.cancel('handover' if self.app.detaching else 'cancelled: TUI closed')
            if self.act['op'] == 'goal' and self.act['started']:
                self.watch_goal(self.act, self.act['cause'])
        self.answer_dry(DRY_CLOSE_S)               # a dry run moves nothing: answer it if it ends now
        if self.dry is not None:
            self.reply(self.dry[0], False, 'handover' if self.app.detaching else 'cancelled', hint=closed)
            self.dry = None
        for req in self.queue:
            self.reply(req, False, 'handover' if self.app.detaching else 'cancelled', hint=closed)
        self.queue = []
        self.persist()


def ended(error):
    """A running act's end the operator caused or already sees (E, Space, a lapsed lease): no refusal notice."""
    return error in ('revoked', 'expired', 'released', 'handover', 'cancelled', 'paused') \
        or error.startswith('overridden: ')


def blockers(app):
    """The desk's runner blockers (app.session adds them to every runner); [] without a desk."""
    desk = getattr(app, 'desk', None)
    return desk.blockers() if desk is not None else []


def run_act(app):
    """The agent act whose run is in flight (started), or None. Its popups close once it ends (ui/popups ``popup``)
    and its procedure answers its own save questions with the safe answer (procedure context ``agent``)."""
    act = getattr(getattr(app, 'desk', None), 'act', None)
    return act if act is not None and act['op'] == 'run' and act.get('started') else None


def step_setting(app, name, key):
    """Left/right on the Agent motion row of Control."""
    from mhseals_testing.harness.settings import BY_KEY
    choices = BY_KEY[name].choices
    value = choices[(choices.index(getattr(app.settings, name)) + (1 if key == 'right' else -1)) % len(choices)]
    setattr(app.settings, name, value)
    app.say(f"{name.replace('_', ' ').capitalize()} {value}")
    desk = getattr(app, 'desk', None)
    if desk is not None:
        desk.setting_changed()               # observe stops a run in flight, as E does


def start(app, ros, registry=None, status=None, clock=time.time):
    """The owner's desk, a feed (startup.py makes it ``app.desk``); None for a guest or a review."""
    if app.watching or app.review or app.args.review is not None:
        return None
    arbiter = getattr(app, 'arbiter', None)
    agent_channel.rotate(active=bool(arbiter and arbiter.table.holder()))
    if status is None:
        from mhseals_testing.ui import status_file
        status = status_file.write
    desk = AgentDesk(app, ros, registry=registry, clock=clock, status=status)
    app.feeds.append(desk)
    return desk

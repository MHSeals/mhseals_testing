"""The agent desk: the session owner reads the agent channel and executes agent acts (spec agent-control 2-4).

Only the owner of a live session has a desk (``start`` returns None for a guest or a review), so only
one process ever acts on a request. Each tick reads at most 64 channel lines, answers claim,
release, stop and mark at once, queues run, goal and ask, and starts the next queued act when no
popup is open and nothing runs. An act runs through the same code as a key (ui/agent_acts.py).

Safety, in one place:

* A human override (Space, manual, RC, a hold, a fault), ``E``, a lapsed lease or a release cancels
  the act at once and nothing resumes it. A run aborts through the runner's interlock path
  (``blockers``, neutral first); while it is paused for an override, the next tick sets
  ``app._runner_cancelled``. A goal is cancelled. A desk popup closes.
* The motion lease: the act's blocking CLI renews every ``motion_s``/3; a lapse expires the claim
  through the arbiter, which stops exactly as ``E`` does.
* Nothing here arms or releases a hold. ``run`` of an armed procedure raises the human arm panel.
* A request past its deadline is answered ``expired``, never executed; a request already answered
  (a crash between act and persist) is skipped. The read position persists in ``registry.agent``.
"""

from collections import deque
import time

from mhseals_testing.harness import agent_channel, agent_config, journal
from mhseals_testing.harness.authority import HUMAN
from mhseals_testing.harness.goals import Goals
from mhseals_testing.ros.arming import muxed
from mhseals_testing.ui import agent_acts

MOTION = ('run', 'goal')
NOTES = 200
PROGRESS_S = 5.0
STATUS_S = 1.0
HINTS = {'keys': 'operator must U and re-arm', 'hold': 'operator must U', 'keyboard': 'operator left manual',
         'rc': 'RC must return to auto', 'fault': 'operator must clear the fault and re-arm',
         'unknown': 'mux state is unknown'}


class AgentDesk:
    """The desk; ``clock`` is the wall clock in seconds (deadlines are ``time_ns`` stamps)."""

    def __init__(self, app, ros, *, registry=None, config=None, clock=time.time, path=None, status=None):
        self.app, self.ros, self.registry, self.clock = app, ros, registry, clock
        self.config = config or agent_config.Watch()
        saved = (registry.read() or {}).get('agent') if registry else None
        self.tail = agent_channel.Tail(path, **(saved or {}))
        if saved:
            self.answered = agent_channel.replied(saved, path)
        else:
            self.tail.seek_end()          # first start: nothing older is ours to answer
            self.answered = set()
        self.path = self.tail.path
        self.queue, self.act, self.busy = [], None, False
        self.notes = deque(maxlen=NOTES)
        self.notices = deque(maxlen=5)
        self.recent = deque(maxlen=20)      # finished or refused acts, newest last
        self.errors, self.saved = set(), None
        self.status, self.status_at = status, None
        self.active_claim = None

    # --- the feed --------------------------------------------------------------------------

    def tick(self, app=None, snapshot=None, now=None):
        self.poll()
        if self.act is not None:
            self.watch()
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
                    'motion_lease_s': lease, 'motion_s': self.lease()['motion_s'], 'cause': act['cause']},
                'queue': [{'id': r['id'], 'op': r['op'], 'who': r['who']} for r in self.queue],
                'recent': list(self.recent), 'notes': list(self.notes)[-20:], 'config_error': self.config.error}

    def blockers(self):
        """Extra runner blockers while an agent run is in flight: polls the channel, enforces the lease."""
        if self.act is None or self.act['op'] != 'run':
            return []
        self.poll()
        cause = self.watch()
        return [f'agent: {cause}'] if cause else []

    # --- the channel -----------------------------------------------------------------------

    def poll(self):
        records, errors = self.tail.read()
        for error in errors:
            if error not in self.errors:     # one notice per distinct error
                self.errors.add(error)
                self.app.notice = f'agent channel: {error}'
        for record in records:
            try:
                getattr(self, 'on_' + record['kind'])(record)
            except (KeyError, TypeError, ValueError, AttributeError) as error:   # a well-formed line, bad values
                if record['kind'] == 'req':
                    self.reply(record, False, 'bad_args', hint=str(error))
                else:
                    self.app.notice = f"agent channel: bad {record['kind']}: {error}"

    def persist(self):
        position = self.tail.position()
        if self.registry is not None and position != self.saved:
            self.saved = dict(position)
            self.registry.update(lambda entry: entry.update(agent=position))

    def reply(self, req, ok=True, error='', state='done', result=None, hint=''):
        agent_channel.reply(req['id'], req['who'], ok=ok, state=state, error=error, result=result,
                            hint=hint, path=self.path)
        if state == 'done':
            label = self.act['label'] if self.act and self.act['id'] == req['id'] else req['op']
            self.recent.append({'id': req['id'], 'op': req['op'], 'who': req['who'], 'label': label,
                                't_ns': time.time_ns(), 'ok': bool(ok), 'error': error, 'hint': hint})
        if state != 'progress':
            self.app.emit('agent_act', id=req['id'], who=req['who'], op=req['op'], state=state, error=error)

    def on_reply(self, record):
        pass

    def on_note(self, record):
        self.touch(record['who'])
        row = {k: record.get(k) for k in ('t_ns', 'who', 'phase', 'tool_id', 'cmd', 'exit', 'dur_s', 'target')}
        self.notes.append(row)
        if record['phase'] != 'pre':          # one journal line per tool call
            self.app.emit('agent_note', who=row['who'], stage=row['phase'], cmd=row['cmd'], exit=row['exit'],
                          dur_s=row['dur_s'])     # 'phase' is the journal's own key

    def on_renew(self, record):
        self.touch(record['who'], record['claim'])
        act = self.act
        if act and act['id'] == record['act'] and act['claim'] == record['claim'] and act['who'] == record['who']:
            act['renewed'] = self.clock()

    def on_req(self, req):
        if req['id'] in self.answered:
            return
        self.answered.add(req['id'])
        args = req.get('args') if isinstance(req.get('args'), dict) else {}
        req = dict(req, args=args)
        self.app.emit('agent_req', id=req['id'], who=req['who'], op=req['op'], args=args, dry=bool(req.get('dry')))
        self.touch(req['who'], req.get('claim'))
        if time.time_ns() > req['deadline_ns']:
            return self.reply(req, False, 'expired', hint='request deadline passed; nothing was done')
        handler = {'claim': self.do_claim, 'release': self.do_release, 'stop': self.do_stop,
                   'mark': self.do_mark, 'run': self.enqueue, 'goal': self.enqueue,
                   'ask': self.enqueue}.get(req['op'])
        if handler is None:
            return self.reply(req, False, 'bad_op', hint=f"unknown op {req['op']!r}")
        handler(req)

    def touch(self, who, claim=None):
        """Renew the idle lease of ``who``'s active claim (any verb or note by it)."""
        arbiter = self.arbiter
        if arbiter is None:
            return
        with arbiter.lock:
            holder = arbiter.table.holder()
            if holder and holder['who'] == who and claim in (None, holder['id']):
                arbiter.table.renew(holder['id'], self.clock())

    @property
    def arbiter(self):
        return getattr(self.app, 'arbiter', None)

    def lease(self):
        return self.config.get()['lease']

    # --- immediate ops ---------------------------------------------------------------------

    def do_claim(self, req):
        if self.arbiter is None:
            return self.reply(req, False, 'no_owner')
        idle = self.lease()['idle_s']
        ack = self.arbiter.handle({'op': 'claim', 'request': req['id'], 'who': req['who'], 'lease_s': idle,
                                   'reason': str(req['args'].get('reason', '')), 'pid': req['pid'],
                                   'host': req['host']})
        holder = ack.get('holder') or {}
        hint = {'busy': f"{holder.get('who')} holds {holder.get('id')}",
                'blocked': 'operator must set Agents to allow'}.get(ack.get('error'), '')
        self.reply(req, ack['ok'], ack.get('error', ''), result={'claim': ack.get('id'), 'lease_s': idle},
                   hint=hint)

    def do_release(self, req):
        claim = req.get('claim') or req['args'].get('claim')
        if self.act and self.act['claim'] == claim and self.act['who'] == req['who']:
            self.cancel('released')
        if self.arbiter is None:
            return self.reply(req, False, 'no_owner')
        ack = self.arbiter.handle({'op': 'release', 'id': claim, 'who': req['who'], 'request': req['id']})
        self.reply(req, ack['ok'], ack.get('error', ''))

    def do_stop(self, req):
        """Space for the agent: hold, neutral, disarm. Always allowed, never needs a claim."""
        if self.act and self.act['op'] in MOTION:
            self.cancel('stop')
        self.app.operator_stop()
        self.reply(req, True, result={'stopped': True})

    def do_mark(self, req):
        number = journal.next_marker_id(journal.read(self.app.paths.journal))
        self.app.emit('marker', kind='agent', id=number, who=req['who'], note=str(req['args'].get('note', '')),
                      t_ns=time.time_ns())
        self.reply(req, True, result={'marker': number})

    # --- acts ------------------------------------------------------------------------------

    def enqueue(self, req):
        if req['op'] in MOTION:
            refusal = self.refusal(req)
            if refusal:
                return self.reply(req, False, refusal[0], hint=refusal[1])
            if req.get('dry'):
                result = agent_acts.dry_run(self.app, req['op'], req['args'], self.app.settings.agent_motion)
                return self.reply(req, True, result=result, hint=result.get('hint', ''))
        self.queue.append(req)
        self.reply(req, True, state='queued')

    def refusal(self, req):
        """(error, hint) when a motion request cannot run now, else None."""
        app, setting = self.app, self.app.settings.agent_motion
        if setting == 'observe':
            return 'motion_observe', 'operator sets Agent motion to confirm or allow'
        found = self.claim_state(req['claim'], req['who'])
        if found:
            return found, 'claim again' if found != 'not_holder' else 'use your own claim'
        if not (muxed(app.profile) or app.profile.use_sim_time):
            return 'no_mux', 'agent motion needs a muxed profile or a sim'
        cause = self.override()
        if cause:
            return 'overridden', HINTS.get(cause, cause)
        if req['op'] == 'run':
            found, name = agent_acts.procedure(agent_acts.run_key(req['args']))
            if found is None or name not in self.config.get()['run']:
                return 'not_allowed', f"agent.yaml run: lists {', '.join(self.config.get()['run'])}"
        else:
            agent_acts.goal_args(req['args'])
        return None

    def claim_state(self, claim_id, who):
        """None for ``who``'s active claim, else why not: revoked, expired, released, not_holder."""
        arbiter = self.arbiter
        if arbiter is None:
            return 'no_owner'
        claim = arbiter.table.get(claim_id)
        if claim is None:
            return arbiter.ended.get(claim_id, 'unknown_claim' if claim_id else 'released')
        if claim['who'] != who:
            return 'not_holder'
        return None if claim['state'] == 'active' else arbiter.ended.get(claim_id, 'revoked')

    def override(self):
        """The human or fault in control right now, or ''."""
        app = self.app
        if app.armed and app.arming.fault():
            return 'fault'
        cause = app.override_cause()
        if cause == 'hold':                      # name the hold: Space's is 'keys'
            mux = getattr(self.arbiter, 'mux', None) or {}
            return str(mux.get('cause') or ((mux.get('holds') or [{}])[0] or {}).get('cause') or 'hold')
        if cause:
            return HUMAN.get(cause, cause)
        if app.manual:
            return 'keyboard'
        if self.act and self.act.get('armed') and not app.armed:
            return 'keys'                       # Space disarmed it (an unmuxed sim has no hold to see)
        return ''

    def start_next(self):
        now_ns, app = time.time_ns(), self.app
        if self.busy or self.act is not None or app.modal is not None or app.running or app.manual:
            for req in [r for r in self.queue if now_ns > r['deadline_ns']]:
                self.queue.remove(req)
                self.reply(req, False, 'busy_ui', hint='a popup was open or something was running')
            return
        req = self.queue.pop(0)
        if req['op'] in MOTION and (refusal := self.refusal(req)):
            return self.reply(req, False, refusal[0], hint=refusal[1])
        args = req['args']
        label = {'run': f'run {agent_acts.procedure(agent_acts.run_key(args))[1]}', 'ask': 'ask',
                 'goal': f"goal {args.get('x')}, {args.get('y')}"}[req['op']]
        self.act = {'id': req['id'], 'op': req['op'], 'who': req['who'], 'claim': req.get('claim'),
                    'req': req, 'label': label, 'state': 'starting', 'cause': None, 'armed': app.armed,
                    'at': self.clock(), 'renewed': self.clock(), 'started': False, 'progress_at': 0.0,
                    'replied': False}
        self.busy = True
        keep = False
        try:
            keep = getattr(self, 'act_' + req['op'])(self.act)
        except agent_acts.STOPPING as error:     # a gate, Nav2 or popup error refuses this act only
            if not self.act['replied']:
                self.finish(self.act, False, 'refused', hint=str(error) or type(error).__name__)
        finally:
            self.busy = False
            act, self.act = self.act, (self.act if keep else None)
            if not keep and act is not None and not act['replied']:   # the TUI is closing or handing over
                self.finish(act, False, 'handover' if app.detaching else 'cancelled', hint='TUI closed')

    def confirm(self, act, title):
        """Under agent_motion confirm, one Enter from the human; returns True to go on."""
        if self.app.settings.agent_motion != 'confirm':
            return True
        self.set_state(act, 'asking')
        choice, why = agent_acts.choose(self.app, title, ('Allow', 'Refuse'), self.config.get()['confirm_s'],
                                        self.watch, clock=self.clock,
                                        message=f"Agent {act['who']} · claim {act['claim']}")
        if choice == 'Allow':
            return True
        cause = act['cause'] or why
        self.finish(act, False, self.error_of(cause) if act['cause'] else 'refused',
                    hint=HINTS.get(cause.split(': ')[-1], f'operator: {cause}'))
        return False

    def act_run(self, act):
        app = self.app
        found, name = agent_acts.procedure(agent_acts.run_key(act['req']['args']))
        if not self.confirm(act, f"Agent {act['who']}: run {name} · Enter allow · Esc refuse"):
            return False
        if 'armed' in found.needs and not app.armed:
            self.set_state(act, 'waiting_arm')
            app.arm_panel()                      # the human's typed confirmation; no verb arms
            if not app.armed:
                return self.finish(act, False, 'not_armed', hint='operator did not arm')
            act['armed'] = True
        if self.watch():
            return self.finish(act, False, self.error_of(act['cause']), hint=act['cause'])
        self.begin(act)
        result, error = agent_acts.run(app, found)
        cause = act['cause'] or self.watch()     # Space aborts in the runner before the desk sees it
        if cause:
            self.finish(act, False, self.error_of(cause), result=dict(result, cause=cause),
                        hint=HINTS.get(cause.split(': ')[-1], ''))
        elif error:
            self.finish(act, False, 'cancelled', result=dict(result, cause=error))
        else:
            self.finish(act, True, result=result)
        return False

    def act_goal(self, act):
        app = self.app
        goal = agent_acts.goal_args(act['req']['args'])
        if not self.confirm(act, f"Agent {act['who']}: goal {goal[0]:.1f}, {goal[1]:.1f} · Enter allow"):
            return False
        nav2 = agent_acts.nav2_of(app)
        lifecycle = nav2.lifecycle(timeout=0.5) if nav2 is not None else {}
        facts = agent_acts.goal_facts(app, lifecycle)
        if self.watch():
            return self.finish(act, False, self.error_of(act['cause']), hint=act['cause'])
        path_m, reason = agent_acts.goal_send(app, goal, facts, act['claim'])
        if reason:
            return self.finish(act, False, 'refused', hint=reason)
        act['lifecycle'] = lifecycle
        self.begin(act, {'path_m': path_m})
        return True                              # the goal goes on; watch() polls it each tick

    def act_ask(self, act):
        args = act['req']['args']
        options = [str(o) for o in (args.get('options') or ('yes', 'no'))][:6]
        self.set_state(act, 'asking')
        choice, why = agent_acts.choose(self.app, f"Agent {act['who']} asks", options,
                                        self.config.get()['ask_s'], self.watch, clock=self.clock,
                                        message=str(args.get('question', ''))[:300], danger=False)
        if choice is not None:
            return self.finish(act, True, result={'choice': choice})
        self.finish(act, False, 'timeout' if why == 'timeout' else 'cancelled', result={'cause': why})
        return False

    def begin(self, act, result=None):
        act['started'], act['renewed'] = True, self.clock()
        self.set_state(act, 'started', result)

    def set_state(self, act, state, result=None):
        act['state'] = state
        self.reply(act['req'], True, state=state, result=result)

    def finish(self, act, ok, error='', result=None, hint=''):
        act['replied'], act['state'] = True, 'done'
        self.reply(act['req'], ok, error, result=result, hint=hint)
        return False

    @staticmethod
    def error_of(cause):
        """The reply error for a cancel cause: revoked, expired, released, overridden: <who>, handover, cancelled."""
        if cause in ('revoked', 'expired', 'released', 'handover') or cause.startswith('overridden: '):
            return cause
        return 'cancelled'

    def cancel(self, cause):
        if self.act is not None and not self.act['cause']:
            self.act['cause'] = cause

    def watch(self):
        """The act's cancel cause (set once, never cleared), acting on it; None while it may go on."""
        act = self.act
        if act is None:
            return None
        if not act['cause'] and act['op'] in MOTION:
            act['cause'] = self.lost(act)
        cause = act['cause']
        if act['op'] == 'run':
            if cause:
                self.app._runner_cancelled = True   # a paused agent run aborts instead of resuming
            elif act['started'] and self.clock() - act['progress_at'] >= PROGRESS_S:
                act['progress_at'] = self.clock()
                self.reply(act['req'], True, state='progress',
                           result={'elapsed_s': round(self.clock() - act['at'], 1), 'notice': str(self.app.notice)})
        elif act['op'] == 'goal' and act['started']:
            self.watch_goal(act, cause)
        return cause

    def lost(self, act):
        """Why a motion act must stop now: claim gone, a human override, or the motion lease lapsed."""
        state = self.claim_state(act['claim'], act['who'])
        if state:
            return state
        human = self.override()
        if human:
            return f'overridden: {human}'
        if act['started'] and self.clock() - act['renewed'] > self.lease()['motion_s']:
            self.arbiter.expire(act['claim'])       # stops exactly as E does
            return 'expired'
        return None

    def watch_goal(self, act, cause):
        goals = self.app.goals
        if goals.active is None or goals.active['claim'] != act['claim']:
            return self.end_goal(act, cause or 'cancelled', None)
        if cause:
            goals.cancel(f'agent: {cause}')
            return self.end_goal(act, cause, None)
        handle = goals.active['goal']
        goals.poll(agent_acts.goal_facts(self.app, act['lifecycle']))
        if goals.active is None:
            status = handle.status if handle.done else 'CANCELED'
            self.end_goal(act, None if status == 'SUCCEEDED' else f'goal {status.lower()}', status)

    def end_goal(self, act, cause, status):
        if cause:
            self.finish(act, False, self.error_of(cause), result={'status': status, 'cause': cause},
                        hint=HINTS.get(cause.split(': ')[-1], ''))
        else:
            self.finish(act, True, result={'status': status})
        self.act = None

    # --- around the acts -------------------------------------------------------------------

    def follow(self):
        """Agent view follow: Activity switches to the agent view when a claim turns active."""
        holder = self.arbiter.table.holder() if self.arbiter is not None else None
        claim = holder['id'] if holder and holder.get('owner') == 'agent' else None
        if claim and claim != self.active_claim and self.app.settings.agent_view == 'follow':
            self.app.activity = 'agent'
        self.active_claim = claim
        notice = str(self.app.notice or '')
        if notice and (not self.notices or self.notices[-1] != notice):
            self.notices.append(notice)

    def write_status(self):
        if self.status is None or (self.status_at is not None and self.clock() - self.status_at < STATUS_S):
            return
        self.status_at = self.clock()
        self.status(self.app, self)

    def close(self):
        """Cancel whatever runs: a takeover never inherits an act (S3)."""
        if self.act is not None:
            self.cancel('handover' if self.app.detaching else 'cancelled: TUI closed')
            if self.act['op'] == 'goal' and self.act['started']:
                self.watch_goal(self.act, self.act['cause'])
        for req in self.queue:
            self.reply(req, False, 'handover' if self.app.detaching else 'cancelled', hint='TUI closed')
        self.queue = []
        self.persist()


def blockers(app):
    """The desk's runner blockers (app.session adds them to every runner); [] without a desk."""
    desk = getattr(app, 'desk', None)
    return desk.blockers() if desk is not None else []


def step_setting(app, name, key):
    """Left/right on the Agent motion or Agent view row of Control."""
    from mhseals_testing.harness.settings import BY_KEY
    choices = BY_KEY[name].choices
    value = choices[(choices.index(getattr(app.settings, name)) + (1 if key == 'right' else -1)) % len(choices)]
    setattr(app.settings, name, value)
    app.notice = f"{name.replace('_', ' ').capitalize()} {value}"


def start(app, ros, registry=None, status=None, clock=time.time):
    """The owner's desk and ``app.goals``; None for a guest or a review."""
    if app.watching or app.review or app.args.review is not None:
        return None
    arbiter = getattr(app, 'arbiter', None)
    agent_channel.rotate(active=bool(arbiter and arbiter.table.holder()))
    app.goals = Goals(agent_acts.Nav2Goals(app), lambda phase, record: app.emit(phase, **record), time.time_ns)
    if status is None:
        from mhseals_testing.ui import status_file
        status = status_file.write
    desk = AgentDesk(app, ros, registry=registry, clock=clock, status=status)
    app.desk = desk
    app.feeds.append(desk)
    return desk

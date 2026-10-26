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
* Nothing here releases a hold. ``run`` of an armed procedure raises the human arm panel, except
  ``run --sim-arm``: it arms (``app.agent_arm``) only when every harness/sim_arm.py check proves a
  simulator, and disarms when the run ends, aborts, passes ``sim_arm.max_s``, loses its lease or meets the
  stop file.
* A request past its deadline is answered ``expired``, never executed; a request already answered
  (a crash between act and persist) is skipped. The read position persists in ``registry.agent``.
"""

from collections import deque
from dataclasses import replace
import os
import threading
import time

from mhseals_hardware import hold_store

from mhseals_testing.harness import agent_channel, agent_config, journal, sim_arm
from mhseals_testing.harness.authority import HUMAN
from mhseals_testing.harness.goals import Goals, gate
from mhseals_testing.ros.arming import muxed
from mhseals_testing.ui import agent_acts, theme

MOTION = ('run', 'goal')
NOTICED = ('claim', 'run', 'goal')   # a refusal of these is a footer notice too, not only an agent-log row
URGENT = ('stop', 'release')   # the reqs a running act's blockers answer; the rest waits for the tick
LIFECYCLE_S = 2.0              # Nav2 lifecycle states are asked at most this often
LIFECYCLE_TIMEOUT_S = 0.1      # per server and step: a down Nav2 costs at most 1.2 s
DRY_TIMEOUT_S = 0.5            # costmap and plan waits of a goal dry run (on its own thread, never the tick)
FENCE_WAIT_S = 1.0             # off the tick only: the first wait for fence_watchdog's latched fence
DRY_CLOSE_S = 0.2              # closing the desk waits this long for a dry run before answering it cancelled
NOTES = 200
PROGRESS_S = 5.0
STATUS_S = 1.0
HINTS = {'keys': 'operator must re-arm and release the hold (U)', 'hold': 'operator must U',
         # Space's hold puts the mux on manual too, so 'keyboard' is Space or manual driving
         'keyboard': 'operator took over (Space or manual): wait until they re-arm and release the hold (U)',
         'rc': 'RC must return to auto', 'fault': 'operator must clear the fault and re-arm',
         'unknown': 'mux state is unknown',
         'geofence': 'fence hold: outside the fence or in its margin facing out; the operator drives inward, '
                     'then releases (U)',
         'costmap_stale': 'costmap hold: the costmap went stale; operator must U once it is fresh',
         'operator_link': 'operator link hold: operator must U once the link is back',
         'fcu_link': 'FCU link hold: operator must U once MAVROS is back',
         'ekf_inputs': 'EKF input hold: operator must U once the sensors are back',
         'approach': 'approach guard hold: operator must U',
         'stop file': 'the sim-arm stop file exists; only the operator removes it',
         'sim-arm timeout': 'the run passed agent.yaml sim_arm.max_s; it was disarmed',
         'motion observe': 'operator set Agent motion to observe: stopped and held; operator must U',
         'agents block': 'operator set Agents to block: stopped and held; operator must U'}


def operator_popup(app):
    """Return the popup a run waits on (e.g. the procedure's "Clear the area and press Enter"), or None.

    Only the operator answers it; an agent reports it and waits (no verb answers a popup).
    """
    modal = getattr(app, 'modal', None)
    if not isinstance(modal, dict):
        return None
    title, message = modal.get('title') or '', modal.get('message') or ''
    return f'{title}: {message}' if title and message else (title or message or None)


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
        self.started_ns = time.time_ns()    # motion requests stamped before this are a replay
        self.queue, self.act, self.busy, self.deferred = [], None, False, []
        self.lifecycle_states, self.lifecycle_at = {}, None
        self.notes = deque(maxlen=NOTES)
        self.notices = deque(maxlen=5)
        self.recent = deque(maxlen=20)      # finished or refused acts, newest last
        self.errors, self.saved = set(), None
        self.status, self.status_at = status, None
        self.active_claim = None
        self.claim_who, self.expired_seen = {}, set()   # agent claim id -> who; lapsed leases already told
        self.dry = None                     # (req, thread, box) of the goal dry run in flight

    # --- the feed --------------------------------------------------------------------------

    def tick(self, app=None, snapshot=None, now=None):
        self.answer_dry()
        self.poll()
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
                    'motion_lease_s': lease, 'motion_s': self.lease()['motion_s'], 'cause': act['cause']},
                'queue': [{'id': r['id'], 'op': r['op'], 'who': r['who']} for r in self.queue],
                'recent': list(self.recent), 'notes': list(self.notes)[-20:], 'config_error': self.config.error}

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

    # --- the channel -----------------------------------------------------------------------

    def poll(self, urgent=False):
        records, errors = self.tail.read()
        for error in errors:
            if error not in self.errors:     # one notice per distinct error
                self.errors.add(error)
                self.app.notice = f'agent channel: {error}'
        records, self.deferred = self.deferred + records, []
        dry = False
        for record in records:
            if urgent and not (record['kind'] in ('renew', 'reply')
                               or (record['kind'] == 'req' and record.get('op') in URGENT)):
                self.deferred.append(record)
                continue
            if record['kind'] == 'req' and record.get('dry') and record.get('op') in MOTION \
                    and record.get('id') not in self.answered:
                if dry or self.dry is not None:   # one at a time: a goal's waits on Nav2 run off the tick
                    self.deferred.append(record)
                    continue
                dry = True
            try:
                getattr(self, 'on_' + record['kind'])(record)
            except (KeyError, TypeError, ValueError, AttributeError, ArithmeticError) as error:   # bad values
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
        if not ok and state == 'done' and req['op'] in NOTICED and not req.get('dry') and not ended(error):
            claim = req.get('claim') or ''
            self.app.notice = (f"Agent {req['who']}{' ' + claim if claim else ''} {req['op']} refused: {error}"
                               + (f' ({hint})' if hint else ''))

    def on_reply(self, record):
        pass

    def on_note(self, record):
        self.touch(record['who'])
        row = {k: record.get(k) for k in ('t_ns', 'who', 'phase', 'tool_id', 'cmd', 'exit', 'dur_s', 'target',
                                          'agent', 'use')}
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
        args = {k: agent_channel.redact(v, agent_channel.TEXT_MAX)       # any writer: the journal stays clean
                if k in agent_channel.TEXT_ARGS and isinstance(v, str) else v for k, v in args.items()}
        # never later than its own stamp allows: a replay keeps no deadline it was not sent with
        deadline = min(req['deadline_ns'], req['t_ns'] + int(self.config.get()['request_s'] * 1e9))
        req = dict(req, args=args, deadline_ns=deadline)
        self.app.emit('agent_req', id=req['id'], who=req['who'], op=req['op'], args=args, dry=bool(req.get('dry')))
        self.touch(req['who'], req.get('claim'))
        if req['op'] in MOTION and req['t_ns'] < self.started_ns:
            return self.reply(req, False, 'expired', hint='sent before this TUI started; nothing was done')
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

    def lifecycle(self):
        """Nav2's lifecycle states, asked at most every LIFECYCLE_S with short waits; {} without Nav2."""
        if self.lifecycle_at is None or self.clock() - self.lifecycle_at >= LIFECYCLE_S:
            nav2 = agent_acts.nav2_of(self.app)
            self.lifecycle_states = nav2.lifecycle(timeout=LIFECYCLE_TIMEOUT_S) if nav2 is not None else {}
            self.lifecycle_at = self.clock()
        return self.lifecycle_states

    # --- immediate ops ---------------------------------------------------------------------

    def do_claim(self, req):
        if self.arbiter is None:
            return self.reply(req, False, 'no_owner')
        idle = self.lease()['idle_s']
        ack = self.arbiter.handle({'op': 'claim', 'request': req['id'], 'who': req['who'], 'lease_s': idle,
                                   'reason': str(req['args'].get('reason', '')), 'pid': req['pid'],
                                   'host': req['host']})
        holder = ack.get('holder') or {}
        error = ack.get('error', '')
        hint = {'busy': f"{holder.get('who')} holds the boat (wait, or ask them)",
                'blocked': 'operator must set Agents to allow'}.get(error, '')
        if error == 'busy' and holder.get('who') == req['who']:   # two agents sharing one id: refuse, say so
            error = 'duplicate_who'     # no hint repeats a claim id: another agent must not drive under it
            hint = (f"the id {req['who']} already holds a claim (pid {holder.get('pid')} on {holder.get('host')}); "
                    'every agent needs its own --who (a subagent too): claim with your own id')
        self.reply(req, ack['ok'], error, result={'claim': ack.get('id'), 'lease_s': idle}, hint=hint)

    def do_release(self, req):
        claim = req.get('claim') or req['args'].get('claim')
        if self.act and self.act['claim'] == claim and self.act['who'] == req['who']:
            self.cancel('released')
        if self.arbiter is None:
            return self.reply(req, False, 'no_owner')
        ack = self.arbiter.handle({'op': 'release', 'id': claim, 'who': req['who'], 'request': req['id']})
        self.reply(req, ack['ok'], ack.get('error', ''))

    def do_stop(self, req):
        """Space for the agent: hold, neutral, disarm. Always allowed, never needs a claim. The stop is the
        agent's, not the operator's: another agent's act ends ``overridden: agent W`` and the hold's hints name W."""
        if self.act and self.act['op'] in MOTION:
            self.cancel('stop' if self.act['who'] == req['who'] else f"overridden: agent {req['who']}")
        self.app.operator_stop(agent=req['who'])
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
                if self.act is not None or self.busy or self.app.running:
                    return self.reply(req, False, 'busy', hint='an act or procedure is running; dry-run after it')
                if req['op'] == 'goal':
                    return self.start_dry(req)
                result = agent_acts.dry_run(self.app, req['op'], req['args'], self.app.settings.agent_motion)
                if req['args'].get('sim_arm') and not self.app.armed:
                    _, checks = self.sim_checks()
                    why = sim_arm.refusal(checks)
                    result = dict(result, sim_arm=[{'check': n, 'ok': ok, 'detail': d} for n, ok, d in checks],
                                  hint=why or ('' if result.get('availability') else 'sim-arm checks pass'))
                return self.reply(req, True, result=result, hint=result.get('hint', ''))
        self.queue.append(req)
        self.reply(req, True, state='queued')

    def start_dry(self, req):
        """A goal dry run on its own thread: Nav2's lifecycle, GetCostmap and the planner can take seconds,
        and the tick (Space) must never wait on them. ``answer_dry`` replies once it ends."""
        box, setting = {}, self.app.settings.agent_motion

        def work():
            try:
                box['result'] = agent_acts.dry_run(self.app, 'goal', req['args'], setting, self.lifecycle(),
                                                   DRY_TIMEOUT_S, fence_wait=FENCE_WAIT_S)
            except Exception as error:      # noqa: BLE001 - answered as the tick would
                box['error'] = error
        thread = threading.Thread(target=work, name='agent-dry', daemon=True)
        self.dry = (req, thread, box)
        thread.start()

    def answer_dry(self, wait_s=0.0):
        """Reply to the goal dry run once its thread ended (waiting at most ``wait_s``)."""
        if self.dry is None:
            return
        req, thread, box = self.dry
        thread.join(wait_s)
        if thread.is_alive():
            return
        self.dry = None
        error = box.get('error')
        if error is None:
            return self.reply(req, True, result=box['result'], hint=box['result'].get('hint', ''))
        bad = isinstance(error, (KeyError, TypeError, ValueError, AttributeError, ArithmeticError))
        self.reply(req, False, 'bad_args' if bad else 'refused', hint=str(error) or type(error).__name__)

    def refusal(self, req):
        """(error, hint) when a motion request cannot run now, else None."""
        app, setting = self.app, self.app.settings.agent_motion
        if setting == 'observe':
            return 'motion_observe', 'operator sets Agent motion to confirm or allow'
        if app.settings.agents == 'block':
            return 'agents_block', 'operator sets Agents to allow'
        found = self.claim_state(req['claim'], req['who'])
        if found:
            return found, 'claim again' if found != 'not_holder' else 'use your own claim'
        if not (muxed(app.profile) or app.profile.use_sim_time):
            return 'no_mux', 'agent motion needs a muxed profile or a sim'
        cause = self.human()
        if cause:
            return 'overridden', self.hint_of(cause, cause)
        if req['op'] == 'run':
            found, name = agent_acts.procedure(agent_acts.run_key(req['args']))
            if found is None or name not in self.config.get()['run']:
                return 'not_allowed', f"agent.yaml run: lists {', '.join(self.config.get()['run'])}"
            task = agent_acts.run_task(req['args'])
            if task is not None and name != 'mission':
                return 'bad_args', '--task is for run mission only'
            if agent_acts.run_profile(req['args']) is not None and name != 'mission':
                return 'bad_args', '--mission-profile is for run mission only'
            if task is not None and task not in agent_acts.mission_tasks():
                return 'bad_args', f"mission tasks: {', '.join(agent_acts.mission_tasks())}"
            if req['args'].get('sim_arm') not in (None, True):
                return 'bad_args', 'sim_arm is true or absent'
            if req['args'].get('sim_arm') and not sim_arm.enabled(self.config.get()['sim_arm']):
                return 'not_allowed', f'sim-arm is off (agent.yaml sim_arm.enabled, {sim_arm.ENABLE_ENV}=1 in the sim)'
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
        if not app.armed and muxed(app.profile):
            return self.saved_hold()            # Space suspends claims until the human releases its hold
        return ''

    def saved_hold(self):
        """The oldest hold the stopped mux saved and restores on arming, as its cause; '' when none."""
        section = (getattr(self.app.args, 'hardware_config', None) or {}).get('command_mux') or {}
        holds = hold_store.load(section.get('state_file') or hold_store.default_path())
        return str(min(holds.values(), key=lambda h: h['since'])['cause']) if holds else ''

    def start_next(self):
        now_ns, app = time.time_ns(), self.app
        if self.busy or self.act is not None or app.modal is not None or app.running or app.manual \
                or self.dry is not None:            # strictly one Nav2 user at a time: a dry run first
            for req in [r for r in self.queue if now_ns > r['deadline_ns']]:
                self.queue.remove(req)
                self.reply(req, False, 'busy_ui', hint='a popup was open or something was running')
            return
        req = self.queue.pop(0)
        if req['op'] in MOTION and (refusal := self.refusal(req)):
            return self.reply(req, False, refusal[0], hint=refusal[1])
        args = req['args']
        label = {'run': ' '.join(filter(None, ('run', agent_acts.procedure(agent_acts.run_key(args))[1],
                                                agent_acts.run_task(args)))), 'ask': 'ask',
                 'goal': f"goal {args.get('x')}, {args.get('y')}"}[req['op']]
        self.act = {'id': req['id'], 'op': req['op'], 'who': req['who'], 'claim': req.get('claim'),
                    'req': req, 'label': label, 'state': 'starting', 'cause': None, 'armed': app.armed,
                    'at': self.clock(), 'renewed': self.clock(), 'started': False, 'progress_at': 0.0,
                    'replied': False}
        self.busy = True
        keep = False
        try:
            keep = getattr(self, 'act_' + req['op'])(self.act)
        except (*agent_acts.STOPPING, ArithmeticError) as error:   # gate, Nav2, popup or math: this act only
            if not self.act['replied']:
                self.finish(self.act, False, 'refused', hint=str(error) or type(error).__name__)
        except KeyboardInterrupt:                # Ctrl+C: the TUI goes away under the act
            self.cancel('handover')
            if not self.act['replied']:
                self.finish(self.act, False, 'handover', hint='TUI interrupted')
            raise
        finally:
            self.busy = False
            act, self.act = self.act, (self.act if keep else None)
            if not keep and act is not None and not act['replied']:   # the TUI is closing or handing over
                self.finish(act, False, 'handover' if app.detaching else 'cancelled', hint='TUI closed')

    def confirm(self, act, title):
        """Under agent_motion confirm, Allow from the human (Refuse is focused); returns True to go on."""
        if self.app.settings.agent_motion != 'confirm':
            return True
        self.set_state(act, 'asking')
        choice, why = agent_acts.choose(self.app, title, ('Allow', 'Refuse'), self.config.get()['confirm_s'],
                                        self.watch, clock=self.clock,
                                        message=f"Agent {act['who']} · claim {act['claim']}")
        if choice == 'Allow':
            return True
        cause = act['cause'] or why or 'refused'      # the Refuse button names no cause
        self.finish(act, False, self.error_of(cause) if act['cause'] else 'refused',
                    hint=self.hint_of(cause, f'operator: {cause}'))
        return False

    def act_run(self, act):
        app = self.app
        found, name = agent_acts.procedure(agent_acts.run_key(act['req']['args']))
        task = agent_acts.run_task(act['req']['args'])
        if not self.confirm(act, f"Agent {act['who']}: run {name}{' ' + task if task else ''} · allow?"):
            return False
        if 'armed' in found.needs and not app.armed and act['req']['args'].get('sim_arm'):
            if not self.arm_sim(act):
                return False
            app.agent_ready = act['who']         # this run's checks passed: its Ready prompts are the agent's
        elif 'armed' in found.needs and not app.armed:
            self.set_state(act, 'waiting_arm')
            act['arming'] = True                 # the mux starts with arming: judged once it reported
            try:
                app.arm_panel()                  # the human's typed confirmation; no verb arms
            finally:
                act['arming'] = False
            if not app.armed:
                return self.finish(act, False, 'not_armed', hint='operator did not arm')
            act['armed'] = True
        try:
            if self.watch():
                return self.finish(act, False, self.error_of(act['cause']), hint=act['cause'])
            # started says who picks the mission task, so 'started' is not read as the course running
            self.begin(act, {'task': task or 'the operator picks it'} if name == 'mission' else None)
            result, error = agent_acts.run(app, found, task, agent_acts.run_profile(act['req']['args']))
            cause = act['cause'] or self.watch()     # Space aborts in the runner before the desk sees it
            if cause:
                self.finish(act, False, self.error_of(cause), result=dict(result, cause=cause),
                            hint=self.hint_of(cause, ''))
            elif error:
                self.finish(act, False, 'cancelled', result=dict(result, cause=error))
            else:
                self.finish(act, True, result=result)
            return False
        finally:
            app.agent_ready = None
            if act.get('sim_armed') and app.armed:    # whatever ended the run: what the desk armed, it disarms
                app.disarm(f"sim auto-arm: {act['cause'] or 'run ended'}", by='agent')

    def sim_checks(self):
        """(facts, harness/sim_arm.py's checks) on the live app; any error reading a fact refuses."""
        config = self.config.get()['sim_arm']
        try:
            facts = sim_arm.gather(self.app)
            return facts, sim_arm.checks(facts, config)
        except Exception as error:      # noqa: BLE001 - an unreadable fact proves nothing: refuse
            return None, [('facts', False, f'unreadable: {error!r}')]

    def clear_stale_holds(self, facts, checks):
        """Owner decision 2026-10-26: at the start of a brand-new sim session (this TUI never armed, and
        the session's own hold file predates it) holds a previous session left there are cleared and
        logged. Every other check must have passed: never outside a verified sim, never a hold raised now.
        Returns the checks to judge (re-gathered after a clear)."""
        app = self.app
        if facts is None or getattr(app, 'ever_armed', False):
            return checks
        try:
            fresh = os.stat(facts.hold_file).st_mtime_ns < self.started_ns
        except OSError:
            return checks
        if not sim_arm.stale_holds(facts, checks, fresh):
            return checks
        hold_store.save(facts.hold_file, {})
        app.emit('sim_arm_stale_holds', who='agent', cleared=list(facts.holds), hold_file=facts.hold_file)
        app.notice = f"Sim auto-arm cleared a previous session's holds: {', '.join(facts.holds)}"
        return self.sim_checks()[1]

    def arm_sim(self, act):
        """Arm a verified sim for ``act`` (``run --sim-arm``); True once armed, else finished refused."""
        app = self.app
        self.set_state(act, 'sim_arming')
        facts, checks = self.sim_checks()
        checks = self.clear_stale_holds(facts, checks)
        why = sim_arm.refusal(checks)
        app.emit('sim_arm', who=act['who'], claim=act['claim'], ok=not why, passed=sim_arm.passed(checks),
                 refusal=why)
        listed = [{'check': n, 'ok': ok, 'detail': d} for n, ok, d in checks]
        if why:
            return self.finish(act, False, 'refused', result={'sim_arm': listed}, hint=why)
        act['arming'] = True                 # the mux starts with arming: judged once it reported
        try:
            why = app.agent_arm(act['who'], sim_arm.passed(checks))
        except BaseException as error:      # whatever escapes arming: what may be armed is disarmed
            why = f'arming failed: {error!r}'
            if app.armed:
                app.disarm(f'sim auto-arm undone: {why}', by='agent')
            if not isinstance(error, Exception):
                raise
        finally:
            act['arming'] = False
        if why or not app.armed:
            if app.armed:
                app.disarm(f'sim auto-arm undone: {why}', by='agent')
            return self.finish(act, False, 'refused', result={'sim_arm': listed},
                               hint=f"sim-arm: {why or 'not armed'}")
        act['armed'] = act['sim_armed'] = True
        act['sim_armed_at'] = act['rechecked_at'] = self.clock()
        act['sim_domain'] = os.environ.get('ROS_DOMAIN_ID', '')
        return True

    def sim_recheck(self, act):
        """Every sim_arm.RECHECK_S while sim-armed: the graph still proves a sim (TOCTOU), else the cause."""
        if self.clock() - act['rechecked_at'] < sim_arm.RECHECK_S:
            return None
        act['rechecked_at'] = self.clock()
        why = sim_arm.recheck(self.app.ros, getattr(self.app.profile, 'ground_truth', None) or '',
                              act['sim_domain'], self.config.get()['sim_arm'])
        if not why:
            return None
        self.app.emit('sim_arm_recheck', who=act['who'], claim=act['claim'], ok=False, refusal=why)
        return f'sim-arm recheck: {why}'

    def act_goal(self, act):
        app = self.app
        goal = agent_acts.goal_args(act['req']['args'])
        facts = agent_acts.goal_facts(app, self.lifecycle())
        reason = gate(replace(facts, costmap_age_s=0.0)) or agent_acts.in_background(
            app, f"Agent {act['who']}: goal {goal[0]:.1f}, {goal[1]:.1f}",
            lambda: agent_acts.fence_check(app, goal, facts),      # toLL may wait: off the key loop
            message='Checking the fence', on_abort=lambda: self.cancel('handover'))
        if reason:                               # never ask Allow for a goal the gates already refuse (the
            #                                      send fetches the costmap fresh and checks it all again)
            return self.finish(act, False, 'refused', hint=reason)
        if not self.confirm(act, f"Agent {act['who']}: goal {goal[0]:.1f}, {goal[1]:.1f} · allow?"):
            return False
        if self.watch():
            return self.finish(act, False, self.error_of(act['cause']), hint=act['cause'])

        def send():                              # off the key loop: Nav2 may take seconds to answer
            lifecycle = self.lifecycle()
            facts = agent_acts.goal_facts(app, lifecycle, fence_wait=FENCE_WAIT_S)
            return lifecycle, agent_acts.goal_send(app, goal, facts, act['claim'], lambda: act['cause'])
        try:
            lifecycle, (path_m, reason) = agent_acts.in_background(
                app, f"Agent {act['who']}: goal {goal[0]:.1f}, {goal[1]:.1f}", send,
                message='Checking Nav2 and sending the goal', on_abort=lambda: self.cancel('handover'))
        except KeyboardInterrupt:
            self.drop_goal(act, 'handover')     # Ctrl+C: a goal that landed meanwhile is cancelled
            raise
        if self.watch():                         # Space, E or a lapse while Nav2 answered
            self.drop_goal(act, act['cause'])
            return self.finish(act, False, self.error_of(act['cause']), hint=act['cause'])
        if reason:
            return self.finish(act, False, 'refused', hint=reason)
        act['lifecycle'] = lifecycle
        self.begin(act, {'path_m': path_m})
        return True                              # the goal goes on; watch() polls it each tick

    def drop_goal(self, act, cause):
        goals = self.app.goals
        if goals.active is not None and goals.active['claim'] == act['claim']:
            goals.cancel(f'agent: {cause}')

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

    def human(self):
        """``override()``, with a Space-style hold an agent's ``stop`` set named ``agent W``."""
        cause = self.override()
        stopper = str(getattr(self.app, 'stopped_by', '') or '')
        if cause in ('keys', 'keyboard') and stopper.startswith('agent '):
            return stopper
        return cause

    @staticmethod
    def hint_of(cause, default=''):
        """The hint for a cancel or refusal cause; ``agent W`` says another agent stopped the boat."""
        key = cause.split(': ')[-1]
        if key.startswith('agent '):
            return (f'{key} stopped the boat (boat_agent stop), not the operator; the operator must re-arm and '
                    'release the hold (U)')
        return HINTS.get(key, default)

    @staticmethod
    def error_of(cause):
        """The reply error for a cancel cause: revoked, expired, released, overridden: <who>, handover, cancelled."""
        if cause in ('revoked', 'expired', 'released', 'handover') or cause.startswith('overridden: '):
            return cause
        return 'cancelled'

    def cancel(self, cause):
        if self.act is not None and not self.act['cause']:
            self.act['cause'] = cause

    def take_over(self):
        """Manual control starts: an agent goal is cancelled in Nav2 now (rc 3, never resumed)."""
        act = self.act
        if act is not None and act['op'] == 'goal' and act['started']:
            self.cancel('overridden: keyboard')
            self.watch_goal(act, act['cause'])

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
                           result={'elapsed_s': round(self.clock() - act['at'], 1), 'notice': str(self.app.notice),
                                   'waiting_for_operator': operator_popup(self.app)})
        elif act['op'] == 'goal' and act['started']:
            self.watch_goal(act, cause)
        return cause

    def lost(self, act):
        """Why a motion act must stop now: claim gone, a human override, or the motion lease lapsed."""
        state = self.claim_state(act['claim'], act['who'])
        if state:
            return state
        setting = self.operator_setting()
        if setting:                                 # switched mid-act: stop as E does, the claim stays
            self.arbiter.halt(setting.replace(' ', '_'))
            self.app.notice = (f"Agent {act['who']} {act['claim']} stopped: {setting}{theme.SEPARATOR}"
                               'neutral, hold (U releases)')
            return f'overridden: {setting}'
        human = '' if act.get('arming') else self.human()
        if human:
            return f'overridden: {human}'
        if act.get('sim_armed') and sim_arm.stop_path().exists():
            return 'stop file'
        if act.get('sim_armed') and self.clock() - act['sim_armed_at'] > self.config.get()['sim_arm']['max_s']:
            return 'sim-arm timeout'
        if act.get('sim_armed'):
            why = self.sim_recheck(act)
            if why:
                return why
        if self.clock() - act['renewed'] > self.lease()['motion_s']:   # in the confirm popup too
            self.arbiter.expire(act['claim'])       # stops exactly as E does
            self.note_expiry()
            return 'expired'
        return None

    def operator_setting(self):
        """'motion observe' or 'agents block' while a Control row forbids agent motion, else ''."""
        settings = self.app.settings
        if settings.agent_motion == 'observe':
            return 'motion observe'
        return 'agents block' if settings.agents == 'block' else ''

    def setting_changed(self):
        """The operator switched Agents or Agent motion: a motion act in flight stops now (``lost``)."""
        if self.act is not None and self.act['op'] in MOTION and self.operator_setting():
            self.watch()

    def note_expiry(self):
        """A lapsed lease (idle or motion) ended a claim and stopped the boat as E: tell the operator once."""
        arbiter = self.arbiter
        if arbiter is None:
            return
        with arbiter.lock:
            self.claim_who.update({c['id']: c['who'] for c in arbiter.table.claims if c['owner'] == 'agent'})
            lapsed = [cid for cid, why in arbiter.ended.items() if why == 'expired' and cid not in self.expired_seen]
        for cid in lapsed:
            self.expired_seen.add(cid)
            if cid in self.claim_who:
                self.app.notice = (f'Agent {self.claim_who[cid]} {cid} lease expired: '
                                   f'stopped{theme.SEPARATOR}neutral, hold (U releases)')

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
                        hint=self.hint_of(cause, ''))
        else:
            self.finish(act, True, result={'status': status})
        self.act = None

    # --- around the acts -------------------------------------------------------------------

    def follow(self):
        """Activity switches to the agent view whenever an agent claim turns active."""
        holder = self.arbiter.table.holder() if self.arbiter is not None else None
        claim = holder['id'] if holder and holder.get('owner') == 'agent' else None
        if claim and claim != self.active_claim:
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
        self.answer_dry(DRY_CLOSE_S)               # a dry run moves nothing: answer it if it ends now
        if self.dry is not None:
            self.reply(self.dry[0], False, 'handover' if self.app.detaching else 'cancelled', hint='TUI closed')
            self.dry = None
        for req in self.queue:
            self.reply(req, False, 'handover' if self.app.detaching else 'cancelled', hint='TUI closed')
        self.queue = []
        self.persist()


def ended(error):
    """A running act's end the operator caused or already sees (E, Space, a lapsed lease): no refusal notice."""
    return error in ('revoked', 'expired', 'released', 'handover', 'cancelled') or error.startswith('overridden: ')


def blockers(app):
    """The desk's runner blockers (app.session adds them to every runner); [] without a desk."""
    desk = getattr(app, 'desk', None)
    return desk.blockers() if desk is not None else []


def step_setting(app, name, key):
    """Left/right on the Agent motion row of Control."""
    from mhseals_testing.harness.settings import BY_KEY
    choices = BY_KEY[name].choices
    value = choices[(choices.index(getattr(app.settings, name)) + (1 if key == 'right' else -1)) % len(choices)]
    setattr(app.settings, name, value)
    app.notice = f"{name.replace('_', ' ').capitalize()} {value}"
    desk = getattr(app, 'desk', None)
    if desk is not None:
        desk.setting_changed()               # observe stops a run in flight, as E does


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

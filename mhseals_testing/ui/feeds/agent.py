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
* Only ``release HOLD --why`` (ui/feeds/agent_ops.py, agent/grants.py) releases a hold: a lease or
  watchdog hold once its fault cleared, an operator hold with a grant, never the e-stop. ``arm`` arms after a countdown banner the
  operator cancels; the e-stop refuses it. ``run`` of an armed procedure raises the human arm panel, except
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

from mhseals_testing.harness import control, operator_state, sim_arm
from mhseals_testing.agent import channel as agent_channel, config as agent_config
from mhseals_testing.harness import background
from mhseals_testing.harness.goals import Goals, gate
from mhseals_testing.procedures import unavailable
from mhseals_testing.ros.arming import muxed
from mhseals_testing.ui import agent_acts
from mhseals_testing.ui.feeds.agent_desk.channel import ChannelMixin
from mhseals_testing.ui.feeds.agent_desk.popups import PopupsMixin
from mhseals_testing.ui.feeds.agent_desk.verbs import VerbsMixin
from mhseals_testing.ui.feeds.agent_desk.lease import LeaseMixin
from mhseals_testing.ui.feeds.agent_desk.popups import operator_popup  # noqa: F401 - re-exported
from mhseals_testing.ui.feeds.agent_desk.common import (  # noqa: F401 - re-exported
    MOTION, NOTICED, URGENT, LIFECYCLE_S, LIFECYCLE_TIMEOUT_S, DRY_TIMEOUT_S, FENCE_WAIT_S, DRY_CLOSE_S, NOTES,
    PART_OPS, FCU_OPS, FCU_CLAIMED, CHANGING, NEVER_PARTS, PROGRESS_S, STATUS_S, NAV2_PREFIX, ATTENTION_S, HINTS)


class AgentDesk(PopupsMixin, ChannelMixin, LeaseMixin, VerbsMixin):
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

    # --- the channel -----------------------------------------------------------------------

        # a refusal stays in the agent panel; the header's one "agent wants attention" (``attention``) points there

    # --- immediate ops ---------------------------------------------------------------------

    def do_set(self, req):
        """boat_agent set KEY=VALUE: any tunable setting (harness/settings.py, owner decision 2026-11-01), as the
        operator's Settings change, under the agent's claim and between runs; armed-unsafe keys (reload.yaml, and
        never amplitude or command_mode) only while disarmed. ``nav2.NODE.PARAM=V`` sets a live Nav2 key."""
        app = self.app
        if app.settings.agents == 'block':
            return self.reply(req, False, 'blocked', hint='operator must set Agents to allow')
        if not req.get('claim'):
            return self.reply(req, False, 'unknown_claim', hint='set needs your claim: --claim ID')
        found = self.claim_state(req['claim'], req['who'])
        if found:
            return self.reply(req, False, found, hint='claim again' if found != 'not_holder' else 'use your own claim')
        asked = req['args'].get('settings') or {}
        if not isinstance(asked, dict) or not asked:
            return self.reply(req, False, 'bad_args', hint='set KEY=VALUE [KEY=VALUE ...]')
        nav2 = {str(k)[len(NAV2_PREFIX):]: v for k, v in asked.items() if str(k).startswith(NAV2_PREFIX)}
        values = {}
        for key, value in asked.items():
            if str(key).startswith(NAV2_PREFIX):
                continue
            values[key], refusal = agent_config.tunable_refusal(str(key), value, app.armed)
            if refusal:
                return self.reply(req, False, refusal[0], hint=refusal[1])
        if app.running or app.manual or self.act is not None or self.busy:
            return self.reply(req, False, 'busy', hint='a procedure or act is running: set between runs, '
                                                       'never mid-segment')
        result = {}
        if nav2:
            ok, error, hint, applied = self.set_nav2(req, nav2)
            if not ok:
                return self.reply(req, False, error, hint=hint, result=applied)
            result['nav2'] = applied
        if values:
            error = self.apply_settings(values, req['who'])
            if error:                     # command_mode's hot switch refused (switch_command_mode): nothing set
                return self.reply(req, False, 'refused', hint=error, result=result)
            result['set'] = values
        self.reply(req, True, result=result)

    def set_nav2(self, req, requested):
        """Live Nav2 keys (``nav2.NODE.PARAM``) through nav2/live.py, the TUI's own apply path: preview,
        fences, a tokened hold, set, read back, roll back on any doubt. Off the key loop. ``(ok, error, hint,
        result)``."""
        import yaml
        live = self.app.live_nav2() if hasattr(self.app, 'live_nav2') else None
        if live is None:
            return False, 'refused', 'no live Nav2 here (a watcher, a review or no Nav2 profile)', None
        values = {k: yaml.safe_load(v) if isinstance(v, str) else v for k, v in requested.items()}
        live.claim = req.get('claim')
        title = f"Agent {req['who']}: setting Nav2 {', '.join(values)}"
        def work():
            receipt = live.apply(values)
            return receipt, (live.readback(values) if hasattr(live, 'readback') else {})
        try:
            receipt, now = agent_acts.in_background(self.app, title, work, message=title)
        except Exception as fault:      # noqa: BLE001 - a Nav2 timeout: told, never raised
            return False, 'refused', f'{type(fault).__name__}: {fault}', None
        status = receipt.get('status')
        changed = {r['key']: r for r in receipt.get('rows', ())}
        # one row per requested key, read back from the node after the apply: an unchanged key says so
        rows = [{'key': key, 'requested': value, 'readback': now.get(key),
                 **({'old': changed[key]['old'], 'new': changed[key]['new']} if key in changed else {'unchanged': True})}
                for key, value in values.items()]
        rows += [{'key': k, 'old': r['old'], 'new': r['new'], 'derived': True} for k, r in changed.items() if k not in values]
        self.app.emit('agent_nav2', who=req['who'], keys=sorted(values), status=status,
                      readback={r['key']: r['readback'] for r in rows if 'readback' in r})
        if status != 'verified':
            why = receipt.get('error') or '; '.join(receipt.get('reasons') or []) or str(receipt.get('refused'))
            return False, 'refused', f'nav2 {status}: {why}', {'rows': rows}
        return True, '', '', {'rows': rows, 'reset': receipt.get('reset')}

    def settings_of(self, req, allowed, idle=True):
        """(``{key: parsed value}``, None) for the request's ``settings``, or (None, (error, hint)): an unknown,
        unlisted, never or out-of-choices key, or with ``idle`` any while a procedure, act or manual drive runs."""
        asked = req['args'].get('settings') or {}
        if not isinstance(asked, dict):
            return None, ('bad_args', 'settings is {key: value}')
        values = {}
        for key, value in asked.items():
            values[key], refusal = agent_config.setting_refusal(str(key), value, allowed)
            if refusal:
                return None, refusal
        app = self.app
        if idle and values and (app.running or app.manual or self.act is not None or self.busy):
            return None, ('busy', 'a procedure or act is running: settings change between runs, never mid-segment')
        return values, None

    def apply_settings(self, values, who):
        """Each value as the operator's change; the first refusal (command_mode's hot switch) stops there."""
        for key, value in values.items():
            error = self.app.set_setting(key, value, who=who)
            if error:
                return error
        return ''


    # --- acts ------------------------------------------------------------------------------

    def enqueue(self, req):
        if req['op'] in MOTION:
            refusal = self.refusal(req)
            if refusal:
                return self.refuse(req, refusal)
            if req.get('dry'):
                if self.act is not None or self.busy or self.app.running:
                    return self.reply(req, False, 'busy', hint='an act or procedure is running; dry-run after it')
                if req['op'] == 'goal':
                    return self.start_dry(req)
                result = agent_acts.dry_run(self.app, req['op'], req['args'], self.app.settings.agent_motion)
                if req['args'].get('settings'):
                    result = dict(result, set=self.settings_of(req, agent_config.settable(
                        self.config.get(), result['procedure']))[0])
                if req['args'].get('sim_arm') and not self.app.armed:
                    _, checks = self.sim_checks()
                    why = sim_arm.refusal(checks)
                    result = dict(result, sim_arm=[{'check': n, 'ok': ok, 'detail': d} for n, ok, d in checks],
                                  hint=why or ('' if result.get('availability') else 'sim-arm checks pass'))
                return self.reply(req, True, result=result, hint=result.get('hint', ''))
        req['renewed'] = self.clock()          # a queued run or goal's CLI renews it as it waits (on_renew)
        self.queue.append(req)
        cause = self.busy_cause()
        self.reply(req, True, state='queued', hint=f'waits up to {self.queue_s():g} s: {cause}' if cause else '')

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
        state = operator_state.current()
        if operator_state.paused(state):         # a request queued before the pause is refused as it starts too
            return 'paused', operator_state.refusal(state)
        if setting == 'observe':
            return 'motion_observe', 'operator sets Agent motion to confirm or allow'
        if app.settings.agents == 'block':
            return 'agents_block', 'operator sets Agents to allow'
        found = self.claim_state(req['claim'], req['who'])
        if found:
            return found, self.claim_hint(found, req['claim'])
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
            refusal = self.settings_of(req, agent_config.settable(self.config.get(), name))[1]
            if refusal:
                return refusal
        else:
            agent_acts.goal_args(req['args'])
        return None

    def refuse(self, req, refusal):
        """Answer ``refusal``'s (error, hint); a paused one carries the operator state, as on_req's does."""
        paused = refusal[0] == 'paused'
        return self.reply(req, False, refusal[0], hint=refusal[1],
                          result={'operator': operator_state.summary(operator_state.current())} if paused else None)

    def busy_cause(self):
        """What keeps the next queued act from starting, or '' (strictly one Nav2 user at a time: a dry run too)."""
        app = self.app
        if app.modal is not None:
            title = app.modal.get('title') if isinstance(app.modal, dict) else ''
            return f'popup {title!r} is open' if title else 'a popup is open'
        if self.busy or self.act is not None:
            return f"agent act {(self.act or {}).get('label', '')!r} is running".replace(" '' ", ' ')
        if app.running:
            return f'procedure {app.running!r} is running'
        if app.manual:
            return 'the operator drives manually'
        return 'a goal dry run is in flight' if self.dry is not None else ''

    def queue_s(self):
        """agent.yaml queue_s: how long a queued run, goal or ask waits behind a busy TUI before busy_ui."""
        return float(self.config.get().get('queue_s', agent_config.QUEUE_S))

    def start_next(self):
        now_ns, app = time.time_ns(), self.app
        motion_s = self.lease()['motion_s']
        for req in [r for r in self.queue if r['op'] in MOTION and self.clock() - r['renewed'] > motion_s]:
            self.queue.remove(req)     # its CLI went away while it waited: nothing starts later for nobody
            self.reply(req, False, 'expired', hint='the CLI stopped waiting while it was queued; nothing was done')
        cause = self.busy_cause()
        if cause:      # an ask waits behind a non-modal state up to ask_s; the rest (and an ask behind a popup) queue_s
            wait = int(self.config.get()['ask_s'] * 1e9)
            queue = int(self.queue_s() * 1e9)
            for req in [r for r in self.queue if now_ns > max(r['deadline_ns'], r['t_ns'] + (
                    wait if r['op'] == 'ask' and app.modal is None else queue))]:
                self.queue.remove(req)
                self.reply(req, False, 'busy_ui', hint=f'{cause} after {self.queue_s():g} s queued; try again after it')
            return
        if not self.queue:
            return
        req = self.queue.pop(0)
        if req['op'] in MOTION and (refusal := self.refusal(req)):
            return self.refuse(req, refusal)
        args = req['args']
        label = {'run': ' '.join(filter(None, ('run', agent_acts.procedure(agent_acts.run_key(args))[1],
                                                agent_acts.run_task(args)))),
                 'ask': f"ask {str(args.get('question') or '')[:60]}".strip(),
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
                self.finish(act, False, 'handover' if app.detaching else 'cancelled', hint=self.closed_hint())

    def act_run(self, act):
        app = self.app
        found, name = agent_acts.procedure(agent_acts.run_key(act['req']['args']))
        task = agent_acts.run_task(act['req']['args'])
        sim = 'armed' in found.needs and not app.armed and act['req']['args'].get('sim_arm')
        blocked = unavailable(found, app.profile, True, app.args.bench_override)
        if blocked:                              # never runnable in this TUI, armed or not: refused, nothing asked
            return self.finish(act, False, 'refused', hint=agent_acts.agent_text(blocked))
        values, refusal = self.settings_of(act['req'], agent_config.settable(self.config.get(), name), idle=False)
        if refusal:                              # agent.yaml changed since the request was read: never a wrong step
            return self.finish(act, False, refusal[0], hint=refusal[1])
        if not self.confirm(act, f"Agent {act['who']}: run {name}{' ' + task if task else ''}"
                                 + ''.join(f' {k}={v}' for k, v in values.items())
                                 + f"{' · sim-arm' if sim else ''} · allow?"):   # Allow arms the sim too
            return False
        self.apply_settings(values, act['who'])  # before the run starts (and before any arming)
        if sim:
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

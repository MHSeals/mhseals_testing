"""The desk's queue: run, goal and ask wait here in their order, one act at a time and never while a popup is open
or anything else runs; each is refused (never executed) when it cannot run now, and answered expired or busy_ui
when it waited too long. The acts themselves run through ui/agent_acts.py, the same code as a key."""

import time

from mhseals_testing.agent import config as agent_config
from mhseals_testing.harness import operator_state, sim_arm
from mhseals_testing.ros.arming import muxed
from mhseals_testing.ui import agent_acts
from mhseals_testing.ui.feeds.agent_desk.common import MOTION


class QueueMixin:
    """Acts and their order: enqueue, the motion refusal, start_next, the acts and their replies."""

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

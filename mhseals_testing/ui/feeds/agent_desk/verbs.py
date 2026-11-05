"""The desk's immediate verbs that act on the stack and the FCU: boat_agent start / stop COMPONENT (the Status
row's own path, under the agent's claim, never a part that arms) and boat_agent fcu (reads need no claim; writes
and the restart need the claim and a disarmed boat), plus Nav2's lifecycle states the goal checks read."""

from mhseals_testing.agent import fcu as agent_fcu
from mhseals_testing.harness.components import ComponentError
from mhseals_testing.ui import agent_acts, parts
from mhseals_testing.ui.feeds.agent_desk.common import (FCU_CLAIMED, LIFECYCLE_S, LIFECYCLE_TIMEOUT_S, NEVER_PARTS,
                                                         PART_OPS)


class VerbsMixin:
    """start / stop COMPONENT, fcu ACTION and the cached Nav2 lifecycle."""

    def lifecycle(self):
        """Nav2's lifecycle states, asked at most every LIFECYCLE_S with short waits; {} without Nav2."""
        if self.lifecycle_at is None or self.clock() - self.lifecycle_at >= LIFECYCLE_S:
            nav2 = agent_acts.nav2_of(self.app)
            self.lifecycle_states = nav2.lifecycle(timeout=LIFECYCLE_TIMEOUT_S) if nav2 is not None else {}
            self.lifecycle_at = self.clock()
        return self.lifecycle_states

    def do_part(self, req):
        """boat_agent start / stop COMPONENT: an allowed stack part through the Status row's own path (ui/parts.py)
        under the agent's claim, with no popup; the row, the notice and the journal name the agent. Never arms."""
        verb = PART_OPS[req['op']]
        found = self.part_refusal(req, verb)
        if found:
            return self.reply(req, False, found[0], hint=found[1])
        key = req['args']['component']
        try:
            done = (parts.start if verb == 'start' else parts.stop)(self.app, key, who=req['who'])
        except ComponentError as error:
            return self.reply(req, False, 'refused', hint=str(error))
        self.reply(req, True, result={'component': key, 'action': verb, 'components': done})

    def part_refusal(self, req, verb):
        """(error, hint) when ``req`` may not start or stop its component now, else None."""
        app, allowed = self.app, self.config.get().get('start', [])
        if app.settings.agents == 'block':
            return 'blocked', 'operator must set Agents to allow'
        if not req.get('claim'):
            return 'unknown_claim', f'{verb} needs your claim: claim --reason "..." --who {req["who"]}, then --claim ID'
        found = self.claim_state(req['claim'], req['who'])
        if found:
            return found, self.claim_hint(found, req['claim'])
        key, stack = req['args'].get('component'), getattr(app, 'stack', None)
        items = stack.items if stack is not None else {}
        if not isinstance(key, str) or key not in items:
            return 'unknown_component', (f"this profile's components: {', '.join(items)}" if items
                                         else "this TUI's profile has no components")
        item = items[key]
        if key not in allowed or item.kind == 'commander' or NEVER_PARTS.search(item.key) \
                or NEVER_PARTS.search(item.part):
            return 'not_allowed', (f"agent.yaml start: lists {', '.join(allowed)}; {item.title} stays the "
                                   "operator's (its Status row)")
        if verb == 'start':      # the parts it needs start first (Stack.plan): each must be an agent's too
            try:
                needs = stack.plan(key, app.ros.nodes())[:-1]
            except ComponentError as error:
                return 'refused', str(error)
            others = [name for name in needs if name not in allowed]
            if others:
                return 'not_allowed', (f"starting {item.title} also starts {', '.join(items[n].title for n in others)}, "
                                       "which is not an agent's: ask the operator")
        if verb == 'stop':
            if app.armed:
                return 'refused', f'armed: the operator stops {item.title} from its Status row (or disarms first)'
            # Dependants stop first, and each stopped part takes the parts its launch brings (the
            # boat's watchdogs ride odometry's launch): none of those may be a part not on the list.
            stopped = [name for name in stack._dependants(key) if stack.running(name)] + [key]
            carried = [name for name, other in items.items() for carrier in stopped
                       if stack.running(carrier) and (other.carried_by == items[carrier].title
                                                      or other.part in items[carrier].brings)]
            others = [name for name in dict.fromkeys(stopped + carried) if name != key and name not in allowed]
            if others:
                return 'not_allowed', (f"stopping {item.title} also stops {', '.join(items[n].title for n in others)}, "
                                       "which is not an agent's: ask the operator")
        return None

    def fcu_work(self):
        """The AgentFcu over this TUI's MavrosLink (ros/fcu.py, the one MAVROS path) and node."""
        if self.fcu is None:
            from mhseals_testing.ros import rates
            from mhseals_testing.ros.fcu import FcuParams, MavrosLink
            node, made = self.app.ros.node, {}

            def params():
                if 'params' not in made:
                    made['params'] = FcuParams(MavrosLink(node))
                return made['params']
            vessel = getattr(getattr(self.app, 'profile', None), 'name', None) or 'fcu'
            self.fcu = agent_fcu.AgentFcu(
                params, agent_fcu.Store(agent_fcu.settings(self.config.get())['store'], vessel), self.config.get,
                lambda window: rates.measure(node, agent_fcu.settings(self.config.get())['rates']['topics'], window))
        return self.fcu

    def do_fcu(self, req):
        """boat_agent fcu get/rates (read-only) and set/revert/restart (claimed, disarmed): owner decision 10-31.
        The work runs off the key loop (its popup keeps Space and E); a write or restart is a notice and a journal
        ``fcu`` event naming the agent. Never arms; the never list (agent_fcu.NEVER) holds whatever agent.yaml says."""
        op, args, who, app = req['op'], req['args'], req['who'], self.app
        if op in FCU_CLAIMED:
            if app.settings.agents == 'block':
                return self.reply(req, False, 'blocked', hint='operator must set Agents to allow')
            if not req.get('claim'):
                return self.reply(req, False, 'unknown_claim',
                                  hint=f'fcu needs your claim: claim --reason "..." --who {who}, then --claim ID')
            found = self.claim_state(req['claim'], who)
            if found:
                return self.reply(req, False, found, hint=self.claim_hint(found, req['claim']))
            if app.armed:
                return self.reply(req, False, 'refused', hint='armed: FCU writes and restarts only while disarmed')
        work, claim = self.fcu_work(), req.get('claim')

        def wrote(name, old, new):       # each write as it lands: a partial set stays visible
            app.notice = f'Agent {who} {claim}: FCU {name} {old}->{new}'
            app.emit('fcu', action='write', ok=True, by='agent', who=who, claim=claim, changes={name: [old, new]},
                     snapshot=work.snapshot.name if work.snapshot else None)
        jobs = {'fcu_get': lambda: work.get(list(args.get('params') or [])),
                'fcu_rates': lambda: work.rates(args.get('window_s'), who),
                'fcu_set': lambda: work.set(dict(args.get('changes') or {}), who, claim, on_write=wrote),
                'fcu_revert': lambda: work.revert(str(args.get('to') or ''), who, claim, on_write=wrote),
                'fcu_restart': lambda: work.restart(who, claim)}
        if op == 'fcu_set' and not args.get('changes'):
            return self.reply(req, False, 'bad_args', hint='fcu set PARAM VALUE [PARAM VALUE ...]')
        titles = {'fcu_get': 'reading FCU parameters', 'fcu_rates': 'measuring rates',
                  'fcu_set': 'setting FCU parameters', 'fcu_revert': f"reverting the FCU to {args.get('to')}",
                  'fcu_restart': 'restarting the FCU'}
        title = f'Agent {who}: {titles[op]}'
        if op in FCU_CLAIMED:
            app.notice = title
        self.reply(req, True, state='started')
        try:
            ok, error, hint, result = agent_acts.in_background(app, title, jobs[op], message=title)
        except Exception as fault:      # noqa: BLE001 - a MAVROS timeout or a missing service: told, not raised
            ok, error, hint, result = False, 'refused', f'{type(fault).__name__}: {fault}', None
        if op in FCU_CLAIMED and result is not None:
            app.notice = (f"Agent {who} {claim}: {titles[op]}"
                          + (f": {', '.join(f'{n} {o}->{v}' for n, (o, v) in result.get('changes', {}).items())}"
                             if result.get('changes') else '') + ('' if ok else f' failed ({hint})'))
            app.emit('fcu', action=op[4:], ok=ok, by='agent', who=who, claim=claim,
                     **{k: result[k] for k in ('changes', 'snapshot') if k in result})
        self.reply(req, ok, error, result=result, hint=hint)

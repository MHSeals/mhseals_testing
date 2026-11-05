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
import socket
import threading
import time

from mhseals_hardware import hold_store

from mhseals_testing.harness import control, operator_state, sim_arm
from mhseals_testing.agent import channel as agent_channel, config as agent_config, fcu as agent_fcu, grants as agent_grants
from mhseals_testing.harness import background
from mhseals_testing.harness.registry import alive as registry_alive
from mhseals_testing.harness.authority import HUMAN
from mhseals_testing.harness.components import ComponentError
from mhseals_testing.harness.goals import Goals, gate
from mhseals_testing.procedures import unavailable
from mhseals_testing.ros.arming import muxed
from mhseals_testing.ui import agent_acts, parts, theme
from mhseals_testing.ui.feeds import agent_ops, authority as authority_feed
from mhseals_testing.ui.feeds.agent_desk.popups import PopupsMixin
from mhseals_testing.ui.feeds.agent_desk.popups import operator_popup  # noqa: F401 - re-exported
from mhseals_testing.ui.feeds.agent_desk.common import (  # noqa: F401 - re-exported
    MOTION, NOTICED, URGENT, LIFECYCLE_S, LIFECYCLE_TIMEOUT_S, DRY_TIMEOUT_S, FENCE_WAIT_S, DRY_CLOSE_S, NOTES,
    PART_OPS, FCU_OPS, FCU_CLAIMED, CHANGING, NEVER_PARTS, PROGRESS_S, STATUS_S, NAV2_PREFIX, ATTENTION_S, HINTS)


class AgentDesk(PopupsMixin):
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
            if self.working and record['kind'] == 'req' and record.get('op') not in URGENT:
                self.deferred.append(record)     # a handler still works (its popup ticks): after it, if in time
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

    def reply(self, req, ok=True, error='', state='done', result=None, hint='', label=None):
        hint = agent_acts.agent_text(hint)
        agent_channel.reply(req['id'], req['who'], ok=ok, state=state, error=error, result=result,
                            hint=hint, path=self.path)
        if state == 'done':
            label = label or (self.act['label'] if self.act and self.act['id'] == req['id'] else req['op'])
            self.recent.append({'id': req['id'], 'op': req['op'], 'who': req['who'], 'label': label,
                                't_ns': time.time_ns(), 'ok': bool(ok), 'error': error, 'hint': hint,
                                'why': str(req.get('why') or '')})
        if state != 'progress':
            self.app.emit('agent_act', id=req['id'], who=req['who'], op=req['op'], state=state, error=error)
        # a refusal stays in the agent panel; the header's one "agent wants attention" (``attention``) points there

    def on_reply(self, record):
        pass

    def on_note(self, record):
        self.touch(record['who'])
        if record['phase'] == 'doing':        # note --doing: the agent's one "doing:" line in the panel
            self.doing[record['who']] = {'text': str(record.get('cmd') or ''), 't_ns': record.get('t_ns')}
            return
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
        for req in self.queue:
            if req['id'] == record['act'] and req.get('claim') == record['claim'] and req['who'] == record['who']:
                req['renewed'] = self.clock()

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
        self.app.emit('agent_req', id=req['id'], who=req['who'], op=req['op'], args=args, dry=bool(req.get('dry')),
                      **({'why': req['why']} if req.get('why') else {}))
        self.touch(req['who'], req.get('claim'))
        if req['op'] in MOTION and req['t_ns'] < self.started_ns:
            return self.reply(req, False, 'expired', hint='sent before this TUI started; nothing was done')
        if time.time_ns() > req['deadline_ns']:
            return self.reply(req, False, 'expired', hint='request deadline passed; nothing was done')
        state = operator_state.current()
        if req['op'] in CHANGING and operator_state.paused(state):
            return self.reply(req, False, 'paused', hint=operator_state.refusal(state),
                              result={'operator': operator_state.summary(state)})
        handler = {'claim': self.do_claim, 'release': self.do_release, 'stop': self.do_stop, 'revoke': self.do_revoke,
                   'mark': self.do_mark, 'run': self.enqueue, 'goal': self.enqueue,
                   'ask': self.enqueue, 'start': self.do_part, 'stop_part': self.do_part, 'set': self.do_set,
                   **dict.fromkeys(FCU_OPS, self.do_fcu),
                   **{op: (lambda r, f=f: f(self, r)) for op, f in agent_ops.HANDLERS.items()}}.get(req['op'])
        if handler is None:
            return self.reply(req, False, 'bad_op', hint=f"unknown op {req['op']!r}")
        self.working = req['op']
        try:
            handler(req)
        finally:
            self.working = None

    def watch_holds(self):
        """Subscribe to each holding watchdog's ~/status as its hold appears, so a release finds it fresh."""
        holds = (getattr(self.arbiter, 'mux', None) or {}).get('holds') or []
        wanted = {str(h.get('by')) for h in holds if isinstance(h, dict) and agent_grants.kind(h) == 'node'}
        ros = getattr(self.app, 'ros', None)
        if not wanted - self.watched or not hasattr(ros, 'watch_beats') or self.clock() - self.watch_at < STATUS_S:
            return
        self.watch_at = self.clock()           # a watchdog not on the graph yet is looked for once a second
        nodes = [n for n in ros.nodes() if n.rsplit('/', 1)[-1] in wanted]
        ros.watch_beats(nodes)
        self.watched |= {n.rsplit('/', 1)[-1] for n in nodes}

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
        self.stops, self.stopped_by = self.stops + 1, req['who']
        self.app.operator_stop(agent=req['who'])
        self.reply(req, True, result={'stopped': True})

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
            fcu = agent_fcu.settings(self.config.get())
            vessel = getattr(getattr(self.app, 'profile', None), 'name', None) or 'fcu'
            self.fcu = agent_fcu.AgentFcu(
                params, agent_fcu.Store(fcu['store'], vessel), self.config.get,
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

    def do_revoke(self, req):
        """A viewer's E (harness/control.py): revoke the active agent claim as the owner's E does. Only a live
        viewer of this session may: an agent has no revoke verb and its pid is no registered viewer."""
        entry = (self.registry.read() or {}) if self.registry is not None else {}
        if not (str(req['who']).startswith('view:') and req.get('host') == socket.gethostname()
                and control.is_viewer(entry, req.get('pid'), registry_alive)):
            return self.reply(req, False, 'refused', hint='only a viewer of this session revokes (E)')
        arbiter = self.arbiter
        holder = arbiter.table.holder() if arbiter is not None else None
        if holder is None or holder.get('owner') != 'agent':
            return self.reply(req, True, result={'revoked': None})
        viewer = req['who'][len('view:'):]
        self.cancel('revoked')
        arbiter.revoke(holder['id'])
        self.app.emit('viewer_revoke', viewer=viewer, claim=holder['id'], who=holder['who'])
        self.app.notice = f"Agent {holder['who']} revoked from viewer {viewer}"
        self.reply(req, True, result={'revoked': holder['id']})

    def do_mark(self, req):
        number = self.app.next_id('marker')
        self.app.emit('marker', kind='agent', id=number, who=req['who'], note=str(req['args'].get('note', '')),
                      tag=str(req['args'].get('tag') or ''), why=req.get('why') or '', t_ns=time.time_ns())
        self.reply(req, True, result={'marker': number})

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

    def claim_hint(self, found, claim_id):
        """What an agent does about ``claim_state``'s answer; an unknown claim names this TUI session's start."""
        if found == 'not_holder':
            return 'use your own claim'
        if found == 'paused_end':
            return (f'claim {claim_id} ended cleanly when the operator returned to normal: it was not kept alive '
                    'while paused (no hold was made); claim again')
        if found == 'unknown_claim' and claim_id:
            record = operator_state.current()
            started = time.strftime('%H:%M', time.localtime(self.started_ns / 1e9))
            after = record.get('restarted_after')
            why = (f", restarted after {operator_state.LABELS.get(after.get('state'), '?')} at "
                   f"{operator_state.clock(after.get('since_ns'))}: re-check the stack, FCU params and RTK"
                   if after else '')
            return f'claim {claim_id} is not in this TUI session (started {started}{why}); claim again'
        return 'claim again'

    def operator_paused(self):
        """The operator declared manual, battery swap or power cycle: an act in flight ends now (a run through
        the runner's interlock path, neutral first; a goal is cancelled), with no lease hold."""
        if self.act is not None and self.act['op'] in MOTION:
            self.cancel('paused')
            self.watch()

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
        holds = authority_feed.saved_holds(self.app)
        return str(holds[0]['cause']) if holds else ''

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
        if key == 'paused':
            return operator_state.refusal(operator_state.current())
        if key.startswith('agent '):
            return (f'{key} stopped the boat (boat_agent stop), not the operator; the operator must re-arm and '
                    'release the hold (U)')
        return HINTS.get(key, default)

    @staticmethod
    def error_of(cause):
        """The reply error for a cancel cause: revoked, expired, released, overridden: <who>, handover, cancelled."""
        if cause in ('revoked', 'expired', 'released', 'handover', 'paused') or cause.startswith('overridden: '):
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
                self.app._runner_cancelled = f'(agent act ended: {cause})'   # a paused agent run aborts, not resumes
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
        if operator_state.paused(operator_state.current()):
            return 'paused'                         # before the lease: a paused claim never lapses
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
            lapsed = [(cid, why) for cid, why in arbiter.ended.items()
                      if why in ('expired', 'lapsed') and cid not in self.expired_seen]
        for cid, why in lapsed:
            self.expired_seen.add(cid)
            if cid in self.claim_who:
                self.app.notice = (f'Agent {self.claim_who[cid]} {cid} lease expired: '
                                   + (f'stopped{theme.SEPARATOR}neutral, hold (U releases)' if why == 'expired'
                                      else 'disarmed and neutral, so no hold'))

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

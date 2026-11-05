"""The desk's lease: an agent's claim through the arbiter (claim, release, a viewer's revoke), its idle and motion
lease, the cancel of an act in flight and the override that causes it. A lapse expires the claim through the
arbiter, which stops exactly as E does; a cause is set once and never cleared, and nothing resumes the act."""

import socket
import time

from mhseals_testing.agent import grants as agent_grants
from mhseals_testing.harness import control, operator_state, sim_arm
from mhseals_testing.harness.authority import HUMAN
from mhseals_testing.harness.registry import alive as registry_alive
from mhseals_testing.ros.arming import muxed
from mhseals_testing.ui import theme
from mhseals_testing.ui.feeds import authority as authority_feed
from mhseals_testing.ui.feeds.agent_desk.common import HINTS, MOTION, PROGRESS_S, STATUS_S
from mhseals_testing.ui.feeds.agent_desk.popups import operator_popup


class LeaseMixin:
    """The claim, the motion lease and override cancel (the contract is desk.py's docstring)."""

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

    def do_claim(self, req):
        if self.arbiter is None:
            return self.reply(req, False, 'no_owner')
        if self.app.settings.agents == 'block':
            return self.reply(req, False, 'blocked', hint='operator must set Agents to allow')
        idle = self.lease()['idle_s']
        ack = self.arbiter.handle({'op': 'claim', 'request': req['id'], 'who': req['who'], 'lease_s': idle,
                                   'reason': str(req['args'].get('reason', '')), 'pid': req['pid'],
                                   'host': req['host']})
        holder = ack.get('holder') or {}
        error = ack.get('error', '')
        hint = f"{holder.get('who')} holds the boat (wait, or ask them)" if error == 'busy' else ''
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
        self.app.say(f"Agent {holder['who']} revoked from viewer {viewer}")
        self.reply(req, True, result={'revoked': holder['id']})

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
                self.app.say(f'Agent {self.claim_who[cid]} {cid} lease expired: '
                             + (f'stopped{theme.SEPARATOR}neutral, hold (U releases)' if why == 'expired'
                                else 'disarmed and neutral, so no hold'))

    def do_stop(self, req):
        """Space for the agent: hold, neutral, disarm. Always allowed, never needs a claim. The stop is the
        agent's, not the operator's: another agent's act ends ``overridden: agent W`` and the hold's hints name W."""
        if self.act and self.act['op'] in MOTION:
            self.cancel('stop' if self.act['who'] == req['who'] else f"overridden: agent {req['who']}")
        self.stops, self.stopped_by = self.stops + 1, req['who']
        self.app.operator_stop(agent=req['who'])
        self.reply(req, True, result={'stopped': True})

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

    def human(self):
        """``override()``, with a Space-style hold an agent's ``stop`` set named ``agent W``."""
        cause = self.override()
        stopper = str(self.app.services.arm.stopped_by or '')
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
                self.app.cancel_run(f'(agent act ended: {cause})')   # a paused agent run aborts, not resumes
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
            self.app.say(f"Agent {act['who']} {act['claim']} stopped: {setting}{theme.SEPARATOR}"
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

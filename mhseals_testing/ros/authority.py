"""The authority arbiter: who is driving, published for everyone (spec authority_hotreload section 3).

It runs in the session owner (the TUI, or a headless run). Every second, and
at once on any change, it turns the latest mux state, the claim table and what
the owner knows about goals, tasks and procedures into one record
(harness/authority.derive) and publishes it latched on AUTHORITY_TOPIC. Agents
ask for claims on CLAIM_TOPIC and read the answer in the ``ack`` field of the
next record. Claims and their sequence number persist in the session registry
(``registry.authority``), so a takeover or exec reload adopts them.

``Arbiter`` has no ROS in it: transport and side effects are injected, so it
is tested with plain callables. ``bind`` wires it to a RosIO node.
"""

import json
import threading
import time

from mhseals_testing.harness import agent_config, operator_state
from mhseals_testing.harness.authority import ClaimTable, derive, events

AUTHORITY_TOPIC = '/mhseals_testing/authority'
CLAIM_TOPIC = '/mhseals_testing/authority/claim'
MUX_STATE_TOPIC = '/command_mux/state'
PAUSE_TOPIC = '/autonomy/pause'      # nav's latched pause requests (the fence's return)
HEARTBEAT_S = 1.0
TICK_S = 0.25
# Ack errors and the exit codes the CLI maps them to (spec section 6, agent-control section 3).
EXIT_CODES = {'busy': 2, 'not_holder': 2, 'revoked': 3, 'expired': 3, 'no_owner': 4, 'blocked': 5, 'paused': 7}


def exit_code(ack):
    """0 for an accepted request, else the CLI exit code of its error."""
    return 0 if ack.get('ok') else EXIT_CODES.get(ack.get('error'), 1)


class Arbiter:
    """Claim table plus publishing loop.

    ``publish(record)`` sends the record; ``emit(phase, **fields)`` journals an
    event. ``sources()`` returns {'nav', 'task', 'procedure', 'graph'} for
    ``derive``; nav goals may carry a ``cancel`` callable. ``neutral()`` and
    ``hold(cause, by)`` stop the boat (zero command, mux hold). ``tui_owner``
    is False for a headless run, which refuses agent claims. ``config`` is the
    hot-reloaded config/agent.yaml (``agent_config.Watch``): its lease bounds
    agent claims. An expired lease stops the boat exactly as ``revoke`` does.
    While the operator has declared a state other than normal (``pause``), agent
    leases are suspended, not lapsed: nothing expires and no lease hold is made, and a new claim is refused.
    Renew and release over the topic must name the claim's ``who``.
    """

    def __init__(self, publish, emit, *, registry=None, tui_owner=False, allow_agents=True,
                 sources=lambda: {}, neutral=lambda: None, hold=lambda cause, by: None,
                 clock=time.time, config=None, quiet=lambda: False):
        self.publish, self.emit, self.registry = publish, emit, registry
        self.tui_owner, self.allow_agents = tui_owner, allow_agents
        self.sources, self.neutral, self.hold, self.clock = sources, neutral, hold, clock
        self.quiet = quiet       # True: disarmed and neutral, so an idle lease lapses with no stop and no hold
        saved = (registry.read().get('authority') or {}) if registry else {}
        self.table = ClaimTable(saved.get('claims', ()), saved.get('seq', 0))
        self.config = config or agent_config.Watch()
        lease = (self.config.get() or {}).get('lease') or {}      # the first claim gets agent.yaml's lease too
        self.table.idle_s = lease.get('idle_s', self.table.idle_s)
        self.table.max_claim_s = lease.get('max_claim_s', self.table.max_claim_s)
        self.ended = {}          # claim id -> 'revoked' | 'expired' | 'lapsed' | 'paused_end': the agent's next call learns why
        self.paused = None       # the operator state record while it is not normal (harness/operator_state.py)
        self.mux, self.mux_at = None, None
        self.pause_request = None   # nav's latest pause request (runner.override_cause); pause() is the operator's
        self.record, self.ack, self.sent_at, self.sent = None, None, None, None
        self.saved = None
        self.lock = threading.RLock()

    def on_mux(self, state):
        with self.lock:
            self.mux, self.mux_at = state, self.clock()

    def on_pause(self, request):
        with self.lock:
            self.pause_request = request

    def handle(self, request):
        """Answer one claim-topic request; the ack rides in the next record."""
        with self.lock:
            now = self.clock()
            op = request.get('op')
            ack = {'request': request.get('request'), 'ok': True, 'op': op}
            if op == 'claim':
                self._claim(request, now, ack)
            elif op in ('release', 'renew'):
                self._own(request, now, ack)
            else:  # revoke is the owner's own key; over the topic it is refused like any other op
                ack.update(ok=False, error='forbidden' if op == 'revoke' else 'bad_op')
            self.ack = ack
            self.tick(force=True)
            return ack

    def _claim(self, request, now, ack):
        if not self.tui_owner:
            ack.update(ok=False, error='no_owner')  # no TUI means no banner to show the claim
        elif not self.allow_agents:
            ack.update(ok=False, error='blocked')
        elif self.paused is not None:            # the desk's paused refusal, for a claim sent on the topic
            ack.update(ok=False, error='paused', hint=operator_state.refusal(self.paused))
        else:
            claim, holder = self.table.claim(
                request.get('owner', 'agent'), request.get('who') or 'agent', now,
                request.get('reason', ''), request.get('lease_s'), request.get('pid'),
                request.get('host'))
            ack.update(ok=claim is not None, id=claim['id'] if claim else None,
                       claim=claim, holder=holder, error='' if claim else 'busy')

    def _own(self, request, now, ack):
        """Renew or release: only the claim's own ``who`` may (G5)."""
        claim_id, op = request.get('id'), request.get('op')
        claim = self.table.get(claim_id)
        if claim is None:
            ack.update(ok=False, error=self.ended.get(claim_id, 'unknown_claim'))
        elif claim['who'] != request.get('who'):
            ack.update(ok=False, error='not_holder', holder=claim['who'])
        elif op == 'release':
            ack.update(ok=self.table.release(claim_id, now), error='')
        elif self.table.renew(claim_id, now):
            ack.update(error='')
        else:
            ack.update(ok=False, error=self.ended.get(claim_id, 'revoked'))

    def _stop(self, cause):
        """Neutral, mux hold ``cause``, cancel every executing goal: what E and a lapsed lease do."""
        self.neutral()
        self.hold(cause, 'tui')
        for goal in (self.sources().get('nav') or {}).get('goals', ()):
            if goal.get('status') == 'executing' and goal.get('cancel'):
                goal['cancel']()

    def revoke(self, claim_id):
        """The owner's revoke key: stop the boat as Space does, cancel the claim's goals, then revoke."""
        with self.lock:
            self._stop('revoke')
            done = self.table.revoke(claim_id, self.clock())
            if done:
                self.ended[claim_id] = 'revoked'
            self.tick(force=True)
            return done

    def halt(self, cause):
        """An operator setting (Agents block, Agent motion observe) ended an agent act: stop as E does;
        the claim stays, suspended under the hold until the operator releases it."""
        with self.lock:
            self._stop(cause)
            self.tick(force=True)

    def expire(self, claim_id):
        """A lapsed motion lease (the desk's): stop as revoke does, then end the claim as ``claim_expire``."""
        with self.lock:
            claim = self.table.get(claim_id)
            if claim is None or claim['state'] != 'active':
                return False
            self._stop('lease')
            self.table.release(claim_id, self.clock(), 'claim_expire')
            self.ended[claim_id] = 'expired'
            self.tick(force=True)
            return True

    def pause(self, record):
        """The operator declared manual, battery swap or power cycle: leases stop counting."""
        with self.lock:
            self.paused = record
            self.tick(force=True)

    def resume(self):
        """The operator is back to normal. An agent claim kept alive during the pause (any verb, renew or
        ``follow --claim``) resumes with a fresh lease; one not heard from for a whole idle lease (or past
        ``max_claim_s``) ends cleanly
        (``paused_end``), with no stop and no hold: it could not drive while paused. Returns the ended claims."""
        with self.lock:
            if self.paused is None:
                return []
            self.paused, now, ended = None, self.clock(), []
            for claim in list(self.table.claims):
                if claim['state'] != 'active' or claim['owner'] != 'agent':
                    continue
                lease = claim['lease_s'] if claim['lease_s'] is not None else self.table.idle_s
                if now - claim['seen_ns'] / 1e9 > lease or now - claim['since_ns'] / 1e9 > self.table.max_claim_s:
                    self.table.release(claim['id'], now, 'claim_release')
                    self.ended[claim['id']] = 'paused_end'
                    ended.append(claim)
                else:
                    self.table.renew(claim['id'], now)
            self.tick(force=True)
            return ended

    def tick(self, force=False):
        """Derive, journal edges, and publish on change or once per HEARTBEAT_S."""
        with self.lock:
            now = self.clock()
            lease = (self.config.get() or {}).get('lease') or {}
            self.table.idle_s = lease.get('idle_s', self.table.idle_s)
            self.table.max_claim_s = lease.get('max_claim_s', self.table.max_claim_s)
            for claim in self.table.expire(now) if self.paused is None else ():
                if self.quiet():              # disarmed and neutral: nothing to stop, so no hold (owner 11-01)
                    self.ended[claim['id']] = 'lapsed'
                    continue
                self._stop('lease')
                self.ended[claim['id']] = 'expired'
            found = self.sources()
            nav = found.get('nav')
            if nav:  # strip the cancel callables: the record is JSON
                nav = {'goals': [{k: v for k, v in g.items() if k != 'cancel'} for g in nav['goals']]}
            record = derive(self.mux, self.table.claims, nav, found.get('task'),
                            found.get('procedure'), found.get('graph'), now, self.record,
                            None if self.mux_at is None else now - self.mux_at,
                            found.get('mux_expected', True), found.get('muxed', True), found.get('saved_holds', ()))
            for event in self.table.drain() + events(self.record, record):
                self.emit(event['phase'], **event['rec'])
            self.record = record
            self._persist()
            key = json.dumps({k: v for k, v in record.items() if k != 't_ns'}, sort_keys=True)
            record = dict(record, ack=self.ack)
            if force or key != self.sent or self.sent_at is None or now - self.sent_at >= HEARTBEAT_S:
                self.sent, self.sent_at = key, now
                self.publish(record)
            return record

    def _persist(self):
        state = {'seq': self.table.seq, 'claims': self.table.claims}
        text = json.dumps(state, sort_keys=True)
        if self.registry and text != self.saved:
            self.saved = text
            self.registry.update(lambda data: data.update(authority=state))

    def close(self, keep_claims=False):
        """Leave the seat. Claims end with the session unless a successor adopts them."""
        with self.lock:
            if not keep_claims:
                for claim in list(self.table.claims):
                    self.table.release(claim['id'], self.clock())
            self.tick(force=True)


def bind(ros, arbiter):
    """Subscribe the arbiter to the mux state, pause and claim requests and run its timer on ``ros``."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    node = ros.node
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(String, AUTHORITY_TOPIC, latched)
    arbiter.publish = lambda record: pub.publish(String(data=json.dumps(record, default=str)))

    def parsed(message):
        try:
            value = json.loads(message.data)
        except ValueError:
            return None
        return value if isinstance(value, dict) else None

    def on_claim(message):
        if (request := parsed(message)) is not None:
            arbiter.handle(request)

    def on_mux(message):
        if (state := parsed(message)) is not None:
            arbiter.on_mux(state)

    node.create_subscription(String, CLAIM_TOPIC, on_claim, 10)
    node.create_subscription(String, MUX_STATE_TOPIC, on_mux, latched)
    node.create_subscription(String, PAUSE_TOPIC, lambda m: arbiter.on_pause(parsed(m)), latched)
    node.create_timer(TICK_S, arbiter.tick)
    return arbiter

"""Who is driving the boat: claims, and the owner derived from them.

Pure and ROS-free. The arbiter (ros/authority.py) feeds the latest mux state,
the claim table, Nav2 goals, task and procedure status and the /cmd_vel
publisher list into ``derive``; it publishes the record and journals
``events``. Times are seconds on one clock, passed in as ``now``. Stamps
(``t_ns``) come from other hosts' clocks, so staleness is aged by local receipt
time: callers pass ``age_s`` (local now minus local receipt time); the stamp
fallback is only right when sender and reader share a clock.

Owners, in display precedence (spec authority_hotreload section 3):
rc > keyboard > hold > agent > task > nav > procedure > external > none.
Humans (rc, keyboard, hold) always win and suspend claims without dropping
them. Anything driving that nobody claimed is ``external`` and shown red.
"""

OWNERS = ('rc', 'keyboard', 'hold', 'agent', 'task', 'nav', 'procedure', 'external', 'none')
HUMAN = {'rc': 'rc', 'manual': 'keyboard', 'hold': 'hold'}
CLAIM_OWNERS = ('agent', 'task', 'nav', 'procedure')
STALE_S = 3.0
CMD_VEL_WINDOW_S = 1.0


def is_unknown(record, now, age_s=None):
    """No record, or one older than STALE_S: clients treat this as paused.

    ``age_s`` is the local time since the record was received; it is
    needed when the sender's clock may differ from ours.
    """
    return record is None or _age(record, now, age_s) > STALE_S


class ClaimTable:
    """Explicit and implicit claims. At most one automated claim is active."""

    def __init__(self, claims=(), seq=0, idle_s=120.0, max_claim_s=3600.0):
        self.claims = [dict(claim) for claim in claims]
        self.idle_s, self.max_claim_s = idle_s, max_claim_s   # agent leases (config/agent.yaml)
        # Never re-issue an id already on record, even if seq was not persisted.
        numbers = [int(c['id'][1:]) for c in self.claims if c['id'][1:].isdigit()]
        self.seq = max([seq, *numbers])
        self.pending = []  # claim life-cycle events since the last drain

    def drain(self):
        events, self.pending = self.pending, []
        return events

    def get(self, claim_id):
        return next((c for c in self.claims if c['id'] == claim_id), None)

    def holder(self):
        """The active claim blocking others, if any; revoked ones block nothing."""
        return next((c for c in self.claims if c['state'] == 'active'), None)

    def claim(self, owner, who, now, reason='', lease_s=None, pid=None, host=None,
              explicit=True, local=False):
        """Add a claim: (claim, None), or (None, holder) when busy.

        Only the arbiter's own implicit claims pass ``local=True``. Every other
        requester is an agent, whatever it says it is (or omits): a guest
        procedure (``--attach``) must not look like our own. An agent claim
        always has a lease, at most ``idle_s``.
        """
        if not local:
            owner = 'agent'
            lease_s = self._lease(lease_s)
        holder = self.holder()
        if holder is not None:
            return None, holder
        self.seq += 1
        claim = {'id': f'c{self.seq}', 'owner': owner, 'who': who, 'host': host, 'pid': pid,
                 'reason': reason, 'since_ns': _ns(now), 'seen_ns': _ns(now),
                 'lease_s': lease_s, 'state': 'active', 'explicit': explicit}
        self.claims.append(claim)
        self._emit('claim', claim, now)
        return claim, None

    def renew(self, claim_id, now):
        claim = self.get(claim_id)
        if claim is None or claim['state'] != 'active':
            return False
        claim['seen_ns'] = _ns(now)
        return True

    def release(self, claim_id, now, phase='claim_release'):
        """End a claim (``claim_revoke`` when revoked)."""
        claim = self.get(claim_id)
        if claim is None:
            return False
        self.claims.remove(claim)
        self._emit(phase, claim, now)
        return True

    def revoke(self, claim_id, now):
        """Mark a claim revoked: it stays on record, never drives and blocks nobody."""
        claim = self.get(claim_id)
        if claim is None or claim['state'] != 'active':
            return False
        claim['state'] = 'revoked'
        self._emit('claim_revoke', claim, now)
        return True

    def expire(self, now):
        """End claims whose lease ran out, or agent claims older than ``max_claim_s``; returns them.

        An agent claim stored without a lease (adopted from an older session) gets ``idle_s``.
        """
        expired = []
        for claim in list(self.claims):
            if claim['state'] != 'active':
                continue
            agent = claim['owner'] == 'agent'
            lease = claim['lease_s'] if claim['lease_s'] is not None else self.idle_s if agent else None
            idle = lease is not None and now - claim['seen_ns'] / 1e9 > lease
            if idle or (agent and now - claim['since_ns'] / 1e9 > self.max_claim_s):
                self.release(claim['id'], now, 'claim_expire')
                expired.append(claim)
        return expired

    def _lease(self, lease_s):
        try:
            lease_s = float(lease_s)
        except (TypeError, ValueError):
            return self.idle_s
        return min(lease_s, self.idle_s) if lease_s > 0 else self.idle_s

    def _emit(self, phase, claim, now):
        self.pending.append({'phase': phase, 't_ns': _ns(now), 'rec': dict(claim)})


def derive(mux, claims, nav, task, procedure, graph, now, prev=None, mux_age_s=None, mux_expected=True,
           muxed=True):
    """The authority record for this instant.

    ``mux``: the command_mux state, or None. ``claims``: the table's claims.
    ``nav``: {'goals': [{'id', 'status', 'by'}]}, ``by`` is 'nav' for goals
    the TUI sent. ``task``: {'status', 'name'} or None. ``procedure``:
    {'name', 'run'} or None. ``graph``: {'cmd_vel': {'age_s', 'nonzero'},
    'publishers': [{'node', 'gid'}], 'own_gids': [...]}. ``prev`` is the last
    record, so ``since_ns`` only moves when the owner or source changes.
    ``mux_age_s`` is the local time since the mux state arrived (see module
    docstring); without it the mux stamp is compared with ``now``.
    ``mux_expected`` is False when a muxed profile is disarmed: a missing or
    stale state is then 'none', not unknown. ``muxed`` False (an unmuxed
    profile) has no mux state at all, so it derives as input 'auto' with no rc
    and no holds and every driver still shows.
    """
    if not muxed:
        mux, mux_age_s = {'input': 'auto', 'rc': None}, 0.0
    record = {'schema': 'mhseals.authority/1', 't_ns': _ns(now), 'owner': 'none', 'source': '',
              'reason': '', 'input': None, 'cause': None, 'holds': [], 'conflict': False,
              'claims': [], 'nav': nav, 'task': task, 'procedure': procedure,
              'rc': mux.get('rc') if isinstance(mux, dict) else None}
    malformed = mux is not None and (not isinstance(mux, dict) or 'input' not in mux
                                     or (mux_age_s is None and 't_ns' not in mux)
                                     or not isinstance(mux.get('rc') or {}, dict))
    gone = mux is None or (not malformed and _age(mux, now, mux_age_s) > STALE_S)
    if gone and not mux_expected:
        record['claims'] = _effective(claims, False)
        return _finish(record, 'none', '', now, prev)
    if mux is None or malformed or _age(mux, now, mux_age_s) > STALE_S:
        # unknown means paused, but the claims stay visible
        record['claims'] = _effective(claims, True)
        record['last_owner'] = _last_known(prev)
        return _finish(record, 'unknown', 'mux state malformed' if malformed else 'no fresh mux state',
                       now, prev)

    record['input'] = mux['input']
    record['cause'] = mux.get('cause')
    record['holds'] = _holds(mux)
    human = HUMAN.get(mux['input'])
    record['claims'] = _effective(claims, human)
    if human:
        return _finish(record, human, _human_source(mux, human), now, prev)

    sources = _sources(record['claims'], task, procedure)
    goals = [g for g in (nav or {}).get('goals', []) if g.get('status') == 'executing']
    if not sources:
        sources = _unclaimed(goals, graph, mux['input'])
    if len(sources) > 1:
        record['conflict'] = True
        names = ', '.join(f"{owner} {text}" for owner, text, _ in sources)
        return _finish(record, 'external', f'? conflict: {names}', now, prev)
    if sources:
        owner, text, reason = sources[0]
        return _finish(record, owner, text, now, prev, reason)
    return _finish(record, 'none', '', now, prev)


def events(prev, record):
    """Journal events for the change from ``prev`` to ``record`` (claim edges excluded)."""
    if prev is None:
        return []
    out = []
    if prev['owner'] != record['owner'] and record['owner'] != 'unknown':
        by = _by(record)
        out.append(_event('authority', record, {
            'from': {'owner': prev['owner'], 'source': prev['source']},
            'to': {'owner': record['owner'], 'source': record['source']},
            'cause': record['cause'] or 'claim', 'reason': record['reason'], 'by': by}))
    old = {c['id']: c['state'] for c in prev['claims']}
    for claim in record['claims']:
        was = old.get(claim['id'])
        if was == 'active' and claim['state'] == 'suspended':
            out.append(_event('claim_suspend', record, {'claim': claim}))
        elif was == 'suspended' and claim['state'] == 'active':
            out.append(_event('claim_resume', record, {'claim': claim}))
    # unknown is transparent: an agent interrupted during a stale gap is still overridden
    if _last_known(prev) in CLAIM_OWNERS and record['owner'] in HUMAN.values():
        out.append(_event('override', record, {
            'by': record['owner'], 'procedure': (prev['procedure'] or {}).get('name'),
            'run': (prev['procedure'] or {}).get('run'), 'holds_invalidated': []}))
    return out


def _last_known(prev):
    """The owner before any run of unknown records ending at ``prev``."""
    if prev is None:
        return 'none'
    return prev.get('last_owner', 'none') if prev['owner'] == 'unknown' else prev['owner']


def _effective(claims, human):
    """Claims as shown: suspended under a human override or unknown mux state.

    A stored revoked (or otherwise non-active) state is kept as it is.
    """
    return [claim if claim.get('state', 'active') not in ('active', 'suspended')
            else dict(claim, state='suspended' if human else 'active') for claim in claims]


def _holds(mux):
    """The mux hold set, oldest first; a lone ``hold`` is a set of one."""
    return list(mux.get('holds') or ([mux['hold']] if mux.get('hold') else []))


def _sources(claims, task, procedure):
    """Top-level automated sources as (owner, source text, reason).

    Goals and tasks started while a claim is active belong to that claim. A
    TUI procedure is the arbiter's own claim, so it only nests under a
    procedure claim and runs beside any other claim as a second source. A
    task the procedure started itself (``procedure['task']``, Mission) is
    an implicit claim of that procedure: it nests, it is no second source.
    A procedure run for an agent claim (``procedure['claim']``) nests under it too.
    """
    active = [c for c in claims if c['state'] == 'active']
    sources = [(c['owner'], _claim_text(c), c['reason']) for c in active]
    if active:
        nests = any(c['owner'] == 'procedure' or c['id'] == (procedure or {}).get('claim') for c in active)
        if procedure and not nests:
            sources.append(('procedure', procedure['name'], ''))
        return sources
    if procedure:
        sources.append(('procedure', procedure['name'], ''))
    if task and task.get('status') == 'RUNNING' and not (procedure and procedure.get('task')):
        sources.append(('task', task.get('name', 'task'), ''))
    return sources


def _unclaimed(goals, graph, input):
    """Nothing claimed: a TUI goal is nav, any other driver is external.

    A /cmd_vel publisher only counts while the mux reads auto.
    """
    if goals:
        out = []
        if any(g.get('by') == 'nav' for g in goals):
            out.append(('nav', 'cursor goal', ''))
        if any(g.get('by') != 'nav' for g in goals):
            out.append(('external', '? goal (unclaimed)', ''))
        return out
    cmd = (graph or {}).get('cmd_vel') or {}
    if input != 'auto':
        return []
    age = cmd.get('age_s')
    if cmd.get('nonzero') and age is not None and age <= CMD_VEL_WINDOW_S:
        own = set(graph.get('own_gids', ()))
        pubs = graph.get('publishers', ())
        names = [p['node'] for p in pubs if p.get('gid') not in own]
        if pubs and not names:
            return []  # every publisher is ours, so not an outside driver
        return [('external', '? ' + (', '.join(names) or 'unknown publisher'), '')]
    return []


def _claim_text(claim):
    where = f" ({claim['host']} pid {claim['pid']})" if claim.get('host') else ''
    return f"claim {claim['id']} {claim['who']}{where}"


def _human_source(mux, human):
    if human == 'rc':
        return f"rc {(mux.get('rc') or {}).get('mode', '')}".strip()
    if human == 'hold':
        hold = (_holds(mux) or [{}])[0]
        return f"hold {hold.get('cause', '')} by {hold.get('by', '?')}".strip()
    return 'keyboard'


def _by(record):
    active = [c['who'] for c in record['claims'] if c['state'] == 'active']
    return active[0] if active and record['owner'] in CLAIM_OWNERS else record['owner']


def _finish(record, owner, source, now, prev, reason=''):
    record['owner'], record['source'], record['reason'] = owner, source, reason
    same = prev and prev['owner'] == owner and prev['source'] == source
    record['since_ns'] = prev['since_ns'] if same else _ns(now)
    return record


def _event(phase, record, fields):
    return {'phase': phase, 't_ns': record['t_ns'], 'rec': fields}


def _age(state, now, age_s):
    return age_s if age_s is not None else now - state['t_ns'] / 1e9


def _ns(seconds):
    return int(seconds * 1e9)

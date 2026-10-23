"""Who is driving the boat: claims, and the owner derived from them.

Pure and ROS-free. The arbiter (ros/authority.py) feeds the latest mux state,
the claim table, Nav2 goals, task and procedure status and the /cmd_vel
publisher list into ``derive``; it publishes the record and journals
``events``. Times are seconds on one clock, passed in as ``now``.

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


def is_unknown(record, now):
    """No record, or one older than STALE_S: clients treat this as paused."""
    return record is None or now - record['t_ns'] / 1e9 > STALE_S


class ClaimTable:
    """Explicit and implicit claims. At most one automated claim is active."""

    def __init__(self, claims=(), seq=0):
        self.claims = [dict(claim) for claim in claims]
        self.seq = seq
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
        procedure (``--attach``) must not look like our own.
        """
        if not local:
            owner = 'agent'
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
        """Drop claims whose lease ran out; lease-less claims never expire."""
        for claim in list(self.claims):
            lease = claim['lease_s']
            if claim['state'] == 'active' and lease is not None and now - claim['seen_ns'] / 1e9 > lease:
                self.release(claim['id'], now, 'claim_expire')

    def _emit(self, phase, claim, now):
        self.pending.append({'phase': phase, 't_ns': _ns(now), 'rec': dict(claim)})


def _ns(seconds):
    return int(seconds * 1e9)

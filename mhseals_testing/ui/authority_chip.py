"""Who drives, as the chip word (ui U4): a pure map from the arbiter's record to ``(kind, WORD)``.

``kind`` is a ``theme.CHIPS`` key (its hue is the ``auth.*`` token) and WORD the upper-case chip
text. ``driving`` adds the freshness rule (authority 3, 4.1): with an arbiter bound, a stale record
reads the red ``? authority stale``; the local guess (KEYS / the running procedure) is only for a
watch guest or a review, which have no arbiter.
"""

from mhseals_testing.ui import theme

STALE = ('unclaimed', '? authority stale')


def age(seconds):
    """``40 s`` under a minute, else ``4m`` (hours as ``2h``)."""
    seconds = max(0, int(seconds))
    if seconds < 60:
        return f'{seconds} s'
    return f'{seconds // 60}m' if seconds < 3600 else f'{seconds // 3600}h'


def since(record, now):
    return age(now - record['since_ns'] / 1e9) if record.get('since_ns') else ''


def _words(*parts):
    return ' '.join(str(p) for p in parts if p not in (None, ''))


def _claim(record, owner):
    return next((c for c in record.get('claims', ()) if c.get('owner') == owner
                 and c.get('state', 'active') == 'active'), {})


def chip(record, now):
    """``(kind, WORD)`` for the record; ``None`` when idle (the header shows a muted ``idle``)."""
    owner = (record or {}).get('owner', 'none')
    if not record or owner == 'none':
        return None
    rc, ago = record.get('rc') or {}, since(record, now)
    if owner == 'keyboard':
        return 'human', 'KEYS'
    if owner == 'rc':
        return 'human', _words('RC', f"{rc['latency_ms']:.0f} ms" if rc.get('latency_ms') else '')
    if owner == 'hold':
        hold = (record.get('holds') or [{}])[0]
        return 'hold', _words('HOLD', hold.get('by') or hold.get('cause'), ago)
    if owner == 'agent':
        return 'agent', _words('AGENT', _claim(record, 'agent').get('who'), ago)
    if owner == 'task':
        return 'ours', _words('TASK', (record.get('task') or {}).get('name'))
    if owner == 'procedure':
        proc = record.get('procedure') or {}
        name = proc.get('name') or _claim(record, 'procedure').get('who', '')
        return 'ours', _words(str(name).upper(), proc.get('progress'))
    if owner == 'nav':
        goal = next(iter((record.get('nav') or {}).get('goals', ())), {})
        return 'nav', _words('GOAL', f"{goal['distance_m']:.0f} m" if goal.get('distance_m') is not None else '')
    source = record.get('source', '')                       # external or unknown: unattributed
    if source.startswith('? goal'):
        return 'unclaimed', '? UNCLAIMED goal'
    return 'unclaimed', source if source.startswith('?') else STALE[1]


class Banner(str):
    """The banner text; ``parts`` are its ``(text, rank)`` pieces so a narrow row drops rank 3 first."""
    parts = ()


WORKING_S = 60     # a note this recent, with no agent claim, shows the unclaimed-activity item
MOTION_S = 10      # motion lease total when the act does not carry its own motion_s
ACT_STATE = {'waiting_arm': 'WAITING ARM', 'asking': 'ASKING'}


def act(desk, claim=None):
    """The desk's current act (``agent_desk['act']``) when it belongs to ``claim`` (or names none)."""
    item = (desk or {}).get('act')
    return item if item and (claim is None or item.get('claim') in (None, claim)) else None


def act_words(item):
    """``run square 34 s``: the desk's label (else the op and its args) and the time it has run."""
    args = item.get('args') or {}
    words = item.get('label') or _words(item.get('op'), *(args.values() if isinstance(args, dict) else args))
    elapsed = item.get('elapsed_s')
    state = item.get('state') if item.get('state') in ('starting', 'queued') else None
    return _words(words, state, age(elapsed) if isinstance(elapsed, (int, float)) and not state else None)


def lease(claim, desk, now):
    """``(text, low)``: ``lease ok`` or ``lease 3 s`` once under a third remains; None without one.

    A running act's motion lease wins over the claim's idle lease.
    """
    doing = act(desk, claim.get('id'))
    if doing and doing.get('motion_lease_s') is not None:
        left, total = doing['motion_lease_s'], doing.get('motion_s') or MOTION_S
    elif claim.get('lease_s'):
        total = claim['lease_s']
        left = total - (now - claim.get('seen_ns', claim.get('since_ns', 0)) / 1e9)
    else:
        return None
    low = left < total / 3
    return (f'lease {max(0, int(left))} s' if low else 'lease ok'), low


def banner(record, now, desk=None):
    """The agent banner text (agent-control 5), or None while no non-TUI claim is on record.

    Every claim whose owner is ``agent`` (a process other than the TUI owner) counts, active,
    suspended or busy; revoked ones do not. States: PAUSED (suspended), WAITING ARM, ASKING,
    IDLE (the desk runs no act) and DRIVING. ``desk`` is the AgentDesk view (``act``, ``queue``,
    ``notes``); without one (a watch guest) the claim alone reads DRIVING.
    """
    claims = [c for c in (record or {}).get('claims', ())
              if c.get('owner') == 'agent' and c.get('state', 'active') != 'revoked']
    if not claims:
        return None
    claim = next((c for c in claims if c.get('state', 'active') == 'active'), claims[0])
    paused = claim.get('state') == 'suspended'
    doing = act(desk, claim.get('id')) if desk is not None else None
    state = ('PAUSED' if paused else ACT_STATE.get((doing or {}).get('state'))
             or ('IDLE' if desk is not None and not doing else 'DRIVING'))
    held = lease(claim, desk, now)
    parts = [(f"{theme.AGENT_GLYPH} AGENT {state}", 0),
             ('RC override' if paused and record.get('owner') == 'rc' else None, 0),
             (claim.get('who'), 0),
             (f'{theme.FOCUS} {act_words(doing)}' if doing else None, 1),
             (held[0] if held else None, 0 if held and held[1] else 2),
             (f"pid {claim['pid']}" if claim.get('pid') else None, 3), (claim.get('reason') or None, 3),
             (since(claim, now) or None, 2)]
    parts = [(str(text), rank) for text, rank in parts if text]
    out = Banner(theme.SEPARATOR.join(text for text, _ in parts))
    out.parts = parts
    return out


def working(desk, record, now):
    """``(who, seconds)`` of the newest note while no agent claim exists: working, not driving."""
    if any(c.get('owner') == 'agent' and c.get('state', 'active') != 'revoked'
           for c in (record or {}).get('claims', ())):
        return None
    notes = (desk or {}).get('notes') or ()
    if not notes:
        return None
    note = notes[-1]
    ago = now - (note.get('t_ns') or 0) / 1e9
    return (note['who'], ago) if ago <= WORKING_S else None


def driving(feed, arbiter, manual, running, now):
    """The chip for the header, Motion subtitle and Driving row.

    ``feed`` is the merged feed view (``authority_record``, ``authority_paused``). With an arbiter a
    paused feed or a missing record is stale; without one the app's own state is the best guess.
    """
    if arbiter is not None:
        if feed.get('authority_paused') or not feed.get('authority_record'):
            return STALE
        return chip(feed['authority_record'], now)
    return (('human', 'KEYS') if manual else ('ours', running.upper()) if running else None)


def control(record, driving_chip, now, setting, **agent):
    """The ``snapshot['control']`` dict the Control rows read; ``setting`` is the Agents value and
    ``agent`` the ``agent_motion``/``agent_view`` settings."""
    return {**agent, 'driving': driving_chip, 'rc': (record or {}).get('rc'),
            'holds': [dict(h, age=age(now - h['since_ns'] / 1e9) if h.get('since_ns') else '')
                      for h in (record or {}).get('holds', ())], 'agents': setting}

"""Who drives, as the chip word (ui U4): a pure map from the arbiter's record to ``(kind, WORD)``.

``kind`` is a ``theme.CHIPS`` key (its hue is the ``auth.*`` token) and WORD the upper-case chip
text. ``driving`` adds the freshness rule (authority 3, 4.1): with an arbiter bound, a stale record
reads the red ``? authority stale``; the local guess (KEYS / the running procedure) is only for a
watch guest or a review, which have no arbiter.
"""

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

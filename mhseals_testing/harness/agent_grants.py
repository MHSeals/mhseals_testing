"""Which mux holds an agent may release (owner decision 2026-11-01). ROS-free.

``boat_agent release HOLD --why "..."``: a lease hold (the arbiter's ``lease``) or a watchdog hold (the TUI's own
``watchdog`` hold, or one a watchdog node made) once its fault has cleared, at once. An operator hold (Space or an
agent's ``stop``: ``keys``; ``agent_stop``, ``revoke``, the Agents and Agent-motion settings, anything else) only with
a grant in the grant file (agent.yaml ``grants``), then at once. The e-stop is never releasable.

The mux releases every hold at once (a tokenless release, as U), so a release goes ahead only when every hold the mux
has is the agent's to release now; the refusal names the one that is not. The grant file::

    grants:
      - who: claude-3f2a          # an agent id, or '*'
        holds: [keys]             # hold causes, or ['*']
        until: '2026-11-02 18:00' # optional, local time
        why: the link watchdog holds after a radio swap (known bug)
"""

from datetime import datetime
from pathlib import Path

LEASE = ('lease',)
OPERATOR = ('keys', 'agent_stop', 'revoke', 'motion observe', 'agents block', 'hold', 'keyboard', 'manual')
ESTOP = ('estop', 'e-stop', 'e_stop')


def load(path):
    """``(grants, error)``: the grant file's entries (none when it is missing), each ``{who, holds, until, why}``."""
    import yaml
    path = Path(path).expanduser()
    if not path.exists():
        return [], ''
    try:
        doc = yaml.safe_load(path.read_text()) or {}
        found = doc.get('grants') or []
        if not isinstance(found, list):
            raise ValueError('grants is a list')
        out = []
        for entry in found:
            if not isinstance(entry, dict) or not isinstance(entry.get('who'), str) \
                    or not isinstance(entry.get('holds'), list):
                raise ValueError('each grant needs who and holds')
            until = entry.get('until')
            out.append({'who': entry['who'], 'holds': [str(h) for h in entry['holds']],
                        'until': datetime.fromisoformat(str(until)).timestamp() if until else None,
                        'why': str(entry.get('why') or '')})
        return out, ''
    except Exception as error:      # noqa: BLE001 - a broken file grants nothing and says why
        return [], f'{path}: {error}'


def granted(grants, who, cause, now):
    """The grant that lets ``who`` release a ``cause`` hold at ``now`` (epoch s), else None."""
    return next((g for g in grants if g['who'] in (who, '*') and ('*' in g['holds'] or cause in g['holds'])
                 and (g['until'] is None or now <= g['until'])), None)


def kind(hold):
    """``lease``, ``watchdog`` (the TUI's own liveness hold), ``node`` (a watchdog node's), ``operator`` or ``estop``."""
    cause, by = str(hold.get('cause') or ''), str(hold.get('by') or '')
    if cause.lower() in ESTOP:
        return 'estop'
    if cause in LEASE:
        return 'lease'
    if cause == 'watchdog' and by in ('', 'tui'):
        return 'watchdog'
    if cause not in OPERATOR and by and by != 'tui' and not by.startswith(('view', 'agent')):
        return 'node'
    return 'operator'


def verdict(holds, name, who, grants, cleared, now):
    """``(error, hint)`` refusing ``who``'s release of hold ``name``, or None when every hold may go now.

    ``holds``: the mux's ``[{cause, by}]``; ``cleared(hold)``: ``(True, '')`` once the hold's fault cleared, else
    ``(False, why)``."""
    if name.lower() in ESTOP:
        return 'not_allowed', 'the e-stop is never releasable by an agent: the operator clears it on the boat'
    if not holds:
        return 'refused', 'the mux holds nothing'
    if name != 'all' and name not in [h.get('cause') for h in holds]:
        return 'bad_args', f"holds now: {', '.join(str(h.get('cause')) for h in holds)} (or all)"
    for hold in holds:
        cause, which = str(hold.get('cause')), kind(hold)
        if which == 'estop':
            return 'not_allowed', 'the e-stop is never releasable by an agent'
        if which == 'operator':
            if granted(grants, who, cause, now) is None:
                return 'not_allowed', (f"{cause} is the operator's hold: release it only with a grant for you in "
                                       'agent_grants.yaml (or ask the operator for U)')
            continue
        ok, why = cleared(hold)
        if not ok:
            return 'refused', f'{cause} hold: its fault has not cleared ({why})'
    return None

"""The control tracker in header row 1: who controls the boat, since when, and how many clients view it.

``item(tracker, now, short)`` renders ``view['control_tracker']`` (harness/control.py) in the reserved
authority hues: a human seat in ``auth.human``, an agent claim in ``auth.agent``, a stale or empty seat
in ``auth.unclaimed``. A viewer's item starts with VIEW and ends with the key that takes control.
"""

import time

from rich.text import Text

from mhseals_testing.ui import theme
from mhseals_testing.ui.authority_chip import age

HUMAN_GLYPH = theme.pick('●', '*')


def item(tracker, now=None, short=False):
    """The tracker as Text (``now``: wall seconds, default the clock), or None without one."""
    if not tracker:
        return None
    now = time.time() if now is None else now
    ctl, sep = tracker['controller'], theme.SEPARATOR
    kind = ctl['kind']
    out = Text()
    if tracker['role'] == 'view':
        out.append('VIEW', style=f"bold {theme.auth_style('human')}".strip())
        out.append(sep if not short else ' ', style=theme.MUTED)
    since = f" {age(now - ctl['since'])}" if ctl.get('since') else ''
    if kind == 'agent':
        out.append(f"{theme.AGENT_GLYPH} {ctl.get('who')} {ctl.get('claim')}", style=theme.auth_style('agent'))
        if not short:
            out.append(f" via {ctl.get('name')}", style=theme.MUTED)
    elif kind == 'human':
        out.append(f"{HUMAN_GLYPH} CTRL ", style=theme.auth_style('human'))
        out.append(ctl.get('name') if not short else (ctl.get('client') or ctl.get('name')),
                   style=f"bold {theme.auth_style('human')}".strip())
        if ctl.get('from') and not short:
            out.append(f" ({ctl['from']})", style=theme.MUTED)
    elif kind == 'unknown':     # a live seat without a fresh authority record: an agent may drive
        out.append('? CTRL unknown', style=theme.auth_style('unclaimed'))
        if not short:
            out.append(f" via {ctl.get('name')}", style=theme.MUTED)
    elif kind == 'stale':
        out.append(f"? CTRL {ctl.get('name')} gone", style=theme.auth_style('unclaimed'))
    else:
        out.append('? no controller', style=theme.auth_style('unclaimed'))
    if kind in ('agent', 'human') and since:
        out.append(since, style=theme.MUTED)
    if tracker.get('handover'):
        out.append(f'{sep}handover', style=theme.WARN)
    viewers = tracker['viewers']
    out.append(f'{sep}{viewers} view' if short else f"{sep}{viewers} viewing", style=theme.MUTED)
    if tracker['role'] == 'view' and not short:
        out.append(f'{sep}O take', style=theme.MUTED)
    return out

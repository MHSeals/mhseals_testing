"""Mission panels: what the task runner is doing and why (ui.feeds.mission.MissionState as ``view['mission']``).

``strip`` is the Drive strip over Activity, shown only while the status is fresh: the stage and
phase, behaviour and elapsed, the goal's why, and the stage ladder. ``pane`` is the same on the Map
tab beside Layers, with the ladder wrapping; ``decisions`` is the timestamped decision log,
coloured by kind. Every stage state has its own glyph, so colour is never the only cue.
"""

import time

from rich.columns import Columns
from rich.console import Group
from rich.text import Text

from mhseals_testing.ui import theme

STRIP_ROWS = 5      # the Drive strip: three interior rows and its border
LADDER = {'done': ('✓', '+'), 'running': ('▸', '>'), 'pending': ('·', '.'), 'skipped': ('↷', '-'),
          'failed': ('✗', 'x')}
STAGE_TOKEN = {'done': 'task.done', 'running': 'task.confirmed', 'skipped': 'task.done', 'failed': 'task.failed'}
KIND_WIDTH = 10     # columns of the decision kind word in the log


def _style(token, extra=''):
    try:
        return f'{theme.PALETTE.style(token)} {extra}'.strip()
    except KeyError:
        return theme.MUTED


def _safe(text):
    text = theme.ascii_safe(str(text))
    return text.encode('ascii', 'replace').decode() if theme.ASCII else text


def headline(status, behaviour=True):
    """``search · search · sweep_lawnmower · 23 s``: stage bold, then phase, behaviour and stage elapsed."""
    sep = theme.SEPARATOR
    out = Text(no_wrap=True, overflow=theme.OVERFLOW)
    out.append(_safe(status.get('stage') or status.get('task') or theme.DASH), style=_style('task.confirmed', 'bold'))
    facts = [status.get('phase')] + ([status.get('behaviour')] if behaviour else [])
    for fact in filter(None, facts):
        out.append(sep, style=theme.MUTED)
        out.append(_safe(fact))
    elapsed = ((status.get('stages') or {}).get(status.get('stage')) or {}).get('elapsed', status.get('elapsed'))
    if isinstance(elapsed, (int, float)):
        out.append(f'{sep}{elapsed:.0f} s', style=theme.MUTED)
    return out


def why(status):
    """``→ the goal's why``, else the status tip, else nothing."""
    goal = status.get('goal') if isinstance(status.get('goal'), dict) else {}
    reason = goal.get('why') or status.get('tip')
    out = Text(no_wrap=True, overflow=theme.OVERFLOW)
    if reason:
        out.append(f'{theme.ARROW} ', style=theme.MUTED)
        out.append(_safe(reason), style=_style('task.goal'))
    return out


def ladder(mission, compact=False):
    """``✓ nav_channel  ✗ docking ×3  ▸ search  · speed_challenge``: glyph, name and attempts past one.

    ``compact`` (the Drive strip) is one line naming only failed and running stages and the next
    pending one; the rest are their glyph. Otherwise the stages wrap between stages, never inside one."""
    entries, next_pending = [], True
    for name, state, attempts, _ in mission.ladder():
        style = _style(STAGE_TOKEN[state], 'bold' if state == 'running' else '') if state in STAGE_TOKEN else None
        named = not compact or state in ('failed', 'running') or (state == 'pending' and next_pending)
        next_pending = next_pending and state != 'pending'
        entry = Text(theme.pick(*LADDER.get(state, LADDER['pending'])) + (f' {_safe(name)}' if named else ''),
                     style=style or '')
        if attempts > 1:
            entry.append(f' {theme.pick("×", "x")}{attempts}', style=theme.MUTED)
        entries.append(entry)
    if not compact:
        return Columns(entries, padding=(0, 2))
    return Text(' ', no_wrap=True, overflow=theme.OVERFLOW).join(entries)


def decision(stamp, event, timed=True):
    """``12:04:31 give up  docking: failed 3x``; an untimed entry (from status.recent) shows dashes."""
    kind = str(event.get('kind', ''))
    out = Text(no_wrap=True, overflow=theme.OVERFLOW)
    if timed:
        out.append(time.strftime('%H:%M:%S ', time.localtime(stamp)) if stamp else f'{theme.DASH * 8} ',
                   style=theme.MUTED)
    kind_word = kind.replace('_', ' ')
    out.append(kind_word.ljust(KIND_WIDTH) if timed else f'{kind_word} ', style=_style('decision.' + kind, 'bold'))
    if event.get('stage'):
        out.append(f'{_safe(event["stage"])}: ')
    out.append(_safe(event.get('reason', '')), style=theme.MUTED)
    return out


def _latest(mission):
    recent = mission.recent(1)
    return decision(*recent[0], timed=False) if recent else None


def strip(view):
    """The Drive strip, or None while there is no fresh status (the slot then takes no rows)."""
    mission = view.get('mission')
    if mission is None or not mission.fresh():
        return None
    status = mission.status
    body = Group(headline(status), why(status), ladder(mission, compact=True))
    return theme.block(body, 'Mission', subtitle=_latest(mission))


def pane(view):
    """The Map tab's Mission box: headline, behaviour, why and the wrapping stage ladder."""
    mission = view.get('mission')
    status = mission.status if mission is not None else {}
    if not status:
        return theme.block(Text('no mission status', style=theme.MUTED), 'Mission')
    age = mission.age()
    subtitle = None if mission.fresh() else Text(f'{age:.0f} s old', style=_style('warn'))
    rows = [headline(status, behaviour=False)]
    if status.get('behaviour'):
        rows.append(Text(_safe(status['behaviour']), no_wrap=True, overflow=theme.OVERFLOW))
    rows += [why(status), ladder(mission)]
    return theme.block(Group(*rows), 'Mission', subtitle=subtitle)


def decisions(view, height):
    """The decision log, newest at the bottom, as many as fit in ``height`` rows."""
    mission = view.get('mission')
    recent = mission.recent(max(1, height - 2)) if mission is not None else []
    if not recent:
        return theme.block(Text('no decisions yet', style=theme.MUTED), 'Decisions')
    count = len(mission.decisions) if mission.decisions else len(recent)
    return theme.block(Group(*(decision(*entry) for entry in recent)), 'Decisions', subtitle=str(count))

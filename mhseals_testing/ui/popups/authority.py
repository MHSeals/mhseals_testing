"""The Authority popup (Enter on the Driving row, ui 5.4): who drives, why, and the claims.

Rows: Driving, Physical, RC, the Claims table (cursor; ``E`` revokes the selected claim),
Inputs, Nav2 and Recent (the journal's ``authority`` events). Everything comes from the
arbiter's record, so a missing field reads as ``-``. The text carries no status glyphs: the
owner is a word (AGENT, OURS, ...), never a mark. Keys are ``popup.authority`` in
config/keymap.yaml: up/down, ``E`` revoke the selected claim, ``U`` release_hold (passed to the app), Esc.
"""

import os
import time

from mhseals_testing.harness import journal
from mhseals_testing.harness.authority import is_unknown
from mhseals_testing.runner import fold

from mhseals_testing.ui import theme

RECENT = 3
KEYS = 'E revoke selected  U release hold  Esc close'
WORDS = {'agent': 'AGENT', 'rc': 'RC', 'keyboard': 'KEYS', 'hold': 'HOLD', 'external': 'UNCLAIMED',
         'none': 'idle', 'task': 'TASK', 'nav': 'GOAL', 'procedure': 'OURS', 'unknown': 'UNKNOWN'}


def word(owner):
    """The owner as a chip word: agents are AGENT, the TUI's own procedures OURS, unheard UNKNOWN."""
    return WORDS.get(owner, 'UNKNOWN')


def span(seconds):
    """``4m12s`` or ``9s``; ``-`` when unknown."""
    if seconds is None:
        return '-'
    seconds = max(0, int(seconds))
    return f'{seconds // 60}m{seconds % 60:02d}s' if seconds >= 60 else f'{seconds}s'


def claim_line(claim, now):
    """One Claims row's value: ``AGENT  tune_mppi  active  4m12s  host pid``."""
    where = ' '.join(filter(None, (claim.get('host'), str(claim['pid']) if claim.get('pid') else '')))
    held = span(now - claim['since_ns'] / 1e9) if claim['state'] in ('active', 'suspended') else '-'
    return (f"{word(claim['owner']):<7}{claim['who']:<14}{claim['state']:<10}{held:<8}"
            f"{where or 'this instance'}").rstrip()


def physical(record):
    if record.get('input') in (None, 'unknown'):
        return 'unknown · hold state unknown'
    holds = record.get('holds') or []
    hold = 'hold ' + ', '.join(h.get('cause', '?') for h in holds) if holds else 'hold off'
    return f"{record.get('input') or '-'}{theme.SEPARATOR}{hold}"


def rc_line(record):
    rc = record.get('rc')
    if not rc:
        return 'no receiver'
    if not rc.get('valid', True):
        return 'receiver lost'
    hz, p95 = rc.get('rate_hz'), rc.get('interval_p95_s')
    parts = [rc.get('mode'), f"{hz:g} Hz" if hz is not None else None,
             f"p95 {p95 * 1000:g} ms" if p95 is not None else None]
    return theme.SEPARATOR.join(p for p in parts if p) or '-'


def inputs_line(record):
    inputs = record.get('inputs') or {}
    return theme.SEPARATOR.join(f'{topic} {theme.pick("←", "<-")} {src or "-"}' for topic, src in inputs.items()) or '-'


def nav_line(record):
    goals = [g for g in (record.get('nav') or {}).get('goals', ()) if g.get('status') == 'executing']
    if not goals:
        return 'no goal'
    return theme.SEPARATOR.join(f"{str(g['id'])[:6]} executing" + (f" ({g['by']})" if g.get('by') else '')
                                for g in goals)


def recent_rows(events, limit=RECENT):
    """The last authority changes from the journal, newest last: ``14:00:02 idle <-> agent (claim)``."""
    out = []
    for event in events:
        if event.get('phase') != 'authority':
            continue
        stamp = event.get('t_ns')
        at = time.strftime('%H:%M:%S', time.localtime(stamp / 1e9)) if stamp else '-'
        old, new = event.get('from') or {}, event.get('to') or {}
        out.append(f"{at} {word(old['owner']) if 'owner' in old else '?'} {theme.pick('↔', '<->')} {word(new['owner']) if 'owner' in new else '?'}"
                   f" ({event.get('cause', '?')})")
    return out[-limit:]


def rows(record, events, now, cursor=0):
    """([(label, text)], [claim id per Claims row]) for the generic popup renderer."""
    if record is None:
        return [('Driving', 'no authority record yet')], []
    if record.get('stale'):
        return [('Driving', 'UNKNOWN · authority stale')], []
    owner = word(record.get('owner', 'unknown'))
    driving = theme.SEPARATOR.join(p for p in (owner, record.get('source'), record.get('reason') and
                                               f'"{record["reason"]}"',
                                               span(now - record['since_ns'] / 1e9) if record.get('since_ns') else None) if p)
    out = [('Driving', driving), ('Physical', physical(record)), ('RC', rc_line(record)), ('Claims', '')]
    claims = record.get('claims') or []
    for index, claim in enumerate(claims):
        mark = theme.CURSOR if index == cursor else '  '
        out.append((f"{mark}{claim['id']}", claim_line(claim, now)))
    if not claims:
        out.append(('', 'none'))
    recent = recent_rows(events)
    out += [('Inputs', inputs_line(record)), ('Nav2', nav_line(record)),
            ('Recent', theme.SEPARATOR.join(recent) or '-')]
    return out, [c['id'] for c in claims]


def modal(record, events, now, state):
    """The popup modal; ``state`` holds the claim ``cursor`` and receives the ``ids`` shown."""
    body, state['ids'] = rows(record, events, now, state['cursor'])
    seq = f"seq {record['seq']}" if record and record.get('seq') is not None else ''
    return dict(kind='message', title=theme.SEPARATOR.join(filter(None, ('Authority', seq))),
                rows=body, width=100, hint=KEYS)


def journal_reader(path):
    """A callable returning the journal events, re-parsed only when the file's size or mtime changes."""
    cache = dict(key=object(), events=[])

    def read():
        try:
            info = os.stat(path)
            key = (info.st_size, info.st_mtime_ns)
        except OSError:
            key = None
        if key != cache['key']:
            cache.update(key=key, events=journal.read(path))
        return cache['events']
    return read


def open(app):
    """Run the popup. ``app.arbiter`` is the arbiter (``record``, ``table``, ``revoke(id)``)."""
    arbiter = getattr(app, 'arbiter', None)
    if arbiter is None:
        return app.show('Authority', message='The authority arbiter is not running')
    state = dict(cursor=0, ids=[])
    events = journal_reader(app.paths.journal)
    def build():
        now = arbiter.clock()
        record = arbiter.record
        if record is not None and (record.get('t_ns') is None or is_unknown(record, now)):
            record = dict(stale=True)
        return modal(record, events(), now, state)
    box = build()

    def refresh():
        state['cursor'] = min(state['cursor'], max(0, len(state['ids']) - 1))
        box.update(build())

    def handle(key):
        raw, key = key, fold(key)
        if key == 'escape':
            return True
        if key in ('up', 'k'):
            state['cursor'] = max(0, state['cursor'] - 1)
        elif key in ('down', 'j'):
            state['cursor'] = min(len(state['ids']) - 1, state['cursor'] + 1)
        elif raw == 'E':   # case matters (K1): fold() would also accept lowercase e
            if state['ids']:
                claim = state['ids'][min(state['cursor'], len(state['ids']) - 1)]
                done = arbiter.revoke(claim)
                app.notice = f'Revoked {claim}' if done else f'{claim} is not active'
        refresh()
    return app.popup(box, handle, refresh)

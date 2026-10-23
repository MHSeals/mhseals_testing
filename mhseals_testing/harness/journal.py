"""The session journal: one append-only JSON line per event, fsynced.

Any process attached to the session appends here (the TUI, guests), so a line is
one ``O_APPEND`` write under ``MAX_LINE`` bytes, which the kernel keeps whole against
other appenders. A crash can still tear the last line; ``read`` skips it.
"""

import json
import os

MAX_LINE = 4000


def _encode(event):
    return (json.dumps(event, sort_keys=True, default=str) + '\n').encode()


def append(path, event):
    """Append ``event`` as one line and fsync; an oversized note is cut to fit."""
    data = _encode(event)
    note = event.get('note')
    if len(data) > MAX_LINE and isinstance(note, str):
        keep = max(0, len(note) - (len(data) - MAX_LINE) - 8)  # JSON escapes can grow a char
        data = _encode(dict(event, note=note[:keep]))
    fd = os.open(path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
    try:
        os.write(fd, data)
        os.fsync(fd)
    finally:
        os.close(fd)


def read(path):
    """All complete events in file order; a torn or non-object line is skipped."""
    try:
        with open(path, encoding='utf-8', errors='replace') as source:
            lines = source.read().splitlines()
    except FileNotFoundError:
        return []
    out = []
    for text in lines:
        try:
            event = json.loads(text)
        except ValueError:
            continue
        if isinstance(event, dict):
            out.append(event)
    return out


def markers(events):
    """Folded marker list in id order: a later ``marker_note`` replaces the note."""
    found = {}
    for e in events:
        if e.get('phase') == 'marker' and 'id' in e:
            found.setdefault(e['id'], dict(id=e['id'], t_ns=e.get('t_ns'), time=e.get('time'),
                                           kind=e.get('kind', 'user'), note=e.get('note', '')))
        elif e.get('phase') == 'marker_note' and e.get('id') in found:
            found[e['id']]['note'] = e.get('note', '')
    return [found[i] for i in sorted(found)]


def next_marker_id(events):
    return max((m['id'] for m in markers(events)), default=0) + 1

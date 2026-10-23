"""Markers in a session journal: the fold, and the edits made while reviewing.

Review never rewrites the bag or old journal lines. An edit is one more
append-only ``events.jsonl`` line flagged ``edited: true``; the reader folds
the latest note per marker id. ROS-free.
"""
import time

from mhseals_testing import data

JOURNAL = 'events.jsonl'


def fold(records):
    """Markers in journal order as ``{id, t_ns, kind, note, edited}``; the latest ``marker_note`` wins."""
    markers = {}
    for rec in records:
        phase = rec.get('phase')
        if phase == 'marker' and 'id' in rec:
            markers[rec['id']] = {'id': rec['id'], 't_ns': rec.get('t_ns', 0), 'kind': rec.get('kind', 'user'),
                                  'note': rec.get('note', ''), 'edited': bool(rec.get('edited'))}
        elif phase == 'marker_note' and rec.get('id') in markers:
            markers[rec['id']]['note'] = rec.get('note', '')
            markers[rec['id']]['edited'] = markers[rec['id']]['edited'] or bool(rec.get('edited'))
    return sorted(markers.values(), key=lambda m: (m['t_ns'], m['id']))


def read(session_dir):
    return fold(data.read_jsonl(session_dir / JOURNAL))


def nearest(markers, t_ns):
    """The marker closest to ``t_ns`` (earlier wins a tie), or None."""
    return min(markers, key=lambda m: (abs(m['t_ns'] - t_ns), m['t_ns']), default=None)


def add_at(session_dir, t_ns, note='', now_ns=None):
    """``b`` in review: a user marker at the playhead. Returns the new marker."""
    markers = read(session_dir)
    record = {'t_ns': int(t_ns), 'phase': 'marker', 'kind': 'user', 'id': max((m['id'] for m in markers), default=0) + 1,
              'note': note, 'rec': False, 'edited': True, 'edit_ns': now_ns if now_ns is not None else time.time_ns()}
    data.append_jsonl(session_dir / JOURNAL, record)
    return {k: record[k] for k in ('id', 't_ns', 'kind', 'note', 'edited')}


def note_nearest(session_dir, t_ns, note, now_ns=None):
    """``B`` in review: set the note of the marker nearest the playhead. Returns it, or None without markers."""
    target = nearest(read(session_dir), t_ns)
    if target is None:
        return None
    data.append_jsonl(session_dir / JOURNAL, {'t_ns': int(t_ns), 'phase': 'marker_note', 'id': target['id'], 'note': note,
                                              'edited': True, 'edit_ns': now_ns if now_ns is not None else time.time_ns()})
    return {**target, 'note': note, 'edited': True}

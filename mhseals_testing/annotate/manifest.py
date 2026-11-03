"""The session manifest: ``<session>/manifest.json``, the labelled index of a session for every consumer.

Schema ``mhseals.session_manifest/1`` (times are wall seconds, the bag's receive-stamp axis):

    session, profile, host, bag      where the data is (``bag`` relative to the manifest)
    time {t0, t1}, built, edits      the span covered; when it was folded; review edits applied
    pending                          review items nobody accepted or rejected yet (the TUI's review chip)
    conditions [{t, wind, waves, current}]   operator scales, 1 calm .. 3 rough, 0 unset
    segments [{id, kind, t0, t1, label, source, status, quality, flags, conditions, note, detail}]
    calibration [{id, kind, t0, t1, command, quality, flags, conditions, fix}]   windows to fit on
    commands [{id, t, text, source, pose, scene, outcome, status}]              operator language, NLC ground truth

``quality`` is the worst effect of what overlaps a segment (ok, degraded, suspect, invalid): tag spans, bad
data, interventions, link holds and the fix tier. A rejected segment is kept with ``status: rejected``; consumers
use ``usable()``. ``load`` is the one reader: it refuses another major version. ROS-free; stdlib besides ``fold``.
"""

from datetime import datetime, timezone
import json
from pathlib import Path

SCHEMA = 'mhseals.session_manifest/1'
FILE = 'manifest.json'
JOURNAL = 'events.jsonl'
REVIEW = ('mark', 'bad', 'intervention', 'said')       # kinds a person should look at
USABLE = ('ok', 'degraded')


def build(session_dir, events=None):
    """The manifest dict of ``session_dir`` from its journal (``events`` to fold instead)."""
    from mhseals_testing import data
    from mhseals_testing.annotate import config, fold as fold_module

    session_dir = Path(session_dir)
    folded = fold_module.fold(data.read_jsonl(session_dir / JOURNAL) if events is None else events)
    header = data.read_json(session_dir / 'session.json', {}) or {}
    items = folded['items']
    spans = fold_module.label_spans(items)
    steady = config.load()['steady']
    segments, calibration, commands = [], [], []
    for entry in items:
        at = fold_module.conditions_at(folded['conditions'], entry['t0'])
        if entry['kind'] == 'said':
            commands.append({'id': entry['id'], 't': entry['t0'], 'text': entry['label'], 'source': entry['source'],
                             'pose': entry['detail'].get('pose'), 'scene': entry['detail'].get('scene'),
                             'outcome': entry['detail'].get('outcome', ''), 'status': entry['status'],
                             'conditions': at, 'note': entry['note']})
            continue
        grade, flags = fold_module.quality(entry, spans) if entry['kind'] in fold_module.LABELLED else ('ok', [])
        segment = {k: entry[k] for k in ('id', 'kind', 't0', 't1', 'label', 'source', 'status', 'note')}
        segment.update(quality=grade, flags=flags, conditions=at, effect=entry['effect'], detail=entry['detail'])
        if entry['kind'] == 'mark':
            before, after = config.tag(entry['label'])['span']
            segment['span'] = [entry['t0'] + before, entry['t0'] + after]     # what the tag downgrades
        segments.append(segment)
        long_enough = entry['t1'] - entry['t0'] >= steady['neutral' if entry['kind'] == 'neutral' else 'calibration']
        if entry['kind'] in ('steady', 'neutral') and long_enough and entry['status'] != 'rejected' and grade in USABLE:
            calibration.append({'id': entry['id'], 'kind': entry['kind'], 't0': entry['t0'], 't1': entry['t1'],
                                'command': entry['detail'].get('command'), 'quality': grade, 'flags': flags,
                                'conditions': at, 'fix': entry['detail'].get('fix'), 'status': entry['status']})
    return {'schema': SCHEMA, 'session': header.get('id', session_dir.name), 'profile': _name(header.get('profile')),
            'host': header.get('host', ''), 'bag': 'bag', 'built': datetime.now(timezone.utc).isoformat(),
            'time': {'t0': folded['t0'], 't1': folded['t1']}, 'edits': folded['edits'],
            'pending': sum(1 for i in items if i['kind'] in REVIEW and i['status'] == 'pending'),
            'conditions': [{'t': t, **c} for t, c in folded['conditions']],
            'segments': segments, 'calibration': calibration, 'commands': commands}


def _name(profile):
    return profile.get('name', '') if isinstance(profile, dict) else (profile or '')


def write(session_dir, events=None):
    """Fold and atomically write ``<session_dir>/manifest.json``; returns the manifest."""
    from mhseals_testing import data
    manifest = build(session_dir, events)
    data.atomic_write(Path(session_dir) / FILE, json.dumps(manifest, indent=1, allow_nan=False, default=str) + '\n')
    return manifest


# --- the reader every consumer uses (stdlib only, so it can be imported by path) ---------------------

def load(path):
    """The manifest at ``path`` (a session folder or the file); ValueError on another major schema version."""
    path = Path(path)
    if path.is_dir():
        path = path / FILE
    doc = json.loads(path.read_text())
    name, _, major = str(doc.get('schema', '')).partition('/')
    if name != SCHEMA.split('/')[0] or major != SCHEMA.split('/')[1]:
        raise ValueError(f'{path}: schema {doc.get("schema")!r} is not {SCHEMA}')
    doc['path'] = str(path)
    return doc


def usable(segment, allow=USABLE):
    """Not rejected and of an allowed quality (suspect and invalid are out by default)."""
    return segment.get('status') != 'rejected' and segment.get('quality', 'ok') in allow


def segments(doc, kinds=None, allow=USABLE):
    """The usable segments of ``kinds`` (all kinds when None), in time order."""
    return [s for s in doc['segments'] if (kinds is None or s['kind'] in kinds) and usable(s, allow)]


def excluded(doc):
    """``(t0, t1, why)`` spans a fit must skip: bad data, interventions, link holds and invalid-tag spans."""
    out = [(s['t0'], max(s['t1'], s['t0'] + 1.0), s['kind']) for s in doc['segments']
           if s['kind'] in ('bad', 'intervention', 'link') and s.get('status') != 'rejected']
    out += [(*s.get('span', (s['t0'], s['t1'])), s['label']) for s in doc['segments']
            if s['kind'] == 'mark' and s.get('effect') == 'invalid' and s.get('status') != 'rejected']
    return sorted(out)


def overlaps(doc, t0, t1):
    """The exclusion reasons overlapping [t0, t1] (empty: clean)."""
    return sorted({why for a, b, why in excluded(doc) if a < t1 and b > t0})

"""Index the sessions under the data root for people and agents. ROS-free (json and PyYAML only).

``rebuild`` reads every ``sessions/<id>/`` and writes, all atomically:

* ``<session>/summary.json`` (``mhseals.summary/1``) from ``session.json``, the bag metadata,
  ``events.jsonl`` and ``runs/*.json``; every input is optional;
* ``INDEX.json`` (``mhseals.index/1``), one short row per session;
* ``README.md``, generated text explaining the layout and schemas.

Nothing here is kept as the source of truth, so a missing or stale index is simply rebuilt
(``ensure``). A plain rosbag2 directory (no ``session.json``) indexes as ``bare``.
Run ``python -m mhseals_testing.harness.index [DIR]`` (DIR is a session or bag; default: the root).
"""

from pathlib import Path
import sys

import yaml

from mhseals_testing import data

SUMMARY_SCHEMA = 'mhseals.summary/1'
INDEX_SCHEMA = 'mhseals.index/1'


def _bag_dirs(session):
    """Bag directories of a session: ``bag/`` itself, its ``*/`` segments, or the directory when it is a bare bag."""
    bag = session / 'bag'
    base = bag if bag.is_dir() else session
    found = [base] if _is_bag(base) else []
    found += sorted(d for d in base.iterdir() if d.is_dir() and _is_bag(d)) if base.is_dir() else []
    return found


def _is_bag(directory):
    return (directory / 'metadata.yaml').exists() or any(directory.glob('*.mcap')) or any(directory.glob('*.db3'))


def _segment(directory):
    """One bag directory as {dir, finalized, empty, bytes, start_ns, duration_s, topics}."""
    files = [f for f in directory.iterdir() if f.suffix in ('.mcap', '.db3')]
    size = sum(f.stat().st_size for f in files)
    meta = {}
    try:
        meta = yaml.safe_load((directory / 'metadata.yaml').read_text()).get('rosbag2_bagfile_information') or {}
    except (OSError, AttributeError, yaml.YAMLError):
        pass
    topics = []
    for entry in meta.get('topics_with_message_count') or []:
        info = entry.get('topic_metadata') or {}
        topics.append({'name': info.get('name'), 'type': info.get('type'), 'count': entry.get('message_count', 0)})
    return {'dir': directory.name, 'finalized': bool(meta), 'empty': size == 0, 'bytes': size,
            'start_ns': (meta.get('starting_time') or {}).get('nanoseconds_since_epoch'),
            'duration_s': (meta.get('duration') or {}).get('nanoseconds', 0) / 1e9, 'topics': topics}


def summarize(session):
    """The summary dict of one session (or bare bag) directory; never raises on missing inputs."""
    session = Path(session)
    header = data.read_json(session / 'session.json')
    bare = not isinstance(header, dict)
    header = {} if bare else header
    segments = [_segment(d) for d in _bag_dirs(session)]
    topics = {}
    for seg in segments:
        for t in seg['topics']:
            merged = topics.setdefault(t['name'], dict(t, count=0))
            merged['count'] += t['count']
    runs = [r for r in (data.read_json(p) for p in sorted((session / 'runs').glob('*.json'))) if isinstance(r, dict)]
    events = data.read_jsonl(session / 'events.jsonl')
    flags = []
    if bare:
        flags.append('bare')
    if segments and all(s['empty'] for s in segments):
        flags.append('empty')
    elif any(not s['finalized'] and not s['empty'] for s in segments):
        flags.append('unfinalized')
    if not bare and not header.get('ended'):
        flags.append('open')
    statuses = [r.get('status') for r in runs]
    outcome = next((s.lower() for s in ('MEASURED', 'ABORTED', 'EXCLUDED') if s in statuses), '')
    clock = header.get('clock') or {}
    profile = header.get('profile')
    starts = [s['start_ns'] for s in segments if s['start_ns']]
    return {
        'id': header.get('id') or session.name, 'title': header.get('title', ''), 'note': header.get('note', ''),
        'pinned': bool(header.get('pinned')), 'profile': profile, 'host': header.get('host'),
        'kind': header.get('kind') or ('sim' if clock.get('domain') == 'sim' else 'boat' if not bare else 'headless'),
        'clock': clock, 'git': header.get('git', {}), 'config': header.get('config', {}),
        'time': {'start': header.get('created') or (min(starts) / 1e9 if starts else None), 'end': header.get('ended'),
                 'duration_s': round(sum(s['duration_s'] for s in segments), 3)},
        'segments': [{k: v for k, v in s.items() if k not in ('topics', 'start_ns')} for s in segments],
        'topics': sorted(topics.values(), key=lambda t: str(t['name'])),
        'markers': [e for e in events if (e.get('kind') or e.get('type')) == 'marker'],
        'runs': [{'id': r.get('id') or r.get('run_id'), 'procedure': r.get('procedure'), 'status': r.get('status')}
                 for r in runs],
        'outcome': outcome, 'flags': flags}


def row(summary):
    """The one-line INDEX.json record of a summary."""
    keys = ('id', 'title', 'host', 'profile', 'kind', 'outcome', 'flags', 'pinned', 'config')
    return {**{k: summary[k] for k in keys}, 'time': summary['time']['start'], 'duration_s': summary['time']['duration_s'],
            'markers': len(summary['markers']), 'runs': len(summary['runs']), 'bytes': sum(s['bytes'] for s in summary['segments'])}


README = """\
# Session data

Generated by `python -m mhseals_testing.harness.index`; do not edit. Everything here is JSON or JSONL,
and every file starts with a `schema` field.

## Layout

```
INDEX.json                 one row per session (schema mhseals.index/1); grep or jq this first
README.md                  this file
sessions/<id>/
  session.json             header: id, title, note, pinned, host, profile, clock, created, ended, git, config
  summary.json             generated digest of the session (schema mhseals.summary/1)
  events.jsonl             markers, procedure start/stop, holds, takeovers
  runs/<run_id>.json       one per procedure run; status MEASURED, ABORTED or EXCLUDED
  bag/                     rosbag2 MCAP (segments as subdirectories when split)
  logs/  snapshots/        child process logs; config snapshots
config-history/  calibration/
```

`INDEX.json` and `summary.json` are rebuilt from the other files, so they can be deleted at any time.
A directory holding only a rosbag2 bag (no `session.json`) is indexed too, with the flag `bare`.

## INDEX.json

`{"schema": "mhseals.index/1", "sessions": [row, ...]}`, newest first. A row has: `id`, `title`, `host`,
`profile`, `kind` (boat, sim, headless), `outcome` (measured, aborted, excluded or empty), `flags`, `pinned`,
`config` (config version ids), `time` (start), `duration_s`, `markers` (count), `runs` (count), `bytes`.

## summary.json

`id, title, note, pinned, profile, host, kind, clock, git, config`, `time {start, end, duration_s}`,
`segments [{dir, finalized, empty, bytes, duration_s}]`, `topics [{name, type, count}]`, `markers [...]`,
`runs [{id, procedure, status}]`, `outcome`, `flags`.

## Flags

* `open`: `session.json` has no `ended` (still recording, or it crashed).
* `unfinalized`: a bag segment has data but no `metadata.yaml`; reindex before reading.
* `empty`: every bag file is 0 bytes.
* `bare`: no `session.json`; topics only.
"""


def _write_all(root, summaries):
    rows = sorted((row(s) for s in summaries), key=lambda r: (str(r['time'] or ''), r['id']), reverse=True)
    data.write_json(root / 'INDEX.json', {'sessions': rows}, INDEX_SCHEMA)
    data.atomic_write(root / 'README.md', README)
    return rows


def rebuild(root=None):
    """Rewrite every summary.json, INDEX.json and README.md under ``root``; returns the index rows."""
    root = Path(root) if root else data.root()
    summaries = []
    for session in sorted((root / 'sessions').glob('*/')) if (root / 'sessions').is_dir() else []:
        if not session.is_dir() or session.name.startswith('.'):
            continue
        summary = summarize(session)
        data.write_json(session / 'summary.json', summary, SUMMARY_SCHEMA)
        summaries.append(summary)
    return _write_all(root, summaries)


def ensure(root=None):
    """Rebuild only when INDEX.json is missing or older than any session file; returns True if it rebuilt."""
    root = Path(root) if root else data.root()
    index = root / 'INDEX.json'
    if index.exists():
        newest = max((p.stat().st_mtime for p in (root / 'sessions').glob('*/*') if p.name != 'summary.json'), default=0)
        if (root / 'README.md').exists() and index.stat().st_mtime >= newest:
            return False
    rebuild(root)
    return True


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    if args:
        summary = summarize(args[0])
        data.write_json(Path(args[0]) / 'summary.json', summary, SUMMARY_SCHEMA)
        print(f'{summary["id"]}: {", ".join(summary["flags"]) or "ok"}')
        return 0
    print(f'{len(rebuild())} sessions indexed under {data.root()}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

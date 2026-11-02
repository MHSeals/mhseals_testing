"""The fast <-> Unity pair archive (``~/.cache/fastsim-pairs``, append-only, read only here) as
judge pairs: [(pair_id, fast RunRecord, unity RunRecord)], with refusals counted."""

from pathlib import Path

from mhseals_testing.judge_pack.roboboat._core import core

core()

from judge.record import Scenario  # noqa: E402
from judge.source import Refused  # noqa: E402

from mhseals_testing.judge_pack.roboboat.sources import SessionSource, json_load, provenance_of  # noqa: E402

ROOT = Path('~/.cache/fastsim-pairs').expanduser()


def _unity_provenance(session):
    runs = sorted(Path(session).glob('runs/*.json'))
    git = ((json_load(runs[-1], {}) or {}).get('provenance') or {}).get('git') if runs else None
    return {'code': {k: (v or {}).get('hash') for k, v in (git or {}).items()}, 'git': git or {}}


def rows(root=ROOT):
    import json
    p = Path(root) / 'index.jsonl'
    return [json.loads(line) for line in p.read_text().splitlines() if line.strip()] if p.exists() else []


def load(root=ROOT, task=None):
    """(pairs, refused [(pair_id, reason)], index rows by pair id)."""
    out, refused, by = [], [], {}
    src = SessionSource()
    for r in rows(root):
        if task and r['condition'].get('task') != task:
            continue
        pid = r['pair_id']
        by[pid] = r
        d = Path(root) / 'pairs' / pid
        sc = Scenario(r['pair_key'], r['truth_sha256'], r.get('seed'), (r.get('options') or {}).get('family'),
                      r.get('condition') or {})
        try:
            fast = src.load(d / 'fast', sc, provenance_of(r.get('fast')), f'{pid}/fast')
            sid = (json_load(d / 'unity' / 'session.json', {}) or {}).get('id') or pid
            unity = src.load(d / 'unity', sc, _unity_provenance(d / 'unity'), f'unity:{sid}')
        except Refused as e:
            refused.append((pid, str(e)))
            continue
        out.append((pid, fast, unity))
    return out, refused, by

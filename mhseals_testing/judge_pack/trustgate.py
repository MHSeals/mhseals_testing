"""Fast tier vs Unity trust gate: G3 (rank agreement) and G4 (per-metric gaps) over the pair archive.

    python -m mhseals_testing.judge_pack.trustgate [--archive ~/.cache/fastsim-pairs] [--json OUT]

Thresholds are the frozen ones of 2026-10-26 (astro_dock .handoff/state/fastsim.md, "Trust gate"); they
live in GATE and are never tuned here. A pair counts only when it is *eligible*: both runs scored, and the
fast twin ran the same nav commit as the Unity run (a twin of another nav is a different variant, kept
apart as `unmatched`). Runs whose fast session carries a blind-spot flag are reported apart as well as in
the gate (the frozen text gates on all eligible pairs and lists the flagged ones separately).

The archive is read only (crane.pair v1 layout: index.jsonl + pairs/<id>/{fast,unity,diff.json}).
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import statistics
import subprocess
from pathlib import Path

# What makes two nav commits the same variant for the fast tier and Unity: the mission logic, its
# configuration and the behaviour trees (a docstring or a launch change elsewhere is the same variant).
NAV_LOGIC = ('mhseals_nav/tasks', 'config', 'mhseals_nav/behavior_trees')

GATE = {
    'g3': {'min_variants': 8, 'min_seeds': 5, 'spearman': 0.80, 'kendall': 0.60, 'top_k': 2,
           'winners_top3': 2, 'regret_pts': 50.0, 'mmrv': 0.08},
    'g4': {'completion_agreement': 0.85, 'duration_median': (0.80, 1.25), 'duration_band': (0.67, 1.5),
           'duration_band_share': 0.80, 'path_median': (0.85, 1.15), 'contacts_tol': 1,
           'contacts_share': 0.90},
}


# ---------------------------------------------------------------- loading

def _score(run_dir: Path) -> dict | None:
    files = sorted(glob.glob(str(run_dir / 'analysis' / 'score.*.json')))
    return json.loads(Path(files[-1]).read_text()) if files else None


def _run_record(run_dir: Path) -> dict:
    files = sorted((run_dir / 'runs').glob('*.json'))
    return json.loads(files[-1].read_text()) if files else {}


def _nav(record: dict) -> str | None:
    git = (record.get('provenance') or {}).get('git') or {}
    nav = git.get('mhseals_nav') or git.get('nav') or {}
    return (nav.get('commit') or nav.get('hash') or '')[:7] or None


def path_length(run_dir: Path, record: dict) -> float | None:
    rel = ((record.get('result') or {}).get('files') or {}).get('track')
    track = run_dir / rel if rel else run_dir / 'track.jsonl'
    if not track.is_file():
        return None
    total, last = 0.0, None
    for line in track.read_text().splitlines():
        if not line.strip():
            continue
        p = json.loads(line)
        if last is not None:
            total += math.hypot(p['x'] - last[0], p['y'] - last[1])
        last = (p['x'], p['y'])
    return total


def side(run_dir: Path) -> dict:
    s, rec = _score(run_dir), _run_record(run_dir)
    fastsim = run_dir / 'fastsim.json'
    flags = json.loads(fastsim.read_text()).get('flags', []) if fastsim.is_file() else []
    if s is None:
        return {'scored': False, 'nav': _nav(rec)}
    tasks = s.get('tasks') or {}
    by_task = (s.get('total') or {}).get('by_task') or {}
    return {
        'scored': True, 'nav': _nav(rec), 'flags': sorted(flags) if isinstance(flags, list) else sorted(flags),
        'points': float((s.get('total') or {}).get('points') or 0),
        'possible': float((s.get('total') or {}).get('possible_scorable') or (s.get('total') or {}).get('possible') or 0),
        'attempted': {t: bool(v.get('attempted')) for t, v in tasks.items()},
        'earned': {t: float(by_task.get(t) or 0) > 0 for t in tasks},
        'duration_s': float((s.get('run') or {}).get('duration_s') or 0),
        'end': ((s.get('run') or {}).get('end') or {}).get('reason'),
        'strikes': int(((s.get('penalties') or {}).get('contacts') or {}).get('strike') or 0),
        'path_m': path_length(run_dir, rec),
        'outcome': (rec.get('result') or {}).get('outcome'),
    }


def logic_id(nav_repo: Path | None, commit: str | None) -> str | None:
    """Tree ids of NAV_LOGIC at a commit (None without a repo or for an unknown commit)."""
    if not (nav_repo and commit):
        return None
    ids = []
    for path in NAV_LOGIC:
        r = subprocess.run(['git', '-C', str(nav_repo), 'rev-parse', f'{commit}:{path}'],
                           capture_output=True, text=True)
        if r.returncode:
            return None
        ids.append(r.stdout.strip())
    return ':'.join(ids)


def load(archive: Path, nav_repo: Path | None = None) -> list[dict]:
    out = []
    for line in (archive / 'index.jsonl').read_text().splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        d = archive / 'pairs' / row['pair_id']
        fast, unity = side(d / 'fast'), side(d / 'unity')
        same_commit = bool(fast.get('nav')) and fast.get('nav') == unity.get('nav')
        fl, ul = logic_id(nav_repo, fast.get('nav')), logic_id(nav_repo, unity.get('nav'))
        match = 'commit' if same_commit else ('logic' if fl and fl == ul else None)
        out.append({'match': match, 'pair_id': row['pair_id'], 'task': (row.get('condition') or {}).get('task'),
                    'profile': (row.get('condition') or {}).get('profile'), 'seed': row.get('seed'),
                    'unity_session': Path(row.get('unity_from', '')).name, 'fast': fast, 'unity': unity,
                    'matched': match is not None, 'variant': (fl or fast.get('nav')),
                    'scored': fast['scored'] and unity['scored']})
    return out


# ---------------------------------------------------------------- statistics

def ranks(values: list[float]) -> list[float]:
    order = sorted(range(len(values)), key=lambda i: values[i])
    r = [0.0] * len(values)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and values[order[j + 1]] == values[order[i]]:
            j += 1
        for k in range(i, j + 1):
            r[order[k]] = (i + j) / 2 + 1
        i = j + 1
    return r


def pearson(a: list[float], b: list[float]) -> float | None:
    if len(a) < 2 or len(set(a)) < 2 or len(set(b)) < 2:
        return None
    ma, mb = statistics.fmean(a), statistics.fmean(b)
    num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    return num / math.sqrt(sum((x - ma) ** 2 for x in a) * sum((y - mb) ** 2 for y in b))


def spearman(a, b):
    return pearson(ranks(a), ranks(b))


def kendall_tau_b(a, b) -> float | None:
    n = len(a)
    conc = disc = ta = tb = 0
    for i in range(n):
        for j in range(i + 1, n):
            da, db = a[i] - a[j], b[i] - b[j]
            if da == 0 and db == 0:
                continue
            if da == 0:
                ta += 1
            elif db == 0:
                tb += 1
            elif (da > 0) == (db > 0):
                conc += 1
            else:
                disc += 1
    denom = math.sqrt((conc + disc + ta) * (conc + disc + tb))
    return (conc - disc) / denom if denom else None


def mmrv(low: dict, ref: dict) -> float:
    """Mean maximum rank violation (SIMPLER): per variant, the largest reference gap to a variant the low tier orders the other way."""
    keys = list(ref)
    worst = []
    for i in keys:
        worst.append(max([abs(ref[i] - ref[j]) for j in keys
                          if j != i and (low[i] < low[j]) != (ref[i] < ref[j])] or [0.0]))
    return statistics.fmean(worst) if worst else 0.0


def top(d: dict, k: int) -> list:
    return sorted(d, key=lambda v: -d[v])[:k]


# ---------------------------------------------------------------- gates

def g3(pairs: list[dict]) -> dict:
    """Per task scope: variants = nav commits; a variant needs the same >= 5 seeds in both tiers."""
    cfg = GATE['g3']
    scopes: dict = {}
    for p in pairs:
        if p['scored'] and p['matched']:
            scopes.setdefault(p['task'], {}).setdefault(p['variant'], {})[p['seed']] = p
    out = {}
    for task, variants in sorted(scopes.items(), key=lambda kv: str(kv[0])):
        common = set.intersection(*(set(v) for v in variants.values())) if variants else set()
        counts = {'variants': len(variants), 'common_seeds': sorted(common),
                  'paired_runs': sum(len(v) for v in variants.values())}
        row = {'counts': counts}
        if len(variants) >= 2 and common:
            norm = lambda s: s['points'] / s['possible'] if s['possible'] else 0.0
            fast = {v: statistics.fmean(norm(variants[v][s]['fast']) for s in common) for v in variants}
            unity = {v: statistics.fmean(norm(variants[v][s]['unity']) for s in common) for v in variants}
            a, b = [fast[v] for v in variants], [unity[v] for v in variants]
            row['descriptive'] = {'fast': fast, 'unity': unity, 'spearman': spearman(a, b),
                                  'kendall_tau_b': kendall_tau_b(a, b), 'pearson': pearson(a, b),
                                  'mmrv': mmrv(fast, unity),
                                  'unity_best_in_fast_top2': top(unity, 1)[0] in top(fast, cfg['top_k']),
                                  'fast_best_in_unity_top2': top(fast, 1)[0] in top(unity, cfg['top_k'])}
        enough = len(variants) >= cfg['min_variants'] and len(common) >= cfg['min_seeds']
        row['status'] = 'measurable' if enough else 'insufficient'
        row['why'] = None if enough else (f"{len(variants)} variant(s) x {len(common)} common seed(s); "
                                          f"G3 needs >= {cfg['min_variants']} x {cfg['min_seeds']} (40 paired runs)")
        out[task] = row
    verdict = 'not measured' if not any(r['status'] == 'measurable' for r in out.values()) else 'see scopes'
    return {'verdict': verdict, 'scopes': out}


def _within(x, lo_hi):
    return lo_hi[0] <= x <= lo_hi[1]


def g4(pairs: list[dict]) -> dict:
    cfg = GATE['g4']
    use = [p for p in pairs if p['scored'] and p['matched']]
    comp = []
    for p in use:
        f, u = p['fast'], p['unity']
        for t in sorted(set(f['attempted']) | set(u['attempted'])):
            if f['attempted'].get(t) or u['attempted'].get(t):
                comp.append(f['earned'].get(t, False) == u['earned'].get(t, False))
    dur = [p['fast']['duration_s'] / p['unity']['duration_s'] for p in use if p['unity']['duration_s'] > 0]
    path = [p['fast']['path_m'] / p['unity']['path_m'] for p in use
            if p['fast']['path_m'] is not None and p['unity']['path_m']]
    cont = [abs(p['fast']['strikes'] - p['unity']['strikes']) <= cfg['contacts_tol'] for p in use]

    def ledger(key):
        d = [p['fast'][key] - p['unity'][key] for p in use if p['fast'][key] is not None and p['unity'][key] is not None]
        return {'n': len(d), 'bias': statistics.fmean(d) if d else None,
                'mae': statistics.fmean(abs(x) for x in d) if d else None}

    checks = {
        'completion': {'value': sum(comp) / len(comp) if comp else None, 'n': len(comp),
                       'pass': bool(comp) and sum(comp) / len(comp) >= cfg['completion_agreement'],
                       'rule': f">= {cfg['completion_agreement']:.0%} of (run, task) pairs agree"},
        'duration_median': {'value': statistics.median(dur) if dur else None, 'n': len(dur),
                            'pass': bool(dur) and _within(statistics.median(dur), cfg['duration_median']),
                            'rule': f"median fast/Unity in {list(cfg['duration_median'])}"},
        'duration_band': {'value': sum(_within(r, cfg['duration_band']) for r in dur) / len(dur) if dur else None,
                          'n': len(dur), 'pass': bool(dur) and sum(_within(r, cfg['duration_band']) for r in dur)
                          / len(dur) >= cfg['duration_band_share'],
                          'rule': f">= {cfg['duration_band_share']:.0%} within {list(cfg['duration_band'])}"},
        'path_median': {'value': statistics.median(path) if path else None, 'n': len(path),
                        'pass': bool(path) and _within(statistics.median(path), cfg['path_median']),
                        'rule': f"median fast/Unity path in {list(cfg['path_median'])}"},
        'contacts': {'value': sum(cont) / len(cont) if cont else None, 'n': len(cont),
                     'pass': bool(cont) and sum(cont) / len(cont) >= cfg['contacts_share'],
                     'rule': f"|fast - Unity| strikes <= {cfg['contacts_tol']} in >= {cfg['contacts_share']:.0%}"},
    }
    flagged = [p['pair_id'] for p in use if p['fast'].get('flags')]
    return {'verdict': 'pass' if use and all(c['pass'] for c in checks.values()) else ('fail' if use else 'no pairs'),
            'eligible_pairs': len(use), 'checks': checks,
            'ledger': {k: ledger(k) for k in ('points', 'duration_s', 'path_m', 'strikes')},
            'flagged_pairs': flagged,
            'per_task': {t: len([p for p in use if p['task'] == t]) for t in sorted({p['task'] for p in use}, key=str)}}


def dedupe(pairs: list[dict]) -> list[dict]:
    """One pair per (Unity session, variant): several fast twins of one logic are the same evidence."""
    seen, out = set(), []
    for p in sorted(pairs, key=lambda p: p['match'] != 'commit'):   # an exact-commit twin wins
        key = (p['unity_session'], p['variant'], p['task'], p['seed'])
        if p['matched'] and key in seen:
            continue
        seen.add(key)
        out.append(p)
    return out


def report(archive: Path, nav_repo: Path | None = None) -> dict:
    pairs = dedupe(load(archive, nav_repo))
    unmatched = [p['pair_id'] for p in pairs if p['scored'] and not p['matched']]
    return {'schema': 'mhseals.trustgate/1', 'archive': str(archive), 'thresholds': 'frozen 2026-10-26',
            'pairs': len(pairs), 'nav_repo': str(nav_repo) if nav_repo else None, 'unmatched_nav': len(unmatched), 'unscored': len([p for p in pairs if not p['scored']]),
            'g3': g3(pairs), 'g4': g4(pairs),
            'rows': [{k: p[k] for k in ('pair_id', 'task', 'seed', 'unity_session', 'matched', 'match')} |
                     {'nav': p['fast'].get('nav'), 'unity_nav': p['unity'].get('nav'),
                      **{f'{s}.{m}': p[s].get(m) for s in ('fast', 'unity')
                         for m in ('points', 'duration_s', 'path_m', 'strikes', 'end')}} for p in pairs]}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--archive', default=str(Path.home() / '.cache' / 'fastsim-pairs'))
    ap.add_argument('--json', help='write the full report here')
    ap.add_argument('--nav-repo', help='an mhseals_nav clone: twins whose mission logic trees are equal count as matched')
    a = ap.parse_args(argv)
    r = report(Path(a.archive).expanduser(), Path(a.nav_repo).expanduser() if a.nav_repo else None)
    if a.json:
        Path(a.json).write_text(json.dumps(r, indent=1, sort_keys=True, default=str))
    print(f"pairs {r['pairs']} (nav-matched eligible {r['g4']['eligible_pairs']}, unmatched {r['unmatched_nav']}, "
          f"unscored {r['unscored']})")
    print(f"G3 {r['g3']['verdict']}: " + '; '.join(f"{t}: {s['status']} ({s['why'] or 'ok'})"
                                                   for t, s in r['g3']['scopes'].items()))
    print(f"G4 {r['g4']['verdict']}: " + '; '.join(
        f"{k} {'PASS' if c['pass'] else 'FAIL'} {c['value'] if c['value'] is None else round(c['value'], 3)} (n {c['n']})"
        for k, c in r['g4']['checks'].items()))


if __name__ == '__main__':
    main()

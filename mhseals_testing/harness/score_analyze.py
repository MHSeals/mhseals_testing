"""Score a finished session by the RoboBoat 2026 rules, headless: ``boat_trial analyze score``.

Reads files only (no ROS), through the session's source adapter
(:mod:`mhseals_testing.analysis.run_record`: kinematic | unity | real, one run record with
declared capabilities; ``--paired`` compares explicit pairs across tiers):
``course_truth.json``, ``track.jsonl`` (ground truth ``{t, x, y, yaw}``, captured by the
harness from its run start), authority changes from ``events.jsonl`` and optional
``states.jsonl`` (course state changes); a harness run is read from its run record
``runs/<run_id>.json`` and its journal slice (``run_record``), and SESSION may name that record.
The run window is the harness's: it starts at the run record's (else ``session.json``)
``run_start_t`` (else the first track sample) and ends at ``time_limit_s``
(none is guessed: without it, the track end with earned rows provisional) or the track end;
the judge cuts it earlier at the exit, a hold, a takeover or out of bounds.
``task_events.jsonl`` holds the boat's claims, never used to judge or to window. Writes
``<session>/analysis/score.<ts>.json`` (schema ``mhseals.score/1``, docs in the module
:mod:`mhseals_testing.analysis.score`). Exit 0 when scored, 2 when it cannot be.
"""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import sys

from mhseals_testing import data
from mhseals_testing.analysis import diagnostics, reliability, run_record, score

CLAIM_EVENTS = ('stage_completed', 'stage_failed', 'stage_skipped', 'gate_cleared', 'docked',
                'undocked', 'debris', 'speed_started', 'speed_finished', 'returned_home')
STAGE_TASK = {'nav_channel': 'entry_gates', 'return_home': 'entry_gates',
              'follow_path': 'debris_clearance', 'docking': 'marina'}   # 2025 stage names


Refused = run_record.Refused
run_window = run_record.run_window


def _claimed_window(task_events):
    """The boat's own task_started .. task_finished: a claim, reported beside the run only."""
    start = next((e['time'] for e in task_events if e.get('event') == 'task_started'), None)
    end = next((e['time'] for e in task_events if e.get('event') == 'task_finished'), None)
    return start, end


def load(session):
    """The run record (``run_record.load``) through the session's source adapter."""
    return run_record.load(session, score.PARAMS['time_limit_s'])


def statuses(session):
    """The boat's /task/status records (mhseals.mission_status/1), if the session kept them."""
    return list(data.read_jsonl(Path(session) / 'task_status.jsonl'))


def claims(task_events, report):
    """The boat's claims beside what ground truth credits, per task (claims never score)."""
    said = [dict(e, t=e.get('time')) for e in task_events
            if e.get('event') in CLAIM_EVENTS + ('decision', 'stage_started', 'spill_count',
                                                 'species_count')]
    out = {}
    for c in said:
        if c['event'] != 'stage_completed' or not c['stage']:
            continue
        task = STAGE_TASK.get(c['stage'], c['stage'])
        judged = report['tasks'].get(task)
        done = bool(judged) and any(r['status'] in ('earned', 'provisional')
                                    for r in judged['rows'])
        out[c['stage']] = dict(task=task, claimed=True, credited=done, false_claim=not done)
    for task, judged in report['tasks'].items():
        credited = any(r['status'] in ('earned', 'provisional') for r in judged['rows'])
        if credited and not any(v['task'] == task for v in out.values()):
            out[task] = dict(task=task, claimed=False, credited=True, missed_claim=True)
    return dict(events=said, by_stage=out)


def motion(track, t0, t1):
    """Path length, time, average/peak speed and time stopped (< 0.1 m/s) in the window."""
    part = [p for p in track if t0 <= p[0] <= t1]
    length = stopped = peak = 0.0
    j = 0
    for i, (a, b) in enumerate(zip(part, part[1:])):
        dt, d = b[0] - a[0], math.dist(a[1:3], b[1:3])
        length += d
        stopped += dt if dt > 0 and d / dt < 0.1 else 0.0
        j = max(j, i)
        while j < len(part) - 1 and part[j][0] - a[0] < 0.5:   # speed over >= 0.5 s
            j += 1
        if part[j][0] - a[0] >= 0.5:
            peak = max(peak, math.dist(a[1:3], part[j][1:3]) / (part[j][0] - a[0]))
    span = (part[-1][0] - part[0][0]) if len(part) > 1 else 0.0
    return dict(path_m=round(length, 1), time_s=round(span, 1),
                mean_speed=round(length / span, 3) if span else None,
                peak_speed=round(peak, 3), stopped_s=round(stopped, 1))


def analyze(session, now=None):
    rec = load(session)
    truth, track, task_events = rec['truth'], rec['track'], rec['task_events']
    window = rec['window'][:2]
    body = score.judge_record(rec)
    if rec['authority_problem']:
        body['run']['warnings'].append(rec['authority_problem'])
    body['run']['window_source'] = rec['window_source']
    body['run']['claimed_window'] = list(_claimed_window(task_events))   # the boat's, unused
    body['run']['source'] = rec['source']           # a label: scoring reads only caps
    if rec['run_id']:
        body['run']['run_id'] = rec['run_id']     # a harness run record (runs/<run_id>.json)
    gen = truth.get('generator') or {}
    body['run'].update(session=rec['session'].name, course=dict(
        name=truth.get('name'), family=truth.get('family'), seed=gen.get('seed'),
        mode=gen.get('mode'), truth_sha256=gen.get('truth_sha256')))
    body['claims'] = claims(task_events, body) if rec['caps']['claims'] else \
        dict(events=[], by_stage={}, unavailable='no task events or status kept')
    body['diagnostics'] = dict(motion=motion(track, window[0], body['run']['scored_until']))
    model = score.course(truth)
    run = score.with_heading([p for p in track
                              if window[0] <= p[0] <= body['run']['scored_until']])
    program = (model['states'].get('harbor_alert.horn') or {}).get('program')
    horn = rec['caps']['horn']
    trigger = score.horn_sounded(rec['changes'], program if horn == 'planned' else None,
                                 horn in ('recorded', 'partial'))[0]
    if run:
        body['diagnostics'].update(diagnostics.diagnose(run, model, body, task_events,
                                                        rec['statuses'], trigger))
        body['penalties']['contacts'] = body['diagnostics']['contacts']['counts']
    body['reliability'] = reliability.run_flags(body, rec['authority_known'],
                                                rec['mission_stop'], rec['mission_stop_why'])
    record = dict(summary=score.summary(body),
                  scorer=dict(version=score.VERSION, rules=score.RULES,
                              handbook_sha256=(truth.get('handbook') or {}).get('sha256')),
                  **body)
    stamp = (now or datetime.now().astimezone()).strftime('%Y%m%dT%H%M%S')
    name = f"score.{rec['run_id']}.{stamp}.json" if rec['run_id'] else f'score.{stamp}.json'
    target = rec['session'] / 'analysis' / name
    data.write_json(target, record, score.SCHEMA)
    return target, record


def metrics(record):
    """The per-run numbers a batch aggregates."""
    out = dict(points=record['total']['points'], provisional=record['total']['provisional'],
               strikes=len(record['penalties']['strikes']),
               entered=float(record['run']['entry_t'] is not None),
               duration_s=record['run']['duration_s'])
    out.update({f'task.{k}': v for k, v in record['total']['by_task'].items()})
    diag = record.get('diagnostics', {})
    out.update({k: v for k, v in diag.get('motion', {}).items() if isinstance(v, (int, float))})
    touch = diag.get('contacts') or {}
    for kind, n in (touch.get('counts') or {}).items():
        out[f'contacts.{kind}'] = n
    end = record['run']['end']['reason']
    out['exited'] = float(record['run'].get('exit_t') is not None)
    rel = record.get('reliability')
    clean = rel['clean'] if rel and reliability.known(rel) else None
    if clean is not None:                    # every category known (v7)
        out['clean_run'] = float(clean)
        if clean:                            # time to the exit, clean runs only (conditional)
            out['clean_time_s'] = record['run']['duration_s']
    ends = ('time_limit', 'intervention', 'out_of_bounds', 'track_end')
    out.update({f'end.{k}': float(k == end) for k in ends})
    if touch:
        out['contact_free'] = float(not touch.get('contacts'))
        out['safe_exit'] = float(out['exited'] and not touch.get('contacts')
                                 and not record['penalties']['out_of_bounds']
                                 and not record['penalties']['interventions'])
        out['near_misses'] = touch.get('near_miss_count')
        out['min_clearance_m'] = touch.get('min_clearance_m')
    for tid, row in (diag.get('timeline') or {}).items():
        out[f'completed.{tid}'] = float(row.get('outcome') == 'completed')
        if row.get('outcome') == 'completed':        # success-conditional: n differs per task
            out[f'time_s.{tid}'] = round(row['end_t'] - row['encounter_t'], 2)
        if row.get('encounter_t') is not None:      # every encounter; incomplete at the cutoff
            end_t = row['end_t'] if row.get('outcome') == 'completed' else \
                record['run']['scored_until']
            out[f'time_cut_s.{tid}'] = round(end_t - row['encounter_t'], 2)
    return out


BINARY = ('entered', 'exited', 'contact_free', 'safe_exit', 'end.', 'completed.', 'clean_run')
LABELS = {
    'time_s.*': 'completed runs only (survivorship): read beside completed.* (completion rate)',
    'time_cut_s.*': 'every run that encountered the task; an incomplete run counts at its '
                    'cutoff, so its mean is a lower bound',
    'binary': 'rates (entered, exited, contact_free, safe_exit, end.*, completed.*): Wilson '
              'interval; differences: Newcombe hybrid score interval',
    'clean_time_s': 'scored time (start to exit) of clean runs only: compared only after the '
                    'failure rate is non-inferior (reliability.decision)',
    'clear': 'interval excludes 0, unadjusted; clear_holm: Holm-adjusted p < 0.05 over every '
             'metric compared (one family)'}


def _betacf(a, b, x):
    """Continued fraction of the regularized incomplete beta (Numerical Recipes 6.4)."""
    c, d = 1.0, 1.0 - (a + b) * x / (a + 1)
    d = 1 / (d if abs(d) > 1e-300 else 1e-300)
    h = d
    for m in range(1, 300):
        for num in (m * (b - m) * x / ((a + 2 * m - 1) * (a + 2 * m)),
                    -(a + m) * (a + b + m) * x / ((a + 2 * m) * (a + 2 * m + 1))):
            d = 1 + num * d
            d = 1 / (d if abs(d) > 1e-300 else 1e-300)
            c = 1 + num / c
            c = c if abs(c) > 1e-300 else 1e-300
            h *= d * c
        if abs(d * c - 1) < 1e-12:
            break
    return h


def t_sf(t, df):
    """Two-sided p of Student's t with ``df`` (real) degrees of freedom."""
    x = df / (df + t * t)
    lb = math.lgamma(df / 2 + 0.5) - math.lgamma(df / 2) - math.lgamma(0.5)
    front = math.exp(lb + (df / 2) * math.log(x) + 0.5 * math.log1p(-x)) if 0 < x < 1 else 0.0
    if x <= 0:
        return 0.0
    if x < (df / 2 + 1) / (df / 2 + 1.5):
        return front * _betacf(df / 2, 0.5, x) / (df / 2)
    return 1 - front * _betacf(0.5, df / 2, 1 - x) / 0.5


def t_quantile(df, p=0.975):
    """t with P(T <= t) = p, by bisection on the exact distribution (df may be fractional)."""
    if df <= 0:
        return math.inf
    lo, hi = 0.0, 1e3
    for _ in range(200):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if t_sf(mid, df) / 2 > 1 - p else (lo, mid)
    return (lo + hi) / 2


def _norm_p(z):
    return math.erfc(abs(z) / math.sqrt(2))


wilson = reliability.wilson


def stats(values, binary=False):
    """n, mean, sd and a 95% interval: Wilson for a rate, else exact-t for the mean."""
    n = len(values)
    mean = sum(values) / n
    sd = math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1)) if n > 1 else 0.0
    out = dict(n=n, mean=round(mean, 3), sd=round(sd, 3))
    if binary:
        k = round(sum(values))
        lo, hi = wilson(k, n)
        return dict(out, k=k, ci95=[round(lo, 3), round(hi, 3)], interval='wilson')
    half = t_quantile(n - 1) * sd / math.sqrt(n) if n > 1 else None
    out['ci95'] = None if half is None else [round(mean - half, 3), round(mean + half, 3)]
    out['interval'] = 't'
    if n > 1 and sd == 0:
        out['note'] = 'sd=0: no spread observed; the interval is not a precision claim'
    return out


def batch(records, points_at_least=None, unjudged=(), excluded=(), procedure='mission'):
    """Aggregate per metric across runs; sources and course seeds are listed; reliability
    (failure rate, categories, finals view) when the reports carry it. ``unjudged``: run
    records without files (``reliability.unjudged``), counted in reliability only.
    ``excluded``: run records of another procedure than the batch's ({run_id, run_key}),
    listed by key with their count and counted nowhere else."""
    rows = [metrics(r) for r in records]
    keys = sorted({k for r in rows for k in r})
    out = dict(runs=len(records),
               sources=sorted({r['run'].get('source', 'unity') for r in records}),
               seeds=sorted({str(r['run'].get('course', {}).get('seed')) for r in records}),
               metrics={k: stats([r[k] for r in rows if r.get(k) is not None],
                                 binary=k.startswith(BINARY)) for k in keys
                        if any(r.get(k) is not None for r in rows)}, labels=LABELS)
    out['time_by_task'] = {k[len('completed.'):]: dict(
        completion=out['metrics'][k],
        time_s_completed_only=out['metrics'].get('time_s.' + k[len('completed.'):]),
        time_cut_s_incl_incomplete=out['metrics'].get('time_cut_s.' + k[len('completed.'):]))
        for k in out['metrics'] if k.startswith('completed.')}
    if all('reliability' in r for r in records):
        out['reliability'] = reliability.batch(records, points_at_least, unjudged)
    if unjudged:
        out['runs_unjudged'] = len(unjudged)
    out['procedure'] = procedure
    if excluded:
        keys = [e['run_key'] for e in excluded]
        out['excluded'] = [dict(run_key=k, count=keys.count(k))
                           for k in sorted(set(keys), key=str)]
    return out


def decide(a, b, b_minus_a, margin=None):
    """Unpaired reliability-first decision: batch ``a`` is the baseline, ``b`` the candidate."""
    if 'reliability' not in a or 'reliability' not in b:
        return None
    ni = reliability.unpaired_ni(a['reliability'], b['reliability'], margin)
    return reliability.decision(ni, b_minus_a.get('clean_time_s'), b_minus_a.get('points'))


def mixing(a, b):
    """Why two batches are not like for like (empty when they are)."""
    why = [f'batch {name} mixes sources {x["sources"]}' for name, x in (('a', a), ('b', b))
           if len(x.get('sources', [])) > 1]
    if set(a.get('sources', [])) != set(b.get('sources', [])):
        why.append(f'sources differ: {a.get("sources")} vs {b.get("sources")}')
    if set(a.get('seeds', [])) != set(b.get('seeds', [])):
        why.append(f'seed sets differ: {a.get("seeds")} vs {b.get("seeds")}')
    return why


def _holm(out):
    tests = sorted((v['p'], k) for k, v in out.items() if v.get('p') is not None)
    m, running = len(tests), 0.0
    for i, (p, k) in enumerate(tests):
        running = max(running, min(1.0, (m - i) * p))
        out[k]['p_holm'] = round(running, 4)
        out[k]['clear_holm'] = running < 0.05
    for v in out.values():
        v.setdefault('clear_holm', False)


def compare(a, b, force=False):
    """Batch b minus batch a per metric. Rates: Newcombe hybrid score interval and a pooled
    two-proportion z p; other metrics: Welch with the exact t quantile. Refuses batches that
    mix sources or differ in seed set unless ``force`` (the reasons are then kept in
    ``_forced``)."""
    why = mixing(a, b)
    if why and not force:
        raise Refused('not like for like: ' + '; '.join(why) + ' (--force to compare anyway)')
    out = {}
    for k in sorted(set(a['metrics']) & set(b['metrics'])):
        x, y = a['metrics'][k], b['metrics'][k]
        diff = y['mean'] - x['mean']
        ci = p = None
        if x.get('interval') == 'wilson' and y.get('interval') == 'wilson':
            l1, u1 = wilson(x['k'], x['n'])
            l2, u2 = wilson(y['k'], y['n'])
            p1, p2 = x['k'] / x['n'], y['k'] / y['n']
            diff = p2 - p1
            ci = [round(diff - math.hypot(p2 - l2, u1 - p1), 3),
                  round(diff + math.hypot(u2 - p2, p1 - l1), 3)]
            pool = (x['k'] + y['k']) / (x['n'] + y['n'])
            se = math.sqrt(pool * (1 - pool) * (1 / x['n'] + 1 / y['n']))
            p = _norm_p(diff / se) if se > 0 else None
        else:
            va, vb = x['sd'] ** 2 / x['n'], y['sd'] ** 2 / y['n']
            var = va + vb
            if var > 0 and x['n'] > 1 and y['n'] > 1:
                df = var ** 2 / (va ** 2 / (x['n'] - 1) + vb ** 2 / (y['n'] - 1))
                half = t_quantile(df) * math.sqrt(var)
                ci = [round(diff - half, 3), round(diff + half, 3)]
                p = t_sf(abs(diff) / math.sqrt(var), df)
        out[k] = dict(diff=round(diff, 3), ci95=ci, p=None if p is None else round(p, 4),
                      clear=bool(ci) and (ci[0] > 0 or ci[1] < 0))
        if ci is None:
            out[k]['note'] = 'no interval: sd=0 in both batches or n < 2'
    _holm(out)
    if why:
        out['_forced'] = why
    return out


LOOPS = {('kinematic', 'unity'): 'loop 1: fast tier <-> Unity',
         ('unity', 'real'): 'loop 2: Unity <-> real boat'}   # pairing labels, never scoring


def _binom_two_sided(k, n):
    """Exact McNemar: two-sided binomial p of k of n discordant pairs at 1/2."""
    tail = sum(math.comb(n, i) for i in range(min(k, n - k) + 1)) / 2 ** n
    return min(1.0, 2 * tail)


def _pairs(pairs_file):
    """Rows {a_session, b_session, seed, order} (``a``/``b`` also accepted), relative paths
    resolved against the pair file's folder."""
    base = Path(pairs_file).parent
    out = []
    for row in data.read_jsonl(Path(pairs_file)):
        a, b = row.get('a_session') or row.get('a'), row.get('b_session') or row.get('b')
        if not a or not b:
            raise Refused(f'{pairs_file}: a pair row without a_session/b_session: {row}')
        out.append(dict(row, a_session=str(base / a), b_session=str(base / b)))
    if not out:
        raise Refused(f'{pairs_file}: no pairs')
    return out


def decide_pairs(recs, b_minus_a, margin=None):
    """Paired reliability-first decision (side a the baseline): pairs whose cleanliness is
    known on both sides; time from pairs clean on both sides (``clean_time_s``)."""
    if not all('reliability' in r for pair in recs for r in pair):
        return None
    known = [(not a['reliability']['clean'], not b['reliability']['clean']) for a, b in recs
             if reliability.known(a['reliability']) and reliability.known(b['reliability'])]
    ni = reliability.paired_ni(known, margin, unknown=len(recs) - len(known))
    ni['pairs_unknown'] = len(recs) - len(known)
    return reliability.decision(ni, b_minus_a.get('clean_time_s'), b_minus_a.get('points'))


def paired(pairs_file, force=False, margin=None):
    """B - A per explicit pair (``mhseals.score_pairs/1``): the pair file says which run
    matches which (same seed and order in the lower and the higher tier), so tiers may differ;
    every pair must be the same source pair. Rates: discordant counts and exact McNemar p;
    other metrics: paired mean difference with an exact-t interval. Capability differences
    between the sides are listed, since they bound what a difference can mean."""
    rows = _pairs(pairs_file)
    recs = [(analyze(r['a_session'])[1], analyze(r['b_session'])[1]) for r in rows]
    kinds = sorted({(a['run']['source'], b['run']['source']) for a, b in recs})
    why = []
    if len(kinds) > 1:
        why.append(f'pairs mix source pairs {kinds}')
    kind = kinds[0]
    loop = 'same source' if kind[0] == kind[1] else LOOPS.get(kind) or LOOPS.get(kind[::-1])
    if loop is None:
        why.append(f'{kind[0]} -> {kind[1]} is not a loop of the double loop')
    if why and not force:
        raise Refused('not comparable: ' + '; '.join(why) + ' (--force to compare anyway)')
    caps = [{k: sorted({str(rec['run']['capabilities'][k]) for rec in side})
             for k in side[0]['run']['capabilities']} for side in zip(*recs)]
    gaps = {k: dict(a=caps[0][k], b=caps[1][k]) for k in caps[0] if caps[0][k] != caps[1][k]}
    per = [(metrics(a), metrics(b)) for a, b in recs]
    out = {}
    for k in sorted({k for ma, mb in per for k in ma} & {k for ma, mb in per for k in mb}):
        both = [(ma[k], mb[k]) for ma, mb in per
                if ma.get(k) is not None and mb.get(k) is not None]
        if not both:
            continue
        diffs = [y - x for x, y in both]
        n = len(diffs)
        mean = sum(diffs) / n
        if k.startswith(BINARY):
            b_only = sum(1 for x, y in both if y > x)
            a_only = sum(1 for x, y in both if x > y)
            p = _binom_two_sided(b_only, b_only + a_only) if b_only + a_only else None
            out[k] = dict(n=n, diff=round(mean, 3), b_only=b_only, a_only=a_only,
                          p=None if p is None else round(p, 4), test='exact McNemar')
            continue
        sd = math.sqrt(sum((d - mean) ** 2 for d in diffs) / (n - 1)) if n > 1 else 0.0
        half = t_quantile(n - 1) * sd / math.sqrt(n) if n > 1 and sd > 0 else None
        p = t_sf(abs(mean) / (sd / math.sqrt(n)), n - 1) if half else None
        out[k] = dict(n=n, diff=round(mean, 3), sd=round(sd, 3), test='paired t',
                      ci95=None if half is None else [round(mean - half, 3),
                                                      round(mean + half, 3)],
                      p=None if p is None else round(p, 4))
        if half is None:
            out[k]['note'] = 'no interval: n < 2 or identical differences'
    for v in out.values():
        v['clear'] = v.get('p') is not None and v['p'] < 0.05
    _holm(out)
    decision = decide_pairs(recs, out, margin)
    result = dict(schema='mhseals.score_pairs/1', pairs_file=str(pairs_file), loop=loop,
                  sources=dict(a=kind[0], b=kind[1]), n_pairs=len(rows),
                  pairs=[{k: r.get(k) for k in ('a_session', 'b_session', 'seed', 'order')}
                         for r in rows],
                  capability_differences=gaps, b_minus_a=out)
    if decision:
        result.update(verdict=decision['line'], decision=decision)
    if why:
        result['forced'] = why
    return result


def _records(paths, procedure='mission'):
    """(scored, unjudged, excluded): every run of ``procedure`` under ``paths``; the records of
    other procedures are ``excluded`` ({run_id, run_key, session}). Each run record of
    a session: one that
    links its files is scored (its session-level track is a copy of the latest run, so it is
    not counted again); one without (aborted, cancelled, errored, refused; or, dc042f84, a
    started run whose partial links hold nothing to score) is classified from its journal and
    record (``reliability.unjudged``). A session without run records is
    scored as a whole."""
    out, unjudged, excluded, seen = [], [], [], set()
    for root in paths:
        sessions = sorted({p.parent for p in Path(root).glob('**/track.jsonl')}
                          | {p.parent.parent for p in Path(root).glob('**/runs/*.json')})
        for session in sessions:
            own = session.parent / f'{session.name}.json'
            if session.parent.name == 'runs' and own.exists() or session in seen:
                continue                    # a run's own copy: scored through its record
            seen.add(session)
            others = []
            runs = run_record.all_runs(session, procedure, others)
            excluded += [dict(e, session=session.name) for e in others]
            for r in runs:
                end = run_record.record_end(r)
                if run_record.has_files(r):
                    try:
                        out.append(analyze(session / 'runs' / f"{r['run_id']}.json")[1])
                        continue
                    except Refused:     # a partial link with nothing to score: the end decides
                        if not end or end['outcome'] == 'complete':
                            raise
                u = reliability.unjudged(r, *run_record.harness_end(session, r))
                unjudged.append(dict(u, session=session.name))
            if not runs and not others and (session / 'track.jsonl').exists():
                out.append(analyze(session)[1])
    return out, unjudged, excluded


def main(argv=None, out=None):
    parser = argparse.ArgumentParser(prog='boat_trial analyze score')
    parser.add_argument('session', help='a session, or with --batch a directory of sessions')
    parser.add_argument('--json', action='store_true', help='print the record that was written')
    parser.add_argument('--batch', action='store_true', help='score every session under it')
    parser.add_argument('--compare', metavar='DIR', help='with --batch: a second batch (B - A)')
    parser.add_argument('--procedure', default='mission', metavar='KEY',
                        help="with --batch: the procedure key whose runs count (default mission); "
                             'runs of other keys are listed as excluded')
    parser.add_argument('--paired', action='store_true',
                        help='SESSION is a pair file (jsonl rows {a_session, b_session, seed, '
                             'order}): B - A per pair, across tiers (fast<->Unity, Unity<->real)')
    parser.add_argument('--force', action='store_true',
                        help='compare batches that mix sources or differ in seed set')
    parser.add_argument('--failure-margin', type=float, default=None,
                        help='non-inferiority margin on the failure rate (default '
                             f"{reliability.RPARAMS['failure_margin']}, owner to confirm)")
    parser.add_argument('--points-at-least', type=float, default=None,
                        help='X for the finals view P(points >= X) (default '
                             f"{reliability.RPARAMS['points_at_least']})")
    args = parser.parse_args(argv)
    if args.paired:
        try:
            result = paired(args.session, args.force, args.failure_margin)
        except Refused as error:
            print(f'refused: {error}', file=sys.stderr)
            return 2
        if result.get('verdict'):
            print(result['verdict'], file=sys.stderr)
        if result.get('forced'):
            print('FORCED pairing: ' + '; '.join(result['forced']), file=sys.stderr)
        print(json.dumps(result, indent=2), file=out or sys.stdout)
        return 0
    if args.batch:
        try:
            scored, unjudged, excluded = _records([args.session], args.procedure)
            first = batch(scored, args.points_at_least, unjudged, excluded, args.procedure)
            result = dict(schema='mhseals.score_batch/1', a=first)
            if args.compare:
                scored, unjudged, excluded = _records([args.compare], args.procedure)
                second = batch(scored, args.points_at_least, unjudged, excluded, args.procedure)
                result.update(b=second, b_minus_a=compare(first, second, args.force))
                decision = decide(first, second, result['b_minus_a'], args.failure_margin)
                if decision:
                    result.update(verdict=decision['line'], decision=decision)
                    print(decision['line'], file=sys.stderr)
                if args.force and mixing(first, second):
                    print('FORCED comparison, not like for like: ' +
                          '; '.join(mixing(first, second)), file=sys.stderr)
                    result['forced'] = mixing(first, second)
        except (Refused, ZeroDivisionError) as error:
            print(f'refused: {error or "no sessions"}', file=sys.stderr)
            return 2
        print(json.dumps(result, indent=2), file=out or sys.stdout)
        return 0
    try:
        target, _ = analyze(args.session)
    except Refused as error:
        print(f'refused: {error}', file=sys.stderr)
        return 2
    record = json.loads(target.read_text())
    print(json.dumps(record, indent=2) if args.json else record['summary'], file=out or sys.stdout)
    return 0

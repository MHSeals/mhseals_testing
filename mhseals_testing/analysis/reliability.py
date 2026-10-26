"""Reliability of a run and of a batch: decision support, never points (owner, 2026-10-26).

Finals give one autonomous attempt, so failures come first and time second (PLAYBOOK §6b):
a faster design wins only if its failure rate is not worse by more than ``failure_margin``.

Per run, from the score report (ground truth and the harness's records, never the boat's
internals), each failure category is ``failed`` True, False, or None (cannot be observed in
this run, with ``why``):

- ``did_not_finish``: no exit through the entry gates before the cut (time limit, track end,
  intervention or out of bounds). Task-mode reports (no full run) are None.
- ``hold`` / ``takeover``: an intervention in the scored run (hold; rc, keyboard, external or
  disarm). None when the authority journal is not aligned to the track.
- ``contact``: a scored strike. None when strikes are unscorable (a real run without a sheet).
- ``false_claim``: the boat claimed a task ground truth does not credit. None without claims.
- ``mission_failure``: the harness's ``mission_stop`` outcome is not ``complete`` (abort or
  error). None without a journaled ``mission_stop``.

``clean`` is True when every category is False, False when any is True, else None (unknown).
A run is *known* only when all six categories are known (v7): a failed run with an unknown
category is left out of the rate just like a clean-looking one, so poor journaling cannot
select failures into the rate. A batch counts the rest apart, with the best and worst case
they could move the rate to.

A harness run record without files (v7) is still a run of the batch (``unjudged``): one that
started (``mission_start``) and ended aborted, cancelled or errored is a failure with its cause
(``terminal``: it never reached the point where the other categories are measured, so it is
known); one the harness refused before ``mission_start`` is "not started (harness refusal)",
listed with its count and kept out of the rate.
"""

import math

RPARAMS = dict(
    failure_margin=0.05,    # owner to confirm (10-26): non-inferiority margin on the failure
                            # rate, absolute (candidate may fail at most this much more often)
    alpha=0.05,             # one-sided level of the non-inferiority test (90% two-sided interval)
    points_at_least=500,    # D, owner to set: X in the finals view P(points >= X)
)
FAILED_VERDICTS = {'ABORTED', 'CANCELED', 'CANCELLED', 'ERROR', 'ERRORED', 'FAIL', 'FAILED',
                   'TIMEOUT'}
CATEGORIES = ('did_not_finish', 'hold', 'takeover', 'contact', 'false_claim', 'mission_failure')
Z95 = 1.959964


def _z(alpha):
    """Upper-tail standard normal quantile (bisection on erfc)."""
    lo, hi = 0.0, 10.0
    for _ in range(100):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if math.erfc(mid / math.sqrt(2)) / 2 > alpha else (lo, mid)
    return (lo + hi) / 2


def wilson(k, n, z=Z95):
    """Wilson score interval for k successes in n."""
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return max(0.0, centre - half), min(1.0, centre + half)


def rate(k, n, z=Z95):
    """{k, n, rate, ci95 (Wilson)} or None when n is 0."""
    if not n:
        return None
    lo, hi = wilson(k, n, z)
    return dict(k=k, n=n, rate=round(k / n, 4), ci95=[round(lo, 4), round(hi, 4)],
                interval='wilson')


# ---------------------------------------------------------------- per run

def _flag(failed, causes=(), why=None):
    out = dict(failed=failed, causes=list(causes))
    if why:
        out['why'] = why
    return out


def run_flags(report, authority_known=True, mission_stop=None, mission_stop_why=None):
    """Per-category failure flags with causes and ``clean`` for one score report."""
    run, pen = report['run'], report['penalties']
    f = {}
    if run.get('mode') == 'task':
        f['did_not_finish'] = _flag(None, why='task mode: not a full run')
    elif run.get('exit_t') is None:
        f['did_not_finish'] = _flag(True, [f"no exit through the entry gates; ended "
                                           f"{run['end']['reason']} at t={run['end']['t']}"])
    else:
        f['did_not_finish'] = _flag(False)
    if authority_known:
        holds = [i for i in pen['interventions'] if i.get('by') == 'hold']
        takes = [i for i in pen['interventions'] if i.get('by') != 'hold']
        for name, got in (('hold', holds), ('takeover', takes)):
            f[name] = _flag(bool(got), [f"{i.get('by')} at t={i['t']}"
                                        + (f" ({i['cause']})" if i.get('cause') else '')
                                        for i in got])
    else:
        why = 'authority journal not aligned to the track (no mission_start)'
        f['hold'], f['takeover'] = _flag(None, why=why), _flag(None, why=why)
    if pen.get('strikes_status') == 'unscorable':
        f['contact'] = _flag(None, why='strikes unscorable (no judge sheet)')
    else:
        f['contact'] = _flag(bool(pen['strikes']), [f"{s.get('object')} at t={s['t']}"
                                                    for s in pen['strikes']])
    claims = report.get('claims') or {}
    if claims.get('unavailable') or 'by_stage' not in claims:
        f['false_claim'] = _flag(None, why=claims.get('unavailable') or 'no claims judged')
    else:
        false = [f"{stage} (task {c['task']})" for stage, c in claims['by_stage'].items()
                 if c.get('false_claim')]
        f['false_claim'] = _flag(bool(false), false)
    if mission_stop is None:
        f['mission_failure'] = _flag(None, why=mission_stop_why or 'no mission_stop recorded')
    else:
        bad = mission_stop.get('outcome') != 'complete'
        f['mission_failure'] = _flag(bad, [
            f"outcome {mission_stop.get('outcome')!r} by {mission_stop.get('by')}"
            + (f" ({mission_stop['cause']})" if mission_stop.get('cause') else '')] if bad
            else [])
    failed = [c for c in CATEGORIES if f[c]['failed']]
    unknown = [c for c in CATEGORIES if f[c]['failed'] is None]
    clean = False if failed else (None if unknown else True)
    return dict(clean=clean, failed=failed, unknown=unknown, flags=f)


def known(rel):
    """Counted in the failure rate: every category known, or a terminal harness failure."""
    return bool(rel.get('terminal')) or (rel['clean'] is not None and not rel['unknown'])


def unjudged(run, started, mission_stop=None):
    """Reliability of a harness run record that links no files (no track to score).

    ``started``: the journal has this run's ``mission_start`` (or the record a run start).
    Returns run_flags' shape plus ``outcome``: ``failure`` (started, then aborted, cancelled or
    errored), ``not_started`` (refused by the harness before the start) or ``unknown``."""
    result = run.get('result') or {}
    verdict = str(result.get('verdict') or '').upper()
    status = str(run.get('status') or '').upper()
    why = (result.get('reason') or result.get('error') or
           '; '.join(map(str, result.get('reasons') or [])) or None)
    base = dict(run_id=run.get('run_id'), verdict=verdict or None, status=status or None)
    if not started:
        return dict(base, outcome='not_started', clean=None, failed=[], unknown=list(CATEGORIES),
                    flags={}, cause='not started (harness refusal)'
                    + (f': {why}' if why else f' (verdict {verdict or "none"})'))
    if mission_stop is not None and mission_stop.get('outcome') == 'complete':
        bad = None
    elif mission_stop is not None:
        bad = f"outcome {mission_stop.get('outcome')!r} by {mission_stop.get('by')}" + (
            f" ({mission_stop['cause']})" if mission_stop.get('cause') else '')
    elif verdict in FAILED_VERDICTS or status == 'ABORTED':
        bad = f'run record verdict {verdict or "none"}, status {status or "none"}' + (
            f' ({why})' if why else '')
    else:
        bad = None
    if bad is None:
        na = 'run record links no files (no track): cannot be judged'
        return dict(base, outcome='unknown', clean=None, failed=[], unknown=list(CATEGORIES),
                    flags={c: _flag(None, why=na) for c in CATEGORIES}, cause=na)
    na = 'run ended before its files were written'
    flags = {c: _flag(None, why=na) for c in CATEGORIES}
    flags['mission_failure'] = _flag(True, [bad])
    flags['did_not_finish'] = _flag(True, [f'ended {bad} before the exit'])
    return dict(base, outcome='failure', terminal=True, clean=False,
                failed=['did_not_finish', 'mission_failure'],
                unknown=[c for c in CATEGORIES if flags[c]['failed'] is None], flags=flags,
                cause=bad)


# ---------------------------------------------------------------- batch

def batch(records, points_at_least=None, unjudged_runs=()):
    """Failure rate (Wilson) over known runs (``known``), per-category rates, the best and worst
    case with the excluded runs, harness refusals listed apart, and the finals view: P(clean
    run) and P(points >= X) for one attempt (``X`` from RPARAMS). ``unjudged_runs``: the
    ``unjudged`` entries of run records without files."""
    x = RPARAMS['points_at_least'] if points_at_least is None else points_at_least
    refused = [u for u in unjudged_runs if u['outcome'] == 'not_started']
    rels = [r['reliability'] for r in records] + [u for u in unjudged_runs
                                                  if u['outcome'] != 'not_started']
    ok = [r for r in rels if known(r)]
    out_ = [r for r in rels if not known(r)]
    k = sum(1 for r in ok if not r['clean'])
    sure = sum(1 for r in out_ if r['clean'] is False)     # failed, another category unknown
    n, u = len(rels), len(out_)
    cats = {}
    for c in CATEGORIES:
        seen = [r['flags'][c]['failed'] for r in rels
                if (r['flags'].get(c) or {}).get('failed') is not None]
        cats[c] = rate(sum(seen), len(seen)) or dict(n=0, note='never observable')
    pts = [r['total']['points'] for r in records]
    out = dict(runs=n, known=len(ok), unknown=u, failures=k,
               failure_rate=rate(k, len(ok)),
               unknown_runs=dict(n=u, failed_with_unknown=sure, clean_with_unknown=u - sure,
                                 best_case=None if not n else rate(k + sure, n),
                                 worst_case=None if not n else rate(k + u, n)),
               failure_rate_bounds_with_unknown=None if not n else [round((k + sure) / n, 4),
                                                                    round((k + u) / n, 4)],
               unjudged=dict(failures=[dict(run_id=r['run_id'], cause=r['cause'])
                                       for r in unjudged_runs if r['outcome'] == 'failure'],
                             unknown=[dict(run_id=r['run_id'], cause=r['cause'])
                                      for r in unjudged_runs if r['outcome'] == 'unknown']),
               not_started=dict(label='not started (harness refusal): not in the failure rate',
                                n=len(refused), runs=[dict(run_id=r['run_id'], cause=r['cause'])
                                                      for r in refused]),
               by_category=cats,
               single_attempt=dict(
                   p_clean=rate(len(ok) - k, len(ok)),
                   points_at_least=x,
                   p_points_at_least=rate(sum(1 for p in pts if p >= x), len(pts))),
               over_timeslot=sum(1 for r in records
                                 if (r['run'].get('timeslot') or {}).get('over')),
               labels=dict(
                   failure_rate='runs with any failure flag / runs whose six flags are all known '
                                '(or that ended aborted, cancelled or errored before files)',
                   bounds='best case: excluded runs clean unless already failed; worst case: '
                          'every excluded run failed',
                   p_points_at_least='earned points only (provisional excluded), scored runs'))
    notes = []
    if u:
        notes.append(f'{u} run(s) with an unknown category are left out of the rate '
                     f'({sure} of them already failed)')
    if refused:
        notes.append(f'{len(refused)} run(s) not started (harness refusal)')
    if notes:
        out['note'] = '; '.join(notes)
    return out


# ---------------------------------------------------------------- comparison

def newcombe(k1, n1, k2, n2, z=Z95):
    """p2 - p1 and its Newcombe hybrid score interval (method 10, unpaired)."""
    l1, u1 = wilson(k1, n1, z)
    l2, u2 = wilson(k2, n2, z)
    p1, p2 = k1 / n1, k2 / n2
    d = p2 - p1
    return d, (d - math.hypot(p2 - l2, u1 - p1), d + math.hypot(u2 - p2, p1 - l1))


def newcombe_paired(both, cand_only, base_only, neither, z=Z95):
    """p_candidate - p_baseline for paired 0/1 outcomes and Newcombe's paired score interval
    (1998, method 10, with his continuity-corrected phi)."""
    a, b, c, d = both, cand_only, base_only, neither
    n = a + b + c + d
    p1, p2 = (a + b) / n, (a + c) / n
    l1, u1 = wilson(a + b, n, z)
    l2, u2 = wilson(a + c, n, z)
    margins = (a + b) * (c + d) * (a + c) * (b + d)
    if not margins:
        phi = 0.0
    else:
        num = a * d - b * c
        phi = (max(num - n / 2, 0.0) if num > 0 else num) / math.sqrt(margins)
    theta = p1 - p2
    lo = theta - math.sqrt(max(0.0, (p1 - l1) ** 2 - 2 * phi * (p1 - l1) * (u2 - p2)
                               + (u2 - p2) ** 2))
    hi = theta + math.sqrt(max(0.0, (u1 - p1) ** 2 - 2 * phi * (u1 - p1) * (p2 - l2)
                               + (p2 - l2) ** 2))
    return theta, (lo, hi)


def non_inferiority(diff, interval, margin=None, alpha=None, method='newcombe', **counts):
    """Candidate minus baseline failure rate: non-inferior when the one-sided upper bound
    (``alpha``) is below ``margin``."""
    margin = RPARAMS['failure_margin'] if margin is None else margin
    alpha = RPARAMS['alpha'] if alpha is None else alpha
    upper = interval[1]
    return dict(test=f'one-sided non-inferiority on failure rate, {method} interval',
                margin=margin, margin_status='owner to confirm', alpha_one_sided=alpha,
                diff=round(diff, 4), upper=round(upper, 4),
                interval=[round(interval[0], 4), round(upper, 4)],
                non_inferior=upper < margin, **counts)


def unpaired_ni(a, b, margin=None, alpha=None):
    """Batch reliability ``a`` (baseline) vs ``b`` (candidate)."""
    alpha = RPARAMS['alpha'] if alpha is None else alpha
    fa, fb = a['failure_rate'], b['failure_rate']
    if not fa or not fb:
        return dict(non_inferior=None, why='no run with every failure category known',
                    excluded=_excluded(a, b))
    diff, ci = newcombe(fa['k'], fa['n'], fb['k'], fb['n'], _z(alpha))
    return non_inferiority(diff, ci, margin, alpha, 'Newcombe hybrid score (unpaired)',
                           baseline=f"{fa['k']}/{fa['n']}", candidate=f"{fb['k']}/{fb['n']}",
                           excluded=_excluded(a, b))


def _excluded(a, b):
    """Runs left out of each side's rate, and refusals, for the verdict line."""
    return {side: dict(unknown=x.get('unknown', 0), runs=x.get('runs', 0),
                       not_started=(x.get('not_started') or {}).get('n', 0))
            for side, x in (('baseline', a), ('candidate', b))}


def paired_ni(pairs, margin=None, alpha=None, unknown=0):
    """``pairs`` [(baseline failed, candidate failed)] with both sides known; ``unknown``
    pairs left out (a side with an unknown category)."""
    alpha = RPARAMS['alpha'] if alpha is None else alpha
    excluded = dict(pairs=dict(unknown=unknown, total=len(pairs) + unknown))
    if not pairs:
        return dict(non_inferior=None, why='no pair with every failure category known',
                    excluded=excluded)
    both = sum(1 for x, y in pairs if x and y)
    cand = sum(1 for x, y in pairs if y and not x)
    base = sum(1 for x, y in pairs if x and not y)
    diff, ci = newcombe_paired(both, cand, base, len(pairs) - both - cand - base, _z(alpha))
    return non_inferiority(diff, ci, margin, alpha, 'Newcombe paired score (method 10)',
                           n_pairs=len(pairs), both_failed=both, candidate_only=cand,
                           baseline_only=base, excluded=excluded)


def excluded_text(ex):
    """'; excluded ...' for the verdict line, or '' when nothing was left out."""
    if not ex:
        return ''
    if 'pairs' in ex:
        p = ex['pairs']
        return f"; excluded {p['unknown']}/{p['total']} pairs (a category unknown)" \
            if p['unknown'] else ''
    parts = [f"{side} {x['unknown']}/{x['runs']}" for side, x in ex.items() if x['unknown']]
    refused = [f"{side} {x['not_started']}" for side, x in ex.items() if x['not_started']]
    text = f"; excluded (a category unknown): {', '.join(parts)}" if parts else ''
    if refused:
        text += f"; not started (harness refusal): {', '.join(refused)}"
    return text


def verdict(ni, time, points):
    """One line from the fixed-sequence decision: failure non-inferiority first; only then
    time (clean runs) and points, each 'clear' when its 95% interval excludes 0."""
    if ni.get('non_inferior') is None:
        return 'no clear difference', 'no clear difference: failure rate not observable ' \
            f"({ni.get('why')})", None
    if not ni['non_inferior']:
        why = 'more failures' if ni['diff'] > 0 else \
            'not shown within the margin: too few runs for the interval'
        return 'not non-inferior', (
            f"not non-inferior ({why}): failure rate {ni['diff']:+.3f}, upper bound "
            f"{ni['upper']:.3f} >= margin {ni['margin']}; time and points not weighed"), None
    gains = dict(time=_gain(time, -1), points=_gain(points, +1))
    faster, worse_pts = gains['time']['better'], gains['points']['worse']
    head = 'better' if faster and not worse_pts else 'no clear difference'
    parts = [f"failures non-inferior (upper {ni['upper']:.3f} < {ni['margin']})"]
    for name, unit in (('time', 's'), ('points', 'pts')):
        g = gains[name]
        if g['diff'] is None:
            parts.append(f'{name} n/a')
        else:
            parts.append(f"{name} {g['diff']:+g} {unit} ({'clear' if g['clear'] else 'noise'})")
    if head == 'better':
        line = 'better (faster and non-inferior): ' + ', '.join(parts)
    else:
        line = 'no clear difference: ' + ', '.join(parts)
    return head, line, gains


def _gain(row, sign):
    if not row or row.get('diff') is None:
        return dict(diff=None, clear=False, better=False, worse=False)
    clear = bool(row.get('clear'))
    return dict(diff=row['diff'], ci95=row.get('ci95'), clear=clear,
                better=clear and sign * row['diff'] > 0, worse=clear and sign * row['diff'] < 0)


def decision(ni, time, points):
    """The decision block: test, gains (only after non-inferiority), verdict and line (with
    how many runs or pairs were excluded)."""
    head, line, gains = verdict(ni, time, points)
    line += excluded_text(ni.get('excluded'))
    return dict(verdict=head, line=line, non_inferiority=ni, gains=gains,
                procedure='fixed sequence: failure non-inferiority (one-sided) first; time '
                          '(clean runs only) and points only if it holds, each clear when its '
                          '95% interval excludes 0. Decision support, not a score.')

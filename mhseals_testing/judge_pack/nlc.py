"""The NLC study's pilot outputs as judge records (interim shim; the NLC program owns its pack).

Reads ``out/<pilot>/episodes.npz`` (per family/arm: pose [seeds, T, 3], t, arrival time) and
``summary.json`` (the NLC evaluator's per-episode numbers: min margin, violation seconds, time to
goal). The evaluator's numbers are used as they are (no second implementation of its semantics):
``nlc.min_margin`` carries its three-valued verdict, the judge adds generic motion metrics, the
paired comparison machinery and failure clustering.

Comparisons: within one disturbance level, arms are paired by scenario seed (exact design);
across levels (D1 vs D2) the same arm and seed index is a block pair (same scenario family and
seed, different disturbance draw), which measures each arm's robustness to the disturbance shift.
"""

import json
from pathlib import Path

import numpy as np

from mhseals_testing.judge_pack._core import core

core()

from judge import render  # noqa: E402
from judge.metric import Metric, MetricValue, Status, unscorable  # noqa: E402
from judge.pack import Pack  # noqa: E402
from judge.packs import generic_motion  # noqa: E402
from judge.record import Header, Robot, RunRecord, Scenario, Validity, se2_channel  # noqa: E402
from judge.run import judge_runs, judge_variants  # noqa: E402
from judge.task import Objective, TaskSpec  # noqa: E402

FIELDS = {'min_margin': 'nlc.min_margin', 'violation_s': 'nlc.violation_s', 'ttg': 'nlc.time_to_goal',
          'peak_excursion': 'nlc.peak_excursion'}


def records(out_dir):
    out_dir = Path(out_dir)
    z = np.load(out_dir / 'episodes.npz', allow_pickle=False)
    summ = json.loads((out_dir / 'summary.json').read_text())
    level = summ.get('level', '?')
    recs = []
    for key in z.files:
        fam, arm, what = (key.split('/') + ['', ''])[:3]
        if what != 'pose':
            continue
        pose, t = z[key], z[f'{fam}/{arm}/t']
        arrive = z[f'{fam}/{arm}/arrived']
        s = (summ.get(fam) or {}).get(arm) or {}
        for i in range(pose.shape[0]):
            t_end = float(arrive[i]) if np.isfinite(arrive[i]) and arrive[i] > 0 else float(t[-1])
            m = t <= t_end + 1e-9
            ch = se2_channel(t[m], pose[i, m, 0], pose[i, m, 1], pose[i, m, 2])
            vals = {name: (s.get(src) or [None] * (i + 1))[i] if isinstance(s.get(src), list) and len(s[src]) > i else None
                    for src, name in FIELDS.items()}
            h = Header(f'{level}/{fam}/{arm}/s{i}', summ.get('tier', 'U*'), 'nlc_pilot', 'nlc_vessel',
                       Robot('nlc-vessel', 'hidden-parameter fastsim'), fam,
                       Scenario(f'nlc-{fam}-s{i}', None, i, fam, {'level': level, 'arm': arm}),
                       {'summary': str(out_dir / 'summary.json'), 'loop': summ.get('loop')},
                       window=(float(t[0]), t_end, 'arrived' if t_end < t[-1] else 'time_limit'))
            recs.append(RunRecord(h, {'pose.se2': ch}, (), {'truth': 'exact'}, Validity(), {},
                                  {'nlc': vals, 'arrived': bool(t_end < t[-1] or s.get('arrived') == 1.0)}))
    return recs, summ


class FromEvaluator(Metric):
    def __init__(self, name, unit, direction):
        self.name, self.unit, self.direction = name, unit, direction

    def compute(self, rec, task):
        v = (rec.attachments.get('nlc') or {}).get(self.name)
        if v is None or not np.isfinite(v):
            return unscorable('not in the NLC summary (or not reached)')
        if self.name == 'nlc.min_margin':
            verdict = 'violated' if v < 0 else 'satisfied'
            return MetricValue(float(v), extra={'verdict': verdict, 'basis': 'nlc.monitor.evaluator'})
        return MetricValue(float(v))


class Violated(Metric):
    name, unit, direction = 'nlc.violated', '', 'lower'

    def compute(self, rec, task):
        v = (rec.attachments.get('nlc') or {}).get('nlc.min_margin')
        return unscorable('no margin') if v is None else MetricValue(float(v < 0))


class Arrived(Metric):
    name, unit, direction = 'nlc.arrived', '', 'higher'

    def compute(self, rec, task):
        return MetricValue(float(bool(rec.attachments.get('arrived'))))


def outcome(rec, values):
    if values.get('nlc.violated') and values['nlc.violated'].value == 1.0:
        return 'violation'
    if values.get('nlc.arrived') and values['nlc.arrived'].value == 0.0:
        return 'no_arrival'
    return 'success'


PACK = Pack('nlc_shim', 'nlc_vessel', {m.name: m for m in (
    FromEvaluator('nlc.min_margin', 'm', 'higher'), FromEvaluator('nlc.violation_s', 's', 'lower'),
    FromEvaluator('nlc.time_to_goal', 's', 'lower'), FromEvaluator('nlc.peak_excursion', 'm', 'lower'),
    Violated(), Arrived())}, outcome=outcome)

TASK = TaskSpec('nlc-pilot', '1', 'nlc_vessel', (
    Objective('nlc.violated', 'lower', 'safety', ni_margin=0.05, margin_cite='screening'),
    Objective('nlc.min_margin', 'higher', 'safety', band=0.25),
    Objective('nlc.arrived', 'higher', 'task', ni_margin=0.1, margin_cite='screening'),
    Objective('nlc.time_to_goal', 'lower', 'efficiency', ni_margin=5.0, band=10.0, margin_cite='screening'),
    Objective('motion.path_length', 'lower', 'diagnostic'),
))
PACKS = (PACK, generic_motion.PACK)


def _by(recs):
    out = {}
    for r in recs:
        c = r.header.scenario.condition
        out[(r.header.task_id, c['arm'], r.header.scenario.seed)] = r
    return out


def arm_pairs(recs, base, cand):
    by = _by(recs)
    return [((f, s), by[(f, base, s)], by[(f, cand, s)]) for (f, a, s) in sorted(by) if a == base and (f, cand, s) in by]


def judge_outputs(path):
    """``path``: the NLC ``out/`` directory (pilot, pilot-D1 inside) or one pilot directory."""
    path = Path(path)
    dirs = [path] if (path / 'episodes.npz').exists() else sorted(p.parent for p in path.glob('*/episodes.npz'))
    sections, levels = {}, {}
    for d in dirs:
        recs, summ = records(d)
        lv = summ.get('level', d.name)
        levels[lv] = recs
        runs = judge_runs(recs, TASK, PACKS)
        sections[f'{lv}:runs'] = runs
        fams = sorted({r.header.task_id for r in recs})
        for base, cand in (('E6', 'TIDE-K'), ('TIDE-0', 'TIDE-K'), ('E1', 'E6')):
            for fam in fams:        # per family: S3's margins are m/s, S1/S2's metres
                pr = [p for p in arm_pairs(recs, base, cand) if p[0][0] == fam]
                if pr:
                    rep = judge_variants(pr, TASK, PACKS, 'exact')
                    rep['arms'] = [base, cand, fam]
                    sections[f'{lv}:{fam}:{cand} vs {base}'] = rep
    if len(levels) == 2:
        (l1, r1), (l2, r2) = sorted(levels.items())
        b1, b2 = _by(r1), _by(r2)
        loops = {r.header.provenance.get('loop') for r in r1} | {r.header.provenance.get('loop') for r in r2}
        for arm in sorted({k[1] for k in b1}):
            for fam in sorted({k[0] for k in b1}):
                pr = [((f, s), b1[(f, a, s)], b2[(f, a, s)]) for (f, a, s) in sorted(b1)
                      if a == arm and f == fam and (f, a, s) in b2]
                if pr:
                    rep = judge_variants(pr, TASK, PACKS, 'block')
                    rep['shift'] = [l1, l2, arm, fam, sorted(x for x in loops if x)]
                    sections[f'{fam}:{arm}: {l2} vs {l1}'] = rep
    main = next((v for k, v in sections.items() if k.endswith('S1:TIDE-K vs E6')), next(iter(sections.values())))
    report = dict(main)
    report['sections'] = {k: {'oneline': render.oneline(v, k), 'report': v} for k, v in sections.items()}
    report['insights'] = _insights(sections) + list(main.get('insights', []))
    return report


def _insights(sections):
    """Cross-section findings: robustness to the disturbance shift per arm, and the headline pairs."""
    from judge.insight import Insight
    out = []
    rows = []
    for k, rep in sections.items():
        if 'shift' not in rep:
            continue
        c = {x['metric']: x for x in rep['comparisons']}
        v, m, t = c.get('nlc.violated'), c.get('nlc.min_margin'), c.get('nlc.time_to_goal')
        rows.append((rep['shift'][2], rep['shift'][3], v, m, t, rep['shift'][4]))
    loops_differ = any(len(r[5]) > 1 for r in rows)
    for arm, fam, v, m, t, loops in rows:
        if v and v['estimate'] is not None and (abs(v['estimate']) >= 0.1 or (m and m['verdict'] in ('better', 'worse'))):
            unit = 'm/s' if fam == 'S3' else 'm'
            out.append(Insight('nlc.shift', 'warn' if v['estimate'] > 0.1 else 'info',
                               f"{fam} {arm}: D1 -> D2 violation rate {v['estimate']:+.2f} [{v['interval'][0]:+.2f}, {v['interval'][1]:+.2f}]"
                               + (f", min margin {m['estimate']:+.2f} {unit} ({m['verdict']})" if m and m['estimate'] is not None else '')
                               + (f", time to goal {t['estimate']:+.1f} s" if t and t['estimate'] is not None else '')
                               + f" (block pairs n={v['n_units']})",
                               scope={'arm': arm, 'family': fam}, data={'violated': v, 'min_margin': m}, basis='statistical'))
    if loops_differ:
        out.append(Insight('nlc.confound', 'warn',
                           'the D1 and D2 pilots ran TIDE from different loop records '
                           f'({", ".join(rows[0][5])}): a D1 -> D2 change of a learned arm mixes the disturbance shift '
                           'with a different learned tube; compare fixed arms (E0/E1/E6) for robustness',
                           scope={}, data={'loops': rows[0][5]}))
    for k, rep in sections.items():
        if 'arms' not in rep:
            continue
        c = {x['metric']: x for x in rep['comparisons']}
        t, v = c.get('nlc.time_to_goal'), c.get('nlc.violated')
        if t and t['estimate'] is not None and v and v['estimate'] is not None and t['verdict'] != 'margin undeclared':
            out.append(Insight('nlc.arms', 'info',
                               f"{k}: time to goal {t['estimate']:+.1f} s [{t['interval'][0]:+.1f}, {t['interval'][1]:+.1f}] "
                               f"({t['verdict']}), violations {v['estimate']:+.2f} ({v['verdict']}), n={t['n_units']}",
                               scope={'section': k}, data={'time_to_goal': t, 'violated': v}, basis='statistical'))
    return [i.as_dict() for i in out]

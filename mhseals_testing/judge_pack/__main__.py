"""python3 -m mhseals_testing.judge_pack COMMAND ... (CRANE_ROOT = a crane checkout with judge/)

  pairs [ROOT] [--task T] [--out DIR]   fast vs Unity over the pair archive: G4, failure modes,
                                         divergence origin, first differing decisions, insights
  session PATH... [--out DIR]           judge harness sessions of any tier (runs report)
  real BAG_OR_SESSION [--out DIR]        the real boat from a rosbag2/MCAP bag (+ journal)
  nlc OUT_DIR [--out DIR]                the NLC pilot outputs: arms paired by seed, D1 vs D2

Each writes report.json, report.md, feed.json and verdict.txt (the one-line verdict) under --out.
"""

import argparse
from pathlib import Path
import sys

# Everything below imports the crane judge lazily: the import-all smoke test imports this module
# on hosts without crane, and the boat never runs it.


def _core():
    from mhseals_testing.judge_pack.roboboat._core import core
    if not core():
        sys.exit('judge core not importable: set CRANE_ROOT to a crane checkout with judge/')


def _by_pair(by_high):
    """Each pair replays through its own fast session's plant (limits and calibration epoch)."""
    from judge.insight.divergence import Replayer

    class ByPair(Replayer):
        horizon_s = None

        def reset_closed_loop(self, high, t0, t1):
            r = by_high.get(high.run_id)
            return r.reset_closed_loop(high, t0, t1) if r else None
    return ByPair()


def cmd_pairs(a):
    from mhseals_testing.judge_pack.roboboat import packs, pairs, replay
    P, refused, by = pairs.load(Path(a.path or pairs.ROOT).expanduser(), a.task)
    reps = {}
    try:
        for pid, fast, unity in P:
            spec = replay.fast_spec(Path(fast.attachments['roboboat.record']['session']))
            if spec:
                reps[unity.run_id] = replay.FastPlantReplayer(*spec)
    except ImportError as e:      # fastsim not importable: the split degrades, the report says so
        print(f'replay unavailable: {e}', file=sys.stderr)
    from judge.run import judge_tiers
    rep = judge_tiers(P, packs.TASK, packs.PACKS, replayer=_by_pair(reps) if reps else None,
                      decision_kinds=('task.decision', 'task.goal'), segmenter=replay.goal_segments)
    rep['refused'] = refused
    rep['archive'] = {'root': str(a.path or pairs.ROOT), 'n_index': len(by),
                      'by_task': _count(r['condition'].get('task') for r in by.values())}
    _emit(rep, a.out, 'fast vs Unity: pair archive')


def cmd_session(a):
    from mhseals_testing.judge_pack.roboboat import packs
    from mhseals_testing.judge_pack.roboboat.sources import SessionSource
    src, refused, recs = SessionSource(), [], []
    for p in a.paths:
        recs += src.load_all(p, refused)
    from judge.run import judge_runs
    rep = judge_runs(recs, packs.TASK, packs.PACKS)
    rep['refused'] = refused
    _emit(rep, a.out, 'sessions')


def cmd_real(a):
    from mhseals_testing.judge_pack.roboboat import real
    rep = real.judge_bag(Path(a.path))
    _emit(rep, a.out, f'real boat: {Path(a.path).name}')


def cmd_nlc(a):
    from mhseals_testing.judge_pack.roboboat import nlc
    rep = nlc.judge_outputs(Path(a.path))
    _emit(rep, a.out, 'NLC pilot')


def _count(xs):
    out = {}
    for x in xs:
        out[x] = out.get(x, 0) + 1
    return out


def _emit(rep, out, title):
    from judge import render
    print(render.oneline(rep))
    if out:
        render.write(rep, out, title)
        print(f'wrote {out}', file=sys.stderr)


def main(argv=None):
    ap = argparse.ArgumentParser(prog='judge_pack', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('pairs')
    p.add_argument('path', nargs='?')
    p.add_argument('--task')
    p.add_argument('--out')
    p.set_defaults(fn=cmd_pairs)
    p = sub.add_parser('session')
    p.add_argument('paths', nargs='+')
    p.add_argument('--out')
    p.set_defaults(fn=cmd_session)
    p = sub.add_parser('real')
    p.add_argument('path')
    p.add_argument('--out')
    p.set_defaults(fn=cmd_real)
    p = sub.add_parser('nlc')
    p.add_argument('path')
    p.add_argument('--out')
    p.set_defaults(fn=cmd_nlc)
    a = ap.parse_args(argv)
    _core()
    a.fn(a)


if __name__ == '__main__':
    main()

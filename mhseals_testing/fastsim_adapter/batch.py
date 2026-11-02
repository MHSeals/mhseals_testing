"""RoboBoat fast-sim batch: seeds -> scored run folders, each with a flat ``metrics.json``.

    python3 -m mhseals_testing.fastsim_adapter.batch --seeds 1:21 --out OUT [--variant V]
        [--config YAML] [--tune JSON] [--workers N] [--allow-dirty]

Needs crane's fast-sim core on PYTHONPATH (a crane checkout's root), mhseals_nav and
``CRANE_DIR`` (that checkout) and ``FASTSIM_TRUTHS`` (the course folder; missing courses are
generated with crane's coursegen). ``--tune`` overrides config ``tune`` keys, e.g.
``'{"params": {"speed": {"cruise": 1.2}}}'``. Run ``OUT/<variant>/s<seed>`` is a harness session
the referee scored; ``metrics.json`` holds the core's numbers and the referee's (cluster metrics).
"""

import argparse
import json
from multiprocessing import get_context
from pathlib import Path
import sys
import time

from mhseals_testing.fastsim_adapter import config as fsconfig


def numbers(run_dir):
    """The run's flat numbers: core (outcome as ``succeeded``, flags, contacts, clearance) and referee."""
    from fastsim.pairs import flat_numbers
    from mhseals_testing.fastsim_adapter.adapter import referee
    run = json.loads((Path(run_dir) / 'run.json').read_text())
    out = flat_numbers({'run': {'sim_s': run['sim_s'], 'contacts': len(run['contacts']),
                                'min_clearance': run['min_clearance_m']}})
    out.update(succeeded=float(run['outcome'] == 'SUCCEEDED'), flags=float(len(run['flags'])),
               error=float(run['error'] is not None))
    out.update(flat_numbers(referee(run_dir)))
    return out


def score(run_dir):
    m = numbers(run_dir)
    (Path(run_dir) / 'metrics.json').write_text(json.dumps(m, indent=1, sort_keys=True) + '\n')
    return m


def run(config=None, tune=None, seeds=(), out='.', variant='base', workers=None, allow_dirty=False):
    """Generate missing courses, run every seed, score each run; returns [(run_dir, metrics)]."""
    from fastsim import identity
    from fastsim.batch import adapter_source, run_batch
    from fastsim.scenario import ScenarioSource
    cfg, base = fsconfig.load(config, tune)
    ScenarioSource(**cfg['scenarios'], base=base).generate(list(seeds))
    where = adapter_source(cfg, base)
    adapter_git = identity.git_state(where) if where else None
    if adapter_git and adapter_git['dirty'] and not allow_dirty:
        raise RuntimeError(f"{adapter_git['source']} has uncommitted changes; commit, or allow dirty")
    import mhseals_nav
    provenance = {'git': {'core': identity.git_state(Path(identity.__file__).parent), 'adapter': adapter_git,
                          'mhseals_nav': identity.git_state(mhseals_nav.__file__)},
                  'config_file': str(Path(config or fsconfig.CONFIG).resolve()), 'tune': tune or {}}
    results = run_batch(cfg, base, seeds, out, variant, workers, provenance)
    dirs = [r.out for r in results]
    n = max(1, min(workers or cfg['workers'], cfg['max_workers'], len(dirs)))
    if n <= 1:
        scored = [score(d) for d in dirs]
    else:
        with get_context('spawn').Pool(n) as pool:
            scored = pool.map(score, dirs, chunksize=1)
    return list(zip(dirs, scored)), results


def main(argv=None):
    p = argparse.ArgumentParser(prog='fastsim_adapter.batch', description=__doc__.split('\n')[0])
    p.add_argument('--config', type=Path, help='default: config/fastsim_roboboat.yaml')
    from fastsim.batch import parse_seeds
    p.add_argument('--seeds', required=True, type=parse_seeds, help='N, FROM:TO or a,b,c')
    p.add_argument('--out', required=True, type=Path)
    p.add_argument('--variant', default='base')
    p.add_argument('--tune', type=json.loads, help='tune overrides (JSON)')
    p.add_argument('--workers', type=int)
    p.add_argument('--allow-dirty', action='store_true', help='run an uncommitted adapter tree')
    a = p.parse_args(argv)
    started = time.perf_counter()
    try:
        scored, results = run(a.config, a.tune, a.seeds, a.out, a.variant, a.workers, a.allow_dirty)
    except RuntimeError as e:
        print(f'fastsim_adapter.batch: {e}', file=sys.stderr)
        return 2
    for r, (_, m) in zip(results, scored):
        print(f"{r.out}: {r.outcome} in {r.sim_s:.0f} s sim, points {m.get('points')}")
    wall = time.perf_counter() - started
    print(f'{len(results)} runs, {wall:.1f} s wall: {len(results) / wall:.2f} runs/s')
    return 0


if __name__ == '__main__':
    sys.exit(main())

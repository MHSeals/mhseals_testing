"""Mission records and sim batches (spec tui section 18, master section 9 ts129).

A mission run is an ordinary ``runs/<run_id>.json`` (schema ``mhseals.run/1``)
with a ``mission`` block. The block's first key, ``summary``, is one line for
people and agents. The referee keeps no points (owner decision): the block
records completion, times, splits, failures and contacts, and comparison is
against earlier runs. Pure and ROS-free; the sim runner is injected.
"""

from pathlib import Path

from .. import data

RUN_SCHEMA = 'mhseals.run/1'
BOAT_DOMAINS = (42, 142)
SIM_DOMAINS = range(200, 229)  # matches crane_experiment's guard


def check_domain(domain):
    """Return ``domain`` as an int, or raise ValueError when it is not safe for the sim."""
    domain = int(domain)
    if domain in BOAT_DOMAINS:
        raise ValueError(f'ROS domain {domain} is the boat; sim runs use '
                         f'{SIM_DOMAINS.start}-{SIM_DOMAINS[-1]}')
    if domain not in SIM_DOMAINS:
        raise ValueError(f'ROS domain {domain} is outside the sim range '
                         f'{SIM_DOMAINS.start}-{SIM_DOMAINS[-1]}')
    return domain


def summary_line(task, outcome, done, total, duration, failures=()):
    """``pair_test: complete, 3/4 elements, 142 s, 1 failure`` (no points)."""
    parts = [f'{task}: {outcome}', f'{done}/{total} elements', f'{duration:.0f} s']
    if failures:
        parts.append(f'{len(failures)} failure{"s" if len(failures) != 1 else ""}')
    return ', '.join(parts)


def mission_block(task, outcome, duration, elements=(), failures=(), contacts=(),
                  plant='crane', profile=None, environment=None, artifacts=None,
                  settings=None):
    """The ``mission`` block, ``summary`` first. ``elements`` are dicts with ``name``, ``done`` and ``t`` (split, s)."""
    elements = [dict(e) for e in elements]
    done = sum(1 for e in elements if e.get('done'))
    failures = list(failures)
    return {
        'summary': summary_line(task, outcome, done, len(elements), duration, failures),
        'task': task, 'plant': plant, 'profile': profile, 'outcome': outcome,
        'duration': duration,
        'completion': {'done': done, 'total': len(elements), 'elements': elements},
        'failures': failures, 'contacts': list(contacts),
        'environment': environment or {}, 'artifacts': artifacts or {},
        'settings': settings or {},
    }


def write_run(run_id, mission, status='MEASURED', started=None, root=None):
    """Write ``runs/<run_id>.json`` with the mission block; returns the path."""
    record = {'run_id': run_id, 'procedure': 'mission', 'status': status,
              'started': started, 'mission': mission}
    target = Path(root) / 'runs' / f'{run_id}.json' if root else data.path('runs', f'{run_id}.json')
    data.write_json(target, record, RUN_SCHEMA)
    return target


def sim_batch(task, run_one, repeat=1, domain=200, profile=None, root=None, name=None):
    """Run ``task`` ``repeat`` times in the sim and record each as a run.

    ``run_one(task, domain, profile, index)`` returns the keyword arguments of
    ``mission_block`` (minus task, plant and profile). Each repeat gets its own
    domain (``domain + index``) so overlapping sims never share a graph; every
    domain is checked first, before anything runs. Returns the batch summary.
    """
    domains = [check_domain(domain + i) for i in range(repeat)]
    runs = []
    for index, dom in enumerate(domains, 1):
        mission = mission_block(task, plant='crane', profile=profile,
                                **run_one(task, dom, profile, index))
        run_id = f'{name or "sim"}_{task}_{index}'
        write_run(run_id, mission, root=root)
        runs.append({'run_id': run_id, 'domain': dom, 'summary': mission['summary']})
    return {'summary': f'{task}: {len(runs)} sim run{"s" if len(runs) != 1 else ""}',
            'task': task, 'profile': profile, 'runs': runs}

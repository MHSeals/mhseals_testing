"""One record per procedure run: ``runs/<run_id>.json`` (``mhseals.run/1``).

A run file is written once when the procedure ends. Its measurements are never
rewritten: excluding a run only sets ``status`` to EXCLUDED, fills ``excluded``
and appends to ``history``. Analysis reads these files and never edits them.
"""

from datetime import datetime
import re

from mhseals_testing import data

SCHEMA = 'mhseals.run/1'
MEASURED, ABORTED, EXCLUDED = 'MEASURED', 'ABORTED', 'EXCLUDED'
ABORT_VERDICTS = {'ABORTED', 'CANCELED'}


def slug(name):
    """``name`` reduced to [a-z0-9-] for use in a file name."""
    return re.sub(r'[^a-z0-9]+', '-', str(name).lower()).strip('-') or 'run'


def status_of(result):
    """MEASURED unless the procedure itself (or one of its results) reported an abort."""
    verdicts = [result.get('verdict'), *(r.get('verdict') for r in result.get('results', []))]
    return ABORTED if ABORT_VERDICTS & set(verdicts) else MEASURED


def next_run_id(directory, procedure):
    """``NNN-<procedure>``, numbered after the run files already in ``directory``."""
    count = len(list(directory.glob('*.json'))) if directory.is_dir() else 0
    return f'{count + 1:03d}-{slug(procedure)}'


def write_run(directory, result, settings=None, schedule=None, now=None, status=None, provenance=None):
    """Write the run file for ``result`` and return its record (``run_id`` included).

    A ``run_id`` already in ``result`` is kept; an existing file is never replaced.
    ``provenance`` (``session.provenance``) is what produced the run: git, config ids, profile hash.
    """
    run_id = result.get('run_id') or next_run_id(directory, result.get('procedure', 'run'))
    target = directory / f'{run_id}.json'
    if target.exists():
        return data.read_json(target)
    record = dict(run_id=run_id, procedure=result.get('procedure'), status=status or status_of(result),
                  excluded=None, time=(now or datetime.now().astimezone()).isoformat(),
                  settings=settings or {}, schedule=schedule or [], provenance=provenance or {},
                  result={k: v for k, v in result.items() if k != 'run_id'}, history=[])
    data.write_json(target, record, SCHEMA)
    return record


def exclude_run(directory, run_id, reason, by, now=None):
    """Mark a run EXCLUDED with who and why; measurements stay as they were."""
    target = directory / f'{run_id}.json'
    record = data.read_json(target)
    if record is None:
        raise FileNotFoundError(f'no run {run_id} in {directory}')
    when = (now or datetime.now().astimezone()).isoformat()
    record['history'].append(dict(time=when, from_status=record['status'], to_status=EXCLUDED,
                                  reason=reason, by=by))
    record['status'] = EXCLUDED
    record['excluded'] = dict(reason=reason, by=by)
    data.write_json(target, record, SCHEMA)
    return record


def load_runs(directory):
    """All run records, oldest id first."""
    return [data.read_json(p) for p in sorted(directory.glob('*.json'))]


def usable(record):
    """True for runs analysis may use: MEASURED and not excluded."""
    return record['status'] == MEASURED and not record.get('excluded')

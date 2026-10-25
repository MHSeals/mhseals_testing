"""Link config versions to sessions and runs. No ROS here.

Pure helpers over a ``History``: the ``config`` block of ``session.json``, the objects copied into
``snapshots/``, the ``config`` and ``staged`` fields of a run record, the ``config_changed`` event,
the per-session counts for ``INDEX.json`` and the guard that stops analysis pooling runs made on
different hardware or FCU configs. Session, run and index writers call these; they do not own files.
"""

from pathlib import Path

from mhseals_testing import data

SUBSYSTEMS = ('hardware', 'nav2', 'fcu', 'rtk')
POOL_KEYS = ('hardware', 'fcu')         # a different id here means a different boat to the analysis


class PoolRefused(Exception):
    """Runs from different configs were pooled without ``--pool``; the message lists the groups."""


def heads(history, subsystems=SUBSYSTEMS):
    """``{sub: id}`` of the newest change per subsystem; None where nothing is recorded (``fcu ?``)."""
    out = {}
    for sub in subsystems:
        head = history.head(sub)
        out[sub] = head['id'] if head else None
    return out


def session_config(history):
    """The ``config`` block for a new ``session.json``."""
    return {'start': heads(history), 'changes': []}


def snapshot(history, change, directory):
    """Copy the object ``change`` produced into ``directory`` (the session's snapshots/); returns the path."""
    target = Path(directory) / f"{change['subsystem']}-{change['id']}.json"
    data.write_json(target, history.version(change), 'mhseals.config_snapshot/1')
    return target


def snapshot_start(history, config, directory):
    """Snapshot every start version of a session; returns the paths written."""
    changes = {c['id']: c for c in history.changes()}
    return [snapshot(history, changes[i], directory) for i in config['start'].values() if i in changes]


def add_change(config, history, change, directory):
    """Append ``change`` to the session's ``config.changes``, snapshot it, and return the ``config_changed`` event."""
    if change['id'] not in config['changes']:
        config['changes'].append(change['id'])
        snapshot(history, change, directory)
    return {'type': 'config_changed', 'id': change['id'], 'subsystem': change['subsystem'],
            'summary': list(change.get('summary') or [])}


def run_config(history, staged=None):
    """The fields ``ctx.record`` adds to a run record: ``config`` as of now, and ``staged`` when set."""
    out = {'config': heads(history)}
    if staged:
        out['staged'] = staged
    return out


def config_runs(runs):
    """``{"hardware:c0142": n}``: how many runs used each version, for ``INDEX.json``."""
    counts = {}
    for run in runs:
        for sub, cid in (run.get('config') or {}).items():
            if cid:
                counts[f'{sub}:{cid}'] = counts.get(f'{sub}:{cid}', 0) + 1
    return counts


def check_pool(runs, pool=False):
    """Refuse runs whose hardware or FCU ids differ unless ``pool``; returns the note the output records."""
    groups = {}
    for run in runs:
        cfg = run.get('config') or {}
        groups.setdefault(tuple(cfg.get(k) for k in POOL_KEYS), []).append(run.get('run_id', '?'))
    if len(groups) <= 1:
        return {'pooled': False, 'groups': len(groups)}
    if not pool:
        def label(ids):
            return ', '.join(k + ' ' + (v or '?') for k, v in zip(POOL_KEYS, ids))

        lines = [f"  {label(ids)}: {len(members)} runs"
                 for ids, members in sorted(groups.items(), key=str)]
        raise PoolRefused('runs span different configs (use --pool to pool anyway):\n' + '\n'.join(lines))
    return {'pooled': True, 'groups': len(groups)}

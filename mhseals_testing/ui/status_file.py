"""``runtime_dir()/status.json``: what the TUI knows, for ``boat_trial status`` (spec agent-control 8.1).

The desk calls ``write`` once a second; the write is atomic, so a reader never sees half a file.
Every section is filled best-effort: a section that fails reads ``{'error': ...}`` and never stops
the UI loop. Perception reads only existing fact keys (``perc.*``); nothing here touches its code.
"""

import math
import os
import socket
import time

from mhseals_testing import data
from mhseals_testing.harness.registry import runtime_dir
from mhseals_testing.ui import feeds as feeds_module

SCHEMA = 'mhseals.status/1'


def path():
    return runtime_dir() / 'status.json'


def write(app, desk, target=None):
    try:
        data.write_json(target or path(), clean(build(app, desk)), SCHEMA)
    except (OSError, ValueError, TypeError):
        pass                                     # status is advisory; the next second tries again


def build(app, desk):
    out = {'t_ns': time.time_ns(), 'host': socket.gethostname(), 'pid': os.getpid()}
    for name, section in SECTIONS.items():
        try:
            out[name] = section(app, desk)
        except Exception as error:  # noqa: BLE001 - one broken section must not cost the rest
            out[name] = {'error': f'{type(error).__name__}: {error}'}
    return out


def clean(value):
    """JSON-safe: non-finite numbers become None, unknown objects their text."""
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [clean(v) for v in value]
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if value is None or isinstance(value, (str, int, bool)):
        return value
    return str(value)


def _session(app, desk):
    return {'profile': app.profile.name, 'sim': bool(app.profile.use_sim_time),
            'muxed': bool(app.profile.muxed), 'dir': str(app.paths.directory), 'owner': os.getpid(),
            'armed': bool(app.armed), 'recording': getattr(app.recorder, 'mode', None)}


def _authority(app, desk):
    record = feeds_module.merged(app.feeds).get('authority_record')
    arbiter = getattr(app, 'arbiter', None)
    mux, at = (arbiter.mux, arbiter.mux_at) if arbiter is not None else (None, None)
    return {'record': record, 'mux_input': (mux or {}).get('input'),
            'mux_age_s': None if at is None else round(arbiter.clock() - at, 2),
            'agents': app.settings.agents, 'agent_motion': app.settings.agent_motion}


def _run(app, desk):
    return {'running': app.running, 'run_id': app.run_id, 'notice': str(app.notice or ''),
            'notices': list(desk.notices)}


def _health(app, desk):
    from mhseals_testing.ui.agent_acts import procedure
    gates = {}
    for name in desk.config.get()['run']:
        found, _ = procedure(name)
        if found is not None:
            gates[name] = app.availability(found) or None
    return {'summary': app.snapshot().get('health'), 'gates': gates}


def _odometry(app, desk):
    row = app.monitor.snapshot()['sensors'].get('odometry', {})
    return {k: row.get(k) for k in ('status', 'age', 'stamp_age', 'values', 'issue')}


def _perception(app, desk):
    return {name: fact.value for name, fact in app.facts.items() if name.startswith('perc.')}


def _nav2(app, desk):
    goals = getattr(app, 'goals', None)
    active = goals.active if goals is not None else None
    return {'goal': None if active is None else {k: active[k] for k in ('claim', 'x', 'y', 'yaw')}}


def _agent(app, desk):
    return desk.summary()


SECTIONS = {'session': _session, 'authority': _authority, 'run': _run, 'health': _health,
            'odometry': _odometry, 'perception': _perception, 'nav2': _nav2, 'agent': _agent}

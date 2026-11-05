"""config/fastsim_roboboat.yaml (``tune`` on top) -> the crane fast-sim core's config.

``tune`` holds what people change (task, mission, profile, mission params, calibration, course family,
workers); ``adapter`` the adapter's options; ``core`` the core's own keys. ``load(path, tune)``
merges ``tune`` overrides (a sweep's parameter set) and returns the core config and its base folder.
"""

import copy
import os
from pathlib import Path

from mhseals_testing.health.config import config_path
from mhseals_testing.strict_yaml import load_strict

CONFIG = config_path('config', 'fastsim_roboboat.yaml')
ADAPTER = 'mhseals_testing.fastsim_adapter.adapter:RoboBoatAdapter'


def merge(base, over):
    out = copy.deepcopy(base)
    for key, value in (over or {}).items():
        out[key] = merge(out[key], value) if isinstance(out.get(key), dict) and isinstance(value, dict) \
            else copy.deepcopy(value)
    return out


def expand(value):
    """$VARS and ~ expanded in every string (scenario folders, generator commands)."""
    if isinstance(value, dict):
        return {k: expand(v) for k, v in value.items()}
    if isinstance(value, list):
        return [expand(v) for v in value]
    return os.path.expanduser(os.path.expandvars(value)) if isinstance(value, str) else value


def domain(doc, tune=None):
    """The core's domain config (before its defaults) from the YAML document and ``tune`` overrides."""
    t = merge(doc['tune'], tune)
    options = {**doc.get('adapter', {}), 'task': t['task'], 'mission': t.get('mission'),
               'profile': t['profile'], 'params': t['params'] or None}
    core = copy.deepcopy(doc.get('core', {}))
    scenarios = core.pop('scenarios', None)
    if scenarios is not None:
        scenarios = expand({**scenarios, 'fields': {'family': t['family'], 'mode': t['mode'],
                                                    **scenarios.get('fields', {})}})
    return {**core, 'scenarios': scenarios, 'calibration': t['calibration'], 'workers': t['workers'],
            'adapter': {'ref': ADAPTER, 'options': options}}


def load(path=None, tune=None):
    """(core config, base folder): the YAML at ``path`` (default the shipped one), ``tune`` merged."""
    from fastsim import config as fsconfig
    path = Path(path or CONFIG)
    base = path.resolve().parent
    return fsconfig.resolve(domain(load_strict(path.read_text(), path), tune), base), base

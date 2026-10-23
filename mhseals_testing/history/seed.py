"""Seed the config history on first run (CH10). Idempotent: running it again records nothing.

1. Optional hardware snapshots (``*.hardware.yaml``) are imported as dated ``import`` versions, oldest first,
   with no trial links (the old report format cannot be linked without guessing).
2. The FCU history starts from a ``.param`` file ("NAME,value" lines) when it has none.
3. Every subsystem reader is ``observe``d, so the current state is the head (c0001... on a fresh store).
"""

import datetime
from fnmatch import fnmatch
from pathlib import Path

import yaml

FCU_NAME = 'audit 10-22 (GSF yaw, no compass)'
FCU_NOTE = '10-19 ad-hoc script: EK3_SRC1_YAW 1->8 (GSF), COMPASS_USE 1->0. As found 2026-10-22.'
# Counters and offsets that change on their own; never stored (config_history.md fcu.yaml `ignore`).
FCU_IGNORE = ('STAT_*', 'INS_GYROFFS_*', 'INS_GYR2OFFS_*', 'INS_GYR3OFFS_*', 'BARO*_GND_PRESS', 'MIS_TOTAL',
              'FORMAT_VERSION')


def read_param_file(path, ignore=FCU_IGNORE):
    """``{name: float}`` from a Mission Planner / MAVProxy ``NAME,value`` file; comments and ignored names dropped."""
    params = {}
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        name, _, value = line.replace('\t', ',').replace(' ', ',').partition(',')
        if any(fnmatch(name, pattern) for pattern in ignore):
            continue
        params[name] = float(value.strip().split(',')[0])
    return params


def _seed_fcu(history, param_file):
    """Record the FCU baseline when the FCU has no history; returns the change or None."""
    if not param_file or history.head('fcu'):
        return None
    params = read_param_file(param_file)
    change = history.record('fcu', params, None, source='seed', applied='observed', meta={'count': len(params)})
    if change:
        history.label(change['id'], name=FCU_NAME, note=FCU_NOTE)
    return change


def _import_hardware(history, folder):
    """Import ``*.hardware.yaml`` files not yet stored, oldest file time first; returns the new changes."""
    done = []
    for file in sorted(Path(folder).glob('*.hardware.yaml'), key=lambda f: (f.stat().st_mtime, f.name)):
        raw = file.read_text()
        value = yaml.safe_load(raw)
        if not isinstance(value, dict):
            continue
        value = {'overrides': value}
        if not value['overrides'] or history.has_object('hardware', value):
            continue                                    # empty, or this content is already a stored version
        when = datetime.datetime.fromtimestamp(file.stat().st_mtime).astimezone().isoformat(timespec='seconds')
        change = history.record('hardware', value, raw, source='import', applied='observed',
                                summary=[file.name], time=when)
        done.append(change)
    return done


def seed(history, readers, *, fcu_param=None, hardware_snapshots=None):
    """Seed ``history``; ``readers`` maps subsystem -> ``() -> (value, raw, meta)`` or None when unavailable.

    Returns the list of changes recorded this call (empty when everything was already seeded).
    """
    made = []
    if hardware_snapshots:
        made += _import_hardware(history, hardware_snapshots)
    made.append(_seed_fcu(history, fcu_param))
    for sub, read in readers.items():
        state = read()
        if state is not None:
            made.append(history.observe(sub, *state))
    return [c for c in made if c]

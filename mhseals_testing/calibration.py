"""Calibration provenance, scope and status (conditions_minimap.md section 4). No ROS here.

``record`` writes what a fit was made under into ``calibration/current.json``; ``status`` is
computed from that record and the config *now*, every time it is displayed. A changed
dependency marks a calibration invalid (``x``) or in need of a wiring check (``?``); nothing is
ever deleted or reverted, the operator reruns or clears it.
"""

import datetime
import fnmatch
from pathlib import Path

import yaml

from mhseals_testing import data
from mhseals_testing.history import digest

SCHEMA = 'mhseals.calibration/1'
INVALID, VERIFY, OK = 'invalid', 'verify', 'ok'
MARKS = {INVALID: 'x', VERIFY: '?', OK: ''}
BATTERY_TOLERANCE_V = 0.8


def load_fields(path=None):
    """config/history/fields.yaml, source tree first, else the installed share directory."""
    source = Path(__file__).resolve().parent.parent / 'config' / 'history' / 'fields.yaml'
    if not source.exists():
        from ament_index_python.packages import get_package_share_directory
        source = Path(get_package_share_directory('mhseals_testing')) / 'config' / 'history' / 'fields.yaml'
    return yaml.safe_load(Path(path or source).read_text())


def _matches(key, block):
    return key in block.get('keys', ()) or any(fnmatch.fnmatchcase(key, p) for p in block.get('patterns', ()))


def scope_of(key, fields=None):
    """``('hardware', None)``, ``('calibration', 'run')``, ``('ignored', None)`` or ``('setting', None)``."""
    fields = (fields or load_fields())['scope']
    if _matches(key, fields['hardware']):
        return 'hardware', None
    for cadence, block in fields['calibration'].items():
        if _matches(key, block):
            return 'calibration', cadence
    if _matches(key, fields.get('ignored', {})):
        return 'ignored', None
    return 'setting', None


def depends_on(fields=None):
    """The ``{invalid: [...], verify: [...]}`` keys the run calibrations (C+/-, curves) depend on."""
    return (fields or load_fields())['scope']['calibration']['run']['depends_on']


def is_yaw_dep(key, fields=None):
    """Whether ``key`` is a setting whose change asks for a yaw check (EK3_SRC1_YAW, COMPASS_USE*, magcal)."""
    return _matches(key, (fields or load_fields())['scope']['yaw_deps'])


def dependency_hash(hardware, keys):
    """Hash of the ``keys`` in ``hardware``; stable under key order, absent keys hash as None
    except ``hardware_epoch``, where absent equals 0 so adding ``hardware_epoch: 0`` later invalidates nothing."""
    values = {k: hardware.get(k) for k in sorted(keys)}
    if 'hardware_epoch' in values and values['hardware_epoch'] is None:
        values['hardware_epoch'] = 0
    return 'sha256:' + digest(values)


def record(direction, fit, hardware, *, extra=None, path=None, fields=None):
    """Store the provenance of one direction's fit in calibration/current.json and return the entry.

    ``fit`` is {id, time, session, run, config}; ``hardware`` is the hardware config at fit time.
    Other directions are kept; the entry is replaced whole. A new fit ends the stored staleness
    verdicts (they judged the old fit), so ``calib.stale`` clears; the session scores are kept.
    """
    deps = depends_on(fields)
    entry = {'fit': fit, **(extra or {}),
             'hardware': {'hash': dependency_hash(hardware, deps['invalid']), 'epoch': hardware.get('hardware_epoch', 0),
                          'verify': dependency_hash(hardware, deps['verify'])}}
    target = Path(path) if path else data.path('calibration', 'current.json')
    current = data.read_json(target, {})
    directions = {**current.get('directions', {}), direction: entry}
    updated = {**current, 'directions': directions}
    tracking = (current.get('provenance') or {}).get('tracking')
    if isinstance(tracking, dict):
        cleared = {a: ({k: v for k, v in row.items() if k != 'staleness'} if isinstance(row, dict) else row)
                   for a, row in tracking.items()}
        updated['provenance'] = {**current['provenance'], 'tracking': cleared}
    data.write_json(target, updated, SCHEMA)
    return entry


def new_fit(now=None):
    """The ``{id, time}`` stamp of one characterization, also written to hardware.yaml's fit map."""
    now = now or datetime.datetime.now().astimezone()
    return {'id': 'f' + now.strftime('%Y%m%dT%H%M%S'), 'time': now.isoformat(timespec='seconds')}


def record_fit(names, fit, hardware, *, context=None, conditions=None, path=None, fields=None, dead=()):
    """``record`` every direction in ``names`` under one ``fit`` stamp.

    ``context`` is what the app knows ({session, run, config, battery_v, muted}); ``conditions``
    is the water summary the fit was made under. Missing pieces are simply left out. Each entry
    says whether its direction was ``dead`` (no response) in this fit; a later fit replaces it.
    """
    context = context or {}
    fit = {**fit, **{k: context[k] for k in ('session', 'run', 'config') if context.get(k)}}
    extra = {'gate': {'muted': bool(context.get('muted'))}}
    if context.get('battery_v') is not None:
        extra['battery_v'] = context['battery_v']
    if conditions:
        extra['conditions'] = conditions
    return {n: record(n, fit, hardware, extra={**extra, 'dead': n in dead}, path=path, fields=fields)
            for n in names}


def load(path=None):
    """The stored calibration record, or an empty one."""
    return data.read_json(Path(path) if path else data.path('calibration', 'current.json'), {'directions': {}})


def _num(change_id):
    return int(change_id[1:])


def _changed_after(history, fit_config):
    """The id of the first hardware change after the config the fit was made under, for messages."""
    if history is None:
        return None
    for change in history.changes('hardware'):
        if fit_config is None or _num(change['id']) > _num(fit_config):
            return change['id']
    return None


def status(entry, hardware, history=None, *, battery_v=None, fields=None):
    """``(state, reason)`` of one direction entry against the hardware config now; worst state wins."""
    deps = depends_on(fields)
    stored = entry.get('hardware', {})
    if stored.get('hash') != dependency_hash(hardware, deps['invalid']):
        after = _changed_after(history, entry.get('fit', {}).get('config'))
        return INVALID, f'hardware changed {after} after fit' if after else 'hardware changed after fit'
    if stored.get('verify') != dependency_hash(hardware, deps['verify']):
        return VERIFY, 'wiring check'
    fitted = entry.get('battery_v')
    if battery_v is not None and fitted is not None and abs(battery_v - fitted) > BATTERY_TOLERANCE_V:
        return VERIFY, f'battery {battery_v:.1f} V vs {fitted:.1f} V at fit'
    if entry.get('gate', {}).get('muted'):
        return VERIFY, 'fitted with the conditions gate muted'
    return OK, ''


def statuses(hardware, history=None, *, current=None, **kw):
    """``{direction: (state, reason)}`` for the stored record, computed now."""
    current = load() if current is None else current
    return {d: status(e, hardware, history, **kw) for d, e in current.get('directions', {}).items()}


def invalid_count(hardware, history=None, *, current=None):
    """The ``calib.invalid`` health fact: how many directions are marked x."""
    return sum(state == INVALID for state, _ in statuses(hardware, history, current=current).values())


def snapshot(hardware, history=None, *, current=None, now=None):
    """The derived ``status`` block written at session close, for agents; never read back."""
    now = now or datetime.datetime.now().astimezone().isoformat(timespec='seconds')
    entries = {d: {'state': s, 'reason': r} for d, (s, r) in statuses(hardware, history, current=current).items()}
    return {'computed': now, 'entries': entries}


def marks(hardware, history=None, *, current=None, battery_v=None):
    """``{direction: mark}`` for the Correction column heads, as the tab draws them (``✗`` invalid, ``?`` verify)."""
    shown = {INVALID: '✗', VERIFY: '?'}
    return {d: shown[s] for d, (s, _) in statuses(hardware, history, current=current, battery_v=battery_v).items()
            if s in shown}


def write_snapshot(hardware, history=None, *, path=None):
    """Add the derived ``status`` block to calibration/current.json (session close); False when there is no record."""
    target = Path(path) if path else data.path('calibration', 'current.json')
    current = data.read_json(target, None)
    if not current or not current.get('directions'):
        return False
    data.write_json(target, {**current, 'status': snapshot(hardware, history, current=current)}, SCHEMA)
    return True

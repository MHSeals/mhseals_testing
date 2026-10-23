"""Config diffs that read like a change note: units, enum names, pins and matrix grids.

``diff`` projects two versions of a subsystem into rows; ``render`` lays the rows out
for a width. Labels and units come from ``config/history/fields.yaml`` (see
config_history.md section 3). ASCII mode swaps the glyphs, and ``•``, ``+`` and ``-``
keep the meaning when colour is off. No ROS here.
"""

from dataclasses import dataclass
from pathlib import Path

import yaml

FIELDS_FILE = Path(__file__).resolve().parents[2] / 'config' / 'history' / 'fields.yaml'
OTHER = 'Other'
_MISSING = object()


@dataclass
class Row:
    """One changed item. ``kind``: change, added, removed or matrix (``grid`` holds its two sides)."""
    group: str
    label: str
    before: object = None
    after: object = None
    unit: str = ''
    kind: str = 'change'
    path: str = ''
    note: str = ''
    grid: dict = None


def load_fields(path=FIELDS_FILE):
    return yaml.safe_load(Path(path).read_text())


def _flatten(node, prefix=()):
    """Leaf values keyed by key path; lists stay whole."""
    if isinstance(node, dict) and node:
        for key, value in node.items():
            yield from _flatten(value, prefix + (str(key),))
    else:
        yield prefix, node


def _grid(old, new, spec, fields):
    """Both sides reshaped to rows x cols, or None when the lists don't fit that shape."""
    rows, cols = fields[spec['rows']], fields[spec['cols']]
    if not all(isinstance(v, (list, tuple)) and len(v) == len(rows) * len(cols) for v in (old, new)):
        return None

    def shape(flat):
        return [list(flat[r * len(cols):(r + 1) * len(cols)]) for r in range(len(rows))]
    return {'rows': rows, 'cols': cols, 'before': shape(old), 'after': shape(new)}


def _place(sub, path, spec, conf):
    """(group, label, unit) of a key path."""
    if sub == 'hardware':
        field = spec.get(path[0], {})
        return field.get('group', OTHER), field.get('label', ' · '.join(path)), field.get('unit', '')
    parts = [p for p in path if p != 'ros__parameters']
    return (conf.get('groups', {}).get(parts[0], parts[0]), ' · '.join(parts[1:]) or parts[0],
            conf.get('units', {}).get(parts[-1], ''))


def _rows(sub, path, old, new, spec, conf, fields, pins):
    group, label, unit = _place(sub, path, spec, conf)
    key = '.'.join(path)
    field = spec.get(path[0], {}) if sub == 'hardware' else {}
    if old is _MISSING or new is _MISSING:
        return [Row(group, label, None if old is _MISSING else old, None if new is _MISSING else new, unit,
                    'removed' if new is _MISSING else 'added', key)]
    if field.get('kind') == 'matrix':
        grid = _grid(old, new, field, fields)
        if grid:
            return [Row(group, label, kind='matrix', path=key, note=field.get('note', ''), grid=grid)]
    if field.get('kind') == 'per' and isinstance(old, list) and isinstance(new, list) and len(old) == len(new):
        rows = []
        for i, (a, b) in enumerate(zip(old, new)):
            if a != b:
                note = f'(pin {pins[a][0]} → {pins[b][0]})' if field.get('pins') and pins and a in pins and b in pins else ''
                rows.append(Row(group, f'{fields["thrusters"][i]} {label}', a, b, unit, path=f'{key}[{i}]', note=note))
        return rows
    return [Row(group, label, old, new, unit, path=key)]


def diff(sub, before, after, pins=None, fields=None):
    """Changed rows between two versions of ``sub``, grouped and in the order fields.yaml lists them.

    Hardware values compare their ``effective`` config. ``pins`` is hardware's J2_PINS, used to
    say which pin an output change means.
    """
    fields = fields or load_fields()
    conf = fields.get(sub, {})
    spec = conf.get('fields', {})

    def flat(value):
        value = value or {}
        if sub == 'hardware' and 'effective' in value:
            value = value['effective']
        return dict(_flatten(value))
    old, new = flat(before), flat(after)
    rank = list(spec)
    rows = []
    for path in sorted(set(old) | set(new), key=lambda p: (rank.index(p[0]) if p[0] in rank else len(rank), p)):
        a, b = old.get(path, _MISSING), new.get(path, _MISSING)
        if a != b:
            rows.extend(_rows(sub, path, a, b, spec, conf, fields, pins))
    groups = conf.get('groups', [])
    order = {name: i for i, name in enumerate(groups if isinstance(groups, list) else [])}
    rows.sort(key=lambda r: order.get(r.group, len(order)))      # stable: field order survives inside a group
    return rows

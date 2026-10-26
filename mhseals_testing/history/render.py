"""Config diffs that read like a change note: units, enum names, pins and matrix grids.

``diff`` projects two versions of a subsystem into rows; ``render`` lays the rows out
for a width. Labels and units come from ``config/history/fields.yaml``.
ASCII mode swaps the glyphs, and ``•``, ``+`` and ``-``
keep the meaning when colour is off. No ROS here.
"""

from dataclasses import dataclass
from pathlib import Path

from rich.text import Text
import yaml

from mhseals_testing.ui import theme

FIELDS_FILE = Path(__file__).resolve().parents[2] / 'config' / 'history' / 'fields.yaml'
OTHER = 'Other'
_MISSING = object()
_ASCII = str.maketrans({'⁺': '+', '⁻': '-', '·': '.', '←': '<', '→': '>', '•': '*', '−': '-'})


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
    choices: dict = None     # enum value -> name


def load_fields(path=FIELDS_FILE):
    return yaml.safe_load(Path(path).read_text())


def _flatten(node, prefix=()):
    """Leaf values keyed by key path; lists stay whole."""
    if isinstance(node, dict):
        for key, value in node.items():
            yield from _flatten(value, prefix + (str(key),))
    else:
        yield prefix, node


def _grid(old, new, spec, fields):
    """Both sides reshaped to rows x cols (a null side becomes all None), or None when the lists don't fit."""
    rows, cols = fields[spec['rows']], fields[spec['cols']]
    if not all(v is None or isinstance(v, (list, tuple)) and len(v) == len(rows) * len(cols) for v in (old, new)):
        return None

    def shape(flat):
        flat = flat if flat is not None else [None] * (len(rows) * len(cols))
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
    return [Row(group, label, old, new, unit, path=key, choices=field.get('choices'))]


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


def glyphs(ascii_):
    """(arrow, left arrow, changed mark, removed mark) for the glyph set."""
    return ('->', '<-', '*', '-') if ascii_ else ('→', '←', '•', '−')


def _number(before, after):
    """Both numbers with the fewest significant digits (from 3) that tell them apart."""
    for sig in range(3, 10):
        a, b = f'{before:.{sig}g}', f'{after:.{sig}g}'
        if a != b:
            break
    return a, b


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _text(value, choices=None):
    if isinstance(value, bool):
        return 'yes' if value else 'no'
    if value is None:
        return 'none'
    if choices and value in choices:
        return str(choices[value])
    return f'{value:.6g}' if _is_num(value) else str(value)


def _change(row):
    """('before', 'after') as words; numbers get just enough digits to differ."""
    if _is_num(row.before) and _is_num(row.after):
        return _number(row.before, row.after)
    return _text(row.before, row.choices), _text(row.after, row.choices)


def _block(title, grid, values, marks, mark, cw):
    """One labelled grid as Text lines: title, column heads, then a row per line; ``marks`` get bold yellow."""
    rows, cols = grid['rows'], grid['cols']
    rl = max(map(len, rows)) + 2
    lines = [Text(' ' * (rl + 1) + title, style='dim'),
             Text(' ' * rl + ''.join(c.rjust(cw - 2).ljust(cw) for c in cols), style='dim')]
    for r, name in enumerate(rows):
        line = Text(name.ljust(rl))
        for c, value in enumerate(values[r]):
            changed = marks[r][c]
            line.append(((mark if changed else ' ') + value).rjust(cw - 2).ljust(cw), style=f'bold {theme.WARN}' if changed else '')
        lines.append(line)
    return lines


def _matrix(row, width, ascii_):
    arrow, left, mark, _ = glyphs(ascii_)
    grid = row.grid
    changed = [[a != b for a, b in zip(x, y)] for x, y in zip(grid['before'], grid['after'])]
    digits = 3
    while digits < 6 and any(f'{a:.{digits}f}' == f'{b:.{digits}f}' for xs, ys, cs in
                             zip(grid['before'], grid['after'], changed) for a, b, c in zip(xs, ys, cs) if c and a is not None and b is not None):
        digits += 1
    fmt = [[['none' if v is None else f'{v:.{digits}f}' for v in line] for line in grid[side]] for side in ('before', 'after')]
    none = [[False] * len(grid['cols']) for _ in grid['rows']]
    cw = max(len(v) for side in fmt for line in side for v in line) + 3
    before = _block('before', grid, fmt[0], none, mark, cw)
    after = _block('after', grid, fmt[1], changed, mark, cw)
    out = [Text(row.label, style='bold')]
    if row.note:
        out[0].append(f'  ({row.note})', style='dim')
    block = before[2].cell_len
    if 2 + 2 * block + 4 <= width:
        out += [Text('  ') + b + Text(' ' * (block - b.cell_len + 4)) + a for b, a in zip(before, after)]
    else:
        out += [Text('  ') + line for line in before + after]
    notes = []
    for r, line in enumerate(changed):
        for c, hit in enumerate(line):
            if hit and None not in (grid['before'][r][c], grid['after'][r][c]):
                was, now = _number(grid['before'][r][c], grid['after'][r][c])
                notes.append(Text(f'{grid["rows"][r]} {left} {grid["cols"][c]}  {was} {arrow} ') .append(now, style='bold'))
    return out + _wrap(notes, width)


def _wrap(items, width):
    """Pieces joined by ' · ' onto lines of at most ``width``, indented two spaces."""
    lines, cur = [], Text('  ')
    for item in items:
        if cur.cell_len > 2 and cur.cell_len + 3 + item.cell_len > width:
            lines.append(cur)
            cur = Text('  ')
        if cur.cell_len > 2:
            cur.append(' · ', style='dim')
        cur.append_text(item)
    return lines + [cur] if cur.cell_len > 2 else lines


def render(rows, width=80, *, meta='', ascii_=None):
    """The rows as a Rich Text for ``width`` columns; ``meta`` sits at the right of the first heading."""
    ascii_ = theme.ASCII if ascii_ is None else ascii_
    arrow, _, _, minus = glyphs(ascii_)
    if not rows:
        return Text('no differences', style='dim')
    plain = [r for r in rows if r.kind == 'change']
    lw = min(max((len(r.label) for r in plain), default=0) + 2, 26)
    vw = max((len(' '.join(_change(r)[:1] + (arrow,) + _change(r)[1:])) + len(r.unit) + bool(r.unit) for r in plain), default=0)
    out, group = [], None
    for row in rows:
        if row.group != group:
            group = row.group
            if row.kind != 'matrix':                  # a matrix title is its own heading
                out.append(Text(group, style='bold'))
        if row.kind == 'matrix':
            out += _matrix(row, width, ascii_)
        elif row.kind in ('added', 'removed'):
            sign = '+' if row.kind == 'added' else minus
            value = _text(row.after if row.kind == 'added' else row.before)
            out.append(Text(f'  {sign} {row.label}  {value}' + (f' {row.unit}' if row.unit else ''), style='dim'))
        else:
            before, after = _change(row)
            unit = f' {row.unit}' if row.unit else ''
            line = Text('  ' + row.label.ljust(lw))
            line.append(before).append(f' {arrow} ', style='dim').append(after + unit, style='bold')
            if row.note:
                line.append(' ' * (vw - len(before) - len(after) - len(unit) - len(arrow) - 2))
                line.append('  ' + row.note.replace('→', arrow), style='dim')
            if line.cell_len > width:                 # too narrow for one line: value goes below the label
                out.append(Text('  ' + row.label))
                line = Text('    ' + before).append(f' {arrow} ', style='dim').append(after + unit, style='bold')
                if row.note:
                    line.append('  ' + row.note.replace('→', arrow), style='dim')
            out.append(line)
    if meta:                                       # right-aligned on the first heading
        out[0].append(' ' * max(2, width - out[0].cell_len - len(meta)) + meta, style='dim')
    for line in out:
        line.rstrip()
    result = Text('\n').join(out)
    if ascii_:                                      # one character each, so the styled spans stay put
        result.plain = result.plain.translate(_ASCII)
    return result


def auto_name(change, rows=(), target=None, ascii_=None):
    """The dim name shown until a person sets one: the first two rows, or what kind of change it was."""
    ascii_ = theme.ASCII if ascii_ is None else ascii_
    arrow, *_ = glyphs(ascii_)
    source = change.get('source', '')
    if source == 'external':
        return 'changed outside boat_test'
    if change.get('revert_of'):
        was = f' "{target["name"]}"' if target and target.get('name') else ''
        return f'{"<-" if ascii_ else "↶"} revert to {change["revert_of"]}{was}'
    if source.endswith('characterize'):
        return 'characterize fit'
    names = []
    for row in rows[:2]:
        if row.kind != 'change':
            names.append(row.label)
        elif isinstance(row.before, bool):
            names.append(row.label)
        else:
            before, after = _change(row)
            names.append(f'{row.label} {before}{arrow}{after}')
    return ', '.join(names)

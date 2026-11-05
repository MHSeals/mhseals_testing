"""What the TUI shows about parameters: each package's installed ``config/schema.yaml``.

One entry per dotted key: ``type``, ``units``, ``tier``, ``doc`` and an optional ``range``.
A schema.yaml may instead be an index (``fragments: [name, ...]``): its entries then live in
``config/schema/<name>.yaml``, one fragment per config file, every key starting ``<name>.``; a
fragment the index does not list follows the listed ones by name.
"""
import math
from pathlib import Path

from mhseals_testing.health.config import share_dir
from mhseals_testing.strict_yaml import load_strict

PACKAGES = ('mhseals_hardware', 'mhseals_nav', 'mhseals_testing')
TYPES = ('float', 'int', 'bool', 'str', 'list', 'map')   # map: a mapping whose contents are free-form
FIELDS = {'type', 'units', 'range', 'tier', 'doc', 'enum', 'nullable', 'optional'}
KINDS = {'float': (int, float), 'int': int, 'bool': bool, 'str': str, 'list': list, 'map': dict}
TIERS = ('live', 'reconfigure', 'restart', 'new-session')


def _check(key, entry, where):
    def bad(why):
        return ValueError(f'{where}: {key}: {why}')
    if not isinstance(entry, dict) or not {'type', 'units', 'tier', 'doc'} <= entry.keys():
        raise bad('needs type, units, tier and doc')
    if set(entry) - FIELDS:
        raise bad('unknown field')
    if entry['type'] not in TYPES:
        raise bad(f"type must be one of {', '.join(TYPES)}")
    if entry['tier'] not in TIERS:
        raise bad(f"tier must be one of {', '.join(TIERS)}")
    if not isinstance(entry['doc'], str) or not isinstance(entry['units'], str):
        raise bad('doc and units are strings')
    bounds = entry.get('range')
    if bounds is not None and not (
            entry['type'] in ('float', 'int', 'list')
            and isinstance(bounds, list) and len(bounds) == 2
            and all(type(v) in (int, float) and math.isfinite(v) for v in bounds)
            and bounds[0] <= bounds[1]):
        raise bad('range is [lo, hi] on a number or a list of numbers')
    if 'enum' in entry and not (isinstance(entry['enum'], list) and entry['enum']):
        raise bad('enum is a non-empty list of the allowed values')
    for flag in ('nullable', 'optional'):
        if not isinstance(entry.get(flag, False), bool):
            raise bad(f'{flag} is true or false')


def _mapping(path):
    document = load_strict(path.read_text(), path) or {}
    if not isinstance(document, dict):
        raise ValueError(f'{path}: expected a mapping of key to entry')
    return document


def _fragments(index, path):
    """{key: entry} from the fragments an index names, then the rest of config/schema/ by name."""
    if set(index) != {'fragments'} or not isinstance(index['fragments'], list) \
            or not all(isinstance(name, str) for name in index['fragments']):
        raise ValueError(f'{path}: an index holds only fragments: [name, ...]')
    folder = path.parent / 'schema'
    names = list(dict.fromkeys(index['fragments']))
    names += sorted(p.stem for p in folder.glob('*.yaml') if p.stem not in names)
    entries = {}
    for name in names:
        fragment = folder / f'{name}.yaml'
        for key, entry in _mapping(fragment).items():
            if key == name:     # the fragment's own settings: closed
                if not (isinstance(entry, dict) and set(entry) <= {'closed'}
                        and isinstance(entry.get('closed', False), bool)):
                    raise ValueError(f'{fragment}: {key}: the fragment settings are {{closed: true|false}}')
                continue
            if not key.startswith(f'{name}.'):
                raise ValueError(f'{fragment}: {key}: keys of this fragment start with {name}.')
            _check(key, entry, fragment)
            entries[key] = entry
    return entries


def load(package):
    """{key: entry} from the package's installed schema; FileNotFoundError when it has none."""
    path = Path(share_dir(package)) / 'config' / 'schema.yaml'
    document = _mapping(path)
    if 'fragments' in document:
        return _fragments(document, path)
    for key, entry in document.items():
        _check(key, entry, path)
    return document


def rows():
    """(package, key, entry) for every package, and {package: reason} for those not loaded."""
    found, missing = [], {}
    for package in PACKAGES:
        try:
            found += [(package, key, entry) for key, entry in load(package).items()]
        except (OSError, ValueError) as error:
            missing[package] = str(error)
    return found, missing


def text(found, missing):
    lines = [f'{package}: not available ({why})' for package, why in missing.items()]
    for package, key, entry in found:
        span = f" [{entry['range'][0]}, {entry['range'][1]}]" if 'range' in entry else ''
        lines.append(f"{package}  {key}  {entry['type']} {entry['units']}{span}  "
                     f"{entry['tier']}  {entry['doc']}")
    return '\n'.join(lines)


def fragment_of(path):
    """The schema fragment that describes the config file ``path`` (config/schema/<stem>.yaml beside it), or None."""
    path = Path(path)
    fragment = path.parent / 'schema' / f'{path.stem}.yaml'
    return fragment if path.suffix == '.yaml' and fragment.is_file() else None


def _value(where, entry, value):
    """Why ``value`` does not fit ``entry``, or None."""
    if value is None:
        return None if entry.get('nullable') else f"{where}: expected {entry['type']}, got null"
    kind = KINDS[entry['type']]
    if not isinstance(value, kind) or (isinstance(value, bool) and entry['type'] != 'bool'):
        return f"{where}: expected {entry['type']}, got {type(value).__name__} {value!r}"
    if 'enum' in entry and value not in entry['enum']:
        return f"{where}: expected one of {entry['enum']}, got {value!r}"
    if 'range' in entry:
        lo, hi = entry['range']
        numbers = value if isinstance(value, list) else [value]
        for number in numbers:
            if not isinstance(number, (int, float)) or isinstance(number, bool) or not lo <= number <= hi:
                return f"{where}: expected within [{lo}, {hi}], got {number!r}"
    return None


def validate(path, doc):
    """Problems of the config document ``doc`` read from ``path`` against its schema fragment (none: []).

    Each names ``file:key`` and what was expected: a described key of the wrong type, outside its range
    or enum; and, in a fragment marked ``<file>: {closed: true}``, any key it does not describe. Key
    patterns: ``*`` is any mapping key, ``[]`` each item of a list (``health.groups.*.checks.[].id``).
    """
    fragment = fragment_of(path)
    if fragment is None:
        return []
    name = Path(path).stem
    raw = _mapping(fragment)
    closed = bool((raw.get(name) or {}).get('closed'))
    patterns = {}
    for key, entry in raw.items():
        if key != name:
            _check(key, entry, fragment)
            patterns[tuple(key.split('.')[1:])] = entry
    prefixes = {pattern[:n] for pattern in patterns for n in range(len(pattern) + 1)}
    problems = []

    def matches(have, want):
        return len(have) == len(want) and all(w in ('*', h) for h, w in zip(have, want))

    def walk(node, at, shown):
        entry = next((e for pattern, e in patterns.items() if matches(at, pattern)), None)
        where = f"{Path(path).name}:{'.'.join(shown) or '(top)'}"
        if entry is not None:
            problem = _value(where, entry, node)
            if problem:
                problems.append(problem)
            if entry['type'] == 'list' and isinstance(node, list) and any(matches(at + ('[]',), p) for p in prefixes):
                for index, item in enumerate(node):
                    walk(item, at + ('[]',), shown + [str(index)])
            return
        if isinstance(node, dict):
            for key, value in node.items():
                child = at + (str(key),)
                if any(matches(child, prefix) for prefix in prefixes):
                    walk(value, child, shown + [str(key)])
                elif closed:
                    problems.append(f"{Path(path).name}:{'.'.join(shown + [str(key)])}: unknown key "
                                    f"(not in {fragment.parent.name}/{fragment.name})")
        elif isinstance(node, list) and any(matches(at + ('[]',), prefix) for prefix in prefixes):
            for index, item in enumerate(node):
                walk(item, at + ('[]',), shown + [str(index)])
        elif closed and at:
            problems.append(f'{where}: not described (not in {fragment.parent.name}/{fragment.name})')
    walk(doc, (), [])
    return problems

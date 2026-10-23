"""Load health checks from YAML: each-expansion, profile templating, user override, lint.

Shipped files live in ``config/health/*.yaml``; ``~/.config/mhseals/health.yaml``
may ``disable`` ids (globs allowed), ``override`` fields of one id, and
override ``probes`` settings. See health/registry.py for the kinds.
"""

import copy
import dataclasses
from fnmatch import fnmatchcase
from pathlib import Path
import re

import yaml

from mhseals_testing.health import registry
from mhseals_testing.health.model import MISSING, SEVERITIES, CheckSpec

USER_FILE = Path.home() / '.config' / 'mhseals' / 'health.yaml'
ANCHORS = ('rose', 'thrusters', 'header.deploy', 'odometry')    # plus sensor:<name>
SPEC_KEYS = {f.name for f in dataclasses.fields(CheckSpec)} - {'params'}
SPEC_TYPES = {'enabled': bool, 'raise_after': (int, float), 'clear_after': (int, float),
              'title': str, 'group': str, 'message': str, 'hint': str}
RESULT_FIELDS = ('value', 'detail', 'label', 'status')           # known to message templates
ID_PATTERN = re.compile(r'[a-z0-9_]+(\.[a-z0-9_]+)*')
FIELD = re.compile(r'\{(\w+)\}')


class ConfigError(ValueError):
    """A health file that cannot be loaded."""


def _health_dir():
    """config/health in the source tree, else the installed share directory."""
    source = Path(__file__).resolve().parents[2] / 'config' / 'health'
    if source.is_dir():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('mhseals_testing')) / 'config' / 'health'
    except (ImportError, KeyError):
        return source


HEALTH_DIR = _health_dir()


@dataclasses.dataclass(frozen=True)
class HealthConfig:
    specs: tuple
    probes: dict
    unmatched: tuple = ()    # override ids and disable patterns that named no check


def render(template, **fields):
    """Fill ``{name}`` from ``fields``; unknown names are left as written."""
    return FIELD.sub(lambda m: str(fields[m[1]]) if m[1] in fields else m[0], template)


RUNTIME_KEYS = ('message', 'hint')


def _fill(value, context):
    """``render`` applied through nested lists and dicts, leaving other types alone."""
    if isinstance(value, str):
        return render(value, **context)
    if isinstance(value, list):
        return [_fill(item, context) for item in value]
    if isinstance(value, dict):
        return {key: _fill(item, context) for key, item in value.items()}
    return value


def profile_context(profile):
    """Scalar profile fields usable as ``{command_topic}`` and the like."""
    if profile is None:
        return {}
    return {key: value for key, value in vars(profile).items()
            if isinstance(value, (str, int, float)) and not isinstance(value, bool)}


def _strings(value):
    """Every string inside nested lists and dicts."""
    if isinstance(value, str):
        yield value
    elif isinstance(value, dict):
        for item in value.values():
            yield from _strings(item)
    elif isinstance(value, list):
        for item in value:
            yield from _strings(item)


def _param_names(kind):
    found = registry.KINDS.get(kind)
    return set(found.params.__dataclass_fields__) if found else set()


def expand(entry, defaults, group, context):
    """One YAML entry as raw check dicts: defaults merged, one per ``each`` row, templated."""
    entry = {**defaults, **{'group': group}, **entry}
    rows = entry.pop('each', None)
    if rows is None:
        rows = [{}]                # no each: one check; an empty each list: none
    out = []
    for row in rows:
        merged = _fill({**entry, **row}, {**context, **row})
        # message and hint are runtime templates: only each-row keys fill at load time
        for key in RUNTIME_KEYS:
            if key in entry or key in row:
                merged[key] = _fill({**entry, **row}[key], row)
        # row keys that are neither spec keys nor kind params only exist to fill templates
        known = SPEC_KEYS | _param_names(merged.get('kind'))
        for key in row:
            if key not in known and not any(f'{{{key}}}' in text for text in _strings(entry)):
                raise ConfigError(f'{merged.get("id", "?")}: each key {key!r} is not a field and fills no template')
        out.append({key: value for key, value in merged.items() if key not in row or key in known})
    return out


def _spec(raw):
    """Validate one raw dict into a CheckSpec; ConfigError names the check."""
    where = raw.get('id', '?')
    if 'id' not in raw:
        raise ConfigError(f'check without an id (kind {raw.get("kind")!r})')
    raw = copy.deepcopy(raw)
    name = raw.get('kind')
    if name not in registry.KINDS:
        raise ConfigError(f'{where}: unknown kind {name!r}')
    fields = {key: raw.pop(key) for key in list(raw) if key in SPEC_KEYS}
    for key, allowed in (('severity', SEVERITIES), ('missing', MISSING)):
        if key in fields and fields[key] not in allowed:
            raise ConfigError(f'{where}: {key} {fields[key]!r} is not one of {allowed}')
    for key, value in fields.items():
        if key in SPEC_TYPES and (isinstance(value, bool) != (SPEC_TYPES[key] is bool)
                                  or not isinstance(value, SPEC_TYPES[key])) \
                and not (key == 'raise_after' and value is None):
            raise ConfigError(f'{where}: {key} {value!r} must be {SPEC_TYPES[key].__name__ if SPEC_TYPES[key] is not (int, float) else "a number"}')
    if not isinstance(fields.get('when', {}), dict):
        raise ConfigError(f'{where}: when must be a mapping')
    try:
        params = registry.build_params(name, raw)
    except (ValueError, TypeError) as error:
        raise ConfigError(f'{where}: {error}') from None
    for key in ('show', 'gates', 'profiles'):
        value = fields.get(key, ())
        fields[key] = (value,) if isinstance(value, str) else tuple(value)
    fields.setdefault('title', fields['id'].replace('.', ' '))
    try:
        return CheckSpec(params=params, **fields)
    except TypeError as error:
        raise ConfigError(f'{where}: {error}') from None


def _read(path):
    try:
        data = yaml.safe_load(Path(path).read_text()) or {}
    except (OSError, yaml.YAMLError) as error:
        raise ConfigError(f'{path}: {error}') from None
    if not isinstance(data, dict):
        raise ConfigError(f'{path}: expected a mapping')
    return data


def _section(data, key, kind, path):
    """``data[key]`` as a ``kind`` (list or dict), empty when absent; ConfigError otherwise."""
    value = data.get(key)
    if value is None:
        return kind()
    if not isinstance(value, kind):
        raise ConfigError(f'{path}: {key} must be a {"list" if kind is list else "mapping"}')
    return value


def _merge(base, extra):
    """Deep-merge ``extra`` over ``base`` for the probes section."""
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value
    return base


def load(directory=None, profile=None, user=None):
    """Specs and probe settings from every ``*.yaml`` in ``directory`` plus the user file.

    ``profile`` (a PlantProfile) fills templates and drops checks whose
    ``profiles`` list excludes it. ``user`` defaults to USER_FILE when ``directory`` does.
    """
    directory = Path(directory or HEALTH_DIR)
    if user is None and directory == HEALTH_DIR:
        user = USER_FILE
    registry.discover()
    context = profile_context(profile)
    raws, probes = [], {}
    for path in sorted(directory.glob('*.yaml')):
        data = _read(path)
        _merge(probes, _section(data, 'probes', dict, path))
        defaults = _section(data, 'defaults', dict, path)
        for entry in _section(data, 'checks', list, path):
            if not isinstance(entry, dict):
                raise ConfigError(f'{path}: every check must be a mapping, got {entry!r}')
            rows = entry.get('each')
            if rows is not None and not (isinstance(rows, list) and all(isinstance(r, dict) for r in rows)):
                raise ConfigError(f'{path}: {entry.get("id", "?")}: each must be a list of mappings')
            raws += expand(entry, defaults, data.get('group', path.stem), context)
    user = Path(user) if user and Path(user).is_file() else None
    override = _read(user) if user else {}
    _merge(probes, _section(override, 'probes', dict, user))
    changes, disabled = _section(override, 'override', dict, user), _section(override, 'disable', list, user)
    for key, change in changes.items():
        if not isinstance(change, dict):
            raise ConfigError(f'{user}: override {key} must be a mapping')
    ids = [str(raw.get('id')) for raw in raws]
    unmatched = tuple(
        [f'override {key}' for key in changes if key not in ids]
        + [f'disable {pattern}' for pattern in disabled
           if not any(fnmatchcase(i, str(pattern)) for i in ids)])
    specs = []
    for raw in raws:
        raw = {**raw, **changes.get(raw.get('id'), {})}
        names = raw.get('profiles') or ()
        names = (names,) if isinstance(names, str) else tuple(names)
        if profile is not None and names and profile.name not in names:
            continue
        if any(fnmatchcase(str(raw.get('id')), pattern) for pattern in disabled):
            raw['enabled'] = False
        specs.append(_spec(raw))
    return HealthConfig(tuple(specs), probes, unmatched)


def _regex_fields(params):
    return [getattr(params, name) for name in ('pattern', 'match') if hasattr(params, name)]


def lint(specs, procedures=(), unmatched=()):
    """Problems in loaded specs as strings; empty when the files are sound.

    ``procedures`` are the real procedure keys that ``gates`` may name;
    ``unmatched`` is ``HealthConfig.unmatched`` (user entries naming no check).
    """
    problems, seen = [f'user file: {entry} matches no check' for entry in unmatched], set()
    for spec in specs:
        add = lambda text, spec=spec: problems.append(f'{spec.id}: {text}')
        if not ID_PATTERN.fullmatch(spec.id):
            add('id must be dotted lowercase words (unresolved template?)')
        if spec.id in seen:
            add('duplicate id')
        seen.add(spec.id)
        if spec.severity not in SEVERITIES:
            add(f'severity {spec.severity!r} is not one of {SEVERITIES}')
        if spec.missing not in MISSING:
            add(f'missing {spec.missing!r} is not one of {MISSING}')
        for anchor in spec.show:
            if anchor not in ANCHORS and not re.fullmatch(r'sensor:\S+', anchor):
                add(f'unknown anchor {anchor!r}')
        for key in spec.gates:
            if key != 'motion' and key not in procedures:
                add(f'gate {key!r} is not a procedure key')
        for pattern in _regex_fields(spec.params):
            try:
                re.compile(pattern)
            except re.error as error:
                add(f'bad regex {pattern!r}: {error}')
        for text in _strings(vars(spec.params)):
            for name in FIELD.findall(text):
                if not name.isdigit():    # {3} is a regex quantifier
                    add(f'unresolved template {{{name}}} in params')
        reference = getattr(spec.params, 'reference', None)
        if reference and not Path(reference).expanduser().exists():
            add(f'reference {reference} does not exist')
        known = {*RESULT_FIELDS, *vars(spec.params), *registry.KINDS[spec.kind].fields}
        for text in (spec.message, spec.hint):
            for chunk in re.findall(r'\{([^{}]*)\}', text):
                if not re.fullmatch(r'\w+', chunk):
                    add(f'template field {{{chunk}}} may not index or call')
                elif chunk not in known and not _named_group(spec, chunk):
                    add(f'template field {{{chunk}}} is unknown')
    return problems


def _named_group(spec, name):
    """True when a log pattern defines ``(?P<name>...)``."""
    return any(f'(?P<{name}>' in pattern for pattern in _regex_fields(spec.params))

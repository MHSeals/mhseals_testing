"""The hardware layer ``boat_agent set`` writes: the operator hardware YAML over mhseals_hardware's defaults. ROS-free.

A key is any top-level key of the defaults (``manual_amplitude``, ``decoupling``), optionally ``hardware.``-prefixed,
or a dotted path into one: a mapping's leaf (``command_mux.ramp``, ``controller.kp``) or a matrix row by the row
names config/history/fields.yaml gives that matrix (``decoupling.yaw``). Values are YAML (``[-0.0375, 0.1, 1.0]``).
``plan`` validates the change against the whole config as the driver would load it, and returns the overrides to
write: only the changed top-level key, a mapping merged with the file's own overrides, never a full copy.
"""

import copy
from pathlib import Path

import yaml

PREFIX = 'hardware.'


def defaults():
    from mhseals_hardware.configuration import load_defaults
    return load_defaults()


def config_file(path=None):
    """The operator hardware YAML this TUI writes (path, else $MHSEALS_HARDWARE_CONFIG, else the user's)."""
    from mhseals_hardware.configuration import config_path
    return config_path(path)


def split(key, known=None):
    """``(top, rest)`` when ``key`` names the hardware layer, else None. ``rest`` is '' for a top-level key."""
    known = defaults() if known is None else known
    bare = key[len(PREFIX):] if key.startswith(PREFIX) else key
    top, _, rest = bare.partition('.')
    return (top, rest) if top in known else None


def matrix_rows(top, fields=None):
    """The row names of a matrix key (fields.yaml ``kind: matrix``, ``rows: axes``), else ()."""
    if fields is None:
        from mhseals_testing.calibration import load_fields
        fields = load_fields()
    spec = ((fields.get('hardware') or {}).get('fields') or {}).get(top) or {}
    if spec.get('kind') != 'matrix':
        return ()
    return tuple(fields.get(spec.get('rows'), ()) or ()), tuple(fields.get(spec.get('cols'), ()) or ())


def parse(value):
    """A typed value from the command line's text (YAML: numbers, lists, on/off, null)."""
    if not isinstance(value, str):
        return value
    try:
        return yaml.safe_load(value)
    except yaml.YAMLError as error:
        raise ValueError(f'{value!r} is not a YAML value: {error}') from None


def overrides(path=None):
    """The operator file's own keys ({} when it does not exist)."""
    from mhseals_hardware.configuration import load_strict_yaml
    target = config_file(path)
    return load_strict_yaml(target) if target.exists() else {}


def current(key, path=None, fields=None):
    """The effective value of ``key`` (defaults overlaid with the operator file); KeyError when unknown."""
    from mhseals_hardware.configuration import load_config
    top, rest = split(key) or (None, None)
    if top is None:
        raise KeyError(key)
    config = load_config(path)
    return _leaf(config, top, rest, fields)


def _leaf(config, top, rest, fields):
    value = config[top]
    if not rest:
        return value
    if isinstance(value, dict):
        if rest not in value:
            raise KeyError(f'{top}.{rest}')
        return value[rest]
    rows, cols = matrix_rows(top, fields) or ((), ())
    if rest not in rows:
        raise KeyError(f'{top}.{rest}')
    width = len(cols)
    i = rows.index(rest) * width
    return list(value[i:i + width])


def plan(key, value, path=None, fields=None):
    """``{'top', 'old', 'new', 'change', 'file'}`` for ``key=value``: ``change`` is the ``update_config`` overrides.
    ValueError names what is wrong (unknown key or row, a wrong shape, a value the driver would refuse)."""
    from mhseals_hardware.configuration import load_config, validate_config
    known = defaults()
    found = split(key, known)
    if found is None:
        raise ValueError(f'unknown hardware key {key!r}')
    top, rest = found
    config = load_config(path)
    try:
        old = _leaf(config, top, rest, fields)
    except KeyError:
        raise ValueError(f'{key}: {top} has no {rest!r} (known: {_children(config, top, fields)})') from None
    new = parse(value)
    if isinstance(known[top], float) and type(new) is int:
        new = float(new)
    if not rest:
        written = new
    elif isinstance(config[top], dict):
        written = dict(overrides(path).get(top) or {}, **{rest: new})
    else:
        rows, cols = matrix_rows(top, fields)
        if not isinstance(new, list) or len(new) != len(cols):
            raise ValueError(f'{key}: a row is {len(cols)} numbers ({", ".join(cols)}), got {new!r}')
        written = [float(v) for v in config[top]]
        i = rows.index(rest) * len(cols)
        written[i:i + len(cols)] = [float(v) for v in new]
    candidate = copy.deepcopy(config)
    candidate[top] = dict(config[top], **written) if isinstance(config[top], dict) else written
    try:
        validate_config(candidate)
    except (ValueError, TypeError, KeyError) as error:
        raise ValueError(f'{key}: {error}') from None
    return {'top': top, 'old': old, 'new': new, 'change': {top: written}, 'file': str(config_file(path))}


def _children(config, top, fields):
    value = config[top]
    if isinstance(value, dict):
        return ', '.join(sorted(value))
    rows = (matrix_rows(top, fields) or ((), ()))[0]
    return ', '.join(rows) or 'nothing (set the whole value)'


def snapshot(path, directory, who, clock_ns):
    """Copy the operator file before a write (None when it does not exist yet); returns the copy's path."""
    source = config_file(path)
    if not source.exists():
        return None
    target = Path(directory) / f'{source.stem}.before-{who}-{clock_ns}{source.suffix}'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(source.read_text())
    return str(target)

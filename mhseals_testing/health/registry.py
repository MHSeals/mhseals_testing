"""Check kinds: pure functions from facts to a Result, registered by name.

A kind never does I/O. Its YAML keys are the fields of a frozen params
dataclass, so unknown or missing keys fail when the config loads.
"""

import dataclasses
import importlib
import pkgutil


KINDS = {}


@dataclasses.dataclass(frozen=True)
class Kind:
    name: str
    params: type
    needs: tuple                 # probes whose facts the kind reads
    watch: object                # params -> {probe: [targets]} the probe must collect
    evaluate: object             # (params, facts) -> Result
    fields: tuple = ()           # result.data keys, usable in message templates


def kind(name, params, needs, watch=lambda p: {}, fields=()):
    """Register ``evaluate(params, facts) -> Result`` as check kind ``name``."""
    def register(evaluate):
        if name in KINDS:
            raise ValueError(f'health kind {name!r} registered twice')
        KINDS[name] = Kind(name, params, tuple(needs), watch, evaluate, tuple(fields))
        return evaluate
    return register


def build_params(name, raw):
    """The kind's params instance from a YAML mapping; ValueError names the problem."""
    cls = KINDS[name].params
    fields = {f.name: f for f in dataclasses.fields(cls)}
    unknown = sorted(set(raw) - set(fields))
    if unknown:
        raise ValueError(f'{name} has no parameter {", ".join(unknown)} (known: {", ".join(fields)})')
    required = [n for n, f in fields.items()
                if f.default is dataclasses.MISSING and f.default_factory is dataclasses.MISSING]
    missing = [n for n in required if n not in raw]
    if missing:
        raise ValueError(f'{name} needs {", ".join(missing)}')
    for key, value in raw.items():
        wanted = fields[key].type
        if wanted in (str, int, float, bool) and not _fits(value, wanted):
            raise ValueError(f'{name} parameter {key} {value!r} must be {wanted.__name__}')
    return cls(**raw)


def _fits(value, wanted):
    """True when ``value`` is a ``wanted`` scalar (ints count as floats, bools only as bools)."""
    if wanted is float:
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    return isinstance(value, wanted) and (wanted is bool or not isinstance(value, bool))


def discover():
    """Import every module in health/kinds so its kinds register themselves."""
    try:
        package = importlib.import_module('mhseals_testing.health.kinds')
    except ModuleNotFoundError as error:
        if error.name != 'mhseals_testing.health.kinds':
            raise
        return
    for info in pkgutil.iter_modules(package.__path__):
        importlib.import_module(f'{package.__name__}.{info.name}')
    return KINDS

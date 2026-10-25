"""What the TUI shows about parameters: each package's installed ``config/schema.yaml``.

One entry per dotted key: ``type``, ``units``, ``tier``, ``doc`` and an optional ``range``.
"""
import math
from pathlib import Path

from mhseals_testing.health.config import share_dir
from mhseals_testing.strict_yaml import load_strict

PACKAGES = ('mhseals_hardware', 'mhseals_nav', 'mhseals_testing')
TYPES = ('float', 'int', 'bool', 'str', 'list')
TIERS = ('live', 'reconfigure', 'restart', 'new-session')


def _check(key, entry, where):
    def bad(why):
        return ValueError(f'{where}: {key}: {why}')
    if not isinstance(entry, dict) or not {'type', 'tier', 'doc'} <= entry.keys():
        raise bad('needs type, tier and doc')
    if set(entry) - {'type', 'units', 'range', 'tier', 'doc'}:
        raise bad('unknown field')
    if entry['type'] not in TYPES:
        raise bad(f"type must be one of {', '.join(TYPES)}")
    if entry['tier'] not in TIERS:
        raise bad(f"tier must be one of {', '.join(TIERS)}")
    if not isinstance(entry['doc'], str) or not isinstance(entry.get('units', ''), str):
        raise bad('doc and units are strings')
    bounds = entry.get('range')
    if bounds is not None and not (
            entry['type'] in ('float', 'int', 'list')
            and isinstance(bounds, list) and len(bounds) == 2
            and all(type(v) in (int, float) and math.isfinite(v) for v in bounds)
            and bounds[0] <= bounds[1]):
        raise bad('range is [lo, hi] on a number or a list of numbers')


def load(package):
    """{key: entry} from the package's installed schema; FileNotFoundError when it has none."""
    path = Path(share_dir(package)) / 'config' / 'schema.yaml'
    document = load_strict(path.read_text(), path) or {}
    if not isinstance(document, dict):
        raise ValueError(f'{path}: expected a mapping of key to entry')
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
        lines.append(f"{package}  {key}  {entry['type']} {entry.get('units', '')}{span}  "
                     f"{entry['tier']}  {entry['doc']}")
    return '\n'.join(lines)

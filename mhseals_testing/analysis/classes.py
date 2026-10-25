"""Map object classes from config/minimap.yaml; no ROS imports.

Each detector or truth label resolves to a glyph, an ASCII letter, a palette
token, a radius and a legend name. An unlisted label resolves to `unknown`.
"""

from collections import namedtuple
from pathlib import Path
from unicodedata import east_asian_width

from mhseals_testing.strict_yaml import load_strict

import yaml

Symbol = namedtuple('Symbol', 'label glyph ascii colour radius name')


def _default_path():
    """config/minimap.yaml in the source tree, else the installed share directory."""
    source = Path(__file__).resolve().parent.parent.parent / 'config' / 'minimap.yaml'
    if source.is_file():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('mhseals_testing')) / 'config' / 'minimap.yaml'
    except (ImportError, KeyError):  # PackageNotFoundError is a KeyError
        return source


FIELDS = ('glyph', 'ascii', 'colour', 'radius', 'name')


def _symbol(label, entry):
    missing = [f for f in FIELDS if not isinstance(entry, dict) or f not in entry]
    if missing:
        raise ValueError(f'class {label!r} is missing {", ".join(missing)}')
    return Symbol(label, entry['glyph'], entry['ascii'], entry['colour'],
                  float(entry['radius']), str(entry['name']))


def check(symbols):
    """Raise ValueError unless each symbol is well formed and glyphs and ASCII codes are unique.

    Values stay raw from YAML so an unquoted `on` or `no` (a bool) is rejected, not stringified.
    """
    for symbol in symbols:
        for field in ('glyph', 'ascii'):
            value = getattr(symbol, field)
            if not isinstance(value, str) or len(value) != 1 or not value.isprintable():
                raise ValueError(f'{symbol.label}: {field} {value!r} must be one printable character (quote it)')
        if not symbol.ascii.isascii():
            raise ValueError(f'{symbol.label}: ascii {symbol.ascii!r} must be ASCII')
        if east_asian_width(symbol.glyph) in 'WF':
            raise ValueError(f'{symbol.label}: glyph {symbol.glyph!r} is wider than one cell')
        if symbol.radius <= 0:
            raise ValueError(f'{symbol.label}: radius must be positive')
    for field in ('glyph', 'ascii'):
        seen = {}
        for symbol in symbols:
            other = seen.setdefault(getattr(symbol, field), symbol.label)
            if other != symbol.label:
                raise ValueError(f'{field} {getattr(symbol, field)!r} used by {other} and {symbol.label}')


TRACKS = {'arrow_s': 1.0, 'predict_s': 3.0, 'unknown_strength': 50, 'prior_strength': 50}
FOV = {'show': True, 'colour': 'fov'}   # colour: the theme.yaml token of the camera cone


def _section(name, given, defaults):
    """A minimap.yaml settings section over its defaults; unknown keys and wrong types raise."""
    given = given or {}
    if not isinstance(given, dict):
        raise ValueError(f'{name} must be a mapping')
    out = dict(defaults)
    for key, value in given.items():
        if key not in defaults:
            raise ValueError(f'{name}.{key} is not a setting')
        if type(value) is not type(defaults[key]) and not (isinstance(defaults[key], float) and type(value) is int):
            raise ValueError(f'{name}.{key} must be a {type(defaults[key]).__name__}')
        out[key] = value
    return out


class ClassTable:
    """The class table with an `unknown` fallback, plus the track and fov drawing settings."""

    def __init__(self, classes, unknown, tokens=None, tracks=None, fov=None):
        """`tokens`, when given, is the set of theme.yaml token names every `colour` must be in."""
        self.tracks, self.fov = _section('tracks', tracks, TRACKS), _section('fov', fov, FOV)
        self.classes = {label: _symbol(label, entry) for label, entry in classes.items()}
        self.unknown = _symbol('unknown', unknown)
        check([*self.classes.values(), self.unknown])
        if tokens is not None:
            for symbol in (*self.classes.values(), self.unknown):
                if symbol.colour not in tokens:
                    raise ValueError(f'{symbol.label}: colour {symbol.colour!r} is not a theme.yaml token')
            if self.fov['colour'] not in tokens:
                raise ValueError(f"fov: colour {self.fov['colour']!r} is not a theme.yaml token")

    def symbol(self, label):
        return self.classes.get(label, self.unknown)

    def __contains__(self, label):
        return label in self.classes

    def __iter__(self):
        """Classes in table order, without the fallback."""
        return iter(self.classes.values())


def load_classes(path=None, tokens=None):
    """Load the class table; any malformed content raises one ValueError naming the file.

    Colours are checked against the theme.yaml tokens (`tokens` overrides, for tests).
    """
    path = Path(path or _default_path())
    try:
        data = load_strict(path.read_text())
        if not isinstance(data, dict):
            raise ValueError('expected a mapping with classes and unknown')
        for key in ('classes', 'unknown'):
            if not data.get(key):
                raise ValueError(f'missing {key}')
        if tokens is None:
            from mhseals_testing.ui.palette import load_theme
            tokens = load_theme(overlay=False)[0].tokens
        return ClassTable(data['classes'], data['unknown'], tokens, data.get('tracks'), data.get('fov'))
    except (ValueError, yaml.YAMLError, AttributeError) as error:
        raise ValueError(f'{path}: {error}') from error

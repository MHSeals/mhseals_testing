"""Map object classes from config/minimap.yaml; no ROS imports.

Each detector or truth label resolves to a glyph, an ASCII letter, a palette
token, a radius and a legend name. An unlisted label resolves to `unknown`.
"""

from collections import namedtuple
from pathlib import Path
from unicodedata import east_asian_width

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


def _symbol(label, entry):
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


class ClassTable:
    """The class table with an `unknown` fallback."""

    def __init__(self, classes, unknown):
        self.classes = {label: _symbol(label, entry) for label, entry in classes.items()}
        self.unknown = _symbol('unknown', unknown)
        check([*self.classes.values(), self.unknown])

    def symbol(self, label):
        return self.classes.get(label, self.unknown)

    def __contains__(self, label):
        return label in self.classes

    def __iter__(self):
        """Classes in table order, without the fallback."""
        return iter(self.classes.values())


def load_classes(path=None):
    data = yaml.safe_load(Path(path or _default_path()).read_text())
    return ClassTable(data['classes'], data['unknown'])

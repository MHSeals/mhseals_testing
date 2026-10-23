"""Map object classes from config/minimap.yaml; no ROS imports.

Each detector or truth label resolves to a glyph, an ASCII letter, a palette
token, a radius and a legend name. An unlisted label resolves to `unknown`.
"""

from collections import namedtuple
from pathlib import Path

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
    return Symbol(label, str(entry['glyph']), str(entry['ascii']), entry['colour'],
                  float(entry['radius']), str(entry['name']))


def check(symbols):
    """Raise ValueError unless glyphs, ASCII letters and their pairs are unique."""
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

"""The image protocol interface: what a terminal image backend implements, and the backends there are.

A backend is a module of this package with a ``BACKEND`` object (a new protocol is one new file):

    name                  the tier it draws (``kitty``, ``sixel``, ``text``), as theme.yaml and the status show it
    rank                  the order `backends()` offers it to the probe; the first that accepts wins
    probe(setting, caps, env, colours) -> bool
                          whether it serves this terminal: theme.yaml ``graphics``, the capability probe's
                          ``caps`` (probe.py), the environment and Rich's colour system
    place(tile, r, c) -> (text, style)
                          the terminal cell that shows cell (r, c) of a pooled tile
    clear(ids) -> str     the escapes that free the images of these pool ids
    cost(tile) -> str     the escapes that paint a tile; their length is what it costs the link

tmux is a transport, not a backend: tmux.py says which tier it carries and wraps kitty's escapes.
"""

from importlib import import_module
from pathlib import Path
from typing import Protocol

import numpy as np

_found = []


class Backend(Protocol):
    name: str
    rank: int

    def probe(self, setting, caps, env, colours): ...

    def place(self, tile, r, c): ...

    def clear(self, ids): ...

    def cost(self, tile): ...


def backends():
    """Every backend of the package, by rank (found once)."""
    if not _found:
        for path in sorted(Path(__file__).parent.glob('*.py')):
            if path.stem not in ('__init__', 'protocol', 'probe', 'tmux'):
                backend = getattr(import_module(f'{__package__}.{path.stem}'), 'BACKEND', None)
                if backend is not None:
                    _found.append(backend)
        _found.sort(key=lambda b: b.rank)
    return list(_found)


def by_name(name):
    return next(b for b in backends() if b.name == name)


def rgba(px, palette):
    lut = np.array([(r, g, b, 255) for r, g, b in palette], np.uint8)
    lut[0, 3] = 0
    return lut[px]

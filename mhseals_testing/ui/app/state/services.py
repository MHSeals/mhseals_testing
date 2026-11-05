"""Narrow facades on App (``app.services``): how one area reads another's state without writing it.

``app.services.bags.scan`` reads the Bags tab's container; a write through a facade raises, so the owning
mixin (core.py OWNERS) stays the one writer. View reads the Thrusters panel's edit this way (``app.services.manual.draft``).
"""

from mhseals_testing.ui.app.state import state_of


class ReadOnly:
    """The fields of one state container, read-only."""

    __slots__ = ('_app', '_group')

    def __init__(self, app, group):
        object.__setattr__(self, '_app', app)
        object.__setattr__(self, '_group', group)

    def __getattr__(self, name):
        return getattr(getattr(state_of(self._app), self._group), name)

    def __setattr__(self, name, value):
        raise AttributeError(f'{self._group}.{name} is written only by its owner (ui/app/core.py OWNERS)')


class Services:
    """``bags``, ``manual``, ``holds``, ``arm`` and ``procedures``: read-only views of those containers."""

    GROUPS = ('bags', 'manual', 'holds', 'arm', 'procedures')

    def __init__(self, app):
        for group in self.GROUPS:
            setattr(self, group, ReadOnly(app, group))

"""Narrow facades on App (``app.services``): how one area reads another's state without writing it.

``app.services.bags.scan`` reads the Bags tab's container; a write through a facade raises, so the container's
owner (the mixin it is named for, ui/app/state) stays the one writer.
"""

from dataclasses import fields

from mhseals_testing.ui.app.state import AppState


class ReadOnly:
    """The fields of one state container, read-only."""

    __slots__ = ('_app', '_group')

    def __init__(self, app, group):
        object.__setattr__(self, '_app', app)
        object.__setattr__(self, '_group', group)

    def __getattr__(self, name):
        return getattr(getattr(self._app.state, self._group), name)

    def __setattr__(self, name, value):
        raise AttributeError(f'app.state.{self._group}.{name} is written only by its owner, the {self._group} '
                             f'mixin (ui/app/state); ask its method')


class Services:
    """One read-only view per container in AppState: ``bags``, ``manual``, ``arm``, ..."""

    GROUPS = tuple(f.name for f in fields(AppState))

    def __init__(self, app):
        for group in self.GROUPS:
            setattr(self, group, ReadOnly(app, group))

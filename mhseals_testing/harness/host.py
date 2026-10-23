"""The adapter between a procedure and whatever runs it (the TUI today, headless later)."""

from typing import Protocol

from mhseals_testing.procedures import Context


class Host(Protocol):
    """What a procedure needs from its host; ``App`` implements it."""

    def prompt(self, message, title='Ready'): ...
    def choose(self, title, options, shortcuts=None, message=None, danger=False, rows=None): ...
    def motion(self, name, **details): ...
    def supervise(self, name, **details): ...
    def observe(self, name, **details): ...
    def record(self, result): ...


def context(host, profile, monitor, settings, stale_after, services):
    """The procedure ``Context`` for ``host``; the TUI and the command line build it alike."""
    return Context(
        profile=profile, monitor=monitor, settings=settings, motion=host.motion,
        prompt=host.prompt, choose=host.choose, record=host.record, supervise=host.supervise,
        observe=host.observe, stale_after=stale_after, services=services)

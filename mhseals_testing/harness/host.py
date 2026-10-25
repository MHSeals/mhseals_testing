"""The adapter between a procedure and whatever runs it."""

from mhseals_testing.procedures import Context


def context(host, profile, monitor, settings, stale_after, services):
    """The procedure ``Context`` for ``host``; the TUI and the command line build it alike."""
    return Context(
        profile=profile, monitor=monitor, settings=settings, motion=host.motion,
        prompt=host.prompt, choose=host.choose, record=host.record, supervise=host.supervise,
        observe=host.observe, stale_after=stale_after, services=services)

"""Neutral models: the plain state and parses that ros/, health/ and ui/ all read.

Stdlib only, and nothing from ros/, health/ or ui/ (test_import_boundaries), so a feed or a probe can use
them without the UI, and the UI imports them instead of the other way round.
"""


class Cancelled(RuntimeError):
    """The operator cancelled a prompt (Q or Escape), or an action refuses and says why (its message)."""

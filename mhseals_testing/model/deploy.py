"""How far the checkouts are from origin, as data: the header's deploy item before any glyph or colour.

``summary`` folds the git_state probe's ``Result.data`` dicts; ui/deploy.py renders it. Stdlib only.
"""

from dataclasses import dataclass

STALE = 3600.0           # a fetch older than this shows its age
COUNTS = ('ahead', 'behind', 'dirty', 'untracked')


@dataclass(frozen=True)
class Deploy:
    counts: dict              # {ahead, behind, dirty, untracked: total over the repos}
    diverged: bool            # some repo is both ahead and behind
    stale_age: float | None   # the oldest fetch's age (s) when older than ``stale``, else None
    code_newer: bool          # the running code is older than the checkout


def summary(states, stale=STALE, code_newer=False):
    """The Deploy of the git_state ``Result.data`` dicts, or None without states (the probe has not run)."""
    states = [s for s in states if s]
    if not states:
        return None
    ages = [s['fetch_age'] for s in states if s.get('fetch_age') is not None]
    return Deploy(counts={name: sum(s.get(name, 0) for s in states) for name in COUNTS},
                  diverged=any(s.get('diverged') for s in states),
                  stale_age=max(ages) if ages and max(ages) > stale else None,
                  code_newer=bool(code_newer))


def age(seconds):
    """``4d``, ``3h``, ``12m`` or ``40s``: the largest whole unit."""
    for unit, size in (('d', 86400), ('h', 3600), ('m', 60)):
        if seconds >= size:
            return f'{int(seconds // size)}{unit}'
    return f'{int(seconds)}s'

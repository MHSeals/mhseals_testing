"""git_state: is this checkout the commit origin has, and is it clean?

Each of ``behind``, ``ahead``, ``dirty`` and ``untracked`` maps a status to the
count that raises to it (``{warn: 1}``); the worst band reached wins. A repo
that is both ahead and behind has diverged and is at least warn.
"""

from dataclasses import dataclass, field

from mhseals_testing.health.model import STATUSES, Result
from mhseals_testing.health.registry import kind

COUNTS = ('behind', 'ahead', 'dirty', 'untracked')


@dataclass(frozen=True)
class Params:
    repo: str
    path: str
    behind: dict = field(default_factory=lambda: {'warn': 1})
    ahead: dict = field(default_factory=lambda: {'info': 1})
    dirty: dict = field(default_factory=lambda: {'warn': 1})
    untracked: dict = field(default_factory=lambda: {'warn': 1})
    fetch: float | None = None       # seconds between fetches; None: the probe's setting


def _band(bands, count):
    """The worst status in ``bands`` whose threshold ``count`` reaches."""
    reached = [status for status, limit in bands.items() if count >= limit]
    return max(reached, key=STATUSES.index, default='ok')


def watch(params):
    return {'git': [(params.repo, params.path, params.fetch)]}


@kind('git_state', Params, needs=('git',), watch=watch,
      fields=(*COUNTS, 'diverged', 'branch', 'hash', 'fetch_age'))
def evaluate(params, facts):
    fact = facts.get(f'git.{params.repo}')
    state = fact.value if fact else None
    if not isinstance(state, dict) or 'error' in state:
        reason = state.get('error') if isinstance(state, dict) else f'no git.{params.repo} fact'
        return Result('unknown', detail=f'{params.repo}: {reason}')
    counts = {name: state[name] for name in COUNTS}
    diverged = counts['ahead'] > 0 and counts['behind'] > 0
    status = max((_band(getattr(params, name), counts[name]) for name in COUNTS),
                 key=STATUSES.index)
    if diverged and status in ('ok', 'info'):
        status = 'warn'
    parts = [f'{label}{counts[name]}' for label, name in
             (('behind ', 'behind'), ('ahead ', 'ahead'), ('modified ', 'dirty'),
              ('untracked ', 'untracked')) if counts[name]]
    detail = f'{params.repo} {state["branch"]}@{state["hash"]}: ' + (', '.join(parts) or 'matches origin')
    if not state['upstream']:
        detail += ' (no upstream)'
    return Result(status, value='diverged' if diverged else '', detail=detail,
                  data={**counts, 'diverged': diverged, 'branch': state['branch'],
                        'hash': state['hash'], 'fetch_age': state['fetch_age']})

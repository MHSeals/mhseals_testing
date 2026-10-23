"""Compare a run against earlier runs of the same challenge (referee.md, owner decisions).

There are no points: a run is judged by completion, time and failures, and shown against
the best, the median and the last N earlier runs of that challenge. Rows come from the
`referee` blocks of INDEX.json / runs JSON (schema mhseals.referee/1); pure, no ROS.
"""

import json
import statistics

LAST_N = 5


def rows(index, challenge):
    """Finished runs of ``challenge`` from INDEX rows, oldest first, as (id, time_s, failed).

    A row carries ``id``, ``t_ns`` or ``started`` for ordering, and a ``referee`` block
    with ``challenges[name].verdict`` and ``details.window_s`` (or ``gate_to_gate_s``).
    Pending, skipped and N/A runs are not comparable and are left out.
    """
    out = []
    for row in sorted(index, key=lambda r: str(r.get('started', r.get('t_ns', '')))):
        got = ((row.get('referee') or {}).get('challenges') or {}).get(challenge)
        if not got or got.get('verdict') not in ('PASS', 'FAIL'):
            continue
        d = got.get('details') or {}
        seconds = d.get('window_s', d.get('gate_to_gate_s'))
        out.append((row.get('id'), seconds if isinstance(seconds, (int, float)) else None,
                    got['verdict'] == 'FAIL'))
    return out


def load(path):
    """Read INDEX.json (a list, or ``{"sessions": [...]}``); a missing or broken file is empty."""
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return []
    data = data.get('sessions', []) if isinstance(data, dict) else data
    return [r for r in data if isinstance(r, dict)]


def compare(history, seconds, n=LAST_N):
    """Best, median and last-``n`` times of earlier PASSed runs, with the deltas of ``seconds``.

    Only passed runs with a time set the reference; ``failures`` counts the failed ones in
    the last ``n``. Deltas are None while there is no reference or no current time.
    """
    times = [t for _, t, failed in history if t is not None and not failed]
    last = times[-n:]
    best = min(times) if times else None
    median = statistics.median(times) if times else None

    def delta(ref):
        return None if ref is None or seconds is None else seconds - ref

    return {'best': best, 'median': median, 'last': last, 'delta_best': delta(best),
            'delta_median': delta(median), 'delta_last': delta(last[-1] if last else None),
            'runs': len(history), 'failures': sum(1 for h in history[-n:] if h[2])}

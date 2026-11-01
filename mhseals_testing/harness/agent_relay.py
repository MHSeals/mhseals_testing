"""tools/boat_agent on a dev machine whose ``$BOAT_AGENT_TARGET`` is a boat host: one ssh hop, and a cache. Stdlib only.

Runs the verb on the target's checkout over ssh (output streamed as it comes) and remembers the last operator state
any output carried (status, a paused reply, a follow event) in ``~/.cache/mhseals/boat_agent/<target>.json``. When
ssh itself fails (exit 255: the boat is off or the link is down) it prints one JSON line, ``unreachable`` (exit 4),
naming that last state: "TUI unreachable: last operator state battery swap since 11:30".
"""

import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

from mhseals_testing.agent_hook import BOAT_AGENT
from mhseals_testing.harness import operator_state

SSH_FAILED = 255
CONNECT_S = 8


def cache_path(target):
    base = Path(os.environ.get('XDG_CACHE_HOME') or Path.home() / '.cache')
    return base / 'mhseals' / 'boat_agent' / f'{target}.json'


def operator_of(doc):
    """The operator section a status, a reply's result or a journal event carries, else None."""
    if not isinstance(doc, dict):
        return None
    found = doc.get('operator')
    if not isinstance(found, dict) and isinstance(doc.get('result'), dict):
        found = doc['result'].get('operator')
    return found if isinstance(found, dict) and found.get('state') in operator_state.STATES else None


def remember(target, found, now_ns=None):
    path = cache_path(target)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps({'operator': found, 'seen_ns': now_ns or time.time_ns()}))
        os.replace(tmp, path)
    except OSError:
        pass                    # the cache is advisory


def unreachable(target, why, now_ns=None):
    """The ``unreachable`` reply, naming the last operator state seen (or that none was)."""
    try:
        cached = json.loads(cache_path(target).read_text())
    except (OSError, ValueError):
        cached = None
    found = (cached or {}).get('operator')
    last = (f"last {operator_state.text(found).replace('operator: ', 'operator state ', 1)} (seen "
            f"{operator_state.clock(cached.get('seen_ns'))})" if found else 'no operator state seen yet')
    paused = operator_state.paused(found)
    hint = (f'TUI unreachable on {target} ({why}): {last}. '
            + ('The boat is expected down: wait for the operator, check again in a few minutes (no retry loop); '
               'after it is back re-check the stack, FCU params and RTK' if paused else
               'The boat is off or the link is down: check again in a minute or ask the operator'))
    return {'ok': False, 'error': 'unreachable', 'target': target, 'hint': hint, 'operator': found,
            'seen_ns': (cached or {}).get('seen_ns'), 't_ns': now_ns or time.time_ns()}


def run(target, argv, out=sys.stdout, popen=subprocess.Popen):
    """Relay one verb; returns its exit code (4 ``unreachable`` when ssh fails)."""
    # ~ expands on the target; its own BOAT_AGENT_TARGET never sends the call on another hop
    remote = ' '.join(['BOAT_AGENT_TARGET=local', BOAT_AGENT, *(shlex.quote(a) for a in argv)])
    proc = popen(['ssh', '-o', f'ConnectTimeout={CONNECT_S}', '-o', 'BatchMode=yes', target, remote],
                 stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    for line in proc.stdout:
        out.write(line)
        out.flush()
        try:
            found = operator_of(json.loads(line))
        except ValueError:
            continue
        if found:
            remember(target, found)
    err = proc.stderr.read()
    code = proc.wait()
    if code != SSH_FAILED:
        sys.stderr.write(err)
        return code
    why = (err.strip().splitlines() or ['ssh failed'])[-1][:200]
    print(json.dumps(unreachable(target, why)), file=out, flush=True)
    return 4


if __name__ == '__main__':
    sys.exit(run(sys.argv[1], sys.argv[2:]))

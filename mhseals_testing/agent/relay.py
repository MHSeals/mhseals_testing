"""tools/boat_agent on a dev machine whose ``$BOAT_AGENT_TARGET`` is a boat host: one ssh hop, and a cache. Stdlib only.

Runs the verb on the target's checkout over ssh (output streamed as it comes) and remembers the last operator state
any output carried (status, a paused reply, a follow event) in ``~/.cache/mhseals/boat_agent/<target>.json``. When
ssh itself fails (exit 255: the boat is off or the link is down) it prints one JSON line, ``unreachable`` (exit 4),
naming that last state: "TUI unreachable: last operator state battery swap since 11:30".
"""

import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import time

from mhseals_testing.agent_hook import BOAT_AGENT
from mhseals_testing.harness import operator_state

SSH_FAILED = 255
CONNECT_S = 8
# a host name or address, with an optional user@: never an ssh option (-oProxyCommand=...) or a path
HOST = re.compile(r'([\w.-]+@)?[A-Za-z0-9][\w.-]*', re.ASCII)


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


def _ns(value, empty=True):
    """A stamp ``operator_state.clock`` can show (or None when ``empty``)."""
    return (value is None and empty) or (type(value) is int and 0 <= value < 2 ** 63)


def cached(target):
    """``(operator, seen_ns)`` from the cache, checked as read; ``(None, None)`` when it is missing or corrupt."""
    try:
        doc = json.loads(cache_path(target).read_text())
    except (OSError, ValueError, RecursionError):
        return None, None
    found = operator_of(doc)
    after = (found or {}).get('restarted_after')
    good = (found and _ns(doc.get('seen_ns'), empty=False)
            and all(_ns(found.get(k)) for k in ('since_ns', 'eta_ns'))
            and all(isinstance(found.get(k, ''), str) for k in ('reason', 'by'))
            and (after is None or isinstance(after, dict) and _ns(after.get('since_ns'))
                 and isinstance(after.get('state', ''), str)))
    return (found, doc['seen_ns']) if good else (None, None)


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
    found, seen_ns = cached(target)
    last = (f"last {operator_state.text(found).replace('operator: ', 'operator state ', 1)} (seen "
            f"{operator_state.clock(seen_ns)})" if found else 'no operator state seen yet')
    paused = operator_state.paused(found)
    hint = (f'TUI unreachable on {target} ({why}): {last}. '
            + ('The boat is expected down: wait for the operator, check again in a few minutes (no retry loop); '
               'after it is back re-check the stack, FCU params and RTK' if paused else
               'The boat is off or the link is down: check again in a minute or ask the operator'))
    return {'ok': False, 'error': 'unreachable', 'target': target, 'hint': hint, 'operator': found,
            'seen_ns': seen_ns, 't_ns': now_ns or time.time_ns()}


def fetch_args(argv):
    """The station's view of ``fetch``'s arguments (the boat parses them again), or None when malformed."""
    import argparse
    parser = argparse.ArgumentParser(prog='boat_agent fetch', add_help=False, exit_on_error=False)
    parser.add_argument('session', nargs='?')
    for flag in ('--who', '--why', '--to'):
        parser.add_argument(flag)
    parser.add_argument('--bwlimit', type=int)
    parser.add_argument('--no-bags', action='store_true')
    try:
        found, extra = parser.parse_known_args(argv)
    except (argparse.ArgumentError, SystemExit):
        return None
    return None if extra else found


def fetch(target, argv, out=sys.stdout, popen=subprocess.Popen, rsync=subprocess.run):
    """``boat_agent fetch`` on the station: the boat names the session (``fetch --where``, logged in its TUI agent
    log), then rsync pulls it here, resumable (``--partial-dir``), throttled, without its open segment."""
    from mhseals_testing import data

    def say(doc, code):
        print(json.dumps(dict(doc, t_ns=time.time_ns())), file=out, flush=True)
        return code
    found = fetch_args(argv[1:])
    if found is None or not (found.why or '').strip():
        return say({'ok': False, 'error': 'bad_args', 'hint': 'fetch [SESSION] --why "..." [--bwlimit KBPS] '
                    '[--no-bags] [--to DIR]'}, 2)
    lines = []

    class Keep:
        def write(self, text):
            lines.append(text)

        def flush(self):
            pass
    code = run(target, [*argv, '--where'], out=Keep(), popen=popen)
    plan = None
    for line in reversed(''.join(lines).splitlines()):
        try:
            plan = json.loads(line)
            break
        except ValueError:
            continue
    if code or not isinstance(plan, dict) or not plan.get('ok'):
        out.write(''.join(lines))
        out.flush()
        return code or 2
    rel, ident = str(plan.get('rel') or ''), str(plan.get('id') or '')
    if not rel or rel.startswith('/') or '..' in rel.split('/') or not ident or '/' in ident or ident in ('.', '..'):
        return say({'ok': False, 'error': 'refused', 'hint': f'the boat named an unsafe path {rel!r}'}, 2)
    base = Path(os.path.expanduser(found.to)) if found.to else data.root() / 'sessions'
    dest = base / ident
    home = Path(BOAT_AGENT).parent.parent.parent.parent.as_posix().removeprefix('~/').removeprefix('~')
    source = f"{target}:{home + '/' if home else ''}{rel}/"
    command = ['rsync', '-a', '--partial-dir=.rsync-partial', '--mkpath', f"--bwlimit={int(plan['bwlimit_kbps'])}",
               '-e', f'ssh -o ConnectTimeout={CONNECT_S} -o BatchMode=yes -o ServerAliveInterval=5',
               *(['--exclude=/bag/'] if found.no_bags else []),
               *(f'--exclude=/bag/{s}/' for s in plan.get('open_segments') or []), source, f'{dest}/']
    started = time.monotonic()
    done = rsync(command, stdin=subprocess.DEVNULL, capture_output=True, text=True)
    doc = {'ok': done.returncode == 0, 'id': ident, 'dest': str(dest), 'source': source,
           'rsync_exit': done.returncode, 'seconds': round(time.monotonic() - started, 1),
           'skipped': plan.get('open_segments') or [], 'bwlimit_kbps': plan['bwlimit_kbps']}
    if done.returncode:
        doc.update(error='rsync', hint=((done.stderr or '').strip().splitlines() or ['rsync failed'])[-1][:200]
                   + '; run fetch again to resume')
    elif plan.get('live'):
        doc['hint'] = 'the session is still recording: fetch again later for the rest'
    return say(doc, 0 if done.returncode == 0 else 2)


def run(target, argv, out=sys.stdout, popen=subprocess.Popen):
    """Relay one verb; returns its exit code (4 ``unreachable`` when ssh fails, 2 for a target that is no host name)."""
    if not HOST.fullmatch(target):
        print(json.dumps({'ok': False, 'error': 'bad_args', 'target': target, 't_ns': time.time_ns(),
                          'hint': 'BOAT_AGENT_TARGET is a host name ([user@]host: letters, digits, . - _)'}),
              file=out, flush=True)
        return 2
    # ~ expands on the target; its own BOAT_AGENT_TARGET never sends the call on another hop
    remote = ' '.join(['BOAT_AGENT_TARGET=local', BOAT_AGENT, *(shlex.quote(a) for a in argv)])
    proc = popen(['ssh', '-o', f'ConnectTimeout={CONNECT_S}', '-o', 'BatchMode=yes', '--', target, remote],
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
    sys.exit((fetch if sys.argv[2:3] == ['fetch'] else run)(sys.argv[1], sys.argv[2:]))

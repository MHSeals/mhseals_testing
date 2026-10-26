"""Keeper: runs one child in its own process group and stops it when its owner is gone.

``python -m mhseals_testing.harness.keeper (--owner PID | --lease FILE --lease-s S) --log LOG
--exit EXIT [--registry FILE] [--grace S] -- argv...``

A unit (harness/unit.py) has no owner: it runs until stopped or until its lease file goes
unrenewed for ``--lease-s`` seconds.

The keeper is the only thing tied to the owner, and only by polling: PDEATHSIG would reach
just the direct child (``ros2 run`` wraps the real node and swallows SIGINT), so the keeper
signals the whole group: SIGINT, SIGTERM after ``grace``, SIGKILL after 2 s more. It stops
the child the same way on SIGTERM/SIGHUP/SIGINT, and writes the return code to the exit
file. With a registry the owner is the registry's owner, and an unexpired handover keeps
the child alive for the next instance. ROS-free.
"""

import argparse
import ctypes
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from mhseals_testing.harness.registry import Registry, alive

PR_SET_PDEATHSIG = 1
POLL = 0.5


def _prepare_child():
    """Own group; default SIGINT/SIGTERM (an inherited SIG_IGN would defeat stop); die with the keeper."""
    os.setpgid(0, 0)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    if sys.platform.startswith('linux'):
        ctypes.CDLL(None, use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGINT)


def owner_gone(owner, registry, now):
    """True when nobody owns the child any more and no handover is pending."""
    entry = registry.read() if registry else None
    if entry is None:
        return not alive(owner)
    current = (entry.get('owner') or {}).get('pid')
    if current is not None:
        return not alive(current)
    handover = entry.get('handover')
    return not (handover and handover.get('deadline', 0) > now)


def _group_alive(pgid):
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    return True


def stop_group(child, grace):
    """INT, TERM, KILL the child's whole group, reaping the leader; returns its return code."""
    pgid = child.pid
    for signum, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 2.0), (signal.SIGKILL, 2.0)):
        try:
            os.killpg(pgid, signum)
        except ProcessLookupError:
            break
        deadline = time.time() + wait
        while time.time() < deadline and (child.poll() is None or _group_alive(pgid)):
            time.sleep(0.05)
        if child.poll() is not None and not _group_alive(pgid):
            break
    return child.wait()


def lease_expired(lease, lease_s, now):
    """True when the lease file is missing or was last renewed more than ``lease_s`` ago."""
    try:
        return now - os.stat(lease).st_mtime > lease_s
    except OSError:
        return True


def run(argv, owner, log, exit_file, registry=None, grace=1.0, lease=None, lease_s=0.0):
    """Keep ``argv`` until it exits, a stop signal, the owner is gone or the lease runs out.

    A unit (harness/unit.py) has no owner pid, only a lease the TUI renews; ``owner`` is None then.
    """
    if owner is None and lease is None:
        raise ValueError('a keeper needs an owner or a lease')
    stopping = []
    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(signum, lambda *_: stopping.append(signum))
    with open(log, 'ab') as output:
        child = subprocess.Popen(argv, preexec_fn=_prepare_child, stdin=subprocess.DEVNULL,
                                 stdout=output, stderr=subprocess.STDOUT)
    pid_file = Path(exit_file).with_suffix('.pid')
    pending = pid_file.with_name(pid_file.name + '.tmp')
    pending.write_text(str(child.pid))
    os.replace(pending, pid_file)
    code, checked = None, 0.0
    while code is None and not stopping:
        time.sleep(0.05)
        if child.poll() is not None:
            code = child.returncode
        elif time.time() - checked >= POLL:
            checked = time.time()
            if owner is not None and owner_gone(owner, registry, checked):
                break
            if lease is not None and lease_expired(lease, lease_s, checked):
                break
    if code is None:
        code = stop_group(child, grace)
    elif _group_alive(child.pid):  # the leader is gone but a grandchild is not
        stop_group(child, grace)
    temporary = Path(str(exit_file) + '.tmp')
    temporary.write_text(str(code))
    os.replace(temporary, exit_file)
    return code


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--owner', type=int)
    parser.add_argument('--log', required=True)
    parser.add_argument('--exit', required=True)
    parser.add_argument('--registry')
    parser.add_argument('--grace', type=float, default=1.0)
    parser.add_argument('--lease')
    parser.add_argument('--lease-s', type=float, default=0.0)
    parser.add_argument('command', nargs='+')
    args = parser.parse_args(argv)
    if args.owner is None and args.lease is None:
        parser.error('--owner or --lease is required')
    run(args.command, args.owner, args.log, args.exit,
        Registry(args.registry) if args.registry else None, args.grace, args.lease, args.lease_s)


if __name__ == '__main__':
    main()

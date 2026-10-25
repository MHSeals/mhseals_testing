"""Named units: one device part kept running on its own computer, by a keeper on a lease.

The TUI runs these verbs on the part's computer (ssh, then ``docker exec`` into the container);
nothing here touches the network (device-topology.md section 4):

``python3 -m mhseals_testing.harness.unit start <unit> [--lease-s S] [--by JSON] -- argv...``
    starts the part, or reports the one already running (a later TUI adopts it that way);
``status [<unit>]``, ``renew <unit>``, ``stop <unit>``: one JSON line per unit.

The keeper puts the part in its own process group and stops the whole group on ``stop``, on
SIGTERM (``docker stop`` reaches it through stop-children.sh) or when nobody renews the lease, so
a dropped ssh session or a crashed TUI never leaves it running for long. Records live in
``$XDG_RUNTIME_DIR/mhseals/units/``; logs under the data root's ``units/<unit>/``.

Motion boundary: the thruster driver and command_mux are refused here, always. A commander part
(Nav2, the mission runner) starts only when the TUI passes the internal ``--commander`` argument,
which it does after an operator's keypress and confirm; it is not an operator-facing option.
Stop is never refused. ROS-free.
"""

import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

from mhseals_testing import data
from mhseals_testing.harness.registry import alive, runtime_dir

LEASE_S = 120.0
START_WAIT = 5.0           # s for the keeper to report its child's pid
STOP_WAIT = 30.0           # s for the keeper to stop the group and write the exit code
NAME = re.compile(r'^mhseals-[a-z0-9][a-z0-9_-]*$')
DRIVERS = {'thruster_pwm_node', 'command_mux'}
COMMANDER_LAUNCHES = {'navigation.launch.py', 'task.launch.py', 'robot.launch.py'}
DEVICE_LAUNCH = 'device.launch.py'


class UnitError(RuntimeError):
    """A unit verb the device refuses; the message says why."""


def units_dir():
    path = runtime_dir() / 'units'
    path.mkdir(parents=True, exist_ok=True, mode=0o700)
    return path


def _files(unit):
    base = units_dir() / unit
    return {key: base.with_suffix('.' + key) for key in ('json', 'lease', 'exit', 'pid', 'lock')}


def _check_name(unit):
    if not NAME.match(unit):
        raise UnitError(f'unit name {unit!r}: mhseals-<part> in lower case')


def _arguments(argv):
    """``{name: value}`` of the ``name:=value`` launch arguments in ``argv``."""
    return dict(token.split(':=', 1) for token in argv if ':=' in token)


def device_part_kind(profile, part):
    """The kind of ``part`` in mhseals_nav's merged device profile; None when it cannot be read."""
    try:
        from mhseals_nav.device_profile import load
        return load(profile)['parts'][part].get('kind', 'sensing')
    except (ImportError, KeyError, OSError, ValueError):
        return None


def commander(argv, kind_of=device_part_kind):
    """True when ``argv`` starts a part that can command motion, or one whose kind cannot be read.

    A device launch with no ``only:=`` starts every part of the profile, so each one is read.
    """
    names = {Path(token).name for token in argv}
    if names & COMMANDER_LAUNCHES:
        return True
    if DEVICE_LAUNCH not in names:
        return False
    arguments = _arguments(argv)
    profile, only = arguments.get('device', ''), arguments.get('only', '')
    parts = [part.strip() for part in only.split(',') if part.strip()]
    if not parts:
        try:
            from mhseals_nav.device_profile import load
            parts = list(load(profile)['parts'])
        except (ImportError, KeyError, OSError, ValueError):
            return True
    return any(kind_of(profile, part) != 'sensing' for part in parts)


def refuse(argv, allow_commander, kind_of=device_part_kind):
    """Raise UnitError for a driver, and for a commander without the TUI's explicit go-ahead."""
    driver = DRIVERS & {Path(token).name for token in argv}
    if driver:
        raise UnitError(f'{", ".join(sorted(driver))} never runs as a unit; '
                        'start it from the TUI on that computer, through arming')
    if not allow_commander and commander(argv, kind_of):
        raise UnitError('this part can command motion; start it from the TUI with a confirm')


class _Lock:
    def __init__(self, path):
        self.path = path

    def __enter__(self):
        self.handle = open(self.path, 'a')
        fcntl.flock(self.handle, fcntl.LOCK_EX)
        return self

    def __exit__(self, *_):
        self.handle.close()


def _read(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def status(unit):
    """One unit's state: running, exited (with its code) or off."""
    _check_name(unit)
    files = _files(unit)
    record = _read(files['json'])
    if record is None:
        return {'unit': unit, 'state': 'off'}
    out = {key: record.get(key) for key in ('unit', 'argv', 'pid', 'keeper', 'started', 'by', 'log',
                                            'lease_s', 'commander')}
    try:
        out['code'] = int(files['exit'].read_text())
    except (OSError, ValueError):
        out['code'] = None
    out['state'] = 'running' if alive(record['keeper']) else ('exited' if out['code'] is not None else 'off')
    try:
        out['lease_age'] = round(time.time() - files['lease'].stat().st_mtime, 1)
    except OSError:
        out['lease_age'] = None
    return out


def statuses():
    return [status(path.stem) for path in sorted(units_dir().glob('mhseals-*.json'))]


def start(unit, argv, lease_s=LEASE_S, by=None, allow_commander=False, kind_of=device_part_kind):
    """Start ``argv`` as ``unit``, or return the running one with ``adopted`` set."""
    _check_name(unit)
    if not argv:
        raise UnitError('start needs a command after --')
    if not lease_s > 0:
        raise UnitError('the lease must be longer than 0 s')
    refuse(argv, allow_commander, kind_of)
    files = _files(unit)
    with _Lock(files['lock']):
        current = status(unit)
        if current['state'] == 'running':
            return dict(current, adopted=True)
        for stale in (files['exit'], files['pid']):
            stale.unlink(missing_ok=True)
        files['lease'].touch()
        os.utime(files['lease'])
        log = data.path('units', unit, time.strftime('%Y%m%d_%H%M%S') + '.log')
        log.parent.mkdir(parents=True, exist_ok=True)
        environment = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_LOGGING_BUFFERED_STREAM='0')
        keeper = subprocess.Popen(
            [sys.executable, '-m', 'mhseals_testing.harness.keeper', '--lease', str(files['lease']),
             '--lease-s', str(lease_s), '--log', str(log), '--exit', str(files['exit']), '--',
             *[str(part) for part in argv]],
            start_new_session=True, env=environment, stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        deadline = time.time() + START_WAIT
        while not files['pid'].exists() and keeper.poll() is None and time.time() < deadline:
            time.sleep(0.02)
        try:
            pid = int(files['pid'].read_text())
        except (OSError, ValueError):
            keeper.kill()
            raise UnitError(f'the keeper for {unit} did not start its command') from None
        record = {'unit': unit, 'argv': list(argv), 'pid': pid,
                  'keeper': keeper.pid, 'started': time.time(), 'by': by or {}, 'log': str(log),
                  'lease_s': lease_s, 'commander': bool(allow_commander)}
        data.write_json(files['json'], record, 'mhseals.unit/1')
        keeper.returncode = 0      # the keeper outlives this process by design; do not warn about it
    return dict(status(unit), adopted=False)


def renew(unit):
    """Extend the lease of a running unit."""
    current = status(unit)
    if current['state'] != 'running':
        raise UnitError(f'{unit} is not running')
    os.utime(_files(unit)['lease'])
    return dict(status(unit))


def stop(unit, wait=STOP_WAIT):
    """Stop a unit (never refused): the keeper stops the whole group, then writes the exit code."""
    _check_name(unit)
    files = _files(unit)
    with _Lock(files['lock']):
        record = _read(files['json'])
        if record is None or not alive(record['keeper']):
            return status(unit)
        os.kill(record['keeper'], signal.SIGTERM)
        deadline = time.time() + wait
        while alive(record['keeper']) and time.time() < deadline:
            time.sleep(0.05)
        if alive(record['keeper']):            # a wedged keeper: end it and its child's group
            os.kill(record['keeper'], signal.SIGKILL)
            try:
                os.killpg(record['pid'], signal.SIGKILL)
            except ProcessLookupError:
                pass
    return status(unit)


def main(argv=None):
    parser = argparse.ArgumentParser(prog='python3 -m mhseals_testing.harness.unit')
    verbs = parser.add_subparsers(dest='verb', required=True)
    begin = verbs.add_parser('start')
    begin.add_argument('unit')
    begin.add_argument('--lease-s', type=float, default=LEASE_S)
    begin.add_argument('--by', default='{}')
    begin.add_argument('--commander', action='store_true', help=argparse.SUPPRESS)  # TUI-internal
    verbs.add_parser('status').add_argument('unit', nargs='?')
    verbs.add_parser('renew').add_argument('unit')
    verbs.add_parser('stop').add_argument('unit')
    argv = sys.argv[1:] if argv is None else list(argv)
    command = []
    if '--' in argv:         # the part's argv is never parsed here, options before it always are
        at = argv.index('--')
        argv, command = argv[:at], argv[at + 1:]
    args = parser.parse_args(argv)
    try:
        if args.verb == 'start':
            results = [start(args.unit, command, args.lease_s, json.loads(args.by), args.commander)]
        elif args.verb == 'status':
            results = [status(args.unit)] if args.unit else statuses()
        else:
            results = [{'renew': renew, 'stop': stop}[args.verb](args.unit)]
    except (UnitError, ValueError) as error:
        print(json.dumps({'error': str(error)}))
        return 1
    for result in results:
        print(json.dumps(result))
    return 0


if __name__ == '__main__':
    sys.exit(main())

"""Named units: one device part kept running on its own computer, by a keeper on a lease.

The TUI runs these verbs on the part's computer (ssh, then ``docker exec`` into the container);
nothing here touches the network (device-topology.md section 4):

``python3 -m mhseals_testing.harness.unit start <unit> [--lease-s S] [--by JSON] -- argv...``
    starts the part, or reports the one already running (a later TUI adopts it that way);
``status [<unit>]``, ``renew <unit>``, ``stop <unit>``: one JSON line per unit;
``status --probe JSON`` adds ``running`` per part from this computer's process table, whoever
started it (a unit, a TUI's keeper, a shell), which is what the TUI shows instead of DDS discovery.

The keeper puts the part in its own process group and stops the whole group on ``stop``, on
SIGTERM (``docker stop`` reaches it through stop-children.sh) or when nobody renews the lease, so
a dropped ssh session or a crashed TUI never leaves it running for long. Records live in
``$XDG_RUNTIME_DIR/mhseals/units/``; logs under the data root's ``units/<unit>/``.

Motion boundary: the thruster driver and command_mux are refused here, except as the one unit the
TUI's arming starts on the computer with the ``thrusters`` capability: the internal ``--driver``
argument, and only under ``mhseals_testing.ros.arm_link``, which stops them as soon as the
operator's DDS heartbeat stops (ros/arm_link.py), never on a lease renewed over ssh. A commander part
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
MAX_LEASE_S = 600.0
START_WAIT = 5.0           # s for the keeper to report its child's pid
STOP_WAIT = 30.0           # s for the keeper to stop the group and write the exit code
NAME = re.compile(r'^mhseals-[a-z0-9][a-z0-9_-]*$')
DRIVERS = {'thruster_pwm_node', 'command_mux'}
ARM_LINK = ('python3', '-m', 'mhseals_testing.ros.arm_link', 'run')
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


def device_parts(arguments):
    """The merged device profile's parts exactly as device.launch.py will build them from ``arguments``.

    The argv's deployment and startup files and its session ``overrides`` are layers too, so
    one of them could otherwise turn a commander into a sensing part unseen.
    """
    from mhseals_nav.device_profile import load
    return load(arguments.get('device', ''), arguments.get('device_deployment_file', ''),
                arguments.get('device_startup_file', ''), arguments.get('overrides', ''))['parts']


def commander(argv, parts_of=device_parts):
    """True when ``argv`` starts a part that can command motion, or one whose kind cannot be read.

    A device launch with no ``only:=`` starts every part of the profile, so each one is read.
    """
    names = {Path(token).name for token in argv}
    if names & COMMANDER_LAUNCHES:
        return True
    if DEVICE_LAUNCH not in names:
        return False
    arguments = _arguments(argv)
    try:
        parts = parts_of(arguments)
    except ModuleNotFoundError as error:
        if (error.name or '').split('.')[0] != 'mhseals_nav':
            return True
        # Still refused (fail closed), but saying why: no part's kind can be read here.
        raise UnitError('mhseals_nav is not installed on this computer, so the part\'s kind '
                        'cannot be read; install it to start device parts as units') from error
    except Exception:          # unreadable counts as commander: fail closed
        return True
    chosen = [part.strip() for part in arguments.get('only', '').split(',') if part.strip()]
    return any(part not in parts or parts[part].get('kind', 'sensing') != 'sensing'
               for part in chosen or parts)


def refuse(argv, allow_commander, parts_of=device_parts, allow_driver=False):
    """Raise UnitError for a driver (unless arming's, under arm_link) and for a commander
    without the TUI's explicit go-ahead."""
    driver = DRIVERS & {Path(token).name for token in argv}
    if driver and not (allow_driver and tuple(argv[:len(ARM_LINK)]) == ARM_LINK):
        raise UnitError(f'{", ".join(sorted(driver))} never runs as a unit; '
                        'start it from the TUI through arming')
    if allow_driver and tuple(argv[:len(ARM_LINK)]) == ARM_LINK:
        from mhseals_testing.ros.arm_link import allowed, split_children
        try:
            why = allowed(split_children(argv[argv.index('--') + 1:]))
        except ValueError as error:
            why = str(error)
        if why:
            raise UnitError(f'arming runs only its own children: {why}')
        return
    if not allow_commander and commander(argv, parts_of):
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


# --- what runs here, whoever started it ---------------------------------------------------------

SELF = 'mhseals_testing.harness.unit'     # this CLI (and the shell that runs it) is never a part


def process_table(proc='/proc'):
    """``[(pid, argv)]`` of every process this computer shows (the container has ``pid: host``)."""
    table = []
    for entry in Path(proc).iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            argv = (entry / 'cmdline').read_bytes().split(b'\0')
        except OSError:
            continue
        argv = [word.decode(errors='replace') for word in argv if word]
        if argv and not any(SELF in word for word in argv):
            table.append((int(entry.name), argv))
    return table


def _node_names(argv):
    """The ROS node names a process may carry: ``__node:=`` remaps and the executable's own name."""
    names = {word.split('__node:=', 1)[1] for word in argv if '__node:=' in word}
    names.add(Path(argv[0]).name)
    if Path(argv[0]).name.startswith('python') and len(argv) > 1:
        names.add(Path(argv[1]).name)          # ros2 run of a Python node: python3 .../lib/pkg/node
    return names


def _launches(argv, spec):
    """True when ``argv`` is a ``ros2 launch`` (or a keeper around one) that starts ``spec``'s part."""
    if 'launch' not in argv:
        return False
    names = [Path(word).name for word in argv]
    arguments = _arguments(argv)
    if DEVICE_LAUNCH in names and arguments.get('device') == spec.get('profile'):
        only = [part.strip() for part in arguments.get('only', '').split(',') if part.strip()]
        return not only or spec.get('part') in only
    return bool(spec.get('launch')) and spec['launch'] in names


def probe(specs, table=None):
    """``{unit: {'running', 'pid', 'how'}}``: whether each part runs on this computer, from its processes.

    ``specs`` are ``{unit, part, profile, launch, nodes}``. A part runs when a launch of it runs
    (device.launch.py with its ``only:=``, or its own launch file), or when every proof node has a
    process (a ``__node:=`` remap or the executable's name). Who started it does not matter: a unit,
    a TUI's keeper or a shell. Nothing here asks DDS, so a discovery dropout cannot change it.
    """
    table = process_table() if table is None else table
    found = {}
    for spec in specs:
        launch = next((pid for pid, argv in table if _launches(argv, spec)), None)
        pids = {}
        for node in spec.get('nodes') or ():
            pids[node] = next((pid for pid, argv in table if node in _node_names(argv)), None)
        if launch is not None:
            found[spec['unit']] = {'running': True, 'pid': launch, 'how': 'launch'}
        elif pids and all(pid is not None for pid in pids.values()):
            found[spec['unit']] = {'running': True, 'pid': min(pids.values()), 'how': 'nodes'}
        else:
            found[spec['unit']] = {'running': False, 'pid': None, 'how': None}
    return found


def part_statuses(specs, table=None):
    """One record per probed part: its unit's status plus ``running`` as this computer sees it."""
    seen = probe(specs, table)
    records = []
    for spec in specs:
        record = status(spec['unit'])
        here = seen[spec['unit']]
        record['running'] = record['state'] == 'running' or here['running']
        record['pid'] = record.get('pid') or here['pid']
        record['how'] = 'unit' if record['state'] == 'running' else here['how']
        records.append(record)
    return records


def start(unit, argv, lease_s=LEASE_S, by=None, allow_commander=False, parts_of=device_parts,
          allow_driver=False):
    """Start ``argv`` as ``unit``, or return the running one with ``adopted`` set."""
    _check_name(unit)
    if not argv:
        raise UnitError('start needs a command after --')
    if not 0 < lease_s <= MAX_LEASE_S:      # NaN fails too: a unit nobody renews must stop
        raise UnitError(f'the lease must be over 0 s and at most {MAX_LEASE_S:.0f} s')
    refuse(argv, allow_commander, parts_of, allow_driver)
    if allow_driver:
        from mhseals_testing.ros.arm_link import LEASE_S as DRIVER_LEASE_S
        if lease_s > DRIVER_LEASE_S:
            raise UnitError(f'the driver\'s lease is at most {DRIVER_LEASE_S:g} s')
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
                  'lease_s': lease_s, 'commander': bool(allow_commander), 'driver': bool(allow_driver)}
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
        # An exit file means the keeper is done: its pids may belong to someone else by now.
        if record is None or files['exit'].exists() or not alive(record['keeper']):
            return status(unit)
        os.kill(record['keeper'], signal.SIGTERM)
        deadline = time.time() + wait
        while alive(record['keeper']) and time.time() < deadline:
            time.sleep(0.05)
        if alive(record['keeper']) and not files['exit'].exists():   # wedged: end it and the group
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
    begin.add_argument('--driver', action='store_true', help=argparse.SUPPRESS)     # arming-internal
    state = verbs.add_parser('status')
    state.add_argument('unit', nargs='?')
    state.add_argument('--probe', help='JSON list of {unit, part, profile, launch, nodes}: '
                       'report those parts as this computer runs them, whoever started them')
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
            results = [start(args.unit, command, args.lease_s, json.loads(args.by), args.commander,
                             allow_driver=args.driver)]
        elif args.verb == 'status' and args.probe:
            specs = json.loads(args.probe)
            for spec in specs:
                _check_name(spec['unit'])
            results = part_statuses(specs)
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

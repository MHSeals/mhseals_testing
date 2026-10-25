"""Parts on other computers, from the TUI: the unit CLI (harness/unit.py) over ssh + docker exec.

One call reaches a part's computer: ``ssh <options> user@address docker exec -i <container> bash -c
'python3 -m mhseals_testing.harness.unit <verb> ...'`` (device-topology.md section 4). The key and
known hosts live in ``~/.config/mhseals/ssh/`` (already mounted in the container); a ``via`` host is
the jump. The keeper owns the part's process group on that computer, so a dropped ssh session or a
crashed TUI never orphans it: the lease simply runs out.

``Units`` keeps each remote part's status for the Status rows, without blocking them: a worker thread
runs the start and stop requests and, every ``period`` s, one call per host that renews the units
this TUI holds and reads, from that computer's own process table, whether each of its parts runs
(``unit status --probe``): whoever started it, and whatever DDS discovery shows. A unit found
running is adopted (renewed from then on), which is how a later TUI on any computer takes over.

The calls share one ssh connection per host (ControlMaster, kept ``PERSIST_S``, with keepalives),
so a status read over the radio is one round trip, and a failed call is retried once. A failed read
keeps the last report: the parts keep their state, and ``trouble()`` says since when the host has
not answered (the TUI raises an issue only past its grace). Stop is never refused; a commander part
starts only with ``commander`` set, which the Stack passes after the operator's confirm.
"""

import json
import os
from pathlib import Path
import queue
import shlex
import subprocess
import threading
import time

from mhseals_testing.harness.registry import runtime_dir
from mhseals_testing.harness.unit import LEASE_S, START_WAIT, STOP_WAIT

CONNECT_S = 10          # the TCP connect and the ssh banner, over the radio
PERSIST_S = 600         # the shared connection outlives a quiet spell
ALIVE_S, ALIVE_COUNT = 5, 3   # a dead link closes the shared connection in 15 s, not at TCP timeout
PERIOD = 5.0            # s between status + renew calls (the TUI's devices probe sets it)
UNIT = ('python3', '-m', 'mhseals_testing.harness.unit')
REASONS = (('timed out', 'timeout'), ('Connection refused', 'refused'), ('No route', 'no route'),
           ('Could not resolve', 'unknown host'), ('Permission denied', 'key refused'),
           ('Host key verification', 'unknown host key'), ('is not running', 'container stopped'),
           ('No such container', 'no container'))


def _key_dir():
    return Path(os.environ.get('XDG_CONFIG_HOME') or Path.home() / '.config') / 'mhseals' / 'ssh'


def _options(token='%C'):
    keys = _key_dir()
    return ['-o', 'BatchMode=yes', '-o', f'ConnectTimeout={CONNECT_S}', '-o', 'ControlMaster=auto',
            '-o', f'ControlPersist={PERSIST_S}', '-o', f'ControlPath={runtime_dir()}/ssh-{token}',
            '-o', f'ServerAliveInterval={ALIVE_S}', '-o', f'ServerAliveCountMax={ALIVE_COUNT}',
            '-i', str(keys / 'id_ed25519'), '-o', f'UserKnownHostsFile={keys / "known_hosts"}']


def _target(host):
    return f'{host.user}@{host.address}' if host.user else host.address


def ssh_argv(host, command, jump=None):
    """argv that runs ``command`` (one shell line) in ``host``'s container, through ``jump`` if given.

    The jump gets the same key and known hosts (``-J`` would not pass them on); ssh expands
    ProxyCommand's own tokens, so its ``%C`` is written ``%%C`` for the inner ssh to expand.
    The remote shell gets one quoted line, so no registry value is ever read as shell syntax.
    """
    argv = ['ssh', *_options()]
    if jump is not None:
        inner = ['ssh', *_options('%%C'), '-W', '%h:%p', _target(jump)]
        argv += ['-o', 'ProxyCommand=' + shlex.join(inner)]
    return argv + [_target(host), shlex.join(['docker', 'exec', '-i', host.container, 'bash', '-c',
                                              command])]


def reason(stderr, code):
    """A short why for a failed call: 'ssh: timeout', 'ssh: key refused', ..."""
    lines = [line for line in (stderr or '').strip().splitlines() if line.strip()]
    text = lines[-1] if lines else f'exit {code}'
    for needle, short in REASONS:
        if needle in text:
            return f'ssh: {short}'
    return f'ssh: {text[:60]}'


def call(host, command, jump=None, timeout=CONNECT_S + 10, retries=1):
    """(JSON lines, error): run one unit command on ``host``; error is set when it was not reached.

    ssh's own failure (exit 255: a dropped shared connection, a banner timeout) is retried
    ``retries`` times at once; a call that used up its time is not.
    """
    try:
        done = subprocess.run(ssh_argv(host, command, jump), capture_output=True, text=True,
                              timeout=timeout, stdin=subprocess.DEVNULL)
    except subprocess.TimeoutExpired:
        return [], 'ssh: timeout'
    except OSError as error:
        return [], f'ssh: {error.strerror or error}'
    if done.returncode == 255 and retries > 0:
        return call(host, command, jump, timeout, retries - 1)
    results = []
    for line in done.stdout.splitlines():
        try:
            found = json.loads(line)
        except ValueError:
            continue
        if isinstance(found, dict):
            results.append(found)
    if done.returncode == 255 or (done.returncode != 0 and not results):
        return [], reason(done.stderr, done.returncode)
    return results, None


def probe_specs(items):
    """What ``unit status --probe`` needs to find each part in the device's process table."""
    return [{'unit': item.unit, 'part': item.part, 'profile': item.profile, 'launch': item.launch,
             'nodes': list(item.nodes)} for item in items]


PROBE_ERROR = ('python3 -c \'import json, sys; lines = sys.argv[2].strip().splitlines(); '
               'print(json.dumps({"probe_error": (lines[-1] if lines else "exit " + sys.argv[1])[:200]}))\'')


def status_command(items):
    """Status of ``items`` as their computer runs them.

    When the probe fails (a device without ``--probe``, an unreadable process table) its last
    stderr line comes back as a ``{"probe_error"}`` line, then the plain unit status: the units
    it names are facts, but a part it calls off may run by hand, so it is no report of off.
    """
    probe = json.dumps(probe_specs(items), separators=(',', ':'), sort_keys=True)
    return (f'{{ err=$({shlex.join([*UNIT, "status", "--probe", probe])} 2>&1 >&3); }} 3>&1 || '
            f'{{ {PROBE_ERROR} "$?" "$err"; {unit_command("status")}; }}')


def unit_command(verb, unit=None, argv=(), by=None, commander=False, lease_s=LEASE_S, driver=False):
    """The unit CLI line for ``verb``; the internal ``--commander`` and ``--driver`` (arming's,
    ros/arm_link.py) are only ever set here."""
    words = [*UNIT, verb, *([unit] if unit else [])]
    if verb == 'start':
        words += ['--lease-s', str(lease_s), '--by', json.dumps(by or {}, sort_keys=True)]
        words += ['--commander'] if commander else []
        words += ['--driver'] if driver else []
        words += ['--', *argv]
    return shlex.join(words)


class Units:
    """Remote parts' status, and their start, stop and lease renewal, off the UI thread.

    ``items`` are the Stack's components (the local ones are ignored), ``hosts`` the registry's
    hosts by name (for ``via``), ``by`` who starts (host, pid, session) and ``renew`` False for a
    TUI that only watches: it reads status but keeps nothing alive.
    """

    def __init__(self, items, hosts=None, by=None, renew=True, run=call, period=PERIOD,
                 clock=time.monotonic):
        # A carried part (the boat's watchdogs, inside odometry's launch) has no unit of its own:
        # adopting a stale one would renew it forever beside odometry's copy.
        self.items = [item for item in items if item.host is not None and not item.carried_by]
        # Its status is still read (from that computer's process table): the device report.
        self.watched = [item for item in items if item.host is not None]
        if hosts is None and any(item.host.via for item in self.watched):
            from mhseals_testing import devices
            hosts = devices.load().hosts
        self.hosts = dict(hosts or {})
        self.by, self.renewing, self.run, self.period = dict(by or {}), renew, run, period
        self.clock = clock
        self.lock = threading.Lock()
        self.records, self.errors, self.held, self.notices = {}, {}, set(), []
        self.since = {}          # host -> when its calls began to fail
        self.read = {}           # host -> when its status was last read
        self.probe_errors = {}   # host -> (why its process probe fails, since when)
        self.started = set()     # the held units this TUI started; the rest of held are adopted
        self.jobs = queue.Queue()
        self.thread = None
        for item in self.watched:
            if not item.host.ssh:
                self.errors[item.host.name] = f'no ssh to {item.host.name}'
                self.since[item.host.name] = clock()

    def status(self, item):
        """(the part's last report or None, why its host does not answer now or None).

        The report outlives a failed read: a blip on the link changes nothing in the row.
        """
        with self.lock:
            return self.records.get(item.unit), self.errors.get(item.host.name)

    def trouble(self):
        """``{host: (why, since, last read or None)}`` for the hosts whose calls fail now."""
        with self.lock:
            return {name: (error, self.since.get(name), self.read.get(name))
                    for name, error in self.errors.items()}

    def probe_trouble(self):
        """``{host: (why, since)}`` for the answering hosts whose process probe fails."""
        with self.lock:
            return dict(self.probe_errors)

    def start(self, item, commander=False):
        with self.lock:
            self.records[item.unit] = {'state': 'running', 'pending': True}
        self.jobs.put(('start', item, commander))

    def stop(self, item):
        with self.lock:
            self.held.discard(item.unit)
            self.started.discard(item.unit)
            self.records[item.unit] = {'state': 'off', 'pending': True}
        self.jobs.put(('stop', item, False))

    def held_by_host(self, adopted=None):
        """{host name: [titles]} of the units this TUI keeps alive.

        ``adopted`` False keeps only the ones it started, True only the ones it adopted.
        """
        found = {}
        with self.lock:
            for item in self.items:
                if item.unit in self.held and adopted in (None, item.unit not in self.started):
                    found.setdefault(item.host.name, []).append(item.title)
        return found

    def take_notices(self):
        with self.lock:
            found, self.notices = self.notices, []
        return found

    # --- worker ------------------------------------------------------------------------------

    def _call(self, host, command, timeout):
        if not host.ssh:
            return [], f'no ssh to {host.name}'
        results, error = self.run(host, command, self.hosts.get(host.via) if host.via else None,
                                  timeout)
        with self.lock:
            if error:                  # keep the last report: the issue, not the row, says it is old
                self.errors[host.name] = error
                self.since.setdefault(host.name, self.clock())
            else:
                self.errors.pop(host.name, None)
                self.since.pop(host.name, None)
        return results, error

    def _note(self, line):
        with self.lock:
            self.notices.append(line)

    def do_start(self, item, commander):
        command = unit_command('start', item.unit, item.argv, self.by, commander)
        results, error = self._call(item.host, command, CONNECT_S + START_WAIT + 10)
        found = results[0] if results else {}
        with self.lock:
            if error or 'error' in found:
                self.records.pop(item.unit, None)
            else:
                self.records[item.unit] = found
                self.held.add(item.unit)
                if not found.get('adopted'):
                    self.started.add(item.unit)
        if error or 'error' in found:
            self._note(f'{item.title} not started on {item.host.name}: {error or found["error"]}')
        else:
            verb = 'adopted' if found.get('adopted') else 'started'
            self._note(f'{item.title} {verb} on {item.host.name} ({item.unit})')

    def do_stop(self, item):
        results, error = self._call(item.host, unit_command('stop', item.unit),
                                    CONNECT_S + STOP_WAIT + 10)
        found = results[0] if results else {}
        if not error and 'error' not in found:
            with self.lock:
                self.records[item.unit] = found
                self.held.discard(item.unit)
                self.started.discard(item.unit)
            self._note(f'{item.title} stopped on {item.host.name}')
        else:
            self._note(f'{item.title} stop on {item.host.name} failed: {error or found["error"]}')
        return error or found.get('error')

    def refresh(self):
        """One call per host: renew what we hold, then read every unit's status (and adopt)."""
        for name in sorted({item.host.name for item in self.watched}):
            mine = [item for item in self.watched if item.host.name == name]
            host = mine[0].host
            with self.lock:
                renew = [item.unit for item in mine if item.unit in self.held] if self.renewing else []
            command = ' ; '.join([*(f'{unit_command("renew", unit)} >/dev/null 2>&1' for unit in renew),
                                  status_command(mine)])
            results, error = self._call(host, command, CONNECT_S + 10)
            if error:
                continue
            found = {record.get('unit'): record for record in results if 'unit' in record}
            # The probe's own failure: a {"probe_error"} line, or the unit CLI's {"error"}.
            failed = next((record.get('probe_error') or record.get('error') for record in results
                           if 'unit' not in record), None)
            with self.lock:
                self.read[name] = self.clock()
                if failed:
                    self.probe_errors[name] = (failed, self.probe_errors.get(name, (None, self.clock()))[1])
                else:
                    self.probe_errors.pop(name, None)
                for item in mine:
                    record = found.get(item.unit, {'unit': item.unit, 'state': 'off'})
                    if failed and record['state'] not in ('running', 'exited'):
                        # Plain unit status: not running as a unit, but it may run by hand.
                        record = {'unit': item.unit, 'state': 'unknown', 'probe_error': failed}
                    current = self.records.get(item.unit, {})
                    if current.get('pending') and record['state'] != current['state']:
                        continue                   # our start or stop is still on its way
                    self.records[item.unit] = record
                    if item.carried_by:
                        continue                   # read only: never adopted or renewed
                    if record['state'] == 'running' and self.renewing:
                        self.held.add(item.unit)   # adopt: from now on this TUI renews it too
                    elif record['state'] != 'running':
                        self.held.discard(item.unit)
                        self.started.discard(item.unit)

    def work(self):
        while True:
            self.refresh()
            try:
                job = self.jobs.get(timeout=self.period)
            except queue.Empty:
                continue
            if job is None:
                return
            verb, item, commander = job
            if verb == 'start':
                self.do_start(item, commander)
            else:
                self.do_stop(item)

    def begin(self):
        """Start the worker (does nothing without remote parts)."""
        if self.watched and self.thread is None:
            self.thread = threading.Thread(target=self.work, name='units', daemon=True)
            self.thread.start()
        return self

    def close(self, stop=True, adopted=False):
        """End the worker; with ``stop`` stop the units this TUI started, and with ``adopted``
        also the ones it adopted (another TUI's, by default left running). The rest are left to
        their leases."""
        if self.thread is not None:
            self.jobs.put(None)
            self.thread.join(timeout=CONNECT_S + STOP_WAIT + 20)
            self.thread = None
        with self.lock:
            held = [item for item in self.items if item.unit in self.held]
            ending = [item for item in held if stop and (adopted or item.unit in self.started)]
        kept = [item.title for item in held if item not in ending]
        failed = [item.title for item in ending if self.do_stop(item)]
        stopped = [item.title for item in ending if item.title not in failed]
        return '; '.join(filter(None, [stopped and f'stopped {", ".join(stopped)}',
                                       failed and f'could not stop {", ".join(failed)}',
                                       kept and f'kept {", ".join(kept)} (lease {LEASE_S:.0f} s)'])) \
            or 'none held'

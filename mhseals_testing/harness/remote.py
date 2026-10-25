"""Parts on other computers, from the TUI: the unit CLI (harness/unit.py) over ssh + docker exec.

One call reaches a part's computer: ``ssh <options> user@address docker exec -i <container> bash -c
'python3 -m mhseals_testing.harness.unit <verb> ...'`` (device-topology.md section 4). The key and
known hosts live in ``~/.config/mhseals/ssh/`` (already mounted in the container); a ``via`` host is
the jump. The keeper owns the part's process group on that computer, so a dropped ssh session or a
crashed TUI never orphans it: the lease simply runs out.

``Units`` keeps each remote part's status for the Status rows, without blocking them: a worker thread
runs the start and stop requests and, every ``PERIOD`` s, one call per host that renews the units
this TUI holds and reads every unit's status. A unit found running is adopted (renewed from then on),
which is how a later TUI on any computer takes over. An ssh failure marks the host unreachable,
which is not off. Stop is never refused; a commander part starts only with ``commander`` set, which
the Stack passes after the operator's confirm.
"""

import json
import os
from pathlib import Path
import queue
import shlex
import subprocess
import threading

from mhseals_testing.harness.registry import runtime_dir
from mhseals_testing.harness.unit import LEASE_S, START_WAIT, STOP_WAIT

CONNECT_S = 6
PERIOD = 30.0            # s between status + renew calls; well inside the lease
UNIT = ('python3', '-m', 'mhseals_testing.harness.unit')
REASONS = (('timed out', 'timeout'), ('Connection refused', 'refused'), ('No route', 'no route'),
           ('Could not resolve', 'unknown host'), ('Permission denied', 'key refused'),
           ('Host key verification', 'unknown host key'), ('is not running', 'container stopped'),
           ('No such container', 'no container'))


def _key_dir():
    return Path(os.environ.get('XDG_CONFIG_HOME') or Path.home() / '.config') / 'mhseals' / 'ssh'


def _options():
    keys = _key_dir()
    return ['-o', 'BatchMode=yes', '-o', f'ConnectTimeout={CONNECT_S}', '-o', 'ControlMaster=auto',
            '-o', 'ControlPersist=60', '-o', f'ControlPath={runtime_dir()}/ssh-%C',
            '-i', str(keys / 'id_ed25519'), '-o', f'UserKnownHostsFile={keys / "known_hosts"}']


def _target(host):
    return f'{host.user}@{host.address}' if host.user else host.address


def ssh_argv(host, command, jump=None):
    """argv that runs ``command`` (one shell line) in ``host``'s container, through ``jump`` if given.

    The jump gets the same key and known hosts (``-J`` would not pass them on).
    """
    argv = ['ssh', *_options()]
    if jump is not None:
        argv += ['-o', 'ProxyCommand=' + shlex.join(['ssh', *_options(), '-W', '%h:%p', _target(jump)])]
    return argv + [_target(host), 'docker', 'exec', '-i', host.container, 'bash', '-c',
                   shlex.quote(command)]


def reason(stderr, code):
    """A short why for a failed call: 'ssh: timeout', 'ssh: key refused', ..."""
    lines = [line for line in (stderr or '').strip().splitlines() if line.strip()]
    text = lines[-1] if lines else f'exit {code}'
    for needle, short in REASONS:
        if needle in text:
            return f'ssh: {short}'
    return f'ssh: {text[:60]}'


def call(host, command, jump=None, timeout=CONNECT_S + 10):
    """(JSON lines, error): run one unit command on ``host``; error is set when it was not reached."""
    try:
        done = subprocess.run(ssh_argv(host, command, jump), capture_output=True, text=True,
                              timeout=timeout, stdin=subprocess.DEVNULL)
    except subprocess.TimeoutExpired:
        return [], 'ssh: timeout'
    except OSError as error:
        return [], f'ssh: {error.strerror or error}'
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


def unit_command(verb, unit=None, argv=(), by=None, commander=False):
    """The unit CLI line for ``verb``; the internal ``--commander`` is only ever set here."""
    words = [*UNIT, verb, *([unit] if unit else [])]
    if verb == 'start':
        words += ['--lease-s', str(LEASE_S), '--by', json.dumps(by or {}, sort_keys=True)]
        words += ['--commander'] if commander else []
        words += ['--', *argv]
    return shlex.join(words)


class Units:
    """Remote parts' status, and their start, stop and lease renewal, off the UI thread.

    ``items`` are the Stack's components (the local ones are ignored), ``hosts`` the registry's
    hosts by name (for ``via``), ``by`` who starts (host, pid, session) and ``renew`` False for a
    TUI that only watches: it reads status but keeps nothing alive.
    """

    def __init__(self, items, hosts=None, by=None, renew=True, run=call, period=PERIOD):
        self.items = [item for item in items if item.host is not None]
        if hosts is None and any(item.host.via for item in self.items):
            from mhseals_testing import devices
            hosts = devices.load().hosts
        self.hosts = dict(hosts or {})
        self.by, self.renewing, self.run, self.period = dict(by or {}), renew, run, period
        self.lock = threading.Lock()
        self.records, self.errors, self.held, self.notices = {}, {}, set(), []
        self.jobs = queue.Queue()
        self.thread = None
        for item in self.items:
            if not item.host.ssh:
                self.errors[item.host.name] = f'no ssh to {item.host.name}'

    def status(self, item):
        """(unit record or None, why the host is unreachable or None)."""
        with self.lock:
            return self.records.get(item.unit), self.errors.get(item.host.name)

    def start(self, item, commander=False):
        with self.lock:
            self.records[item.unit] = {'state': 'running', 'pending': True}
        self.jobs.put(('start', item, commander))

    def stop(self, item):
        with self.lock:
            self.held.discard(item.unit)
            self.records[item.unit] = {'state': 'off', 'pending': True}
        self.jobs.put(('stop', item, False))

    def held_by_host(self):
        """{host name: [titles]} of the units this TUI keeps alive."""
        found = {}
        with self.lock:
            for item in self.items:
                if item.unit in self.held:
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
            if error:
                self.errors[host.name] = error
                for item in self.items:            # nothing is known about an unreachable host
                    if item.host.name == host.name:
                        self.records.pop(item.unit, None)
            else:
                self.errors.pop(host.name, None)
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
            self._note(f'{item.title} stopped on {item.host.name}')
        else:
            self._note(f'{item.title} stop on {item.host.name} failed: {error or found["error"]}')
        return error or found.get('error')

    def refresh(self):
        """One call per host: renew what we hold, then read every unit's status (and adopt)."""
        for name in sorted({item.host.name for item in self.items}):
            mine = [item for item in self.items if item.host.name == name]
            host = mine[0].host
            with self.lock:
                renew = [item.unit for item in mine if item.unit in self.held] if self.renewing else []
            command = ' ; '.join([*(f'{unit_command("renew", unit)} >/dev/null 2>&1' for unit in renew),
                                  unit_command('status')])
            results, error = self._call(host, command, CONNECT_S + 10)
            if error:
                continue
            found = {record.get('unit'): record for record in results}
            with self.lock:
                for item in mine:
                    record = found.get(item.unit, {'unit': item.unit, 'state': 'off'})
                    current = self.records.get(item.unit, {})
                    if current.get('pending') and record['state'] != current['state']:
                        continue                   # our start or stop is still on its way
                    self.records[item.unit] = record
                    if record['state'] == 'running' and self.renewing:
                        self.held.add(item.unit)   # adopt: from now on this TUI renews it too
                    elif record['state'] != 'running':
                        self.held.discard(item.unit)

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
        if self.items and self.thread is None:
            self.thread = threading.Thread(target=self.work, name='units', daemon=True)
            self.thread.start()
        return self

    def close(self, stop=True):
        """End the worker; with ``stop`` stop every unit we hold (else their leases run out)."""
        if self.thread is not None:
            self.jobs.put(None)
            self.thread.join(timeout=CONNECT_S + STOP_WAIT + 20)
            self.thread = None
        with self.lock:
            held = [item for item in self.items if item.unit in self.held]
        if not stop:
            return f'kept {", ".join(item.title for item in held)} (lease {LEASE_S:.0f} s)' if held \
                else 'none held'
        failed = [item.title for item in held if self.do_stop(item)]
        stopped = [item.title for item in held if item.title not in failed]
        return '; '.join(filter(None, [stopped and f'stopped {", ".join(stopped)}',
                                       failed and f'could not stop {", ".join(failed)}'])) or 'none held'

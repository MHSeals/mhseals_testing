"""The session registry: who owns the boat, which children run, who may adopt them.

One JSON file in the runtime directory, read-modify-written under a flock and
replaced atomically, so keepers and other instances can read it without a lock.
An owner whose pid is dead is stale: it can be claimed, and its children adopted
or stopped, without a handshake. The clock, sleep, kill and liveness test are
injectable so two instances can be simulated in one test. ROS-free.
"""

import fcntl
import json
import os
from pathlib import Path
import signal
import tempfile
import time

HANDOVER_TIMEOUT = 5.0


def runtime_dir():
    """``$XDG_RUNTIME_DIR/mhseals`` (else ``/tmp/mhseals-<uid>``), mode 0700."""
    base = os.environ.get('XDG_RUNTIME_DIR')
    path = Path(base) / 'mhseals' if base else Path(tempfile.gettempdir()) / f'mhseals-{os.getuid()}'
    path.mkdir(parents=True, exist_ok=True, mode=0o700)
    return path


def alive(pid):
    """True for a running process; a zombie is dead for our purposes."""
    try:
        state = Path(f'/proc/{int(pid)}/stat').read_text().rsplit(')', 1)[1].split()[0]
    except (OSError, ValueError, IndexError):
        return False
    return state != 'Z'


class Busy(RuntimeError):
    """Another live instance owns the session; ``entry`` is its registry record."""

    def __init__(self, entry):
        super().__init__(f"session owned by pid {entry['owner']['pid']}")
        self.entry = entry


class Registry:
    def __init__(self, path=None, alive=alive, clock=time.time, sleep=time.sleep, kill=os.kill):
        self.path = Path(path) if path else runtime_dir() / 'session.json'
        self.alive, self.clock, self.sleep, self.kill = alive, clock, sleep, kill

    def read(self):
        try:
            return json.loads(self.path.read_text())
        except (OSError, ValueError):
            return None

    def _locked(self, change):
        """Apply ``change(entry)`` to the entry (a dict, {} when absent) under the lock."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.path.with_name(self.path.name + '.lock'), 'w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            entry = self.read() or {}
            result = change(entry)
            temporary = self.path.with_name(f'{self.path.name}.{os.getpid()}.tmp')
            temporary.write_text(json.dumps(entry, indent=1))
            os.replace(temporary, self.path)
            return result

    def state(self):
        """The entry, {} when there is none yet."""
        return self.read() or {}

    def owner(self):
        """The recorded owner, live or not, or None when the seat is free."""
        return self.state().get('owner')

    def stale(self):
        owner = self.owner()
        return owner is not None and not self.alive(owner['pid'])

    def claim(self, owner, session=None):
        """Become the owner; raises Busy while another live process owns it.

        ``session`` None keeps the recorded session (an adopter continues it).
        """
        def change(entry):
            current = entry.get('owner')
            if current and current['pid'] != owner['pid'] and self.alive(current['pid']):
                raise Busy(entry)
            entry.update(owner=owner, handover=None)
            if session is not None:
                entry['session'] = session
            entry.setdefault('children', {})
            entry.setdefault('guests', {})
        self._locked(change)

    def update(self, change=None, **fields):
        """Set ``fields``, or run ``change(entry)`` on the entry, under the lock."""
        self._locked(change or (lambda entry: entry.update(fields)))

    def set_children(self, children):
        self.update(children=children)

    def children(self):
        return self.state().get('children', {})

    def request_handover(self, to_pid, timeout=HANDOVER_TIMEOUT):
        """Ask the owner to detach (SIGUSR1) and wait for the seat to empty. True if it let go."""
        owner = self.owner()
        if owner is None:
            return True
        deadline = self.clock() + timeout
        # The keepers read the deadline: an unexpired handover keeps the children alive.
        self.update(handover={'to': to_pid, 'deadline': deadline})
        if self.alive(owner['pid']):
            self.kill(owner['pid'], signal.SIGUSR1)
        while self.clock() < deadline:
            if self.owner() is None:
                return True
            self.sleep(0.1)
        return self.owner() is None

    def release(self, pid=None, keep_children=True):
        """Give up the seat (only ``pid``'s, when given); drop the children unless handed on."""
        def change(entry):
            if pid is not None and (entry.get('owner') or {}).get('pid') != pid:
                return
            entry['owner'] = None
            if not keep_children:
                entry.update(children={}, handover=None)
        self._locked(change)

    def add_guest(self, pid, node, procedure):
        self._locked(lambda entry: entry.setdefault('guests', {}).update(
            {str(pid): {'node': node, 'procedure': procedure}}))

    def remove_guest(self, pid):
        self._locked(lambda entry: entry.setdefault('guests', {}).pop(str(pid), None))

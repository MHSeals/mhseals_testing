"""The session registry: who owns the boat, which children run, who may adopt them.

One JSON file in the runtime directory, read-modify-written under a flock and
replaced atomically, so keepers and other instances can read it without a lock.
ROS-free.
"""

import fcntl
import json
import os
from pathlib import Path
import signal
import tempfile
import time


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
    def __init__(self, path=None):
        self.path = Path(path) if path else runtime_dir() / 'session.json'

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

    def claim(self, owner, session):
        """Become the owner; raises Busy while another live process owns it."""
        def change(entry):
            current = entry.get('owner')
            if current and current['pid'] != owner['pid'] and alive(current['pid']):
                raise Busy(entry)
            entry.update(owner=owner, session=session, handover=None)
            entry.setdefault('children', {})
            entry.setdefault('guests', {})
        self._locked(change)

    def update(self, **fields):
        self._locked(lambda entry: entry.update(fields))

    def request_handover(self, to_pid, timeout=5.0):
        """Ask the owner to detach (SIGUSR1) and wait until it lets go; returns the entry."""
        entry = self.read() or {}
        owner = entry.get('owner')
        self.update(handover={'to': to_pid, 'deadline': time.time() + timeout})
        if owner and alive(owner['pid']):
            os.kill(owner['pid'], signal.SIGUSR1)
        deadline = time.time() + timeout
        while time.time() < deadline:
            entry = self.read() or {}
            if entry.get('owner') is None:
                return entry
            time.sleep(0.1)
        raise TimeoutError('the owner did not let go')

    def release(self, keep_children):
        """Give up ownership; drop the children unless they are handed on."""
        self.update(owner=None, **({} if keep_children else {'children': {}, 'handover': None}))

    def add_guest(self, pid, node, procedure):
        self._locked(lambda entry: entry.setdefault('guests', {}).update(
            {str(pid): {'node': node, 'procedure': procedure}}))

    def remove_guest(self, pid):
        self._locked(lambda entry: entry.setdefault('guests', {}).pop(str(pid), None))

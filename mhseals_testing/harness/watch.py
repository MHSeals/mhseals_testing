"""Watch config files, keep the last good version, report each distinct error once.

Pure and ROS-free (spec authority_hotreload section 7, HR1). A file is polled
by ``stat`` (mtime_ns, size, inode), never inotify, which misses edits made
through bind mounts. A changed file is parsed only after its stat key has been
the same on two polls, so a half-written file is never read. A bad parse keeps
the last good value; the same error text twice is reported once. Our own saves
call ``reload`` to skip the settle wait.

Any commit that adds a YAML to the TUI registers it here.
"""

from dataclasses import dataclass, field
import hashlib
from pathlib import Path
import time

import yaml

PARSE_ERRORS = (OSError, ValueError, KeyError, TypeError, yaml.YAMLError)


@dataclass
class Watched:
    """One config surface: ``load(path)`` parses and validates, ``apply(value)`` takes effect."""
    name: str
    path: Path
    load: object
    apply: object = None
    subsystem: str = None   # config-history subsystem observed on success
    key: object = field(default=None, repr=False)      # stat key last applied or rejected
    seen: object = field(default=None, repr=False)     # stat key at the previous poll
    error: str = None
    loaded_at: float = None


@dataclass
class Change:
    """The outcome of one reload: ``report`` is False for an already-announced error."""
    name: str
    ok: bool
    error: str = None
    sha: str = None
    report: bool = True


def stat_key(path):
    """The file's (mtime_ns, size, inode); None when it cannot be read."""
    try:
        stat = Path(path).stat()
    except OSError:
        return None
    return stat.st_mtime_ns, stat.st_size, stat.st_ino


def describe(error):
    """One line for an error, with the YAML line number when the parser gave one."""
    mark = getattr(error, 'problem_mark', None)
    text = str(getattr(error, 'problem', None) or error).splitlines()[0] if str(error) else type(error).__name__
    return f'line {mark.line + 1}: {text}' if mark is not None else text


class Watcher:
    def __init__(self, clock=time.time):
        self.watches = {}
        self.clock = clock

    def add(self, watched):
        """Register ``watched``, taking the file as it is now as the baseline."""
        watched.key = watched.seen = stat_key(watched.path)
        watched.loaded_at = self.clock()
        self.watches[watched.name] = watched
        return watched

    def poll(self):
        """Reload every settled change; returns the list of ``Change``."""
        changes = []
        for watched in self.watches.values():
            key, previous = stat_key(watched.path), watched.seen
            watched.seen = key
            if key == watched.key:
                continue
            if key != previous:
                continue  # still changing: an in-place save caught mid-write is not read
            changes.append(self.reload(watched.name))
        return changes

    def reload(self, name):
        """Parse now (our own save, or a settled edit); keeps the old value on error."""
        watched = self.watches[name]
        watched.key = watched.seen = stat_key(watched.path)
        try:
            value = watched.load(watched.path)
            sha = self.sha(watched.path)
            if watched.apply is not None:
                watched.apply(value)
        except PARSE_ERRORS as error:
            text = describe(error)
            fresh = text != watched.error
            watched.error = text
            return Change(name, False, text, report=fresh)
        watched.error, watched.loaded_at = None, self.clock()
        return Change(name, True, sha=sha)

    @staticmethod
    def sha(path):
        try:
            return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:12]
        except OSError:
            return None

    def rows(self):
        """(name, ok, detail, loaded_at) per surface, for the Config row and popup."""
        return [(w.name, w.error is None, w.error or w.path.name, w.loaded_at)
                for w in self.watches.values()]

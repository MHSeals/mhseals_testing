"""The logs probe: known-failure lines from /rosout and from managed child logs.

Two feeds, no polling of ROS: ``on_rosout`` takes ``rcl_interfaces/Log`` messages
(WARN and above only; the level filter runs first, so INFO traffic costs one
compare) and ``tail`` reads the new bytes of every ``<name>.log`` that
ProcessManager writes. Fast DDS prints its RTPS errors to a child's stderr, never
to /rosout, which is why the tails exist.

The probe is told what to look for through ``watch`` (the ``log_pattern`` params of
every enabled check). Each line is first offered to the suppress patterns, which
count and swallow it, then to the others; a match is remembered for the pattern's
``hold`` seconds and published as one fact per pattern (see ``fact_name``).
"""

from collections import deque
from fnmatch import fnmatchcase
import re

from mhseals_testing.health.model import Fact

WARN = 30                    # rcl_interfaces/Log level: WARN, ERROR 40, FATAL 50
MAX_LINE = 4000              # longer lines are cut before matching


def fact_name(params):
    """The fact a ``log_pattern`` check reads: checks sharing these fields share one window."""
    return f'logs:{params.pattern}|{",".join(params.nodes)}|{params.key}|{params.hold:g}'


class LogProbe:
    def __init__(self, clock):
        self.clock = clock
        self._watches = {}       # fact name -> params
        self._regex = {}         # pattern -> compiled
        self._hits = {}          # fact name -> deque of (time, key, groups, line)
        self._suppressed = {}    # fact name -> lines swallowed so far
        self._offsets = {}       # log path -> bytes consumed
        self._partial = {}       # log path -> trailing text without a newline yet

    def watch(self, params_list):
        """Replace the watched patterns; windows of patterns that stay are kept."""
        self._watches = {fact_name(p): p for p in params_list}
        for name in list(self._hits):
            if name not in self._watches:
                del self._hits[name]
        for name in list(self._suppressed):
            if name not in self._watches:
                del self._suppressed[name]
        for p in params_list:
            if p.pattern not in self._regex:
                self._regex[p.pattern] = re.compile(p.pattern)

    def on_rosout(self, message):
        """Feed one rcl_interfaces/Log; its logger name becomes a node path."""
        if message.level >= WARN:
            self.feed('/' + message.name.replace('.', '/'), message.msg)

    def tail(self, directory):
        """Feed the lines appended to ``<directory>/*.log`` since the last call.

        Logs belong to one session (a fresh directory), so a file is read from its
        start the first time it is seen. A file that shrank was truncated: reread it.
        """
        for path in sorted(directory.glob('*.log')):
            try:
                size = path.stat().st_size
                offset = self._offsets.get(path, 0)
                if size < offset:
                    offset = 0
                    self._partial.pop(path, None)
                if size == offset:
                    continue
                with path.open('rb') as handle:
                    handle.seek(offset)
                    chunk = handle.read(size - offset)
            except OSError:
                continue
            self._offsets[path] = offset + len(chunk)
            text = self._partial.pop(path, '') + chunk.decode(errors='replace')
            *lines, rest = text.split('\n')
            if rest:
                self._partial[path] = rest
            for line in lines:
                self.feed('/' + path.stem, line)

    def feed(self, node, text, now=None):
        """Offer one line from ``node`` to the watched patterns."""
        now = self.clock() if now is None else now
        text = text.rstrip('\r')[:MAX_LINE]
        if not text.strip():
            return
        found = [(name, p, self._regex[p.pattern].search(text)) for name, p in self._watches.items()
                 if not p.nodes or any(fnmatchcase(node, glob) for glob in p.nodes)]
        found = [(name, p, m) for name, p, m in found if m]
        swallowed = [name for name, p, _ in found if p.suppress]
        for name in swallowed:
            self._suppressed[name] = self._suppressed.get(name, 0) + 1
        if swallowed:
            return
        for name, p, match in found:
            groups = {k: v for k, v in match.groupdict().items() if v is not None}
            key = groups.get(p.key, '') if p.key else ''
            self._hits.setdefault(name, deque()).append((now, key, groups, text.strip()))

    def facts(self, now=None):
        """One Fact per watched pattern: matches in the last ``hold`` s, for its worst key.

        value: the count for the most frequent key (0 when quiet); label: that key;
        data: ``counts`` per key, the named ``groups`` and the ``line`` of its latest match.
        Suppress patterns publish the number of lines swallowed so far.
        """
        now = self.clock() if now is None else now
        out = {}
        for name, p in self._watches.items():
            if p.suppress:
                out[name] = Fact(self._suppressed.get(name, 0), time=now)
                continue
            hits = self._hits.get(name, deque())
            while hits and now - hits[0][0] > p.hold:
                hits.popleft()
            counts, latest = {}, {}
            for _, key, groups, line in hits:
                counts[key] = counts.get(key, 0) + 1
                latest[key] = (groups, line)
            worst = max(counts, key=counts.get) if counts else ''
            groups, line = latest.get(worst, ({}, ''))
            out[name] = Fact(counts.get(worst, 0), label=worst,
                             data={'counts': counts, 'groups': groups, 'line': line}, time=now)
        return out

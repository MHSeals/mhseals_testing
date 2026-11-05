"""The background orchestrator's open questions and proposals (D-20261101-31), for the agent panel's one line.
ROS-free, read-only.

They are events in mhseals_agents ``state/events/*.jsonl`` (one file per station and day, see its
``state/README.md``): an item's kind is its first event's type, and it stays open until an event of it sets a
closing ``status`` (``./ev question ID --status done``, a proposal adopted or dropped). Every station counts.

mhseals_agents is ``$MHSEALS_AGENTS`` when set, else agent.yaml ``background``; neither (or no state/events there):
nothing to show. This package never guesses another repo's checkout path.
"""

import json
import os
import time
from pathlib import Path

CLOSED = ('done', 'closed', 'dropped', 'revoked', 'expired', 'adopted', 'answered')   # ev's, plus the plain words
KINDS = ('question', 'proposal')
EVERY_S = 5.0       # a frame reads the cached counts; the files are re-read at most this often
MAX_BYTES = 2 << 20  # a re-read reads at most this much, the newest files first: a frame never waits on a big log


def root(configured=''):
    """The mhseals_agents checkout, or None. A set ``$MHSEALS_AGENTS`` is the only place looked (tests point it away)."""
    env = os.environ.get('MHSEALS_AGENTS')
    candidate = env if env is not None else configured
    if candidate and (Path(candidate).expanduser() / 'state' / 'events').is_dir():
        return Path(candidate).expanduser()
    return None


def read(agents):
    """``{'question': [...], 'proposal': [...]}``: the open items, oldest first, each ``{id, title, station, ts,
    branch, verdict}``."""
    items, texts, budget = {}, [], MAX_BYTES
    for path in sorted((Path(agents) / 'state' / 'events').glob('*.jsonl'),
                       key=lambda p: (p.stem.rsplit('-', 1)[-1], p.name), reverse=True):   # newest day first
        if budget <= 0:
            break
        try:
            with open(path, 'rb') as handle:
                data = handle.read(budget)
        except OSError:
            continue
        budget -= len(data)
        texts.append((path, data.decode('utf-8', 'replace')))
    for path, text in reversed(texts):
        lines = text.splitlines()
        station = path.stem.rsplit('-', 1)[0]
        for line in lines:
            try:
                event = json.loads(line)
            except ValueError:
                continue
            if not isinstance(event, dict) or not event.get('id') or not event.get('type'):
                continue
            ident = str(event['id'])
            item = items.setdefault(ident, {'id': ident, 'kind': str(event['type']), 'station': station,
                                            'ts': str(event.get('ts', '')), 'title': '', 'closed': False})
            for key in ('title', 'branch', 'verdict'):
                if event.get(key):
                    item[key] = str(event[key])
            if event.get('status') in CLOSED or event['type'] == 'landed':
                item['closed'] = True
    open_items = sorted((i for i in items.values() if i['kind'] in KINDS and not i['closed']), key=lambda i: i['ts'])
    return {kind: [i for i in open_items if i['kind'] == kind] for kind in KINDS}


def line(found):
    """``background: N questions · M proposals``; '' when nothing is open."""
    questions, proposals = len(found.get('question') or ()), len(found.get('proposal') or ())
    if not questions and not proposals:
        return ''
    plural = lambda n, word: f"{n} {word}{'' if n == 1 else 's'}"
    return f"background: {plural(questions, 'question')} · {plural(proposals, 'proposal')}"


class Reader:
    """``get()``: the open items, re-read at most every ``EVERY_S`` and only when an events file changed."""

    def __init__(self, configured=lambda: '', clock=time.monotonic):
        self.configured, self.clock = configured, clock
        self.at, self.key, self.found = None, None, {kind: [] for kind in KINDS}

    def get(self):
        now = self.clock()
        if self.at is not None and now - self.at < EVERY_S:
            return self.found
        self.at = now
        try:
            agents = root(self.configured())
            files = sorted((agents / 'state' / 'events').glob('*.jsonl')) if agents else []
            key = tuple((str(p), p.stat().st_mtime_ns, p.stat().st_size) for p in files)
            if key != self.key:
                self.key, self.found = key, read(agents) if agents else {kind: [] for kind in KINDS}
        except Exception:           # noqa: BLE001 - a file that moved mid-read, a bad line: the last counts stand
            pass
        return self.found

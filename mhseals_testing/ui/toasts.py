"""Health toasts: a pure model of the short-lived notices shown bottom-right (health.md 5.4).

Only engine transitions toast, and only warn and fail raise one; info and unknown never do.
The model takes the clock as an argument and never reads keys. While a modal is open nothing is
shown, and what arrived meanwhile collapses into one summary toast when it closes.
"""

import dataclasses

from mhseals_testing.ui import theme

LIFETIME = {'warn': 6.0, 'ok': 3.0, 'fail': None}   # seconds; None stays until recovery or ack
TITLES = {'warn': ' Warning ', 'fail': ' Error ', 'ok': ' Recovered ', 'info': ' Health '}
MAX_SHOWN = 3
DEDUPE = 30.0                                       # a repeat inside this window bumps the count


@dataclasses.dataclass
class Toast:
    id: str
    kind: str                # warn, fail, ok (recovered) or info (summary)
    text: str
    born: float
    count: int = 1
    expires: float | None = None

    @property
    def title(self):
        return TITLES[self.kind]


class Toasts:
    def __init__(self):
        self._items = {}         # id -> Toast, in arrival order
        self._seen = {}          # id -> (first raise time, count): dedupe outlives expiry and recovery
        self._raised = set()     # ids that toasted when they raised: only they announce recovery
        self._backlog = 0        # toasts raised while suppressed
        self._was_suppressed = False

    def push(self, id, kind, text, now):
        """Raise (warn, fail) or recover (ok) a toast; other kinds are ignored."""
        if kind == 'ok':
            if id not in self._raised:
                return
            self._raised.discard(id)
            self._items.pop(id, None)
        elif kind in ('warn', 'fail'):
            self._raised.add(id)
        else:
            return
        first, count = self._seen.get(id, (now, 0))
        if kind != 'ok':
            if now - first > DEDUPE:
                first, count = now, 0
            self._seen[id] = (first, count + 1)
            count += 1
        else:
            count = 1
        life = LIFETIME[kind]
        self._items.pop(id, None)
        self._items[id] = Toast(id, kind, text, now, count, None if life is None else now + life)
        self._backlog += 1

    def transition(self, transition, title, now):
        """Feed one engine Transition; `title` names the check."""
        detail = transition.result.detail
        self.push(transition.id, transition.new, f'{title}: {detail}' if detail else title, now)

    def ack(self, id=None):
        """Dismiss one toast, or every standing one."""
        for key in [id] if id else list(self._items):
            self._items.pop(key, None)

    def visible(self, now, suppressed=False):
        """The toasts to draw, newest last, at most MAX_SHOWN, then `hidden(now)` more."""
        items = self._live(now)
        if suppressed:
            self._was_suppressed = True
            return []
        if self._was_suppressed:                  # a modal just closed: summarise what it hid
            self._was_suppressed = False
            arrived = self._backlog
            if len(items) > 1 and arrived > 1:
                kept = [t for t in items if t.kind == 'fail']
                summary = Toast('summary', 'info', f'Health: {arrived} new{theme.SEPARATOR}! details', now,
                                expires=now + LIFETIME['warn'])
                self._items = {t.id: t for t in kept}
                self._items['summary'] = summary
                items = list(self._items.values())
        self._backlog = 0
        return items[-MAX_SHOWN:]

    def hidden(self, now):
        return max(0, len(self._live(now)) - MAX_SHOWN)

    def _live(self, now):
        self._items = {k: t for k, t in self._items.items() if t.expires is None or t.expires > now}
        return list(self._items.values())

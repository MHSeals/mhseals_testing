"""The preview's sanity checklist: a side panel the operator marks pass, fail or note (harness/preview.py).

Shown only when boat_tui preview sets ``BOAT_PREVIEW_CHECKLIST`` (the checklist JSON) and
``BOAT_PREVIEW_RESULTS`` (the directory of ``preview-<date>.md``, appended per mark, never rewritten).
W (keymap global ``checklist``) turns checking on and off. While on: up/down or j/k move, p pass, f fail, n note (a popup), Esc leaves;
every other key (Space, q, a, ...) acts as usual.
"""

import json
import os
from pathlib import Path

from rich.console import Group
from rich.text import Text

from mhseals_testing.harness import preview
from mhseals_testing.ui import keymap, theme
from mhseals_testing.ui.panels import window

WIDTH = 44
STATE = {'pass': 'PASS', 'fail': 'FAIL', 'note': 'WARN', None: 'OFF'}


class Checklist:
    def __init__(self, data, results=None, toggles=('W',)):
        self.toggles = set(toggles)
        self.ref = data.get('ref', '')
        self.items = list(data.get('items') or [])
        self.results = Path(results) if results else None
        self.marks = {}          # item id -> (mark, note)
        self.cursor = 0
        self.active = False
        self.started = False     # the results file got this preview's header

    @classmethod
    def from_env(cls, environ=None):
        """The checklist boat_tui preview hands the TUI, else None (any other boat_test)."""
        environ = os.environ if environ is None else environ
        path = environ.get('BOAT_PREVIEW_CHECKLIST')
        if not path:
            return None
        try:
            data = json.loads(Path(path).read_text())
        except (OSError, ValueError):
            return None
        try:
            bound = keymap.key_map(keymap.load()[0].doc.get('global'))
            toggles = [key for key, action in bound.items() if action == 'checklist'] or ['W']
        except Exception:                 # a broken user keymap: the TUI reports it; W still works here
            toggles = ['W']
        return cls(data, environ.get('BOAT_PREVIEW_RESULTS') or Path(path).parent, toggles)

    def mark(self, mark, note=''):
        """Mark the cursor item and append the line to the results file; returns the notice."""
        if not self.items:
            return 'checklist is empty'
        item = self.items[self.cursor]
        self.marks[item['id']] = (mark, note)
        line = preview.result_line(item, mark, note)
        if self.results is not None:
            try:
                path = preview.results_path(self.results)
                with open(path, 'a') as out:
                    if not self.started:
                        out.write(preview.header_line(self.ref))
                        self.started = True
                    out.write(line)
            except OSError as error:
                return f'checklist not saved: {error}'
        self.cursor = min(len(self.items) - 1, self.cursor + 1)
        return f"{mark} {item['text'][:40]}"

    def handle(self, app, raw):
        """Apply ``raw`` when it is a checklist key; True when consumed."""
        if raw in self.toggles:
            self.active = not self.active
            app.notice = 'checklist: p pass · f fail · n note · Esc leaves' if self.active else 'checklist off'
            return True
        if not self.active:
            return False
        if raw in ('up', 'k', 'down', 'j'):
            step = -1 if raw in ('up', 'k') else 1
            self.cursor = max(0, min(len(self.items) - 1, self.cursor + step))
        elif raw in ('p', 'f'):
            app.notice = self.mark('pass' if raw == 'p' else 'fail')
        elif raw == 'n':
            note = app.ask_text('Checklist note', self.items[self.cursor]['text'] if self.items else '')
            if note:
                app.notice = self.mark('note', note)
        elif raw == 'escape':
            self.active = False
            app.notice = 'checklist off'
        else:
            return False
        return True

    def counts(self):
        marks = [m for m, _ in self.marks.values()]
        return {m: marks.count(m) for m in preview.MARKS}

    def panel(self, height):
        """The side panel: items by group, the cursor item's detail under them."""
        rows, last = [], None
        for index, item in enumerate(self.items):
            if item['group'] != last:
                rows.append(('heading', item['group']))
                last = item['group']
            rows.append(('item', index))
        at = next((n for n, row in enumerate(rows) if row == ('item', self.cursor)), 0)
        _, visible = window(rows, at, max(1, height - 5))
        lines = []
        for kind, value in visible:
            if kind == 'heading':
                lines.append(Text(value, style=f'{theme.MUTED} bold'))
                continue
            item = self.items[value]
            mark = self.marks.get(item['id'], (None, ''))[0]
            label = Text(item['text'], no_wrap=True, overflow=theme.OVERFLOW)
            lines.append(theme.row(STATE[mark], label, self.active and value == self.cursor))
        current = self.items[self.cursor] if self.items else {}
        detail = current.get('detail') or (self.marks.get(current.get('id'), (None, ''))[1]) or current.get('text', '')
        counts = self.counts()
        subtitle = f"{counts['pass']}✓ {counts['fail']}✗ {len(self.items) - len(self.marks)} left"
        body = Group(*lines, Text(''), Text(detail, style=theme.MUTED, no_wrap=True, overflow=theme.OVERFLOW))
        keys = 'p pass · f fail · n note · Esc leave' if self.active else f'{min(self.toggles)} check'
        return theme.block(body, 'Checklist', self.active, theme.ascii_safe(subtitle), keys=keys)

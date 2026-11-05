"""The Labels tab: review a session's marks and segments and set the water conditions (ui/panels/labels.py).

It shows the live session, or any closed one with < >. Accept, reject, tag, note, split and merge are
append-only ``label_edit`` lines, and the manifest beside the bag is rewritten after each
(annotate/manifest.py). Review edits wait for disarmed and still; the conditions rows always work.
"""

import dataclasses
import time

from mhseals_testing import data
from mhseals_testing.annotate import config, fold as fold_module, manifest as manifest_module
from mhseals_testing.ui import keymap

LISTED = ('mark', 'bad', 'intervention', 'said', 'procedure', 'steady', 'neutral', 'link', 'fence')
OUTCOMES = ('done', 'partly', 'failed', 'unsafe', 'ignored')     # what a language command led to
EDITS = ('accept', 'reject', 'undo', 'split', 'merge', 'note_label', 'replay', 'tag')   # tab.labels: wait for disarmed


@dataclasses.dataclass
class LabelsState:
    """What the Labels tab owns, and nothing else reads but through it."""
    session: object = None       # the session folder shown; None is the live one
    came_from: str = None        # the tab L or Esc returns to
    conditions: dict = None      # wind, waves, current as last set here (the next mark carries them)
    cache: tuple = (None, None)  # (stamp, manifest) of the session shown


class Labels:
    """Mixed into App."""

    @property
    def labels(self):
        """The Labels tab's own state (LabelsState), made on first use."""
        state = self.__dict__.get('_labels_state')
        if state is None:
            state = self.__dict__['_labels_state'] = LabelsState()
        return state

    def labels_session(self):
        return self.labels.session or self.paths.directory

    def labels_live(self):
        return self.labels_session() == self.paths.directory

    def labels_doc(self):
        """The folded manifest of the shown session, re-folded when its journal changed."""
        folder = self.labels_session()
        try:
            stamp = (str(folder), (folder / manifest_module.JOURNAL).stat().st_mtime_ns)
        except OSError:
            stamp = (str(folder), None)
        if self.labels.cache[0] != stamp:
            self.labels.cache = (stamp, manifest_module.build(folder))
        return self.labels.cache[1]

    def labels_rows(self):
        doc = self.labels_doc()
        items = [s for s in doc['segments'] if s['kind'] in LISTED]
        items += [dict(c, kind='said', t0=c['t'], t1=c['t'], label=c['text']) for c in doc['commands']]
        return sorted(items, key=lambda i: (i['t0'], i['id']))

    def labels_count(self):
        return len(config.load()['scales']) + len(self.labels_rows())

    def labels_selected(self):
        rows, cursor = self.labels_rows(), self.cursors.get('labels', 0) - len(config.load()['scales'])
        return rows[cursor] if 0 <= cursor < len(rows) else None

    def labels_model(self):
        """What the Labels tab draws (ui/panels/labels.py)."""
        doc, scales = self.labels_doc(), config.load()['scales']
        current = fold_module.conditions_at([(c['t'], c) for c in doc['conditions']], float('inf')) or {}
        cursor = min(self.cursors.get('labels', 0), max(0, self.labels_count() - 1))
        return {'session': doc['session'], 'live': self.labels_live(), 'scales': scales,
                'conditions': {k: int(current.get(k, 0)) for k in scales}, 'rows': self.labels_rows(),
                'armed': [s for s in doc['segments'] if s['kind'] == 'armed'],
                'calibration': {c['id'] for c in doc['calibration']}, 'commands': doc['commands'],
                'cursor': cursor, 'selected': self.labels_selected(), 'pending': doc['pending'],
                'readonly': bool(self.armed or self.moving()), 't0': doc['time']['t0'], 't1': doc['time']['t1']}

    def labels_write(self, phase, **fields):
        """Journal on the live session (it rides in the bag), or append to a closed one; then the manifest."""
        if self.labels_live():
            self.emit(phase, **fields)
        else:
            data.append_jsonl(self.labels_session() / manifest_module.JOURNAL,
                              {'t_ns': time.time_ns(), 'phase': phase, 'rec': False, 'edited': True, **fields})
        self.labels.cache = (None, None)
        manifest_module.write(self.labels_session())

    def edit_label(self, action, value=None):
        item = self.labels_selected()
        if action != 'undo' and item is None:
            return
        self.labels_write('label_edit', target=None if action == 'undo' else item['id'], action=action, value=value)

    def set_condition(self, index, step):
        scales = config.load()['scales']
        current = dict(fold_module.conditions_at(
            [(c['t'], {k: c[k] for k in scales}) for c in self.labels_doc()['conditions']], float('inf'))
            or dict.fromkeys(scales, 0))
        current[scales[index]] = max(0, min(3, current[scales[index]] + step))
        self.labels.conditions = current
        self.labels_write('conditions_set', **current)

    def on_labels(self, key):
        """tab.labels keys; True when consumed. Edits wait for disarmed and still, the conditions never do."""
        scales = len(config.load()['scales'])
        cursor, action = self.cursors.get('labels', 0), keymap.dispatch('tab.labels', key)
        if action in ('scale_down', 'scale_up') and cursor < scales:
            self.set_condition(cursor, -1 if action == 'scale_down' else 1)
        elif action in ('session_prev', 'session_next'):
            self.labels_step(-1 if action == 'session_prev' else 1)
        elif action in EDITS:
            if self.armed or self.moving():
                self.notice = 'Labels: review edits wait for disarmed; b, N and " still mark'
            else:
                self.labels_edit_key(action, key)
        else:
            return False
        return True

    def labels_edit_key(self, action, key):
        """One tab.labels edit; ``key`` is the digit of a ``tag``."""
        item = self.labels_selected()
        if action == 'undo':
            self.edit_label('undo')
        elif item is None:
            return
        elif action == 'accept':
            self.edit_label('pending' if item['status'] == 'accepted' else 'accept')
        elif action == 'reject':
            self.edit_label('pending' if item['status'] == 'rejected' else 'reject')
        elif action == 'tag' and item['kind'] == 'said':
            index = int(key) - 1
            if index < len(OUTCOMES):
                self.edit_label('outcome', OUTCOMES[index])
        elif action == 'tag' and item['kind'] == 'mark' and key in config.load()['tags']:
            self.edit_label('tag', config.load()['tags'][key]['label'])
        elif action == 'note_label':
            text = self.text_input('Said' if item['kind'] == 'said' else 'Note', item.get('label', '') or item['kind'])
            if text:
                self.edit_label('text' if item['kind'] == 'said' else 'note', text)
        elif action == 'split' and item['t1'] > item['t0']:
            inside = [i['t0'] for i in self.labels_rows() if i['kind'] == 'mark' and item['t0'] < i['t0'] < item['t1']]
            middle = (item['t0'] + item['t1']) / 2
            self.edit_label('split', min(inside, key=lambda t: abs(t - middle)) if inside else middle)
        elif action == 'merge':
            rows = self.labels_rows()
            after = next((i for i in rows if i['kind'] == item['kind'] and i['t0'] >= item['t1']
                          and i['id'] != item['id']), None)
            if after is not None:
                self.edit_label('merge', after['id'])
        elif action == 'replay':
            self.labels_replay(item)

    def labels_step(self, step):
        """< >: the previous or next session in the Bags list (the live one first)."""
        folders = [self.paths.directory] + [b.path for b in self.bags_view()[0]
                                            if not b.legacy and b.path != self.paths.directory]
        here = folders.index(self.labels_session()) if self.labels_session() in folders else 0
        self.labels.session = folders[max(0, min(len(folders) - 1, here + step))]
        self.cursors['labels'] = 0
        self.labels.cache = (None, None)

    def labels_replay(self, item):
        """Enter: play a closed session from just before the item (the Bags review)."""
        if self.labels_live():
            self.notice = 'Live session: its replay opens from Bags once it is closed'
            return
        bag = next((b for b in self.bags_view()[0] if b.path == self.labels_session()), None)
        if bag is not None:
            self.open_bag_review(bag)
            if self.review:
                self.review.go(item['t0'] - 5, self.clock())

    def review_chip(self):
        """``L Review N`` for the footer once a session has items to review and nothing moves; else None."""
        if self.armed or self.moving() or self.tab == 'Labels':
            return None
        try:
            pending = self.labels_doc()['pending'] if self.labels_live() else 0
        except (OSError, ValueError):
            return None
        return ('L', f'Review {pending}') if pending else None

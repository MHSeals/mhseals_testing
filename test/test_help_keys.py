"""The ? help names only bound keys, and every page action the app handles has a help row.

HELP labels read '[Screen ]keys'; the screen picks a keymap stack, the keys resolve in it.
Popups are left out: each draws its own key hint line.
"""
import re

import pytest

from mhseals_testing.ui import keymap
from mhseals_testing.ui.app import HELP, LEFTRIGHT, UPDOWN

SCREENS = {   # label prefix -> keymap stack (outermost first); no prefix is global
    None: ['global'],
    'Drive': ['global', 'tab.drive'],
    'Status': keymap.STACKS['drive/status'],
    'Thrusters': keymap.STACKS['drive/thrusters'],
    'Activity': keymap.STACKS['drive/activity'],
    'Map': keymap.STACKS['map'],
    'Correction': keymap.STACKS['correction'],
    'Bags': keymap.STACKS['bags'],
    'Fusion': keymap.STACKS['fusion'],
    'Review': keymap.STACKS['review/drive'],
    'Manual': ['manual'],
    'Run': ['tab.drive', 'runner'],   # m (Manual) takes over a run
}
NAMES = {'Shift Tab': 'backtab', 'Space': 'space', 'Enter': 'enter', 'Esc': 'escape', 'Tab': 'tab'}
# Bound in config/keymap.yaml but not handled by the app yet, so help leaves them out.
UNHANDLED = {('panel.procedures', 'fold'), ('panel.procedures', 'unfold'),
             ('panel.activity', 'view_prev'), ('panel.activity', 'view_next'), ('panel.activity', 'zoom'),
             ('view.minimap', 'course'), ('view.minimap', 'centre'), ('view.minimap', 'goal_mode'),
             ('view.minimap', 'cancel_goal'), ('tab.bags', 'send'), ('review', 'goto')}
COVERED = ('global', 'tab.drive', 'tab.correction', 'tab.map', 'tab.bags', 'tab.fusion', 'panel.status',
           'panel.thrusters', 'panel.procedures', 'panel.activity', 'view.minimap', 'review',
           'review.activity', 'manual', 'runner')


def parse(label):
    """(screen, [key names]) of one HELP label."""
    first, _, rest = label.partition(' ')
    screen = first if first in SCREENS else None
    text = rest if screen else label
    ctrl = text.startswith('Ctrl ')
    text = text.removeprefix('Ctrl ').replace('arrows', 'left down up right')
    text = text.replace(UPDOWN, 'up down').replace(LEFTRIGHT, 'left right')
    for name, key in NAMES.items():
        text = re.sub(rf'\b{name}\b', key, text) if name.isalpha() else text.replace(name, key)
    text = text.replace('1-4', '1 2 3 4')
    keys = [k for k in text.replace('/', ' ').split() if k]
    return screen, ['ctrl-' + k for k in keys] if ctrl else keys


def owner(doc, stack, key):
    """(context, action) that ``key`` resolves to in ``stack``, innermost first, or None."""
    for name in reversed(stack):
        action = keymap.key_map(doc.get(name)).get(key)
        if action:
            return name, action
    return None


def test_help_names_only_handled_bound_keys():
    doc = keymap.load(user=None)[0].doc
    for label, _ in HELP:
        screen, keys = parse(label)
        assert keys, label
        for key in keys:
            found = owner(doc, SCREENS[screen], key)
            assert found, f'{label!r}: {key} is not bound there'
            assert found not in UNHANDLED, f'{label!r}: {key} ({found[1]}) is not handled yet'


def test_every_handled_action_has_a_help_row():
    doc = keymap.load(user=None)[0].doc
    helped = {owner(doc, SCREENS[screen], key) for screen, keys in map(parse, (l for l, _ in HELP))
              for key in keys}
    missing = [(name, action) for name in COVERED for action in keymap.bindings(doc.get(name))
               if (name, action) not in helped and (name, action) not in UNHANDLED]
    assert not missing, f'bound without a help row: {missing}'
    assert not UNHANDLED & helped


pytestmark = pytest.mark.smoke   # tools/smoke

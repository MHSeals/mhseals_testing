"""The ? help names only bound keys, and every page action the app handles has a help row.

The help and hint rows live in config/keymap.yaml (help, hints); README.md's key table is generated.
HELP labels read '[Screen ]keys'; the screen picks a keymap stack, the keys resolve in it.
Popups are left out: each draws its own key hint line.
"""

import pytest

from pathlib import Path

from mhseals_testing.strict_yaml import load_strict
from mhseals_testing.ui import keymap
from mhseals_testing.ui.app import help as help_module

COVERED = ('global', 'tab.drive', 'tab.correction', 'tab.map', 'tab.bags', 'tab.fusion', 'panel.status',
           'panel.thrusters', 'panel.procedures', 'panel.activity', 'view.minimap', 'review',
           'review.activity', 'manual', 'runner')
# Bound in config/keymap.yaml but not handled by the app yet, so help leaves them out.
UNHANDLED = {('panel.procedures', 'fold'), ('panel.procedures', 'unfold'),
             ('panel.activity', 'view_prev'), ('panel.activity', 'view_next'), ('panel.activity', 'zoom'),
             ('view.minimap', 'course'), ('view.minimap', 'centre'), ('view.minimap', 'goal_mode'),
             ('view.minimap', 'cancel_goal'), ('tab.bags', 'send'), ('review', 'goto')}
ROOT = Path(__file__).resolve().parents[1]


def parse(label):
    """(stack, [key names]) of one help row as keymap.yaml writes it."""
    stack, keys = keymap.help_screen(label)
    return stack, keymap.label_keys(keys)


def owner(doc, stack, key):
    """(context, action) that ``key`` resolves to in ``stack``, innermost first, or None."""
    for name in reversed(stack):
        action = keymap.key_map(doc.get(name)).get(key)
        if action:
            return name, action
    return None


def rows(doc):
    return [row[0] for row in doc['help']]


def test_help_names_only_handled_bound_keys():
    doc = keymap.load(user=None)[0].doc
    for label in rows(doc):
        stack, keys = parse(label)
        assert keys, label
        for key in keys:
            found = owner(doc, stack, key)
            assert found, f'{label!r}: {key} is not bound there'
            assert found not in UNHANDLED, f'{label!r}: {key} ({found[1]}) is not handled yet'


def test_every_handled_action_has_a_help_row():
    doc = keymap.load(user=None)[0].doc
    helped = {owner(doc, stack, key) for stack, keys in map(parse, rows(doc)) for key in keys}
    missing = [(name, action) for name in COVERED for action in keymap.bindings(doc.get(name))
               if (name, action) not in helped and (name, action) not in UNHANDLED]
    assert not missing, f'bound without a help row: {missing}'
    assert not UNHANDLED & helped



def test_the_tables_come_from_the_keymap():
    doc = keymap.load(user=None)[0].doc
    assert len(help_module.HELP) == len(doc['help'])
    assert set(help_module.PANEL_HINTS) == {g.removeprefix('panel.') for g in doc['hints'] if g.startswith('panel.')}


def test_a_help_or_hint_key_that_is_not_bound_fails_validation():
    doc = load_strict((ROOT / 'config' / 'keymap.yaml').read_text())
    doc['help'].append(['Map Q', 'nothing'])
    doc['hints']['manual'].append(['W', 'upper-case w is not manual'])
    errors = keymap.validate(doc)
    assert any("'Map Q': Q is not bound" in e for e in errors)
    assert any("hints.manual: 'W'" in e for e in errors)
    assert not keymap.validate(doc, docs=False)        # a user's rebinds never trip on help text


def test_readme_key_table_is_generated():
    text = (ROOT / 'README.md').read_text()
    assert '<!-- keys: config/keymap.yaml -->' in text
    assert help_module.readme(text) == text, 'run python3 -m mhseals_testing.ui.app.help --readme'


pytestmark = pytest.mark.smoke   # tools/smoke

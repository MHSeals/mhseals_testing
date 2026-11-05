"""The shipped config/keymap.yaml: counts, binding rules and the Space/E invariants.

Self-contained until ui/keymap.py (ts011) exists; the ACTIONS coverage test then switches on.
"""

import importlib
import os

import pytest
import yaml

PATH = os.path.join(os.path.dirname(__file__), '..', 'config', 'keymap.yaml')
NAMED = set('space enter escape tab backtab backspace up down left right shift-left shift-right ctrl-up '
            'ctrl-down ctrl-left ctrl-right ctrl-h ctrl-j ctrl-k ctrl-l focus-out any'.split())
STACKS = {
    'drive/status': ['global', 'tab.drive', 'panel.status'],
    'drive/motion': ['global', 'tab.drive', 'panel.motion'],
    'drive/conditions': ['global', 'tab.drive', 'panel.conditions'],
    'drive/thrusters': ['global', 'tab.drive', 'panel.thrusters'],
    'drive/procedures': ['global', 'tab.drive', 'panel.procedures'],
    'drive/activity': ['global', 'tab.drive', 'panel.activity'],
    'drive/minimap': ['global', 'tab.drive', 'view.minimap'],
    'correction': ['global', 'tab.correction'],
    'map': ['global', 'tab.map', 'view.minimap'],
    'bags': ['global', 'tab.bags'],
    'review/drive': ['global', 'review', 'review.activity'],
    'review/map': ['global', 'review', 'view.minimap'],
}
MODAL = ('manual', 'runner', 'minimap.goal', 'popup.')


@pytest.fixture(scope='module')
def doc():
    with open(PATH) as f:
        doc = yaml.safe_load(f)
    return {k: v for k, v in doc.items() if k not in ('help', 'hints', 'fixed')}   # rows, not contexts (test_help_keys)


def binds(ctx):
    return {a: ks for a, ks in (ctx or {}).items() if a not in ('override', 'passthrough')}


def key_map(ctx):
    out = {}
    for action, keys in binds(ctx).items():
        for k in keys:
            assert k not in out, '%s bound to %s and %s' % (k, out[k], action)
            out[k] = action
    return out


def test_counts_and_schema(doc):
    assert doc['schema'] == 'mhseals.keymap/1'
    contexts = {k: v for k, v in doc.items() if k != 'schema'}
    assert len(contexts) == 44
    assert sum(len(binds(c)) for c in contexts.values()) == 279


def test_keys_known_and_unique_per_context(doc):
    for name, ctx in doc.items():
        if name != 'schema':
            for k in key_map(ctx):
                assert len(k) == 1 or k in NAMED, '%s: unknown key %r' % (name, k)


def test_layered_children_only_shadow_listed_overrides(doc):
    for stack, chain in STACKS.items():
        resolved = {}
        for name in chain:
            allowed = set((doc[name] or {}).get('override', []))
            for k, a in key_map(doc[name]).items():
                assert k not in resolved or resolved[k] in allowed, '%s: %s shadows %s' % (stack, name, resolved[k])
                resolved[k] = a


def test_space_stops_and_E_revokes_everywhere(doc):
    for stack, chain in STACKS.items():
        resolved = {}
        for name in chain:
            resolved.update(key_map(doc[name]))
        assert resolved['space'] == 'stop', stack
        assert resolved['E'] == 'revoke_agent', stack
    for name, ctx in doc.items():
        if not name.startswith(MODAL) or name == 'popup.text':
            continue
        own = key_map(ctx)
        for a in ctx.get('passthrough', []):
            for k in doc['global'].get(a, []):
                assert k not in own, '%s: %s collides with passthrough %s' % (name, k, a)
                own[k] = a
        assert own.get('space') in ('stop', 'abort'), name
        assert own.get('E') in ('revoke_agent', 'revoke_selected'), name


def test_locked_actions_stay_bound(doc):
    for a in ('stop', 'quit', 'revoke_agent'):
        assert doc['global'][a]


def test_every_action_is_bound_or_marked_unbound(doc):
    try:
        actions = importlib.import_module('mhseals_testing.ui.actions')
    except ImportError:
        pytest.skip('ui/actions.py (ts011) is not on this branch yet')
    bound = {a for ctx in doc.values() if isinstance(ctx, dict) for a in binds(ctx)}
    unbound = set(getattr(actions, 'UNBOUND', ()))
    missing = set(actions.ACTIONS) - bound - unbound
    assert not missing, sorted(missing)


# Every key a ui/ file compared as a literal before it asked the keymap: (scope, key, action), frozen when
# the file's literals were converted (test_no_literal_key_checks), so a keymap edit cannot move them silently.
LITERALS = [
    # popups/stop.py, popups/hold.py, popups/authority.py, actions/arm.py, actions/settings.py (areas 3, 4, 6)
    *[('popup.stop', k, 'close') for k in ('escape', 'q', 'enter')],
    *[('popup.hold', k, 'decline') for k in ('escape', 'n')],
    ('popup.authority', 'escape', 'close'), ('popup.authority', 'enter', 'detail'),
    ('popup.authority', 'E', 'revoke_selected'),
    *[('popup.authority', k, a) for a, ks in (('up', 'uk'), ('down', 'dj')) for k in (a, ks[1])],
    ('popup.arming', 'space', 'stop'), ('popup.arming', 'escape', 'cancel'), ('popup.arming', 'E', 'revoke_agent'),
    ('popup.pins_wait', 'space', 'stop'), ('popup.pins_wait', 'escape', 'stop'),
    ('panel.status', 'left', 'adjust_down'), ('panel.status', 'right', 'adjust_up'), ('panel.status', 'e', 'params'),
    ('panel.status', 'h', None), ('panel.status', 'l', None),
    *[('tab.correction', k, a) for k, a in (('left', 'column_prev'), ('h', 'column_prev'), ('right', 'column_next'),
                                            ('l', 'column_next'), ('r', 'rerun_direction'), ('R', 'run_all'),
                                            ('A', 'apply_pending'), ('I', 'hardware_changed'))],
    # popups/power.py, health.py, conditions.py, magcal.py (popup.text), config.py (folded)
    *[(s, k, 'close') for s in ('popup.power', 'popup.health', 'popup.conditions') for k in ('escape', 'q')],
    ('popup.conditions', 'up', 'up'), ('popup.conditions', 'k', 'up'),
    ('popup.conditions', 'down', 'down'), ('popup.conditions', 'j', 'down'),
    ('popup.text', 'enter', 'confirm'), ('popup.text', 'escape', 'cancel'),
    ('popup.config', 'up', 'up'), ('popup.config', 'k', 'up'), ('popup.config', 'down', 'down'),
    ('popup.config', 'j', 'down'), ('popup.config', 'enter', 'apply'), ('popup.config', 'escape', 'close'),
    # popups/catalog.py, popups/course.py (its NUDGE letters too)
    *[('popup.catalog', k, a) for k, a in (('up', 'up'), ('k', 'up'), ('down', 'down'), ('j', 'down'),
                                           ('enter', 'detail'), ('escape', 'close'), ('q', 'close'))],
    *[('popup.catalog_detail', k, a) for a, ks in (('up', ('up', 'k', '[')), ('down', ('down', 'j', ']')),
                                                   ('close', ('escape', 'q'))) for k in ks],
    *[('popup.course', k, a) for k, a in (
        ('escape', 'leave'), ('g', 'corner_add'), ('G', 'corner_here'), ('n', 'nudge_north'), ('s', 'nudge_south'),
        ('e', 'nudge_east'), ('w', 'nudge_west'), ('+', 'step_up'), ('=', 'step_up'), ('-', 'step_down'),
        ('_', 'step_down'), ('x', 'remove'), ('up', 'up'), ('down', 'down'), ('K', 'move_up'), ('J', 'move_down'),
        ('o', 'on_exit'), ('y', 'apply'), ('k', None), ('j', None))],
    # popups/__init__.py (show folds, prompt folds, text fields), camera_view.py (case kept), actions/popups.py
    ('popup.show', 'k', 'up'), ('popup.show', 'j', 'down'), ('popup.show', 'up', 'close'), ('popup.show', 'z', 'close'),
    ('popup.prompt', 'enter', 'continue'), ('popup.prompt', 'escape', 'cancel'), ('popup.prompt', 'q', 'cancel'),
    ('popup.text', 'backspace', 'erase'), ('popup.text', 'ctrl-h', 'erase'), ('popup.text', '\x7f', 'erase'),
    ('popup.text', 'space', None), ('global', 'U', 'release_hold'),
    *[('popup.perception', k, a) for k, a in (
        ('d', 'boxes'), ('D', 'floor'), ('f', 'freeze'), ('s', 'save'), ('+', 'rate_up'), ('=', 'rate_up'),
        ('-', 'rate_down'), ('_', 'rate_down'), ('k', 'up'), ('j', 'down'), ('up', None), ('down', None),
        ('v', 'view'), ('escape', 'back'), ('q', 'close'))],
    # popups/history.py (case kept)
    *[('popup.history', k, a) for k, a in (
        ('escape', 'close'), ('q', 'close'), ('[', 'tab_prev'), ('left', 'tab_prev'), (']', 'tab_next'),
        ('right', 'tab_next'), ('up', 'up'), ('k', 'up'), ('down', 'down'), ('j', 'down'), ('K', 'scroll_up'),
        ('J', 'scroll_down'), ('l', 'detail'), ('d', 'base'), ('enter', 'revert'), ('y', 'confirm'),
        ('n', 'name'), ('N', 'note'))],
    # actions/labels.py (tab.labels alone, case kept)
    *[('tab.labels', k, a) for k, a in (
        ('left', 'scale_down'), ('h', 'scale_down'), ('right', 'scale_up'), ('l', 'scale_up'), ('<', 'session_prev'),
        ('>', 'session_next'), ('y', 'accept'), ('r', 'reject'), ('u', 'undo'), ('s', 'split'), ('m', 'merge'),
        ('n', 'note_label'), ('enter', 'replay'), ('0', None), *((str(d), 'tag') for d in range(1, 10)))],
    # actions/bags.py: review (folded), the Bags list, Activity, popup.run, the Recording row
    *[('review', k, a) for k, a in (
        ('left', 'seek_back'), ('h', 'seek_back'), ('right', 'seek_fwd'), ('l', 'seek_fwd'),
        ('shift-left', 'seek_back_big'), ('{', 'seek_back_big'), ('shift-right', 'seek_fwd_big'),
        ('}', 'seek_fwd_big'), (',', 'jump_prev'), ('.', 'jump_next'), ('<', 'rate_down'), ('>', 'rate_up'),
        ('p', 'play'), ('escape', 'leave'))],
    ('review.activity', 'z', 'zoom'), ('global', 'q', 'quit'), ('panel.activity', 'v', 'view_next'),
    *[(('global', 'tab.drive'), k, a) for k, a in (('a', 'arm'), ('x', 'report'), ('m', 'manual'),
                                                   ('+', 'amplitude_up'), ('=', 'amplitude_up'),
                                                   ('-', 'amplitude_down'), ('_', 'amplitude_down'))],
    *[('global', k, a) for k, a in (('\t', 'focus_next'), ('backtab', 'focus_prev'), ('up', 'up'), ('k', 'up'),
                                    ('down', 'down'), ('j', 'down'), ('?', 'help'), ('o', 'odometry'),
                                    ('ctrl-h', 'focus_left'), ('ctrl-left', 'focus_left'), ('ctrl-l', 'focus_right'),
                                    ('ctrl-right', 'focus_right'), ('ctrl-k', 'focus_up'), ('ctrl-up', 'focus_up'),
                                    ('ctrl-j', 'focus_down'), ('ctrl-down', 'focus_down'), ('enter', 'activate'))],
    *[(('global', 'tab.bags'), k, a) for k, a in (('enter', 'activate'), ('p', 'pin'), ('n', 'title'),
                                                  ('d', 'delete'), ('P', 'prune'), ('F', 'finalize'))],
    *[('popup.run', k, a) for k, a in (('X', 'exclude'), ('up', 'prev'), ('k', 'prev'), ('down', 'next'),
                                       ('j', 'next'), ('escape', 'close'), ('q', 'close'))],
    (('global', 'panel.activity'), 'v', 'view_next'), (('global', 'panel.activity'), 'h', None),
    # actions/annotate.py, actions/procedures.py
    ('global', 'b', 'mark'), ('global', 'N', 'bad_data'), ('global', '"', 'say'), ('global', 'B', 'mark_note'),
    # actions/manual.py: the Thrusters panel (its key arrives folded)
    *[(STACKS['drive/thrusters'], k, a) for k, a in (
        ('left', 'pulse_down'), ('h', 'pulse_down'), ('right', 'pulse_up'), ('l', 'pulse_up'), ('0', 'neutral'),
        ('enter', 'activate'), ('1', 'wire_1'), ('2', 'wire_2'), ('3', 'wire_3'), ('4', 'wire_4'), ('r', 'reverse'),
        ('s', 'save'), ('u', 'undo'), ('i', 'thruster_set'), ('c', 'clear_decoupling'))],
]


@pytest.mark.parametrize('scope, key, action', LITERALS)
def test_the_old_literal_keys_resolve_to_the_same_action(scope, key, action):
    from mhseals_testing.ui import keymap
    km = keymap.load(user=None)[0]
    assert km.dispatch(scope, key) == action, '%s %r' % (scope, key)

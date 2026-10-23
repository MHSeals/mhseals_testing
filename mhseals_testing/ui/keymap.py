"""Validate the keymap (config/keymap.yaml).

A keymap maps context -> action id -> key names. Contexts are layered
(``global`` -> ``tab.*`` -> ``panel.*``/``view.*``, review on top) or modal
(``popup.*``, ``manual``, ``runner``, ``minimap.goal``: own keys plus the global
keys of their ``passthrough`` actions, nothing from outside). A layered child
may shadow an ancestor's key only for actions it lists in ``override``.

Rules (see ``validate``):
  1 a key bound twice in a context      4 unknown key, action or context name,
                                          or a malformed (non list / mapping) value
  2 undeclared shadowing                5 Space stops and E revokes in every stack
  3 modal key collides with passthrough 6 stop, quit, revoke_agent stay bound
"""

import yaml

from mhseals_testing.ui.actions import ACTIONS

KEY_NAMES = set('space enter escape tab backtab backspace up down left right shift-left shift-right '
                'ctrl-up ctrl-down ctrl-left ctrl-right ctrl-h ctrl-j ctrl-k ctrl-l focus-out any'.split())
OPTIONS = ('override', 'passthrough')
MODAL_NAMES = ('manual', 'runner', 'minimap.goal')
NO_SPACE = ('popup.text',)               # Space types a space there; nothing moves while it is open
LOCKED = ('stop', 'quit', 'revoke_agent')
STACKS = {                               # outermost first
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


class KeymapError(ValueError):
    """A keymap that fails validation; ``errors`` lists every violation."""

    def __init__(self, errors):
        super().__init__('invalid keymap:\n  ' + '\n  '.join(errors))
        self.errors = errors


def is_modal(name):
    return name.startswith('popup.') or name in MODAL_NAMES


def _known(name):
    return is_modal(name) or any(name in chain for chain in STACKS.values())


def bindings(context):
    """action -> keys of one context, without its options."""
    return {a: list(k) for a, k in (context or {}).items() if a not in OPTIONS}


def key_map(context):
    """key -> action for one context (the first binding wins on a duplicate)."""
    out = {}
    for action, keys in bindings(context).items():
        for key in keys:
            out.setdefault(key, action)
    return out


def _stack_keys(doc, stack):
    """key -> action for a stack, innermost context winning; a modal adds its passthrough keys."""
    out = {}
    for name in reversed(stack):
        context = doc.get(name) or {}
        for key, action in key_map(context).items():
            out.setdefault(key, action)
        for action in context.get('passthrough', []):
            for key in (doc.get('global') or {}).get(action, []):
                out.setdefault(key, action)
    return out


def _shape_errors(doc, known=True):
    """Rule 4: contexts are mappings of action -> list of strings; names are known if ``known``."""
    if not isinstance(doc, dict):
        return ['keymap must be a mapping of contexts']
    errors = []
    for name, context in doc.items():
        if name == 'schema':
            continue
        if known and not _known(name):
            errors.append('unknown context %r' % name)
        if context is None:
            continue
        if not isinstance(context, dict):
            errors.append('context: %s must be a mapping' % name)
        elif not all(isinstance(k, list) and all(isinstance(x, str) for x in k) for k in context.values()):
            errors.append('context: %s action keys must be a list of strings' % name)
    return errors


def validate(doc):
    """Return every rule violation in ``doc`` as a list of strings (empty = valid)."""
    errors = _shape_errors(doc)
    if errors:
        return errors
    contexts = {n: c for n, c in doc.items() if n != 'schema'}
    for name, context in contexts.items():
        seen = {}
        for action, keys in bindings(context).items():
            if action not in ACTIONS:
                errors.append('%s: unknown action %r' % (name, action))                   # 4
            for key in keys:
                if len(key) != 1 and key not in KEY_NAMES:
                    errors.append('%s: unknown key %r (%s)' % (name, key, action))       # 4
                elif key in seen:
                    errors.append('%s: %s bound to both %s and %s' % (name, key, seen[key], action))  # 1
                seen[key] = action
        for option in OPTIONS:
            for action in (context or {}).get(option, []):
                if action not in ACTIONS:
                    errors.append('%s: unknown action %r in %s' % (name, action, option))  # 4
    for stack, chain in STACKS.items():
        owner = {}                                  # key -> (context, action) so far
        for name in chain:
            allowed = (contexts.get(name) or {}).get('override', [])
            for key, action in key_map(contexts.get(name)).items():
                if key in owner and owner[key][1] not in allowed:
                    errors.append('%s: %s %s shadows %s %s without override' % (
                        stack, name, key, owner[key][0], owner[key][1]))                  # 2
                owner[key] = (name, action)
        errors += _invariants(stack, _stack_keys(doc, chain), modal=False)                # 5
    for name, context in contexts.items():
        if not is_modal(name):
            continue
        own = key_map(context)
        for action in (context or {}).get('passthrough', []):
            for key in (doc.get('global') or {}).get(action, []):
                if key in own:
                    errors.append('%s: %s (%s) collides with passthrough %s' % (name, key, own[key], action))  # 3
        if name not in NO_SPACE:
            errors += _invariants(name, _stack_keys(doc, [name]), modal=True)             # 5
    for action in LOCKED:
        if not (doc.get('global') or {}).get(action):
            errors.append('global: %s must stay bound' % action)                          # 6
    return errors


def _invariants(label, keys, modal):
    """Rule 5: Space stops (modals may abort) and E revokes (popup.authority revokes the selected)."""
    errors = []
    if keys.get('space') not in (('stop', 'abort') if modal else ('stop',)):
        errors.append('%s: space is %s, not stop' % (label, keys.get('space')))
    if keys.get('E') not in (('revoke_agent', 'revoke_selected') if modal else ('revoke_agent',)):
        errors.append('%s: E is %s, not revoke_agent' % (label, keys.get('E')))
    return errors

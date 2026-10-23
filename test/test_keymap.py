"""Keymap validation rules 1-6 and the ui.md negative cases, with inline YAML."""

import yaml

from mhseals_testing.ui import keymap

BASE = """
schema: mhseals.keymap/1
global: {stop: [space], quit: [q], arm: [a], mark: [b], revoke_agent: [E], up: [up, k]}
tab.drive: {characterize: [f]}
tab.correction: {}
tab.map: {}
tab.bags: {}
panel.status: {}
panel.motion: {}
panel.conditions: {}
panel.thrusters: {}
panel.procedures: {}
panel.activity: {}
view.minimap: {}
review: {}
review.activity: {}
popup.show: {passthrough: [stop, revoke_agent], close: [any]}
popup.text: {confirm: [enter], cancel: [escape]}
"""


def doc(**changes):
    """BASE with whole contexts replaced: doc(**{'tab.drive': {...}})."""
    out = yaml.safe_load(BASE)
    for name, context in changes.items():
        out[name.replace('__', '.')] = context
    return out


def with_global(**changes):
    d = doc()
    d['global'].update(changes)
    return keymap.validate(d)


def errors(**changes):
    return keymap.validate(doc(**changes))


def has(errs, text):
    return any(text in e for e in errs)


def test_base_is_valid():
    assert keymap.validate(doc()) == []


def test_rule1_key_bound_twice():
    assert has(errors(tab__drive={'characterize': ['f'], 'manual': ['f']}), 'f bound to both')


def test_rule2_shadow_needs_override():
    assert has(errors(panel__status={'characterize': ['a']}), 'shadows')
    assert errors(panel__status={'characterize': ['a'], 'override': ['arm']}) == []


def test_rule3_modal_collides_with_passthrough():
    assert has(errors(popup__show={'passthrough': ['stop'], 'close': ['space']}), 'collides with passthrough')


def test_rule4_unknown_key_and_action():
    assert has(errors(tab__drive={'characterize': ['f1']}), "unknown key 'f1'")
    assert has(errors(tab__drive={'frobnicate': ['f']}), "unknown action 'frobnicate'")


def test_rule5_space_stops_and_e_revokes_everywhere():
    assert has(errors(tab__bags={'abort': ['space']}), 'bags: space is')
    assert has(with_global(stop=['s']), 'space is')
    assert has(with_global(revoke_agent=['V']), 'E is')
    assert has(errors(popup__show={'close': ['any']}), 'popup.show: space is')
    assert not has(errors(), 'popup.text')                      # popup.text is exempt


def test_rule4_shape_and_context_names():
    assert has(errors(tab__bags=['x']), 'context: tab.bags must be a mapping')
    assert has(errors(tab__bags={'abort': 'ctrl-a'}), 'context: tab.bags action keys must be a list of strings')
    assert has(errors(tab__drvie={}), "unknown context 'tab.drvie'")


def test_rule6_locked_actions_stay_bound():
    assert has(with_global(quit=[]), 'quit must stay bound')


# The three negative cases from ui.md section 7.
def test_negative_removed_override():
    d = doc(tab__drive={'station': ['s']}, panel__thrusters={'save': ['s'], 'override': ['station']})
    assert keymap.validate(d) == []
    d['panel.thrusters'] = {'save': ['s']}
    assert has(keymap.validate(d), 'panel.thrusters s shadows tab.drive station')


def test_negative_passthrough_collision():
    d = doc(popup__show={'passthrough': ['stop', 'revoke_agent'], 'close': ['E']})
    assert has(keymap.validate(d), 'popup.show: E (close) collides with passthrough revoke_agent')


def test_negative_review_h_shadows_history():
    d = doc(review={'seek_back': ['H']})
    d['global']['history'] = ['H']
    assert has(keymap.validate(d), 'review H shadows global history')
    d['review']['override'] = ['history']
    assert keymap.validate(d) == []

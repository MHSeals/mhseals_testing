"""Keymap validation rules 1-6, overlay, resolve and hints, with inline YAML."""

import pytest
import yaml

from mhseals_testing.ui import keymap
from mhseals_testing.ui.keymap import Keymap, KeymapError

BASE = """
schema: mhseals.keymap/1
fixed: [{key: space, action: stop, also: [abort], why: stop}, {key: E, action: revoke_agent, why: revoke}]
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


def test_overlay_rebinds_and_unbinds():
    merged = keymap.overlay(doc(), {'global': {'arm': ['A'], 'mark': []}, 'tab.drive': {'manual': ['m']}})
    assert merged['global']['arm'] == ['A'] and 'mark' not in merged['global']
    assert merged['tab.drive']['manual'] == ['m']
    assert doc()['global']['arm'] == ['a']                 # input untouched


def test_overlay_shape_errors():
    with pytest.raises(KeymapError, match='arm must be a list|action keys must be a list'):
        keymap.overlay(doc(), {'global': {'arm': 'ctrl-a'}})
    with pytest.raises(KeymapError, match='tab.bags must be a mapping'):
        keymap.overlay(doc(), {'tab.bags': ['x']})


def test_load_applies_overlay_and_falls_back_when_bad(tmp_path):
    base = tmp_path / 'base.yaml'
    base.write_text(BASE)
    user = tmp_path / 'user.yaml'
    user.write_text('global: {arm: [A]}\n')
    km, warning = keymap.load(base, user)
    assert warning is None and km.resolve(['global'], 'A') == 'arm' and km.resolve(['global'], 'a') is None
    assert keymap.load(base, tmp_path / 'missing.yaml')[0].resolve(['global'], 'a') == 'arm'
    for text in ('global: {stop: []}\n', 'global: {arm: ctrl-a}\n', 'tab.drvie: {arm: [x]}\n', 'global: [\n'):
        user.write_text(text)                                   # unbound stop, shape, name, syntax
        km, warning = keymap.load(base, user)
        assert warning and 'ignored' in warning
        assert km.resolve(['global'], 'a') == 'arm' and km.resolve(['global'], 'space') == 'stop'


def test_resolve_innermost_wins_and_modal_passthrough():
    km = Keymap(doc(panel__status={'characterize': ['a'], 'override': ['arm']}))
    assert km.resolve(['global', 'tab.drive', 'panel.status'], 'a') == 'characterize'
    assert km.resolve(['global', 'tab.drive', 'panel.status'], 'k') == 'up'
    assert km.resolve(['popup.show'], 'space') == 'stop'
    assert km.resolve(['popup.show'], 'a') == 'close'           # any
    assert km.resolve(['popup.text'], 'a') is None


def test_hints_ordered_by_priority_and_follow_rebinding():
    km = Keymap(doc())
    hints = km.hints(['global', 'tab.drive'])
    assert hints[:3] == [('space', 'stop'), ('a', 'arm'), ('b', 'mark')]
    assert ('up', 'up') not in hints and ('f', 'characterize') in hints
    assert ('A', 'arm') in Keymap(keymap.overlay(doc(), {'global': {'arm': ['A']}})).hints(['global'])


pytestmark = pytest.mark.smoke   # tools/smoke


def test_rule8_fixed_keys_cannot_move():
    assert not keymap.validate(doc())
    moved = doc(**{'tab__drive': {'characterize': ['E']}})
    assert any('E is fixed to revoke_agent' in e for e in keymap.validate(moved))
    dropped = doc(); dropped['global']['stop'] = ['x']
    assert any('global: space is fixed to stop' in e for e in keymap.validate(dropped))
    without = doc(); del without['fixed']
    assert any(e.startswith('fixed:') for e in keymap.validate(without))
    wrong = doc(); wrong['fixed'][1]['key'] = 'R'
    assert any('keymap.FIXED' in e for e in keymap.validate(wrong))
    with pytest.raises(KeymapError, match='fixed'):
        keymap.overlay(doc(), {'fixed': []})


def test_dispatch_matches_and_keys_name_raw_keys():
    km = Keymap(doc(**{'popup__show': {'passthrough': ['stop', 'revoke_agent'], 'close': ['any'], 'up': ['k']}}))
    assert km.dispatch('global', 'q') == 'quit' and km.dispatch(['global'], 'Q') is None
    assert km.dispatch('global', 'Q', fold=True) == 'quit'
    assert km.dispatch('global', 'press:space') is None and km.dispatch('global', 'press:space', press=True) == 'stop'
    assert km.matches('popup.show', 'up', 'k') and km.matches('popup.show', 'close', 'z')   # any
    assert km.matches('popup.show', 'stop', 'space') and not km.matches('global', 'quit', None)
    assert km.keys('global', 'up') == ['up', 'k']
    tabbed = Keymap(doc(**{'global': {**yaml.safe_load(BASE)['global'], 'focus_next': ['tab']}}))
    assert tabbed.dispatch('global', '\t') == 'focus_next'


def test_current_rereads_a_changed_file(tmp_path, monkeypatch):
    path = tmp_path / 'keymap.yaml'
    path.write_text(BASE)
    monkeypatch.setattr(keymap, 'shipped_file', lambda: path)
    monkeypatch.setattr(keymap, 'USER_FILE', tmp_path / 'none.yaml')
    monkeypatch.setattr(keymap, 'LIVE', dict(keymap=None, stamp=None, checked=0.0))
    monkeypatch.setattr(keymap, 'RECHECK_S', 0.0)
    assert keymap.matches('global', 'quit', 'q')
    path.write_text(BASE.replace('quit: [q]', 'quit: [Q]'))
    import os
    os.utime(path, ns=(1, 1))
    assert keymap.matches('global', 'quit', 'Q') and not keymap.matches('global', 'quit', 'q')
    path.write_text('global: {')                      # broken mid-session: the last good keymap stays
    os.utime(path, ns=(2, 2))
    assert keymap.matches('global', 'quit', 'Q')

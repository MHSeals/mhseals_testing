"""The arming, stop, hold and authority UI keys go through config/keymap.yaml (audit finding 5).

Each scope's shipped bindings are pinned to the keys the code compared before the change, so a keymap edit
cannot quietly rebind or drop an arm, stop or release key; Space and E stay the fixed keys (keymap.STOP_KEY,
keymap.REVOKE_KEY), handled before any scope.
"""
import pytest

from mhseals_testing.ui import keymap

pytestmark = pytest.mark.smoke


def shipped(scope, key, **flags):
    return keymap.load(user=None)[0].dispatch(scope, key, **flags)


def test_space_and_e_are_the_fixed_keys():
    assert (keymap.STOP_KEY, keymap.REVOKE_KEY) == ('space', 'E')


@pytest.mark.parametrize('key', ['escape', 'q', 'enter'])
def test_the_stop_popup_closes_on_its_old_keys(key):
    assert shipped('popup.stop', key) == 'close'


@pytest.mark.parametrize('key, action', [('escape', 'close'), ('up', 'up'), ('k', 'up'), ('K', 'up'),
                                         ('down', 'down'), ('j', 'down'), ('J', 'down'), ('enter', 'detail')])
def test_the_authority_popup_keys_fold_as_before(key, action):
    assert shipped('popup.authority', key, fold=True) == action


def test_only_a_capital_e_revokes_the_selected_claim():
    assert shipped('popup.authority', 'e', fold=True) is None and shipped('popup.authority', 'E', fold=True) is None
    assert shipped('popup.authority', keymap.REVOKE_KEY) == 'revoke_selected'

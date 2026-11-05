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


@pytest.mark.parametrize('key', ['escape', 'press:escape'])
def test_esc_cancels_the_arm_waits_and_space_is_not_theirs_to_rebind(key):
    assert shipped('popup.arming', key, press=True) == 'cancel'
    assert shipped('popup.arming', 'space', press=True) == 'stop'


@pytest.mark.parametrize('key', ['space', 'escape', 'press:space', 'press:escape'])
def test_space_or_esc_stops_the_pins_edit_wait(key):
    assert shipped('popup.pins_wait', key, press=True) == 'stop'


def test_the_arm_waits_check_the_fixed_stop_key_before_the_keymap():
    import inspect
    from mhseals_testing.ui.app.actions import arm
    from mhseals_testing.ui.app.actions.settings_rows import hardware
    from mhseals_testing.ui.popups import STOP_KEYS
    assert STOP_KEYS == ('space', 'press:space')
    for source in (inspect.getsource(arm.Arm.await_mux), inspect.getsource(arm.Arm.wait_for_driver),
                   inspect.getsource(arm.Arm.read_boat_config), inspect.getsource(hardware.Hardware.apply_hardware)):
        assert 'key in STOP_KEYS' in source
        first = source.index('key in STOP_KEYS')
        assert all(first < i for i in (source.find('keymap.matches'),) if i >= 0)


@pytest.mark.parametrize('key, action', [('left', 'adjust_down'), ('right', 'adjust_up'), ('e', 'params'),
                                         ('h', None), ('l', None)])
def test_the_status_row_keys_are_the_old_ones(key, action):
    assert shipped('panel.status', key) == action
    assert shipped('global', 'enter') == 'activate'


@pytest.mark.parametrize('key, action', [('left', 'column_prev'), ('h', 'column_prev'), ('right', 'column_next'),
                                         ('l', 'column_next'), ('r', 'rerun_direction'), ('R', 'run_all'),
                                         ('A', 'apply_pending'), ('I', 'hardware_changed'), ('enter', None)])
def test_the_correction_tab_keys_are_the_old_ones(key, action):
    assert shipped('tab.correction', key) == action


def test_a_status_step_passes_its_direction_not_the_raw_key():
    from mhseals_testing.ui.app.actions.settings_rows.status import STEPS
    assert STEPS == {'adjust_down': 'left', 'adjust_up': 'right'}

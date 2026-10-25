"""P on the Thrusters panel: a row's J2 PWM pin, validated, drafted, saved by s, read back."""

import pytest
import yaml

from mhseals_hardware.configuration import load_config
from mhseals_hardware.odroid_pwm import J2_PINS
from mhseals_testing.ui.popups import pins
from test_app import make_app, pins_app

OFF_TABLE = '/sys/devices/platform/test.pwm/pwm/pwmchip*'   # output 1 here frees pin 7
CHIPS = [J2_PINS[output][3] for output in range(1, 5)]


def free_pin_7(tmp_path):
    (tmp_path / 'hardware.yaml').write_text(
        'frequency: 50.0\nchannel_map: [1, 2, 3, 4]\n'
        f"pwm_chips: {yaml.safe_dump([OFF_TABLE, *CHIPS[1:]], default_flow_style=True).strip()}\n")


@pytest.mark.parametrize('text, error', [('x', 'a J2 pin number'), ('22', 'pin 22 has no PWM'),
                                         ('13', 'MOSFET enable'), ('12', 'already drives output 2')])
def test_parse_pin_rejects_non_pwm_mosfet_and_taken_pins(text, error):
    with pytest.raises(ValueError, match=error):
        pins.parse_pin(text, CHIPS, 1)


def test_parse_pin_moves_only_that_output():
    assert pins.parse_pin('7', [OFF_TABLE, *CHIPS[1:]], 1) == CHIPS
    assert pins.parse_pin('7', CHIPS, 1) == CHIPS            # its own pin is no duplicate
    assert pins.output_pin([OFF_TABLE, *CHIPS[1:]], 1) is None


def test_pin_edit_is_a_draft_that_s_saves_and_reads_back(tmp_path, monkeypatch):
    free_pin_7(tmp_path)
    app, processes, seen = pins_app(tmp_path, monkeypatch, [('enabled', '')])
    asked = []
    app.ask_text = lambda title, message, text, check: asked.append((title, text)) or '7'
    app.dispatch('P')
    assert asked == [('FL pin', '')]                          # off the table: no pin to show
    assert app.draft['pwm_chips'] == CHIPS and processes.started == []
    assert load_config(app.args.config)['pwm_chips'][0] == OFF_TABLE   # nothing written yet
    app.dispatch('s')
    assert load_config(app.args.config)['pwm_chips'] == CHIPS
    assert app.notice.startswith('Pins saved; driver restarted, sysfs showed')
    assert app.typed == ['arm'] and processes.started == ['hardware'] == processes.stopped
    assert not app.armed and not app.pins_unverified and app.draft is None


def test_mapping_only_save_leaves_the_chip_paths_out_of_the_file(tmp_path, monkeypatch):
    app, processes, _ = pins_app(tmp_path, monkeypatch, [('enabled', '')])
    app.dispatch('r'), app.dispatch('s')
    assert 'pwm_chips' not in yaml.safe_load((tmp_path / 'hardware.yaml').read_text())
    assert processes.started == [] and app.notice.startswith('Mapping saved')


def test_duplicate_pin_is_rejected_and_nothing_drafted(tmp_path, monkeypatch):
    app, processes, _ = pins_app(tmp_path, monkeypatch, [('enabled', '')])
    app.ask_text = lambda *a: '33'
    app.dispatch('P')
    assert app.notice == 'Pin rejected: pin 33 already drives output 4' and app.draft is None


@pytest.mark.parametrize('state', ['armed', 'pending'])
def test_pin_edit_and_its_save_are_refused_unless_disarmed(tmp_path, state):
    free_pin_7(tmp_path)
    app, _ = make_app(tmp_path)
    app.focus = 'thrusters'
    app.edit_mapping([1, 2, 3, 4], [False] * 4, CHIPS)        # drafted while disarmed
    if state == 'armed':
        app.arming.armed = True
    else:
        app.arming.pending = 1.0
    app.ask_text = lambda *a: pytest.fail('no prompt')
    app.dispatch('P')
    assert app.notice == pins.REFUSED
    app.dispatch('s')
    assert app.notice == pins.REFUSED and app.draft is not None
    assert load_config(app.args.config)['pwm_chips'][0] == OFF_TABLE


def test_arming_waits_for_a_readback_after_a_pin_change(tmp_path, monkeypatch):
    free_pin_7(tmp_path)
    app, processes, _ = pins_app(tmp_path, monkeypatch, [('unexported', ''), ('enabled', '')])
    monkeypatch.setattr(pins, 'READBACK_TIMEOUT', 0)
    app.ask_text = lambda *a: '7'
    app.dispatch('P'), app.dispatch('s')
    assert 'did not show' in app.notice and app.pins_unverified
    assert 'not read back' in ' '.join(app.arm_blockers())
    app.arm_panel()
    assert app.notice.startswith('Arming refused: pins changed') and not app.armed
    processes.started.clear(), processes.stopped.clear()      # the fake runs each name once
    app.dispatch('s')                                           # no draft: s retries the readback
    assert app.notice.startswith('Pins saved; driver restarted, sysfs showed')
    assert not app.pins_unverified and app.pin_refusal() == ''


def test_keymap_binds_pin_to_P_on_the_thrusters_panel():
    from mhseals_testing.ui import keymap
    loaded, _ = keymap.load()
    assert loaded.doc['panel.thrusters']['pin'] == ['P']


def test_a_pin_draft_refuses_arming_and_test_pulses(tmp_path):
    free_pin_7(tmp_path)
    app, _ = make_app(tmp_path)
    app.edit_mapping([1, 2, 3, 4], [False] * 4, CHIPS)
    assert app.pins_moved() and 'unsaved pin change' in app.pin_refusal()
    assert 'unsaved pin change' in ' '.join(app.arm_blockers())
    app.arm_panel()
    assert app.notice.startswith('Arming refused: unsaved pin change') and not app.armed
    app.arming.armed = True                                    # even if armed, no test pulse
    app.nudge(0, 10)
    assert app.notice.startswith('Test pulses refused: unsaved pin change')


def test_a_failed_save_after_the_write_leaves_arming_refused(tmp_path, monkeypatch):
    free_pin_7(tmp_path)
    app, processes, _ = pins_app(tmp_path, monkeypatch, [('enabled', '')])
    monkeypatch.setattr(app, 'announce', lambda change: type('C', (), {'ok': False})())
    app.ask_text = lambda *a: '7'
    app.dispatch('P'), app.dispatch('s')                       # written, then the reload fails
    assert app.pins_unverified and processes.started == []
    assert load_config(app.args.config)['pwm_chips'] == CHIPS
    assert 'not read back' in app.pin_refusal()


def test_a_new_app_refuses_arming_on_pins_not_verified(tmp_path, monkeypatch):
    free_pin_7(tmp_path)
    app, _, _ = pins_app(tmp_path, monkeypatch, [('enabled', '')])
    app.ask_text = lambda *a: '7'
    app.dispatch('P'), app.dispatch('s')                       # confirmed: the marker follows
    assert pins.verified_hash(app) == pins.pin_hash(app.args.hardware_config)
    assert make_app(tmp_path)[0].pin_refusal() == ''           # restart on verified pins: armable
    config = tmp_path / 'hardware.yaml'                        # a hand edit, or M saved unread
    config.write_text(config.read_text().replace('channel_map', 'pwm_channels: [1, 0, 0, 0]\nchannel_map'))
    fresh, _ = make_app(tmp_path)
    fresh.profile = app.profile
    assert 'differ from the last readback' in fresh.pin_refusal()
    fresh.arm_panel()
    assert fresh.notice.startswith('Arming refused: pins differ') and not fresh.armed


def test_a_first_unread_pin_change_survives_a_restart(tmp_path, monkeypatch):
    free_pin_7(tmp_path)
    app, _, _ = pins_app(tmp_path, monkeypatch, [('unexported', '')])
    monkeypatch.setattr(pins, 'READBACK_TIMEOUT', 0)
    app.ask_text = lambda *a: '7'
    app.dispatch('P'), app.dispatch('s')
    assert 'did not show' in app.notice
    fresh, _ = make_app(tmp_path)
    fresh.profile = app.profile
    assert not fresh.pins_unverified and 'differ from the last readback' in fresh.pin_refusal()

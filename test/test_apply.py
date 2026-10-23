"""Hot-applying hardware pins: readback, rollback, timeout and the armed refusal."""

import pytest

from mhseals_testing.harness.apply import HardwareApply
from mhseals_testing.ui.popups.pins import parse_pins, pin_text

OLD = dict(pwm_channels=[0, 0, 0, 0], mosfet_line=29)
NEW = dict(pwm_channels=[1, 0, 0, 0], mosfet_line=29)


class Plant:
    """A fake config file and driver; ``driver`` decides what the readback shows."""

    def __init__(self, driver, armed=False):
        self.file, self.saves, self.driver, self.is_armed, self.now = dict(OLD), [], driver, armed, 0.0

    def save(self, changes):
        self.saves.append(changes)
        self.file.update(changes)

    def read(self, keys):
        return self.driver(self, keys)

    def sleep(self, seconds):
        self.now += seconds

    serving = True

    def apply(self, changes=NEW):
        return HardwareApply(self.save, self.read, lambda: self.is_armed, lambda: self.file,
                             serving=lambda: self.serving, clock=lambda: self.now, sleep=self.sleep).stage(changes)


def follows(plant, keys):
    return {key: plant.file[key] for key in keys}


def test_applied_when_the_driver_shows_the_new_pins():
    plant = Plant(follows)
    assert plant.apply().ok
    assert plant.file == NEW


def test_applied_after_a_late_readback():
    plant = Plant(lambda p, keys: follows(p, keys) if p.now >= 0.5 else dict(OLD))
    assert plant.apply().ok


def test_rolled_back_when_the_driver_keeps_the_old_pins():
    plant = Plant(lambda p, keys: {key: OLD[key] for key in keys})
    result = plant.apply()
    assert result.status == 'rolled_back'
    assert plant.file == OLD


def test_timeout_without_an_answer_restores_the_file():
    plant = Plant(lambda p, keys: None)
    result = plant.apply()
    assert result.status == 'timeout' and plant.file == OLD
    assert 1.9 <= plant.now <= 2.2


def test_refused_while_armed_writes_nothing():
    plant = Plant(follows, armed=True)
    assert plant.apply().status == 'refused'
    assert plant.saves == []


def test_unchanged_pins_are_not_written():
    plant = Plant(follows)
    assert plant.apply(OLD).ok and plant.saves == []


def test_invalid_write_is_reported():
    plant = Plant(follows)
    plant.save = lambda changes: (_ for _ in ()).throw(ValueError('bad'))
    assert plant.apply().status == 'invalid'


def test_pin_text_round_trips():
    assert parse_pins(pin_text(OLD)) == OLD


@pytest.mark.parametrize('text', ['', '1 2 3', 'a b c d e', '0 0 0 0 -1'])
def test_bad_pin_text_is_rejected(text):
    with pytest.raises(ValueError):
        parse_pins(text)


def test_saved_for_the_next_arm_when_no_driver_runs():
    plant = Plant(lambda p, keys: pytest.fail('no readback without a driver'))
    plant.serving = False
    result = plant.apply()
    assert result.status == 'saved' and not result.ok
    assert plant.file == NEW and plant.now == 0.0


def test_failed_restore_tells_the_operator_to_fix_the_file():
    plant = Plant(lambda p, keys: None)

    def save(changes):
        if changes == OLD:
            raise ValueError('locked')
        plant.file.update(changes)
    plant.save = save
    result = plant.apply()
    assert result.status == 'invalid' and 'fix hardware.yaml' in result.detail

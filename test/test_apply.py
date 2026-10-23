"""Hot-applying hardware pins (readback, rollback, timeout, armed refusal) and correction staging."""

import pytest

from mhseals_hardware.configuration import load_config, update_config
from mhseals_testing import data
from mhseals_testing.harness.apply import HardwareApply, IDENTITY, SCHEMA, Staging
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


C = [1.0, 0.1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


@pytest.fixture
def staging(tmp_path):
    config = tmp_path / 'hardware.yaml'
    update_config(dict(decoupling=C), config)
    return Staging(config, tmp_path / 'state.json')


def test_abort_restores_c_and_clears_backup(staging):
    with pytest.raises(KeyboardInterrupt):
        with staging.staged():
            assert load_config(staging.config)['decoupling'] == IDENTITY
            raise KeyboardInterrupt
    assert load_config(staging.config)['decoupling'] == C
    assert staging.pending() is None and staging.blockers() == []


def test_crash_leaves_backup_that_blocks_motion(staging):
    staging.raw_correction()                       # the process dies here: no restore runs
    again = Staging(staging.config, staging.state)  # the next start
    assert again.pending()['decoupling'] == C
    assert again.blockers()
    staging.raw_correction()                       # a repeat must not back up the identity
    assert again.pending()['decoupling'] == C
    assert again.restore() and load_config(staging.config)['decoupling'] == C
    assert again.blockers() == []


def test_commit_writes_result_and_drops_backup(staging):
    staging.raw_correction()
    new = [1.0, 0.2, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    staging.commit(new)
    assert load_config(staging.config)['decoupling'] == new
    assert staging.pending() is None
    assert not staging.restore()


def test_other_state_survives_stage_restore_and_commit(staging):
    history = {'thrusters': [1, 2]}
    data.write_json(staging.state, dict(characterize=history), SCHEMA)
    staging.raw_correction()
    assert data.read_json(staging.state)['characterize'] == history
    assert staging.restore()
    assert data.read_json(staging.state) == dict(schema=SCHEMA, characterize=history)
    staging.raw_correction()
    staging.commit(C)
    assert data.read_json(staging.state)['characterize'] == history
    assert staging.pending() is None

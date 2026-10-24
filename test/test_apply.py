"""Pin text parsing and correction staging."""

import pytest

from mhseals_hardware.configuration import load_config, update_config
from mhseals_testing import data
from mhseals_testing.harness.apply import IDENTITY, SCHEMA, Staging
from mhseals_testing.ui.popups.pins import parse_pins, pin_text

OLD = dict(pwm_channels=[0, 0, 0, 0], mosfet_line=29)


def test_pin_text_round_trips():
    assert parse_pins(pin_text(OLD)) == OLD


@pytest.mark.parametrize('text', ['', '1 2 3', 'a b c d e', '0 0 0 0 -1'])
def test_bad_pin_text_is_rejected(text):
    with pytest.raises(ValueError):
        parse_pins(text)


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

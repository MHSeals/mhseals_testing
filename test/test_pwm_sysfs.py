from dataclasses import replace
from pathlib import Path

import pytest

from mhseals_testing.health import legacy
from mhseals_testing.health.pwm_sysfs import sample


@pytest.fixture
def chips(tmp_path):
    paths = [tmp_path / f'pwmchip{i}' for i in range(4)]
    for chip in paths:
        pin = chip / 'pwm0'
        pin.mkdir(parents=True)
        (chip / 'npwm').write_text('1')
        for key, value in [('enable', '1'), ('period', '20000000'), ('duty_cycle', '1500000')]:
            (pin / key).write_text(value)
    return paths


def test_one_acquisition_projects_without_reread(chips):
    observation = sample(chips, [0] * 4, verified_local=True, clock=lambda: 10)
    assert observation.neutral(10, 1) is True
    for chip in chips:
        (chip / 'pwm0' / 'duty_cycle').write_text('1600000')
    assert legacy.pwm_readback_rows(observation, True)[2] == ['ON 1500us / 50Hz'] * 4
    assert observation.health(10, 1, True)[0] == 'OK'
    assert observation.neutral(12, 1) is None and observation.receipt == 10
    assert sample(chips, [0] * 4, verified_local=True, clock=lambda: 10).neutral(10, 1) is False
    assert sample(chips, [0] * 4, clock=lambda: 10).neutral(10, 1) is None


def test_missing_unexported_disabled_and_invalid(chips):
    pin = chips[0] / 'pwm0'
    (pin / 'enable').write_text('0')
    assert sample(chips, [0] * 4, verified_local=True, clock=lambda: 10).neutral(10, 1)
    for child in pin.iterdir():
        child.unlink()
    pin.rmdir()
    observation = sample(chips, [0] * 4, verified_local=True, clock=lambda: 10)
    assert observation.outputs[0].state == 'unexported' and observation.neutral(10, 1)
    (chips[0] / 'npwm').unlink()
    chips[0].rmdir()
    assert sample(chips, [0] * 4, verified_local=True, clock=lambda: 10).neutral(10, 1) is None
    assert not replace(observation, verified=False).known(10, 1)


@pytest.mark.parametrize('count', [0, 3])
def test_incomplete_and_duplicate(chips, count):
    assert sample(chips[:count], [0] * count, verified_local=True, clock=lambda: 10).neutral(10, 1) is None
    assert not sample([chips[0]] * 4, [0] * 4, verified_local=True, clock=lambda: 10).known(10, 1)


@pytest.mark.parametrize('field,value', [('period', '0'), ('period', '-1'),
                                         ('period', '10000000'), ('duty_cycle', '21000000'),
                                         ('duty_cycle', '-1'), ('duty_cycle', 'nan'),
                                         ('enable', 'invalid')])
def test_invalid_sample_is_not_neutral(chips, field, value):
    (chips[0] / 'pwm0' / field).write_text(value)
    observation = sample(chips, [0] * 4, verified_local=True, clock=lambda: 10)
    assert observation.neutral(10, 1) is None
    assert observation.health(10, 1)[0] == 'N/A'


@pytest.mark.parametrize('frequency', [float('nan'), float('inf'), 0, -1, None])
def test_invalid_frequency_configuration(chips, frequency):
    observation = sample(chips, [0] * 4, frequency, verified_local=True, clock=lambda: 10)
    assert observation.neutral(10, 1) is None


def test_aliases_do_not_create_distinct_physical_outputs(chips, tmp_path):
    aliases = [tmp_path / f'alias{i}' for i in range(4)]
    for alias in aliases:
        alias.symlink_to(chips[0], target_is_directory=True)
    observation = sample(aliases, [0] * 4, verified_local=True, clock=lambda: 10)
    assert len({output.identity for output in observation.outputs}) == 1
    assert observation.neutral(10, 1) is None


@pytest.mark.parametrize('capability,channel', [
    ('1', 999), ('1', 1), ('0', 0), ('-1', 0), ('bad', 0), ('', 0), (None, 0),
])
def test_absent_channel_requires_valid_kernel_capability(tmp_path, capability, channel):
    paths = [tmp_path / f'pwmchip{i}' for i in range(4)]
    for chip in paths:
        chip.mkdir()
        if capability is not None:
            (chip / 'npwm').write_text(capability)
    observation = sample(paths, [channel] * 4, verified_local=True, clock=lambda: 10)
    assert [output.state for output in observation.outputs] == ['unavailable'] * 4
    assert observation.neutral(10, 1) is None
    assert observation.health(10, 1)[0] == 'N/A'


def test_real_unexported_channels_are_known_off(tmp_path):
    paths = [tmp_path / f'pwmchip{i}' for i in range(4)]
    for chip in paths:
        chip.mkdir()
        (chip / 'npwm').write_text('2')
    observation = sample(paths, [1] * 4, verified_local=True, clock=lambda: 10)
    assert [output.state for output in observation.outputs] == ['unexported'] * 4
    assert observation.neutral(10, 1) is True
    assert observation.health(10, 1)[0] == 'OK'


def test_delayed_acquisition_expires_oldest_channel(chips, monkeypatch):
    now = [10.]
    read_text = Path.read_text

    def delayed_read(path, *args, **kwargs):
        if path == chips[1] / 'pwm0' / 'enable':
            now[0] = 20.
        return read_text(path, *args, **kwargs)

    monkeypatch.setattr(Path, 'read_text', delayed_read)
    observation = sample(chips, [0] * 4, verified_local=True, clock=lambda: now[0])
    assert now[0] == 20 and observation.receipt == 10
    assert observation.neutral(now[0], 1) is None
    assert observation.health(now[0], 1)[0] == 'N/A'
    assert observation.neutral(9, 1) is None
    assert observation.receipt == 10

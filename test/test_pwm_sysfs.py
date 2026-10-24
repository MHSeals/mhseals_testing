from dataclasses import replace

import pytest

from mhseals_testing.health import legacy
from mhseals_testing.health.pwm_sysfs import sample


@pytest.fixture
def chips(tmp_path):
    paths = [tmp_path / f'pwmchip{i}' for i in range(4)]
    for chip in paths:
        pin = chip / 'pwm0'
        pin.mkdir(parents=True)
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
    chips[0].rmdir()
    assert sample(chips, [0] * 4, verified_local=True, clock=lambda: 10).neutral(10, 1) is None
    assert not replace(observation, verified=False).known(10, 1)


@pytest.mark.parametrize('count', [0, 3])
def test_incomplete_and_duplicate(chips, count):
    assert sample(chips[:count], [0] * count, verified_local=True).neutral(10, 1) is None
    assert not sample([chips[0]] * 4, [0] * 4, verified_local=True).known(10, 1)

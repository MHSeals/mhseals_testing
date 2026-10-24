from dataclasses import replace

import pytest

from mhseals_testing.health.pwm import Output, Readback


def record():
    return Readback('odroid', 10., tuple(
        Output(str(i), 'enabled', 20_000_000, 1_500_000) for i in range(4)), True)


def test_receipt_shared_by_pure_projections():
    observation = record()
    assert observation.neutral(10.5, 1) is True
    assert observation.health(10.5, 1, armed=True)[0] == 'OK'
    assert observation.neutral(11.01, 1) is None
    assert observation.health(11.01, 1)[0] == 'N/A'
    assert observation.receipt == 10
    assert observation.neutral(9, 1) is None


@pytest.mark.parametrize('count', [0, 3])
def test_incomplete_configuration_is_unknown(count):
    assert replace(record(), outputs=record().outputs[:count]).neutral(10, 1) is None


@pytest.mark.parametrize('change', [
    {'backend': 'none'}, {'backend': 'remote'}, {'verified': False},
    {'configuration_valid': False}, {'receipt': None}, {'receipt': float('nan')},
    {'receipt': float('inf')},
])
def test_missing_or_invalid_source(change):
    assert replace(record(), **change).neutral(10, 1) is None


@pytest.mark.parametrize('output', [
    Output('0', 'unavailable'), Output('0', 'invalid'),
    Output('0', 'enabled', 0, 0), Output('0', 'enabled', 20, 21),
    Output('0', 'enabled', 20_000_000, float('nan')),
    Output('0', 'enabled', 20_000_000, True),
    Output('0', 'disabled', -1, 0), Output('0', 'enabled', 20_000_000, 1_000_000),
])
def test_invalid_channels_are_unknown(output):
    assert replace(record(), outputs=(output, *record().outputs[1:])).neutral(10, 1) is None


def test_identity_and_channel_states():
    observation = record()
    assert replace(observation, outputs=(observation.outputs[0],) * 4).neutral(10, 1) is None
    for state in ('disabled', 'unexported'):
        output = Output('0', state, 20_000_000, 0) if state == 'disabled' else Output('0', state)
        assert replace(observation, outputs=(output, *observation.outputs[1:])).neutral(10, 1)
    moving = replace(observation.outputs[0], duty_ns=1_600_000)
    assert replace(observation, outputs=(moving, *observation.outputs[1:])).neutral(10, 1) is False

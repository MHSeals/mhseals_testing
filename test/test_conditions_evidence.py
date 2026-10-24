from dataclasses import replace

import pytest

from mhseals_testing.health.pwm import Output, Readback
from mhseals_testing.harness.conditions_evidence import CommandObservation, EvidenceGate


def inputs():
    return dict(afloat=True, bench=False, now=10., epoch=1, local=(0, 0, 0), raw_active=False,
                command=CommandObservation(10., (0, 0, 0), (0, 0, 0), False, 1, 10., 10.),
                readback=Readback('topic', 10., tuple(Output(str(i), 'pulse', duty_ns=1500000)
                                                     for i in range(4)), True))


@pytest.mark.parametrize('change', [dict(afloat=None), dict(afloat=False), dict(bench=True),
    dict(readback=None), dict(command=None), dict(local=None), dict(raw_active=None),
    dict(local=(.01, 0, 0)), dict(raw_active=True), dict(now=12.01), dict(now=9.)])
def test_missing_and_active_evidence_never_admits(change):
    args = inputs() | change
    assert not EvidenceGate(output_ttl=2, command_ttl=2).update(**args).eligible


@pytest.mark.parametrize('change', [dict(receipt=9., applied=(1, 0, 0)),
    dict(effort=(1e-12, 0, 0)), dict(effort=None), dict(effort_receipt=7.), dict(raw_receipt=None), dict(raw_active=True),
    dict(applied=(float('nan'), 0, 0)), dict(epoch=2), dict(receipt=11.)])
def test_independent_command_proof(change):
    args = inputs()
    args['command'] = replace(args['command'], **change)
    assert not EvidenceGate(output_ttl=2, command_ttl=2).update(**args).eligible


@pytest.mark.parametrize('change', [dict(verified=False), dict(configuration_valid=False),
    dict(receipt=11.), dict(outputs=()), dict(backend='none')])
def test_actual_readback_required(change):
    args = inputs()
    args['readback'] = replace(args['readback'], **change)
    assert not EvidenceGate(output_ttl=2, command_ttl=2).update(**args).eligible


def test_expiry_and_unknown_interval_restart_coast():
    gate = EvidenceGate(output_ttl=2, command_ttl=2)
    args = inputs()
    assert gate.update(**args).coast_started == 10
    assert gate.update(**(args | dict(now=12))).eligible
    assert not gate.update(**(args | dict(now=12.1))).eligible
    args['readback'] = replace(args['readback'], receipt=13.)
    args['command'] = replace(args['command'], receipt=13., effort_receipt=13., raw_receipt=13.)
    resumed = gate.update(**(args | dict(now=13.)))
    assert resumed.eligible and resumed.coast_started == 13
    assert gate.update(**(args | dict(now=13.1, local=(1, 0, 0)))).cut
    assert gate.update(**(args | dict(now=13.2))).coast_started == 13.2
    assert gate.update(**(args | dict(now=13.3, epoch=2))).cut


@pytest.mark.parametrize('ttl', [0, -1, float('nan'), True])
def test_positive_policy(ttl):
    with pytest.raises(ValueError):
        EvidenceGate(output_ttl=ttl, command_ttl=1)

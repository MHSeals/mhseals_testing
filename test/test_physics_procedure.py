"""The physics characterization steps against fakes: records, provenance, refusals."""
from contextlib import contextmanager
import json
import math
from types import SimpleNamespace

import pytest
import yaml

from mhseals_hardware.configuration import config_sha, load_config, update_config
from mhseals_testing import physics_record
from mhseals_testing.analysis import physics as fits
from mhseals_testing.procedures import Settings, physics


def make_ctx(tmp_path, answers=(), runner=None, status=None, **settings):
    config = tmp_path / 'hardware.yaml'
    if not config.exists():
        update_config(dict(reverse_thrust_ratio=0.78), config)
    answers, records = list(answers), []

    def ask_text(title, message, text='', check=None):
        answer = answers.pop(0)
        if answer is not None and check:
            check(answer)
        return answer

    @contextmanager
    def motion(name, **details):
        yield runner

    services = dict(user_config=config, physics_root=tmp_path / 'char', ask_text=ask_text,
                    fit_context=lambda: dict(session='s1', run='r1', battery_v=15.9, muted=False),
                    driver_status=lambda: status)
    ctx = SimpleNamespace(
        settings=Settings(**settings), services=services, record=records.append, motion=motion,
        observe=motion, prompt=lambda *a, **k: '', monitor=SimpleNamespace(clock=lambda: 0.0),
        history=lambda since=float('-inf'), until=float('inf'), name='odometry':
            [s for s in (runner.series.get(name, []) if runner else []) if since <= s[0] <= until])
    return ctx, records


def saved(result):
    record = yaml.safe_load(open(result['path']))
    assert record['schema'] == physics_record.SCHEMA
    return record


def test_weigh_in_records_mass_cg_and_bifilar_inertia_with_provenance(tmp_path):
    ctx, records = make_ctx(tmp_path, ['24.6', '0.05', '0', '-0.02', '2.0', '0.4', '1.5'])
    result = physics.dock(ctx)
    record = saved(result)
    assert record['step'] == 'weigh_in' and record['mass'] == 24.6 and record['cg'] == [0.05, 0.0, -0.02]
    assert record['iz']['value'] == pytest.approx(24.6 * 9.80665 * 0.16 * 4 / (16 * math.pi ** 2 * 1.5))
    assert record['units']['mass'] == 'kg' and record['counts']['entries'] == 7
    hardware = record['hardware']
    assert hardware['keys']['reverse_thrust_ratio'] == 0.78 and hardware['sim_derived'] is False
    assert hardware['hash'].startswith('sha256:') and hardware['physics_hash'].startswith('sha256:')
    assert record['fit']['session'] == 's1' and record['battery_v'] == 15.9
    assert record['conditions'] == {'verdict': 'unknown'}
    assert records[-1]['verdict'] == 'SAVED'


def test_weigh_in_without_a_swing_keeps_the_inertia_a_prior(tmp_path):
    ctx, _ = make_ctx(tmp_path, ['24.6', '0', '0', '0', ''])
    assert saved(physics.dock(ctx))['iz'] == {'value': None, 'method': 'none: prior'}


def test_cancel_or_out_of_range_entry_refuses_and_writes_nothing(tmp_path):
    ctx, records = make_ctx(tmp_path, [None])
    assert physics.dock(ctx)['verdict'] == 'REFUSED'
    ctx, _ = make_ctx(tmp_path, ['500'])
    with pytest.raises(ValueError):
        physics.dock(ctx)       # the popup's check keeps it open on a value out of range
    assert not (tmp_path / 'char').exists()


def test_driver_check_refuses_a_pending_restart_or_another_file(tmp_path):
    config = tmp_path / 'hardware.yaml'
    update_config(dict(reverse_thrust_ratio=0.78), config)
    good = dict(config_sha=config_sha(load_config(config)), effective_config_sha='x', pending_restart_keys=[])
    assert physics_record.driver_check(good, config, True) == (
        {'config_sha': good['config_sha'], 'effective_config_sha': 'x', 'pending_restart_keys': []}, '')
    assert 'restart' in physics_record.driver_check(dict(good, pending_restart_keys=['decoupling']), config, True)[1]
    assert 'another hardware file' in physics_record.driver_check(dict(good, config_sha='0'), config, True)[1]
    assert physics_record.driver_check(None, config, True)[1] and not physics_record.driver_check(None, config, False)[1]


def test_a_record_needs_units_and_counts_and_hashes_its_samples(tmp_path):
    header = dict(step='x', fit=dict(id='f1'))
    with pytest.raises(ValueError):
        physics_record.write(header, dict(units={}), root=tmp_path)
    path = physics_record.write(header, dict(units={}, counts={}), samples=[[0.0, 1.0]], root=tmp_path)
    record = yaml.safe_load(path.read_text())
    text = (tmp_path / record['samples_file']['path']).read_text()
    assert record['samples_file']['sha256'] == physics_record.sha256(text) and json.loads(text) == [[0.0, 1.0]]

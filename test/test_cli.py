"""boat_trial check: exit codes, selection, JSON and --lint without ROS."""

import json

import pytest

from mhseals_testing.harness import cli
from mhseals_testing.health import registry
from mhseals_testing.health.config import HealthConfig
from mhseals_testing.health.kinds import threshold  # noqa: F401  (registers 'threshold')
from mhseals_testing.health.model import CheckSpec, Fact

VALUES = {'v': 3.9}


@pytest.fixture(autouse=True)
def kinds():
    registry.discover()
    VALUES['v'] = 3.9


def config(severity='fail'):
    params = registry.build_params('threshold', {'fact': 'v', 'warn': 3.5, 'fail': 3.3})
    return HealthConfig((CheckSpec(id='power.cell', kind='threshold', group='Power', params=params,
                                   severity=severity, gates=('go',), title='Cell'),
                         CheckSpec(id='other.x', kind='threshold', group='Other', params=params)), {})


def collect():
    return {'v': Fact(VALUES['v'])}, {}, {}


def run(window=3.0, **kw):
    t = [0.0]
    clock = lambda: t[0]
    sleep = lambda s: t.__setitem__(0, t[0] + s)
    return cli.run_check(config(), 'boat', collect=collect, clock=clock, sleep=sleep,
                         window=window, host='h', **kw)


def test_ok_warn_and_fail_exit_codes():
    report, code = run()
    assert code == 0 and report['summary']['ok'] == 2
    VALUES['v'] = 3.4
    assert run()[1] == 0                                  # warn passes by default
    assert run(fail_on='warn')[1] == 1
    VALUES['v'] = 3.0
    assert run()[1] == 2 and run(fail_on='warn')[1] == 2


def test_blocked_gate_exits_2_with_reasons():
    VALUES['v'] = 3.0
    report, code = run(gate='go', only=['power'])
    assert code == 2 and report['gates']['go']['blocking'] == ['Cell: 3.00']


def test_only_selects_group_or_id_glob():
    assert [c['id'] for c in run(only=['Power'])[0]['checks']] == ['power.cell']
    assert [c['id'] for c in run(only=['other.*'])[0]['checks']] == ['other.x']


def test_report_has_the_documented_json_fields():
    report, _ = run()
    assert {'time', 'host', 'profile', 'summary', 'gates', 'checks'} <= set(report)
    assert set(report['checks'][0]) == {'id', 'group', 'status', 'value', 'detail', 'hint',
                                        'data', 'age', 'source'}


def test_main_json_and_exit_code(capsys):
    assert cli.main(['check', '--window', '0', '--json', '-']) == 0
    assert json.loads(capsys.readouterr().out)['profile'] == 'boat'


def test_lint_runs_without_ros_and_reports_problems(capsys, monkeypatch, nav_share):
    assert cli.main(['check', '--lint']) == 0
    assert 'no problems' in capsys.readouterr().out
    bad = config()
    bad = HealthConfig(bad.specs, {'graph': {}}, ())
    monkeypatch.setattr(cli.health_config, 'load', lambda profile=None: bad)
    assert cli.main(['check', '--lint']) == 2
    assert 'positive period' in capsys.readouterr().out


def test_bad_arguments_exit_2():
    with pytest.raises(SystemExit) as exit_:
        cli.main(['check', '--fail-on', 'loud'])
    assert exit_.value.code == 2


def test_missing_fact_reports_unknown_at_once():
    def nothing():
        return {}, {}, {}
    t = [0.0]
    report, code = cli.run_check(config(), 'boat', collect=nothing, clock=lambda: t[0],
                                 sleep=lambda s: t.__setitem__(0, t[0] + s), window=5.0, host='h')
    assert {c['status'] for c in report['checks']} == {'unknown'}
    assert report['summary']['unknown'] == 2 and code == 0
    assert 'unknown' in cli.table(report)


def test_gate_with_a_non_procedure_key_exits_2(monkeypatch, capsys):
    monkeypatch.setattr(cli, 'procedure_keys', lambda: {'go'})
    monkeypatch.setattr(cli.health_config, 'load', lambda profile=None: config())
    assert cli.main(['check', '--gate', 'typo', '--window', '0']) == 2
    assert "'typo'" in capsys.readouterr().err


def test_default_collector_samples_host_facts(capsys):
    assert cli.main(['check', '--window', '0', '--json', '-', '--only', 'compute.load', 'compute.disk']) in (0, 1, 2)
    checks = json.loads(capsys.readouterr().out)['checks']
    assert any(c['status'] != 'unknown' for c in checks if c['id'].startswith('compute'))


def test_all_unknown_is_not_a_pass(capsys):
    assert cli.main(['check', '--window', '0', '--only', 'compute.load'],
                    collect=lambda: ({}, {}, {})) == 3
    assert 'NOTHING CHECKED' in capsys.readouterr().err


def test_banner_counts_partial_unknown():
    report = {'checks': [{}, {}], 'summary': {'unknown': 1}}
    assert cli.unchecked(report) == '1 of 2 checks unknown (no sample from their probes)'
    assert cli.unchecked({'checks': [], 'summary': {'unknown': 0}}) == ''

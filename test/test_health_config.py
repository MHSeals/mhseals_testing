from dataclasses import dataclass, field

import pytest

from mhseals_testing.health import registry
from mhseals_testing.health.config import ConfigError, load, render
from mhseals_testing.health.model import Result
from mhseals_testing.profiles import load_profile


@dataclass(frozen=True)
class RateParams:
    topic: str
    hz: float = 1.0
    pattern: str = '.*'
    names: list = field(default_factory=list)


@pytest.fixture(autouse=True)
def rate_kind(monkeypatch):
    monkeypatch.setattr(registry, 'KINDS', {})
    registry.kind('rate', RateParams, needs=['rates'])(lambda params, facts: Result('ok'))


def write(tmp_path, text, name='a.yaml'):
    (tmp_path / name).write_text(text)
    return tmp_path


def specs(tmp_path, text, **kwargs):
    return load(write(tmp_path, text), user=kwargs.pop('user', False), **kwargs)


def test_each_expands_and_templates_ids(tmp_path):
    config = specs(tmp_path, """
group: Sensors
defaults: {severity: fail}
checks:
  - id: rate.{name}
    kind: rate
    topic: '{topic}'
    each: [{name: gps, topic: /gps/fix, hz: 5}, {name: imu, topic: /imu/raw}]
""")
    gps, imu = config.specs
    assert (gps.id, gps.params.topic, gps.params.hz, gps.group, gps.severity) == \
        ('rate.gps', '/gps/fix', 5, 'Sensors', 'fail')
    assert imu.id == 'rate.imu' and imu.params.hz == 1.0
    assert not hasattr(gps.params, 'name')


def test_profile_fields_fill_templates_and_filter(tmp_path):
    text = """
checks:
  - {id: cmd, kind: rate, topic: '{command_topic}', profiles: [boat]}
  - {id: sim, kind: rate, topic: /x, profiles: [crane_direct]}
"""
    boat = load_profile('boat')
    config = specs(tmp_path, text, profile=boat)
    assert [s.id for s in config.specs] == ['cmd']
    assert config.specs[0].params.topic == boat.command_topic


def test_regex_braces_and_message_fields_survive(tmp_path):
    config = specs(tmp_path, """
checks:
  - {id: log, kind: rate, topic: /t, pattern: 'a\\d{2}', message: '{topic} slow {value}'}
""")
    assert config.specs[0].params.pattern == r'a\d{2}'
    assert config.specs[0].message == '{topic} slow {value}'


def test_profile_fields_do_not_fill_runtime_templates(tmp_path):
    config = specs(tmp_path, """
checks:
  - {id: log, kind: rate, topic: '{command_topic}', message: 'node {name} died', hint: '{description}'}
  - {id: row, kind: rate, topic: /t, message: '{name} slow', each: [{name: gps}]}
""", profile=load_profile('boat'))
    log, row = config.specs
    assert (log.message, log.hint) == ('node {name} died', '{description}')
    assert row.message == 'gps slow'


@pytest.mark.parametrize('check, text', [
    ('kind: nope', "unknown kind"),
    ('kind: rate', "needs topic"),
    ('kind: rate, topic: /t, bogus: 1', "no parameter bogus"),
])
def test_bad_checks_fail_at_load(tmp_path, check, text):
    with pytest.raises(ConfigError, match=text):
        specs(tmp_path, f"checks:\n  - {{id: a, {check}}}\n")


def test_unknown_each_key_fails_unless_it_fills_a_template(tmp_path):
    with pytest.raises(ConfigError, match="each key 'hzz'"):
        specs(tmp_path, "checks: [{id: 'rate.{name}', kind: rate, topic: /t, each: [{name: gps, hzz: 5}]}]")


@pytest.mark.parametrize('field', ['severity: loud', 'missing: never', 'when: [x]'])
def test_invalid_enums_and_when_fail_at_load(tmp_path, field):
    with pytest.raises(ConfigError, match=field.split(':')[0]):
        specs(tmp_path, f"checks: [{{id: a, kind: rate, topic: /t, {field}}}]")


def test_user_file_disables_and_overrides(tmp_path):
    shipped = tmp_path / 'shipped'
    shipped.mkdir()
    write(shipped, """
probes: {git: {fetch: 0}, host: {subnet: 10.0.0.0/24}}
checks:
  - {id: rate.a, kind: rate, topic: /a}
  - {id: rate.b, kind: rate, topic: /b}
  - {id: other, kind: rate, topic: /c}
""")
    user = write(tmp_path, """
disable: ['rate.*']
override: {other: {severity: info, hz: 7}}
probes: {git: {fetch: 900}}
""", name='user.yaml') / 'user.yaml'
    config = load(shipped, user=user)
    assert {s.id: s.enabled for s in config.specs} == {'rate.a': False, 'rate.b': False, 'other': True}
    other = config.specs[2]
    assert (other.severity, other.params.hz) == ('info', 7)
    assert config.probes == {'git': {'fetch': 900}, 'host': {'subnet': '10.0.0.0/24'}}


@pytest.mark.parametrize('text, match', [
    ('checks: [{kind: rate, topic: /t}]', 'without an id'),
    ('checks: {id: a}', 'checks must be a list'),
    ('checks: [oops]', 'must be a mapping'),
    ('defaults: [1]', 'defaults must be a mapping'),
    ('checks: [{id: a, kind: rate, topic: /t, each: x}]', 'each must be a list'),
])
def test_malformed_files_raise_config_error(tmp_path, text, match):
    with pytest.raises(ConfigError, match=match):
        specs(tmp_path, text)


def test_malformed_user_file_raises_config_error(tmp_path):
    shipped = tmp_path / 'shipped'
    shipped.mkdir()
    write(shipped, 'checks: [{id: a, kind: rate, topic: /t}]')
    user = write(tmp_path, 'override: {a: 3}', name='user.yaml') / 'user.yaml'
    with pytest.raises(ConfigError, match='override a must be a mapping'):
        load(shipped, user=user)


def test_scalar_list_fields_become_one_item(tmp_path):
    spec = specs(tmp_path, 'checks: [{id: a, kind: rate, topic: /t, show: rose, gates: motion}]').specs[0]
    assert (spec.show, spec.gates) == (('rose',), ('motion',))


def test_render_leaves_unknown_fields():
    assert render('{a} {b}', a=1) == '1 {b}'

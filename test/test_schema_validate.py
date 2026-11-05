"""schema.validate checks a config file against its fragment, and load_strict runs it on every load with a path."""
import logging
from pathlib import Path

import pytest
import yaml

from mhseals_testing import schema, strict_yaml
from mhseals_testing.strict_yaml import load_strict

OWN = Path(__file__).resolve().parents[1]


def entry(kind, **more):
    return dict(type=kind, units='', tier='live', doc='d', **more)


@pytest.fixture
def config(tmp_path):
    """A config dir with demo.yaml described by schema/demo.yaml; returns (write config, write fragment)."""
    (tmp_path / 'schema').mkdir()

    def write(doc):
        path = tmp_path / 'demo.yaml'
        path.write_text(yaml.safe_dump(doc))
        return path

    def fragment(entries, closed=False):
        doc = {f'demo.{key}': value for key, value in entries.items()}
        if closed:
            doc['demo'] = {'closed': True}
        (tmp_path / 'schema' / 'demo.yaml').write_text(yaml.safe_dump(doc))
    return write, fragment


def test_types_ranges_enums_and_nulls_name_file_key_and_expectation(config):
    write, fragment = config
    fragment({'tune.rate': entry('float', range=[0, 10]), 'tune.mode': entry('str', enum=['a', 'b']),
              'tune.count': entry('int'), 'tune.life': entry('float', nullable=True), 'tune.on': entry('bool')})
    path = write({'tune': {'rate': 3, 'mode': 'a', 'count': 2, 'life': None, 'on': False}})
    assert schema.validate(path, yaml.safe_load(path.read_text())) == []
    path = write({'tune': {'rate': 11.0, 'mode': 'c', 'count': True, 'life': 'x', 'on': 1}})
    assert sorted(schema.validate(path, yaml.safe_load(path.read_text()))) == sorted([
        'demo.yaml:tune.rate: expected within [0, 10], got 11.0',
        "demo.yaml:tune.mode: expected one of ['a', 'b'], got 'c'",
        'demo.yaml:tune.count: expected int, got bool True',
        "demo.yaml:tune.life: expected float, got str 'x'",
        'demo.yaml:tune.on: expected bool, got int 1'])


def test_wildcards_and_list_items(config):
    write, fragment = config
    fragment({'probes.*.period': entry('float'), 'groups.*.checks.[].id': entry('str')}, closed=True)
    path = write({'probes': {'a': {'period': 1}, 'b': {'period': 'x'}},
                  'groups': {'G': {'checks': [{'id': 'ok'}, {'id': 3, 'idd': 'typo'}]}}})
    assert sorted(schema.validate(path, yaml.safe_load(path.read_text()))) == [
        'demo.yaml:groups.G.checks.1.id: expected str, got int 3',
        'demo.yaml:groups.G.checks.1.idd: unknown key (not in schema/demo.yaml)',
        "demo.yaml:probes.b.period: expected float, got str 'x'"]


def test_unknown_keys_count_only_in_a_closed_fragment(config):
    write, fragment = config
    path = write({'tune': {'rate': 1, 'raet': 2}})
    fragment({'tune.rate': entry('float')})
    assert schema.validate(path, yaml.safe_load(path.read_text())) == []
    fragment({'tune.rate': entry('float')}, closed=True)
    assert schema.validate(path, yaml.safe_load(path.read_text())) == [
        'demo.yaml:tune.raet: unknown key (not in schema/demo.yaml)']


def test_a_file_without_a_fragment_is_not_checked(tmp_path):
    path = tmp_path / 'free.yaml'
    path.write_text('anything: 1\n')
    assert schema.validate(path, {'anything': 1}) == []


def test_load_strict_warns_now_and_raises_when_strict(config, caplog, monkeypatch):
    write, fragment = config
    fragment({'tune.rate': entry('float')}, closed=True)
    path = write({'tune': {'rate': 'fast'}})
    with caplog.at_level(logging.WARNING, logger='mhseals_testing.strict_yaml'):
        assert load_strict(path.read_text(), path) == {'tune': {'rate': 'fast'}}
    assert "demo.yaml:tune.rate: expected float, got str 'fast'" in caplog.text
    assert load_strict(path.read_text()) == {'tune': {'rate': 'fast'}}     # no path: nothing to look up
    monkeypatch.setattr(strict_yaml, 'STRICT', True)
    with pytest.raises(ValueError, match=r"demo\.yaml:tune\.rate: expected float, got str 'fast'"):
        load_strict(path.read_text(), path)


@pytest.mark.smoke
@pytest.mark.parametrize('path', sorted((OWN / 'config').glob('*.yaml')), ids=lambda p: p.name)
def test_every_shipped_config_validates_clean(path):
    assert schema.validate(path, load_strict(path.read_text())) == []

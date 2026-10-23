from dataclasses import dataclass

import pytest

from mhseals_testing.health import registry
from mhseals_testing.health.model import Result


@dataclass(frozen=True)
class Params:
    topic: str
    hz: float = 1.0


@pytest.fixture(autouse=True)
def empty_registry(monkeypatch):
    monkeypatch.setattr(registry, 'KINDS', {})


def test_kind_registers_with_declared_fields():
    registry.kind('rate', Params, needs=['rates'], fields=['count'])(lambda p, f: Result('ok'))
    found = registry.KINDS['rate']
    assert (found.needs, found.fields, found.watch(None)) == (('rates',), ('count',), {})


def test_double_register_is_an_error():
    registry.kind('rate', Params, needs=[])(lambda p, f: Result('ok'))
    with pytest.raises(ValueError, match='registered twice'):
        registry.kind('rate', Params, needs=[])(lambda p, f: Result('ok'))


def test_build_params_fills_defaults():
    registry.kind('rate', Params, needs=[])(lambda p, f: Result('ok'))
    assert registry.build_params('rate', {'topic': '/t'}) == Params('/t', 1.0)


@pytest.mark.parametrize('raw, text', [
    ({'topic': '/t', 'bogus': 1}, 'no parameter bogus'),
    ({}, 'needs topic'),
])
def test_build_params_rejects_unknown_and_missing(raw, text):
    registry.kind('rate', Params, needs=[])(lambda p, f: Result('ok'))
    with pytest.raises(ValueError, match=text):
        registry.build_params('rate', raw)

"""docs/health.md stays true to the code: its YAML loads and lints, and it names every kind and check key."""

import dataclasses
from pathlib import Path
import re

import yaml

from mhseals_testing.health import registry
from mhseals_testing.health.config import load, lint
from mhseals_testing.health.model import CheckSpec

ROOT = Path(__file__).resolve().parents[1]
DOC = (ROOT / 'docs' / 'health.md').read_text()


def test_every_registered_kind_is_documented():
    registry.discover()
    shipped = [k.name for k in registry.KINDS.values()
               if k.evaluate.__module__.startswith('mhseals_testing.health.kinds.')]  # not test fakes
    assert shipped
    for name in shipped:
        assert f'`{name}`' in DOC, name


def test_every_check_key_is_documented():
    for field in dataclasses.fields(CheckSpec):
        if field.name != 'params':
            assert f'`{field.name}`' in DOC, field.name


def test_documented_yaml_example_loads_and_lints_clean(tmp_path):
    blocks = re.findall(r'```yaml\n(.*?)```', DOC, re.S)
    example = next(b for b in blocks if 'checks:' in b)
    assert yaml.safe_load(example)['checks']
    (tmp_path / 'example.yaml').write_text(example)
    config = load(tmp_path, user=False)
    assert config.specs and lint(config.specs) == []


def test_readme_links_the_health_doc():
    assert 'docs/health.md' in (ROOT / 'README.md').read_text()

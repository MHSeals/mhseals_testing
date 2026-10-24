"""Course YAML is authority; JSON is a deterministic Unity input artifact."""
import json
from pathlib import Path
import sys
from types import ModuleType

import pytest

from mhseals_testing.course import compile_course

ROOT = Path(__file__).parents[1]


@pytest.fixture
def installed_courses(tmp_path, monkeypatch):
    """Resolve a copied installed share outside the working directory."""
    import shutil
    share = tmp_path / 'installed/share/mhseals_testing'
    shutil.copytree(ROOT / 'config/courses', share / 'config/courses')
    module = ModuleType('ament_index_python.packages')
    module.get_package_share_directory = lambda package: str(share)
    monkeypatch.setitem(sys.modules, 'ament_index_python', ModuleType('ament_index_python'))
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', module)
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path / 'cache'))
    monkeypatch.chdir(tmp_path)
    return share


@pytest.mark.parametrize('name', ['circle_test', 'pair_test'])
def test_shipped_semantics_and_deterministic_generation(installed_courses, name):
    """Shipped values equal the former JSON authoring values exactly."""
    path = compile_course(name)
    original = {
        'name': name,
        'hide': ['SpeedChallenge', 'NavigationChannel', 'GateTraversal', 'Marina', 'DeliveryBoats'],
        'jitter': 0.0,
        'elements': ([{'task': name, 'template': 'RedBuoy0', 'label': 'yellow_buoy',
                       'x': 9.0, 'y': 0.0, 'color': '#f2c20f'}] if name == 'circle_test' else [
            {'task': name, 'template': 'RedPoleBuoy0', 'label': 'red_pole_buoy', 'x': 7.0, 'y': 1.5},
            {'task': name, 'template': 'GreenPoleBuoy0', 'label': 'green_pole_buoy', 'x': 7.0, 'y': -1.5}]),
    }
    assert json.loads(path.read_text()) == original
    assert compile_course(name) == path
    assert path.parent.name == 'course-json'
    assert not list((installed_courses / 'config/courses').glob('*.json'))


@pytest.mark.parametrize('raw', [
    '[]', 'name: bad', 'elements: []',
    'elements: [{template: A, x: 1, x: 2}]',
    'elements: &loop [*loop]',
    'elements: [{template: A, x: .nan}]',
    'elements: [{template: A, x: .inf}]',
    'elements: [{template: A, x: true}]',
])
def test_invalid_source_has_no_artifact(tmp_path, monkeypatch, raw):
    """No cache directory or generated files appear for invalid source bytes."""
    path = tmp_path / 'invalid.yaml'
    path.write_text(raw)
    cache = tmp_path / 'cache'
    monkeypatch.setenv('XDG_CACHE_HOME', str(cache))
    with pytest.raises(ValueError):
        compile_course(str(path))
    assert not cache.exists()
    assert path.read_text() == raw

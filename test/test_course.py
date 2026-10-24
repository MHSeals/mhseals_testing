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


def test_actual_main_prints_only_unity_path_and_consumes_edits(installed_courses, monkeypatch, capsys):
    """The existing entry-point main exports current coordinates, never sampling."""
    from mhseals_testing.harness import cli
    def forbidden(*args, **kwargs):
        pytest.fail('course entered an operational path')
    monkeypatch.setattr(cli, 'load_profile', forbidden)
    monkeypatch.setattr(cli.health_config, 'load', forbidden)
    monkeypatch.setattr(cli, 'host_collector', forbidden)
    assert cli.main(['course', 'pair_test'], collect=forbidden) == 0
    captured = capsys.readouterr()
    first = Path(captured.out.strip())
    assert captured.out == str(first) + '\n' and not captured.err
    original = json.loads(first.read_text())
    source = installed_courses / 'config/courses/pair_test.yaml'
    before = source.read_text()
    source.write_text(before.replace('y: 1.5', 'y: 2.5'))
    edited = source.read_bytes()
    assert cli.main(['course', str(source)]) == 0
    second = Path(capsys.readouterr().out.strip())
    assert first != second and first.exists()
    consumed = json.loads(second.read_text())  # same file-read contract as Unity
    assert original['elements'][0]['y'] == 1.5
    assert consumed['elements'][0]['y'] == 2.5
    assert source.read_bytes() == edited


@pytest.mark.parametrize('source', ['missing_course', '/missing/course.yaml'])
def test_actual_main_invalid_input_exits_without_stdout(installed_courses, capsys, source):
    """A failed substitution cannot be mistaken for a generated input path."""
    from mhseals_testing.harness.cli import main
    assert main(['course', source]) == 2
    captured = capsys.readouterr()
    assert not captured.out and 'boat_trial course:' in captured.err


@pytest.mark.parametrize('verb', ['plan', 'apply', 'set', 'revert', 'observe', 'log'])
def test_course_route_does_not_enable_dormant_config_verbs(verb):
    """The separate config parser remains the catalog-only surface."""
    from mhseals_testing.harness.cli import main
    with pytest.raises(SystemExit) as error:
        main(['config', verb])
    assert error.value.code == 2


def test_actual_main_malformed_yaml_has_no_artifact(tmp_path, monkeypatch, capsys):
    """Parser errors use normal CLI failure, before creating the cache."""
    from mhseals_testing.harness.cli import main
    source = tmp_path / 'malformed.yaml'
    source.write_text('elements: [')
    cache = tmp_path / 'cache'
    monkeypatch.setenv('XDG_CACHE_HOME', str(cache))
    assert main(['course', str(source)]) == 2
    captured = capsys.readouterr()
    assert not captured.out and 'boat_trial course:' in captured.err
    assert not cache.exists()

from pathlib import Path
from types import SimpleNamespace

import pytest

from mhseals_testing import data
from mhseals_testing.configuration import Selection
from mhseals_testing.harness import session
from mhseals_testing.startup_owner import navigation_args, prepare_session


def profile(**kwargs):
    return SimpleNamespace(outputs='odroid', components={
        'navigation': {'launch': ['mhseals_nav', 'navigation.launch.py'],
                       'args': {'sim': 'false'}, 'limits': True}}, **kwargs)


def test_final_profile_overlay_and_reserved_reference():
    selected = navigation_args(profile(), {'navigation': {'sim': 'true', 'use_lidar': 'false'}})
    assert selected['sim'] == 'true'
    assert selected['use_lidar'] == 'false'
    assert 'limits_file' in selected
    for extra in ('config_scope', 'unknown'):
        with pytest.raises(ValueError):
            navigation_args(profile(), {'navigation': {extra: 'wrong'}})


def test_watch_and_adopt_reuse_reference_without_preparation(tmp_path, monkeypatch):
    from mhseals_testing import startup_owner as owner
    created = session.create('boat', directory=tmp_path / 'session')
    reference = Selection(str(tmp_path / 'store'), 'scope', 'a' * 64, ('hardware', 'navigation'))
    session.persist_startup_selection(created.directory, reference.record())
    checked = []
    def validate(self):
        checked.append(self)
        return SimpleNamespace(documents={
            'hardware-pure': {'effective': {'output_backend': 'odroid'}},
            'nav2-compiler': {'invocation': {'sim': False, 'use_lidar': True}}})
    monkeypatch.setattr(Selection, 'validate', validate)
    monkeypatch.setattr(owner, 'bootstrap', lambda *a: pytest.fail('must not bootstrap'))
    monkeypatch.setattr(owner, 'config_path', lambda *a: pytest.fail('must not read authoring inputs'))
    args = SimpleNamespace(config='/missing/mutable.yaml')
    before = {f.name: f.read_bytes() for f in created.directory.iterdir() if f.is_file()}
    for mode in ('watch', 'adopt'):
        assert prepare_session(profile(), args, created.directory, mode, {}) == reference
    assert len(checked) == 2
    assert before == {f.name: f.read_bytes() for f in created.directory.iterdir() if f.is_file()}


def test_legacy_adoption_and_domain_expansion_refuse(tmp_path):
    created = session.create('boat', directory=tmp_path)
    with pytest.raises(ValueError, match='legacy'):
        prepare_session(profile(), SimpleNamespace(), tmp_path, 'adopt', {})
    reference = Selection(str(tmp_path / 'store'), 'scope', 'a' * 64, ('hardware',))
    session.persist_startup_selection(created.directory, reference.record())
    with pytest.raises(ValueError, match='domains differ'):
        prepare_session(profile(), SimpleNamespace(), tmp_path, 'adopt', {})


def test_reference_is_write_once_and_preserves_user_metadata(tmp_path):
    created = session.create('boat', directory=tmp_path)
    session.update(created, title='Keep me', note='recording')
    record = Selection(str(tmp_path / 'store'), 'scope', 'a' * 64, ('hardware',)).record()
    session.persist_startup_selection(tmp_path, record)
    session.update(created, pinned=True)
    session.persist_startup_selection(tmp_path, record)
    with pytest.raises(ValueError, match='immutable'):
        session.persist_startup_selection(tmp_path, dict(record, scope='other'))
    header = data.read_json(created.file)
    assert header['title'] == 'Keep me' and header['note'] == 'recording' and header['pinned']
    assert header['startup_selection'] == record


def test_real_owner_empty_root_persists_then_adopts_captured_inputs(tmp_path, monkeypatch):
    import shutil
    import sys
    from types import ModuleType
    from mhseals_testing import startup_owner as owner
    from mhseals_testing.configuration import navigation
    source = Path(navigation.__file__).parents[1]
    share = tmp_path / 'share'
    for name in ('launch', 'config'):
        shutil.copytree(source / name, share / name)
    shutil.copytree(source / 'mhseals_nav/behavior_trees', share / 'behavior_trees')
    packages = ModuleType('ament_index_python.packages')
    packages.get_package_share_directory = lambda name: str(share)
    monkeypatch.setitem(sys.modules, 'ament_index_python', ModuleType('ament_index_python'))
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    monkeypatch.setenv('MHSEALS_CONFIG_ROOT', str(tmp_path / 'store'))
    operator = tmp_path / 'operator.yaml'
    operator.write_text('manual_yaw_scale: 0.5\n')
    args = SimpleNamespace(config=str(operator))
    created = session.create('boat', directory=tmp_path / 'session')
    selected_profile = profile()
    selected_profile.outputs = 'topic'
    reference = prepare_session(selected_profile, args, created.directory, 'own', {})
    header = data.read_json(created.file)
    assert header['startup_selection'] == reference.record()
    assert len(reference.validate().documents) == 4
    captured = reference.validate().documents['hardware-pure']
    assert captured['baseline']['output_backend'] == 'odroid'
    assert captured['effective']['output_backend'] == 'topic'
    operator.write_text('invalid yaml: [')
    (share / 'config/nav2_params.yaml').write_text('invalid yaml: [')
    assert prepare_session(selected_profile, args, created.directory, 'adopt', {}) == reference
    for changed in ({'sim': 'true'}, {'use_lidar': 'false'}):
        with pytest.raises(ValueError, match='invocation differs'):
            prepare_session(selected_profile, args, created.directory, 'adopt', {'navigation': changed})
    selected_profile.outputs = 'odroid'
    with pytest.raises(ValueError, match='backend differs'):
        prepare_session(selected_profile, args, created.directory, 'watch', {})


def test_robot_nav2_source_argument_is_forwarded_into_preparation():
    robot = SimpleNamespace(outputs='none', components={'robot': {
        'launch': ['mhseals_nav', 'robot.launch.py'],
        'args': {'nav2_params_file': '/selected/nav.yaml', 'sim': 'false', 'fcu': 'false'}}})
    invocation = navigation_args(robot, {})
    assert invocation['params_file'] == '/selected/nav.yaml'
    assert 'limits_file' in invocation
    with pytest.raises(ValueError, match='nav2_params_file'):
        navigation_args(robot, {'robot': {'params_file': '/ignored.yaml'}})

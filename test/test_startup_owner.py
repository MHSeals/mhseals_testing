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
    monkeypatch.setattr(Selection, 'validate', lambda self: checked.append(self))
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

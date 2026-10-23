from datetime import datetime, timezone
import json
import os

import pytest

from mhseals_testing import data
from mhseals_testing.harness import session

NOW = datetime(2026, 10, 23, 14, 2, 1, tzinfo=timezone.utc)


@pytest.fixture(autouse=True)
def root(tmp_path, monkeypatch):
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path / 'bags'))
    return tmp_path / 'bags'


def test_layout_and_header(root):
    made = session.create('boat', host='blastoise', now=NOW, git={'astro_dock': 'abc'}, config={'hardware': 'v1'})
    assert made.id == '20261023_140201_blastoise_boat'
    assert made.directory == root / 'sessions' / made.id
    assert made.logs.is_dir() and made.snapshots.is_dir()
    header = json.loads(made.file.read_text())
    assert list(header)[0] == 'schema' and header['schema'] == 'mhseals.session/1'
    assert (header['title'], header['note'], header['pinned']) == ('', '', False)
    assert header['host'] == 'blastoise' and header['profile'] == 'boat'
    assert header['clock'] == {'domain': 'wall', 'time_scale': 1.0}
    assert header['git'] == {'astro_dock': 'abc'} and header['config'] == {'hardware': 'v1'}


def test_same_second_ids_are_unique():
    ids = {session.create('boat', host='h', now=NOW).id for _ in range(5)}
    assert len(ids) == 5 and '20261023_140201_h_boat_02' in ids


def test_names_are_sanitised_so_they_cannot_escape():
    made = session.create('../evil name', host='a/b', now=NOW)
    assert made.directory.parent == data.path('sessions')
    assert '/' not in made.id


def test_never_cwd_relative(tmp_path, monkeypatch):
    monkeypatch.setenv('MHSEALS_DATA', 'relative')
    monkeypatch.delenv('ROS_WS', raising=False)
    monkeypatch.setenv('HOME', str(tmp_path / 'home'))
    elsewhere = tmp_path / 'cwd'
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)
    made = session.create('boat', host='h', now=NOW)
    assert made.directory.is_absolute() and not made.directory.is_relative_to(elsewhere)
    assert list(elsewhere.iterdir()) == []


def test_update_is_atomic_and_limited(root, monkeypatch):
    made = session.create('boat', host='h', now=NOW)
    session.update(made, title='square', pinned=True)
    header = json.loads(made.file.read_text())
    assert header['title'] == 'square' and header['pinned'] is True and header['host'] == 'h'
    assert not [p for p in made.directory.iterdir() if p.name.endswith('.tmp')]
    with pytest.raises(ValueError):
        session.update(made, host='other')
    before = made.file.read_text()
    monkeypatch.setattr(os, 'replace', lambda *a: (_ for _ in ()).throw(OSError('boom')))
    with pytest.raises(OSError):
        session.update(made, note='lost')
    assert made.file.read_text() == before      # a failed edit leaves the old header whole
    assert not [p for p in made.directory.iterdir() if p.name.endswith('.tmp')]


def test_legacy_scan_is_read_only(root):
    (root / 'boat_20261001_100000').mkdir(parents=True)
    (root / 'boat_20261001_100000' / 'metadata.yaml').write_text('x')
    (root / 'crane_full_stack_20260901_090000').mkdir()
    (root / 'crane_full_stack_20260901_090000' / 'metadata.yaml').write_text('x')
    (root / 'old').mkdir()
    os.utime(root / 'old', (1.9e9, 1.9e9))      # no stamp: mtime (2030) decides
    (root / 'old.report.json').write_text('{}')
    (root / 'logs').mkdir()
    (root / 'junk').mkdir()
    session.create('boat', host='h', now=NOW)
    before = sorted(p for p in root.rglob('*'))
    names = [s['name'] for s in session.scan_legacy()]
    # name order (crane > boat) differs from time order (boat is newer)
    assert names == ['old', 'boat_20261001_100000', 'crane_full_stack_20260901_090000']
    assert sorted(root.rglob('*')) == before


def test_legacy_scan_without_root(root):
    assert session.scan_legacy() == []

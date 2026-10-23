import json
import os
from pathlib import Path

import pytest

from mhseals_testing import data
from mhseals_testing.ui.app import parse_args


@pytest.fixture
def root(tmp_path, monkeypatch):
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path / 'bags'))
    return tmp_path / 'bags'


def test_root_precedence_and_never_cwd(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.delenv('MHSEALS_DATA', raising=False)
    monkeypatch.setenv('ROS_WS', str(tmp_path / 'ws'))
    assert data.root() == tmp_path / 'ws' / 'bags'
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path / 'data'))
    assert data.root() == tmp_path / 'data'
    monkeypatch.delenv('MHSEALS_DATA')
    monkeypatch.delenv('ROS_WS')
    assert data.root().parts[-2:] == ('astro_dock', 'bags')
    assert not str(data.root()).startswith(str(tmp_path))


def test_relative_env_is_ignored_not_resolved(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv('MHSEALS_DATA', 'bags')
    monkeypatch.setenv('ROS_WS', 'ws')
    assert data.root().parts[-2:] == ('astro_dock', 'bags')
    assert not str(data.root()).startswith(str(tmp_path))
    monkeypatch.setenv('ROS_WS', str(tmp_path / 'abs'))
    assert data.root() == tmp_path / 'abs' / 'bags'


def test_path_stays_under_root(root):
    assert data.path('sessions', 'a', 'session.json') == root / 'sessions/a/session.json'
    for bad in [('..', 'x'), ('/etc', 'passwd'), ('a', '..', '..', 'x')]:
        with pytest.raises(ValueError):
            data.path(*bad)


def test_atomic_replace_keeps_old_file_on_failure(root, monkeypatch):
    target = data.path('sessions', 's', 'session.json')
    data.write_json(target, {'id': 1}, 'mhseals.session/1')
    data.write_json(target, {'id': 2}, 'mhseals.session/1')
    assert data.read_json(target)['id'] == 2

    def boom(*_):
        raise OSError('disk')
    monkeypatch.setattr(os, 'replace', boom)
    with pytest.raises(OSError):
        data.write_json(target, {'id': 3}, 'mhseals.session/1')
    assert data.read_json(target)['id'] == 2
    assert [p.name for p in target.parent.iterdir()] == ['session.json']


def test_schema_is_first_field(root):
    target = data.path('runs', 'r.json')
    data.write_json(target, {'a': 1, 'schema': 'stale'}, 'mhseals.run/1')
    assert list(json.loads(target.read_text()))[0] == 'schema'
    assert data.read_json(target)['schema'] == 'mhseals.run/1'


def test_torn_line_is_skipped(root):
    target = data.path('events.jsonl')
    data.append_jsonl(target, {'n': 1}, schema='mhseals.event/1')
    data.append_jsonl(target, {'n': 2})
    with open(target, 'a') as handle:
        handle.write('{"n": 3, "tor')
    assert [r['n'] for r in data.read_jsonl(target)] == [1, 2]
    assert data.read_jsonl(data.path('missing.jsonl')) == []


def test_long_line_is_refused(root):
    with pytest.raises(ValueError):
        data.append_jsonl(data.path('e.jsonl'), {'blob': 'x' * data.MAX_LINE})


def test_append_after_torn_line_is_read_back(root):
    log = root / 'x.jsonl'
    data.append_jsonl(log, {'a': 1})
    with open(log, 'a') as handle:
        handle.write('{"torn":')
    data.append_jsonl(log, {'a': 2})
    assert data.read_jsonl(log) == [{'a': 1}, {'a': 2}]


def test_write_json_mode_matches_append_and_is_preserved(root):
    new, log = root / 'new.json', root / 'x.jsonl'
    old = os.umask(0o002)  # the common user-private-group default
    try:
        data.write_json(new, {}, 's/1')
        data.append_jsonl(log, {})
    finally:
        os.umask(old)
    assert new.stat().st_mode == log.stat().st_mode
    new.chmod(0o640)
    data.write_json(new, {'a': 1}, 's/1')
    assert new.stat().st_mode & 0o777 == 0o640


def test_nan_is_refused_and_leaves_no_file(root):
    with pytest.raises(ValueError):
        data.write_json(root / 'a.json', {'x': float('nan')}, 'mhseals.t/1')
    with pytest.raises(ValueError):
        data.append_jsonl(root / 'a.jsonl', {'x': float('inf')})
    assert not (root / 'a.json').exists()
    assert not (root / 'a.jsonl').exists() or (root / 'a.jsonl').read_text() == ''


def test_torn_utf8_tail_is_skipped(root):
    target = root / 'a.jsonl'
    data.append_jsonl(target, {'a': 1})
    with open(target, 'ab') as handle:
        handle.write('{"name":"é'.encode()[:-1])    # cut inside the two-byte character
    assert data.read_jsonl(target) == [{'a': 1}]
    data.append_jsonl(target, {'a': 2})
    assert data.read_jsonl(target) == [{'a': 1}, {'a': 2}]


def test_data_root_is_root_and_ignores_install_path(monkeypatch, tmp_path):
    monkeypatch.delenv('MHSEALS_DATA', raising=False)
    monkeypatch.setenv('ROS_WS', str(tmp_path / 'ws'))
    assert data.data_root() == tmp_path / 'ws' / 'bags'


def test_default_session_bag_lands_under_root(monkeypatch, tmp_path):
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path / 'd'))
    monkeypatch.chdir(tmp_path)
    args, _ = parse_args([])
    assert Path(args.output).parent == tmp_path / 'd'


def test_relative_data_env_does_not_land_in_cwd(monkeypatch, tmp_path):
    monkeypatch.setenv('MHSEALS_DATA', 'rel_bags')
    monkeypatch.chdir(tmp_path)
    args, _ = parse_args([])
    assert Path(args.output).is_absolute()
    assert not str(args.output).startswith(str(tmp_path))

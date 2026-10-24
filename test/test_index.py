import json
from pathlib import Path

import pytest

from mhseals_testing import data
from mhseals_testing.harness import index

META = """rosbag2_bagfile_information:
  starting_time: {nanoseconds_since_epoch: 1700000000000000000}
  duration: {nanoseconds: 5000000000}
  topics_with_message_count:
  - topic_metadata: {name: /odom/local, type: nav_msgs/msg/Odometry}
    message_count: 50
"""


@pytest.fixture
def root(tmp_path, monkeypatch):
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path / 'bags'))
    return tmp_path / 'bags'


def session(root, name, header=True, **extra):
    directory = root / 'sessions' / name
    directory.mkdir(parents=True)
    if header:
        data.write_json(directory / 'session.json', {'id': name, 'host': 'h', 'profile': 'boat', 'created': '2026-10-23T14:00:00Z',
                                                      **extra}, 'mhseals.session/1')
    return directory


def test_finalized_session(root):
    d = session(root, 'a', ended='2026-10-23T14:01:00Z')
    (d / 'bag').mkdir()
    (d / 'bag' / 'bag_0.mcap').write_bytes(b'x' * 10)
    (d / 'bag' / 'metadata.yaml').write_text(META)
    data.append_jsonl(d / 'events.jsonl', {'phase': 'marker', 'kind': 'user', 'id': 1, 'note': 'n'})
    data.append_jsonl(d / 'events.jsonl', {'kind': 'hold'})
    data.write_json(d / 'runs' / 'r1.json', {'id': 'r1', 'procedure': 'square', 'status': 'MEASURED'}, 'mhseals.run/1')
    s = index.summarize(d)
    assert s['flags'] == [] and s['outcome'] == 'measured'
    assert s['time']['duration_s'] == 5.0 and len(s['markers']) == 1
    assert s['topics'] == [{'name': '/odom/local', 'type': 'nav_msgs/msg/Odometry', 'count': 50}]


def test_unfinalized_empty_and_open(root):
    d = session(root, 'u')
    (d / 'bag').mkdir()
    (d / 'bag' / 'bag_0.mcap').write_bytes(b'x')
    assert index.summarize(d)['flags'] == ['unfinalized', 'open']
    e = session(root, 'e', ended='x')
    (e / 'bag').mkdir()
    (e / 'bag' / 'bag_0.mcap').write_bytes(b'')
    assert index.summarize(e)['flags'] == ['empty']
    assert index.summarize(session(root, 'n', ended='x'))['flags'] == []   # no bag at all


def test_bare_bag_dir(root, tmp_path):
    bare = tmp_path / 'navbag'
    bare.mkdir()
    (bare / 'a.mcap').write_bytes(b'x')
    (bare / 'metadata.yaml').write_text(META)
    s = index.summarize(bare)
    assert s['flags'] == ['bare'] and s['id'] == 'navbag' and s['kind'] == 'headless'
    assert s['topics'][0]['count'] == 50


def test_rebuild_writes_index_summary_readme(root):
    session(root, 'old', ended='x', created='2026-10-22T00:00:00Z')
    session(root, 'new', ended='x', clock={'domain': 'sim', 'time_scale': 1.0})
    session(root, 'bare', header=False)
    rows = index.rebuild()
    assert [r['id'] for r in rows] == ['new', 'old', 'bare']
    assert json.loads((root / 'INDEX.json').read_text())['schema'] == 'mhseals.index/1'
    assert rows[0]['kind'] == 'sim'
    assert (root / 'sessions' / 'old' / 'summary.json').exists()
    assert not list(root.glob('sessions/*/index.md'))


def test_ensure_rebuilds_missing_or_stale_only(root):
    d = session(root, 'a', ended='x')
    assert index.ensure() is True and index.ensure() is False
    (root / 'INDEX.json').unlink()
    assert index.ensure() is True
    import os
    os.utime(d / 'session.json', (4e9, 4e9))
    assert index.ensure() is True


def test_readme_golden(root):
    index.rebuild()
    assert (root / 'README.md').read_text() == (Path(__file__).parent / 'fixtures' / 'data_README.md').read_text()


def test_legacy_flat_bags_are_listed_read_only(root):
    legacy = root / 'boat_20261001_120000'
    legacy.mkdir(parents=True)
    (legacy / 'metadata.yaml').write_text(META)
    (legacy / 'a.mcap').write_bytes(b'x')
    session(root, 'new', ended='x')
    rows = index.rebuild()
    assert {r['id'] for r in rows} == {'new', 'boat_20261001_120000'}
    assert 'legacy' in next(r for r in rows if r['id'] != 'new')['flags']
    assert not (legacy / 'summary.json').exists()


def test_system_and_user_markers_counted_in_row(root):
    d = session(root, 'm')
    for i, kind in enumerate(('user', 'system'), 1):
        data.append_jsonl(d / 'events.jsonl', {'phase': 'marker', 'kind': kind, 'id': i})
    data.append_jsonl(d / 'events.jsonl', {'phase': 'hold'})
    s = index.summarize(d)
    assert len(s['markers']) == 2 and index.row(s)['markers'] == 2

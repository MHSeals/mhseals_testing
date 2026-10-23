import pytest

from mhseals_testing import data
from mhseals_testing.harness import bags


@pytest.fixture
def root(tmp_path, monkeypatch):
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path / 'data'))
    return tmp_path / 'data'


def session(root, name, start, size=0, runs=0, markers=0, closed=True, **meta):
    folder = root / 'sessions' / name
    segment = folder / 'bag' / '00'
    segment.mkdir(parents=True)
    (segment / 'bag_0.mcap').write_bytes(b'x' * size)
    if closed:
        (segment / 'metadata.yaml').write_text('x')
    data.write_json(folder / 'session.json', {'start': start, 'end': start + 60, **meta}, bags.SCHEMA)
    for index in range(runs):
        data.write_json(folder / 'runs' / f'r{index}.json', {}, 'mhseals.run/1')
    for index in range(markers):
        data.append_jsonl(folder / 'events.jsonl', {'phase': 'marker', 'id': index})
    return folder


def test_scan_lists_newest_first_with_flags(root):
    session(root, 'a', 1000, size=10, runs=2, title='first')
    session(root, 'b', 2000, size=20, markers=1, closed=False)
    session(root, 'c', 3000)
    found = bags.scan(root)
    assert [b.id for b in found] == ['c', 'b', 'a']
    a, b, c = found[2], found[1], found[0]
    assert (a.title, a.duration, a.outcome, a.empty, a.unfinalized) == ('first', 60, '2 runs', False, False)
    assert b.unfinalized and b.markers == 1 and not b.empty
    assert c.empty and a.size > 10


def test_recording_session_is_not_unfinalized(root):
    session(root, 'live', 1000, closed=False)
    assert bags.scan(root, recording={'live'})[0].recording
    assert not bags.scan(root, recording={'live'})[0].unfinalized


def test_legacy_layout_is_listed_with_its_report(root):
    legacy = root / 'boat_20261018_192251'
    legacy.mkdir(parents=True)
    (legacy / 'metadata.yaml').write_text('x')
    data.write_json(root / 'boat_20261018_192251.report.json', {'results': [{}, {}]}, 'r')
    (root / 'logs').mkdir()
    found = bags.scan(root)
    assert [b.id for b in found] == ['boat_20261018_192251']
    assert found[0].legacy and found[0].outcome == '2 results' and not found[0].empty


def test_set_meta_round_trips_and_keeps_other_fields(root):
    folder = session(root, 'a', 1000, profile={'sim': True})
    bag = bags.scan(root)[0]
    bags.set_meta(bag, title='wind test', pinned=True)
    again = bags.scan(root)[0]
    assert (again.title, again.pinned, again.sim) == ('wind test', True, True)
    assert data.read_json(folder / 'session.json')['start'] == 1000
    with pytest.raises(ValueError):
        bags.set_meta(bag, size=1)


def test_set_meta_on_legacy_uses_a_sidecar(root):
    (root / 'old').mkdir(parents=True)
    (root / 'old' / 'metadata.yaml').write_text('x')
    bags.set_meta(bags.scan(root)[0], note='n')
    assert bags.scan(root)[0].note == 'n' and (root / 'old.meta.json').exists()


def test_prune_never_touches_pinned_or_recording(root):
    session(root, 'pinned', 1, size=500, pinned=True)
    session(root, 'live', 2, size=500)
    session(root, 'empty_old', 3)
    session(root, 'old', 4, size=500, runs=1)
    session(root, 'new', 5, size=500, runs=1)
    found = bags.scan(root, recording={'live'})
    ids = [b.id for b in bags.prune_candidates(found, budget=0)]
    assert ids == ['empty_old', 'old', 'new']
    assert not {'pinned', 'live'} & set(ids)


def test_prune_stops_once_under_budget(root):
    session(root, 'old', 1, size=3000, runs=1)
    session(root, 'new', 2, size=3000, runs=1)
    found = bags.scan(root)
    total = sum(b.size for b in found)
    assert [b.id for b in bags.prune_candidates(found, total)] == []
    assert [b.id for b in bags.prune_candidates(found, total - 1)] == ['old']


def test_delete_refuses_pinned_and_recording(root):
    session(root, 'p', 1, pinned=True)
    session(root, 'r', 2)
    session(root, 'x', 3)
    by_id = {b.id: b for b in bags.scan(root, recording={'r'})}
    for name in ('p', 'r'):
        with pytest.raises(PermissionError):
            bags.delete(by_id[name])
        assert by_id[name].path.exists()
    bags.delete(by_id['x'])
    assert not by_id['x'].path.exists()


def test_disk_reports_filesystem_and_bag_bytes(root):
    session(root, 'a', 1, size=1234)
    used, total, size = bags.disk(root)
    assert 0 < used <= total and size >= 1234
    assert bags.disk(root / 'missing')[2] == 0


def test_legacy_folder_with_bag_data_is_not_empty_or_pruned(root):
    legacy = root / 'boat_x'
    legacy.mkdir(parents=True)
    (legacy / 'boat_x_0.mcap').write_bytes(b'x' * 10)
    found = bags.scan(root)
    assert not found[0].empty
    assert bags.prune_candidates(found, budget=10**9) == []


def test_summary_flags_decide_empty_when_present(root):
    folder = session(root, 'a', 1000, size=10)
    data.write_json(folder / 'summary.json', {'flags': ['empty']}, 'mhseals.summary/1')
    assert bags.scan(root)[0].empty


def test_legacy_scan_skips_folders_without_a_bag(root):
    (root / 'holding' / 'notes').mkdir(parents=True)
    (root / 'holding' / 'notes' / 'a.txt').write_text('x')
    (root / 'real' / 'sub').mkdir(parents=True)
    (root / 'real' / 'sub' / 'metadata.yaml').write_text('x')
    assert [b.id for b in bags.scan(root)] == ['real']

import os

from mhseals_testing import data
from mhseals_testing.harness import markers


def _session(tmp_path):
    bag = tmp_path / 'bag_00'
    bag.mkdir()
    (bag / 'bag_00_0.mcap').write_bytes(b'x' * 64)
    (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information: {}\n')
    data.append_jsonl(tmp_path / 'events.jsonl', {'t_ns': 100, 'phase': 'marker', 'kind': 'user', 'id': 1, 'note': ''})
    data.append_jsonl(tmp_path / 'events.jsonl', {'t_ns': 900, 'phase': 'marker', 'kind': 'system', 'id': 2, 'note': 'sim reset'})
    return tmp_path


def _snapshot(root):
    return {str(p): (p.stat().st_mtime_ns, p.stat().st_size) for p in (root / 'bag_00').iterdir()}


def test_add_at_playhead_folds_in_time_order_and_flags_edit(tmp_path):
    s = _session(tmp_path)
    m = markers.add_at(s, 500, 'wobble')
    assert m['id'] == 3 and m['edited']
    got = markers.read(s)
    assert [x['id'] for x in got] == [1, 3, 2]
    assert got[1]['note'] == 'wobble' and got[1]['edited'] and not got[0]['edited']


def test_note_nearest_picks_closest_and_latest_note_wins(tmp_path):
    s = _session(tmp_path)
    assert markers.note_nearest(s, 800, 'first')['id'] == 2
    markers.note_nearest(s, 850, 'second')
    got = {m['id']: m for m in markers.read(s)}
    assert got[2]['note'] == 'second' and got[2]['edited'] and got[1]['note'] == ''


def test_note_without_markers_is_none(tmp_path):
    assert markers.note_nearest(tmp_path, 5, 'x') is None


def test_torn_line_skipped_and_bag_untouched(tmp_path):
    s = _session(tmp_path)
    before = _snapshot(s)
    with open(s / 'events.jsonl', 'a') as f:
        f.write('{"phase": "mark')
    markers.add_at(s, 300)
    markers.note_nearest(s, 100, 'n')
    assert len(markers.read(s)) == 3
    assert _snapshot(s) == before

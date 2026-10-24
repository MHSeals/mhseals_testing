"""Segment finalize: keeper-child reindex, never on the recording segment."""

from pathlib import Path

from mhseals_testing.harness.bags import EMPTY, UNINDEXED, finalize, segment_state


def seg(tmp_path, files, name='bag_00'):
    d = tmp_path / name
    d.mkdir()
    for fname, data in files.items():
        (d / fname).write_bytes(data)
    return d


class Runner:
    def __init__(self, index=True, code=0):
        self.calls, self.index, self.code = [], index, code

    def __call__(self, argv):
        self.calls.append((argv, sorted(p.name for p in Path(argv[3]).iterdir())))
        if self.index and self.code == 0:
            (Path(argv[3]) / 'metadata.yaml').write_text('x')
        return self.code


def test_refused_on_recording_segment(tmp_path):
    d = seg(tmp_path, {'a_0.mcap': b'x'})
    run = Runner()
    assert finalize(d, run, recording=d)['status'] == 'refused'
    assert not run.calls


def test_only_zero_byte_files_is_empty_without_reindex(tmp_path):
    d = seg(tmp_path, {'a_0.mcap': b''})
    run = Runner()
    assert segment_state(d) == EMPTY
    assert finalize(d, run)['status'] == 'empty'
    assert not run.calls and not list(d.glob('*.mcap'))


def test_zero_byte_removed_before_reindex(tmp_path):
    d = seg(tmp_path, {'a_0.mcap': b'data', 'a_1.mcap': b''})
    run = Runner()
    assert segment_state(d) == UNINDEXED
    assert finalize(d, run, recording=tmp_path / 'bag_01')['status'] == 'done'
    assert run.calls[0][1] == ['a_0.mcap']


def test_indexed_untouched_and_failure_reported(tmp_path):
    done = seg(tmp_path, {'a_0.mcap': b'x', 'metadata.yaml': b'm'})
    run = Runner()
    assert finalize(done, run)['status'] == 'indexed' and not run.calls
    bad = seg(tmp_path, {'a_0.mcap': b'x'}, 'bag_01')
    assert finalize(bad, Runner(code=1))['status'] == 'failed'


def test_flat_live_session_markers_come_from_sibling_journal(tmp_path):
    """A flat live session keeps its journal beside the folder as <name>.events.jsonl."""
    from mhseals_testing import data
    from mhseals_testing.harness import bags
    (tmp_path / 'live').mkdir()
    for i in (1, 2):
        data.append_jsonl(tmp_path / 'live.events.jsonl', {'phase': 'marker', 'kind': 'user', 'id': i})
    assert bags._markers(tmp_path / 'live') == 2

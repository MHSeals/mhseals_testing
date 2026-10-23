"""Replay trail: loaded from trail.jsonl and cut at the playhead."""
import json

from mhseals_testing.ui.minimap import trail


def _write(tmp_path, lines):
    f = tmp_path / 'minimap' / 'trail.jsonl'
    f.parent.mkdir()
    f.write_text('\n'.join(lines) + '\n')


def _crumb(t, x=0.0):
    return json.dumps({'schema': trail.SCHEMA, 't_ns': t, 'x': x, 'y': 0.0, 'fix': 6})


def test_trail_ends_at_the_playhead(tmp_path):
    _write(tmp_path, [_crumb(t * 10**9, t) for t in range(10)])
    crumbs = trail.load(tmp_path)
    assert [c['x'] for c in trail.upto(crumbs, 4 * 10**9)] == [0, 1, 2, 3, 4]
    assert trail.upto(crumbs, -1) == []
    assert len(trail.upto(crumbs, 99 * 10**9)) == 10


def test_load_sorts_and_drops_bad_lines(tmp_path):
    bad = json.dumps({'schema': 'other/1', 't_ns': 1, 'x': 0, 'y': 0})
    _write(tmp_path, [_crumb(3), '{"t_ns": 2, "x": ', bad, _crumb(1), '{"t_ns": 5, "x": NaN, "y": 0}', _crumb(2)])
    assert [c['t_ns'] for c in trail.load(tmp_path)] == [1, 2, 3]


def test_missing_file_is_an_empty_trail(tmp_path):
    assert trail.load(tmp_path) == []

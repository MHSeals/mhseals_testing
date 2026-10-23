"""Headless referee analysis writes analysis/referee.<ts>.json and --json prints exactly that."""
import io
import json

import pytest

from mhseals_testing import data
from mhseals_testing.harness import referee_analyze as ra
from test.test_referee import TRUTH, line


def session(tmp_path, track, task='nav_channel', events=()):
    root = tmp_path / 'sessions' / 's1'
    root.mkdir(parents=True)
    (root / 'course_truth.json').write_text(json.dumps(TRUTH))
    (root / 'track.jsonl').write_text(''.join(
        json.dumps(dict(t=t, x=x, y=y, yaw=yaw)) + '\n' for t, x, y, yaw in track))
    (root / 'events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in events))
    (root / 'session.json').write_text(json.dumps(dict(task=task)))
    return root


def test_json_equals_the_written_file(tmp_path):
    root = session(tmp_path, line((0, 0), (16, 0)))
    out = io.StringIO()
    assert ra.main([str(root), '--json'], out=out) == 0
    [written] = (root / 'analysis').glob('referee.*.json')
    assert json.loads(out.getvalue()) == json.loads(written.read_text())
    assert json.loads(written.read_text())['verdict'] == 'PASS'
    assert data.read_json(written)['schema'] == 'mhseals.referee/1'


def test_fail_exits_one_and_prints_the_summary(tmp_path, capsys):
    root = session(tmp_path, line((0, 0), (10, 0)))
    out = io.StringIO()
    assert ra.main([str(root)], out=out) == 1
    assert out.getvalue().strip() == 'nav_channel: FAIL · 0/1 PASS · 0 contacts'


def test_unjudgeable_session_is_refused(tmp_path, capsys):
    assert ra.main([str(tmp_path / 'missing')]) == 2
    root = session(tmp_path, line((0, 0), (1, 0)), task='nope')
    assert ra.main([str(root)]) == 2
    assert 'unknown task' in capsys.readouterr().err

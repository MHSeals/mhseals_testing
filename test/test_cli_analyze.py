"""boat_trial analyze, index and check --require, (harness-gaps P7). No ROS."""

import json

from mhseals_testing import data
from mhseals_testing.harness import cli
from test_referee_analyze import line, session


def test_analyze_referee_runs_the_referee_main(tmp_path, capsys):
    root = session(tmp_path, line((0, 0), (16, 0)))
    assert cli.main(['analyze', 'referee', str(root), '--json']) == 0
    [written] = (root / 'analysis').glob('referee.*.json')
    assert json.loads(capsys.readouterr().out) == json.loads(written.read_text())
    assert cli.main(['analyze', 'referee', str(tmp_path / 'missing')]) == 2


def test_analyze_dispatches_through_the_registry(monkeypatch):
    seen = []
    monkeypatch.setitem(cli.ANALYSES, 'fake', __name__)
    monkeypatch.setattr(__import__(__name__), 'main', lambda argv: seen.append(argv) or 7, raising=False)
    assert cli.main(['analyze', 'fake', 's1', '--run', '002-square']) == 7
    assert seen == [['s1', '--run', '002-square']]


def test_index_json_rebuilds_or_summarizes(data_root, capsys):
    one = data_root / 'sessions' / '20261025_120000_b_boat'
    one.mkdir(parents=True)
    data.write_json(one / 'session.json', {'id': one.name, 'profile': 'boat'}, 'mhseals.session/1')
    assert cli.main(['index', '--json']) == 0
    index = json.loads(capsys.readouterr().out)
    assert index['schema'] == 'mhseals.index/1' and [r['id'] for r in index['sessions']] == [one.name]
    assert (data_root / 'INDEX.json').exists()
    assert cli.main(['index', str(one), '--json']) == 0
    summary = json.loads(capsys.readouterr().out)
    assert summary['schema'] == 'mhseals.summary/1' and 'open' in summary['flags']
    assert cli.main(['index', str(data_root / 'nope')]) == 2


def report(*rows):
    checks = [{'id': i, 'group': g, 'status': s, 'value': '', 'detail': ''} for i, g, s in rows]
    summary = {st: sum(c['status'] == st for c in checks) for st in ('ok', 'info', 'unknown', 'warn', 'fail')}
    return {'summary': summary, 'gates': {}, 'checks': checks}


def test_require_exits_3_when_a_group_was_skipped(monkeypatch, capsys):
    found = report(('sys.load', 'System', 'ok'), ('perc.hz', 'Perception', 'unknown'))
    monkeypatch.setattr(cli, 'run_check', lambda *a, **k: (found, 0))
    assert cli.main(['check', '--json', '-']) == 0
    capsys.readouterr()
    assert cli.main(['check', '--require', 'perception', '--require', 'System', '--json', '-']) == 3
    out = capsys.readouterr()
    assert json.loads(out.out)['required'] == {'perception': {'evaluated': 0, 'total': 1},
                                               'System': {'evaluated': 1, 'total': 1}}
    assert 'perception' in out.err
    assert cli.main(['check', '--require', 'Absent']) == 3
    assert cli.main(['check', '--require', 'system']) == 0
    monkeypatch.setattr(cli, 'run_check', lambda *a, **k: (found, 2))
    assert cli.main(['check', '--require', 'perception']) == 2

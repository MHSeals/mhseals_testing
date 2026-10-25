"""boat_trial analyze, index and check --require, plus run provenance (harness-gaps P7, P6). No ROS."""

import json

from mhseals_testing import data
from mhseals_testing.harness import cli, runs, session as session_module
from mhseals_testing.health.model import Fact
from mhseals_testing.history import History
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
    monkeypatch.setattr(cli, 'ros_collector', lambda *a: None)
    assert cli.main(['check', '--json', '-']) == 0
    capsys.readouterr()
    assert cli.main(['check', '--require', 'perception', '--require', 'System', '--json', '-']) == 3
    out = capsys.readouterr()
    assert json.loads(out.out)['required'] == {
        'perception': {'evaluated': 0, 'total': 1, 'not_ok': ['perc.hz']},
        'System': {'evaluated': 1, 'total': 1, 'not_ok': []}}
    assert 'perception' in out.err
    assert cli.main(['check', '--require', 'Absent']) == 3
    assert cli.main(['check', '--require', 'system']) == 0
    monkeypatch.setattr(cli, 'run_check', lambda *a, **k: (found, 2))
    assert cli.main(['check', '--require', 'perception']) == 2


def test_provenance_names_the_running_overlay_by_its_source_commit(tmp_path, monkeypatch):
    """M9: the code that runs (an overlay sync's .source_commit), not the workspace checkout."""
    import sys
    from types import ModuleType
    source = tmp_path / 'overlay' / 'src' / 'mhseals_nav'
    source.mkdir(parents=True)
    (source / 'package.xml').write_text('<package/>')
    (source / '.source_commit').write_text('c351f3f+dirty')
    share = tmp_path / 'install' / 'share' / 'mhseals_nav'
    share.mkdir(parents=True)
    (share / 'package.xml').symlink_to(source / 'package.xml')       # colcon --symlink-install

    def share_directory(name):
        if name != 'mhseals_nav':
            raise LookupError(name)
        return str(share)
    packages = ModuleType('ament_index_python.packages')
    packages.get_package_share_directory = share_directory
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    session_module.running_sources.cache_clear()
    try:
        found = session_module.provenance()['running']
    finally:
        session_module.running_sources.cache_clear()
    assert found == {'mhseals_nav': {'commit': 'c351f3f+dirty', 'source': str(source)}}


def test_runs_record_provenance(tmp_path, data_root):
    history = History()
    history.record('nav2', {'a': 1}, source='tui:test')
    history.record('hardware', {'b': 1}, source='tui:test')
    history.record('nav2', {'a': 2}, source='tui:test')
    facts = {'git.mhseals_nav': Fact({'hash': 'abc123', 'branch': 'main', 'dirty': 1, 'untracked': 0,
                                      'behind': 0}),
             'git.broken': Fact({'error': 'not a repo'}),
             'host.load': Fact(0.2),
             'host.squirtle-jetson.image_digest': Fact('sha256:ff'),
             'host.squirtle-jetson.git.mhseals_nav': Fact('def456')}
    found = session_module.provenance(facts, history, 'boat')
    assert found['git'] == {'mhseals_nav': {'hash': 'abc123', 'branch': 'main', 'dirty': 1, 'untracked': 0}}
    assert found['config'] == {'nav2': 'c0003', 'hardware': 'c0002'}
    assert found['camera'] == {'host.squirtle-jetson.image_digest': 'sha256:ff',
                               'host.squirtle-jetson.git.mhseals_nav': 'def456'}
    assert len(found['profile']['sha256']) == 64 and found['profile']['path'].endswith('boat.yaml')
    assert {k: v for k, v in session_module.provenance().items() if k != 'running'} == {'git': {}, 'camera': {}}
    record = runs.write_run(tmp_path, {'procedure': 'Square'}, provenance=found)
    assert data.read_json(tmp_path / f"{record['run_id']}.json")['provenance'] == found
    assert runs.write_run(tmp_path, {'procedure': 'Square'})['provenance'] == {}

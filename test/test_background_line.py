"""The agent panel's background line (D-20261101-31): open questions and proposals from mhseals_agents state."""

import json
from types import SimpleNamespace

from mhseals_testing.harness import background
from mhseals_testing.ui.panels import agent_log


def events(root, station, *lines):
    path = root / 'state' / 'events' / f'{station}-20261102.jsonl'
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as out:
        for line in lines:
            out.write(json.dumps(line) + '\n')


def test_open_questions_and_proposals_from_every_station_count_until_closed(tmp_path, monkeypatch):
    monkeypatch.setenv('MHSEALS_AGENTS', str(tmp_path))
    events(tmp_path, 'innovation', {'ts': '1', 'type': 'question', 'id': 'q1', 'title': 'which fence?'},
           {'ts': '2', 'type': 'proposal', 'id': 'p1', 'title': 'faster return', 'branch': 'bg/ret'},
           {'ts': '3', 'type': 'issue', 'id': 'i1', 'title': 'not counted'})
    events(tmp_path, 'innovation-bg', {'ts': '4', 'type': 'question', 'id': 'q2', 'title': 'second'},
           {'ts': '5', 'type': 'proposal', 'id': 'p1', 'verdict': 'pass'},
           {'ts': '6', 'type': 'question', 'id': 'q1', 'status': 'done'})
    found = background.read(background.root())
    assert [i['id'] for i in found['question']] == ['q2'] and found['proposal'][0]['verdict'] == 'pass'
    assert background.line(found) == 'background: 1 question · 1 proposal'
    assert background.line(background.read(tmp_path / 'none')) == ''


def test_the_root_comes_from_env_or_config_never_a_fixed_home(tmp_path, monkeypatch):
    events(tmp_path, 'x', {'ts': '1', 'type': 'question', 'id': 'q'})
    monkeypatch.setenv('MHSEALS_AGENTS', str(tmp_path / 'missing'))
    assert background.root(str(tmp_path)) is None              # a set env is the only place looked
    monkeypatch.delenv('MHSEALS_AGENTS')
    assert background.root(str(tmp_path)) == tmp_path


def test_the_reader_rereads_only_after_its_interval(tmp_path, monkeypatch):
    now = [0.0]
    reader = background.Reader(clock=lambda: now[0])
    monkeypatch.setenv('MHSEALS_AGENTS', str(tmp_path))
    events(tmp_path, 'x', {'ts': '1', 'type': 'question', 'id': 'q'})
    assert len(reader.get()['question']) == 1
    events(tmp_path, 'x', {'ts': '2', 'type': 'question', 'id': 'q', 'status': 'done'})
    assert len(reader.get()['question']) == 1
    now[0] += background.EVERY_S
    assert reader.get()['question'] == []


def test_the_panel_shows_one_quiet_line_on_top_and_enter_lists_them():
    found = {'question': [{'id': 'q2', 'station': 'bg', 'title': 'second'}],
             'proposal': [{'id': 'p1', 'station': 'bg', 'title': 'faster return', 'branch': 'bg/ret',
                           'verdict': 'pass'}]}
    desk = {'background': found, 'notes': [{'who': 'claude-1', 'tool_id': 't', 'cmd': 'ls', 'exit': 0,
                                            't_ns': 1}]}
    rows = agent_log.rows(desk)
    assert rows[0]['what'] == 'background: 1 question · 1 proposal' and len(rows) == 2
    shown = []
    app = SimpleNamespace(show=lambda title, **kw: shown.append((title, kw['message'])))
    agent_log.detail(app, desk, 0)
    assert shown[0][0] == rows[0]['what']
    assert shown[0][1].splitlines() == ['question q2 (bg): second', 'proposal p1 · bg/ret · sim pass (bg): faster return']
    assert agent_log.rows({'notes': desk['notes']})[0]['who'] == 'claude-1'     # nothing open: no line

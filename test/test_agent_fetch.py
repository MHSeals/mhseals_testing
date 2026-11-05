"""boat_agent fetch: the boat names the session (fetch --where), the station rsyncs it, throttled. No ssh or rsync."""

import io
import json
import subprocess

import pytest

from mhseals_testing import data
from mhseals_testing.agent import channel as ch, cli as agent_cli, relay
from mhseals_testing.harness import cli

T = 'blastoise-odroid'


class Proc:
    def __init__(self, out, code=0):
        self.stdout, self.stderr, self.code = io.StringIO(out), io.StringIO(''), code

    def wait(self):
        return self.code


class Reg:
    def __init__(self, session=None, pid=None):
        self.session, self.pid = session, pid

    def owner(self):
        return {'pid': self.pid} if self.pid else None

    def alive(self, pid):
        return True

    def state(self):
        return {'session': str(self.session) if self.session else None}


def where(argv, registry):
    out = io.StringIO()
    code = agent_cli.main(cli.build_parser().parse_args(argv), registry=registry, out=out,
                          settings={'request_s': 5.0, 'fetch': {'bwlimit_kbps': 1500}})
    return code, json.loads(out.getvalue().splitlines()[-1])


def test_the_boat_half_names_the_session_its_open_segment_and_logs_why(data_root, monkeypatch):
    monkeypatch.setenv('ROS_WS', str(data_root))
    session = data_root / 'sessions' / '20261104_101500_pool'
    for seg in ('00', '01'):
        (session / 'bag' / seg).mkdir(parents=True)
    code, doc = where(['fetch', '--who', 'a', '--why', 'check sweep 2', '--where'], Reg(session, pid=7))
    assert code == 0 and doc['rel'] == 'sessions/20261104_101500_pool' and doc['open_segments'] == ['01']
    assert doc['bwlimit_kbps'] == 1500 and doc['live'] is True
    assert [n['cmd'] for n in data.read_jsonl(ch.channel_path()) if n['kind'] == 'note'] == \
        ['fetch 20261104_101500_pool to the station: check sweep 2']
    code, doc = where(['fetch', '--who', 'a', '--why', 'w', '--where', '--bwlimit', '99'], Reg(session))
    assert doc['open_segments'] == [] and doc['bwlimit_kbps'] == 99          # not the TUI's live session
    assert where(['fetch', '../x', '--who', 'a', '--why', 'w', '--where'], Reg())[0] == 2
    assert where(['fetch', '--who', 'a', '--why', 'w'], Reg())[1]['error'] == 'bad_args'   # not relayed


def test_the_station_pulls_with_rsync_throttled_and_without_the_open_segment(data_root):
    sent, pulled = [], []

    def popen(argv, **kw):
        sent.append(argv[-1])
        return Proc(json.dumps({'ok': True, 'id': 's1', 'rel': 'bags/sessions/s1', 'open_segments': ['03'],
                                'live': True, 'bwlimit_kbps': 1500}) + '\n')

    def rsync(argv, **kw):
        pulled.append(argv)
        return subprocess.CompletedProcess(argv, 0, '', '')
    out = io.StringIO()
    assert relay.fetch(T, ['fetch', 's1', '--who', 'a', '--why', 'qc'], out=out, popen=popen, rsync=rsync) == 0
    assert sent[0].endswith("fetch s1 --who a --why qc --where")
    argv = pulled[0]
    assert '--bwlimit=1500' in argv and '--exclude=/bag/03/' in argv and '--partial-dir=.rsync-partial' in argv
    assert argv[-2] == f'{T}:astro_dock/bags/sessions/s1/' and argv[-1] == f"{data_root / 'sessions' / 's1'}/"
    doc = json.loads(out.getvalue())
    assert doc['ok'] and 'still recording' in doc['hint']


def test_a_live_session_names_only_its_open_split_so_closed_splits_come_along(data_root, monkeypatch):
    """11-04 pool: the boat records one segment per session, so excluding the whole open segment left the
    station with no bag at all while the session recorded (the 01:35 surge+ hold could not be checked)."""
    monkeypatch.setenv('ROS_WS', str(data_root))
    session = data_root / 'sessions' / '20261105_011850_pool'
    (session / 'bag' / '00').mkdir(parents=True)
    for n in (0, 1, 2, 10):
        (session / 'bag' / '00' / f'00_{n}.mcap').write_bytes(b'')
    code, doc = where(['fetch', '--who', 'a', '--why', 'w', '--where'], Reg(session, pid=7))
    assert code == 0 and doc['open_files'] == ['bag/00/00_10.mcap']
    assert where(['fetch', '--who', 'a', '--why', 'w', '--where'], Reg(session))[1]['open_files'] == []
    pulled = []

    def rsync(argv, **kw):
        pulled.append(argv)
        return subprocess.CompletedProcess(argv, 0, '', '')
    plan = dict(doc, rel='bags/sessions/s1', id='s1')
    out = io.StringIO()
    assert relay.fetch(T, ['fetch', '--who', 'a', '--why', 'qc'], out=out, rsync=rsync,
                       popen=lambda argv, **kw: Proc(json.dumps(plan) + '\n')) == 0
    assert '--exclude=/bag/00/00_10.mcap' in pulled[0] and '--exclude=/bag/00/' not in pulled[0]
    assert json.loads(out.getvalue())['skipped'] == ['bag/00/00_10.mcap']


@pytest.mark.parametrize('plan', [{'ok': True, 'id': 's1', 'rel': '../../etc', 'bwlimit_kbps': 1},
                                  {'ok': True, 'id': '..', 'rel': 'bags/x', 'bwlimit_kbps': 1},
                                  {'ok': False, 'error': 'no_session'}])
def test_the_station_refuses_an_unsafe_or_failed_plan_and_pulls_nothing(data_root, plan):
    pulled = []
    code = relay.fetch(T, ['fetch', '--who', 'a', '--why', 'qc'], out=io.StringIO(),
                       popen=lambda argv, **kw: Proc(json.dumps(plan) + '\n'), rsync=lambda *a, **k: pulled.append(a))
    assert code == 2 and pulled == []


def test_the_station_needs_a_why(data_root):
    out = io.StringIO()
    assert relay.fetch(T, ['fetch'], out=out, popen=None, rsync=None) == 2
    assert json.loads(out.getvalue())['error'] == 'bad_args'


pytestmark = pytest.mark.smoke   # tools/smoke

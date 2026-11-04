"""boat_agent quality (harness/quality.py): rates, gaps and commanded vs measured coverage, no ROS (a fake reader)."""

import io
import json

import pytest

from mhseals_testing import data
from mhseals_testing.harness import cli, quality
from mhseals_testing.agent import cli as agent_cli
from mhseals_testing.profiles import load_profile

S = 10 ** 9


def session(root, profile='crane_full_stack'):
    path = root / 'sessions' / '20261104_100000_pool'
    (path / 'bag' / '00').mkdir(parents=True)
    (path / 'bag' / '00' / 'metadata.yaml').write_text('rosbag2_bagfile_information: {}\n')
    data.write_json(path / 'session.json', {'id': path.name, 'profile': profile}, 'mhseals.session/1')
    return path


def reader_of(messages):
    def read(path, topics=None, on_segment=None, **_):
        on_segment(path, None)
        for topic, t_ns in sorted(messages, key=lambda m: m[1]):
            if topics is None or topic in topics:
                yield topic, '', b'', t_ns
    return read


def test_rates_gaps_and_coverage_from_the_profile_topics(data_root):
    path = session(data_root)
    profile = load_profile('crane_full_stack')
    cmd, odom = profile.command_topic, profile.odometry
    messages = [(cmd, int(t * S / 10)) for t in range(0, 100)]            # 10 s commanded at 10 Hz
    messages += [(odom, int(t * S / 20)) for t in range(0, 100)]          # odometry 0-5 s, then silent
    messages += [(odom, 10 * S)]
    doc = quality.summarize(path, reader=reader_of(messages))
    assert doc['topics'][cmd]['role'] == 'command' and abs(doc['topics'][cmd]['rate_hz'] - 10) < 0.2
    assert doc['topics'][odom]['max_gap_s'] > 4.9 and 'gaps' in doc['topics'][odom]['flags']
    assert 0.45 < doc['coverage']['measured_share'] < 0.55 and doc['coverage']['commanded_s'] > 9.8
    assert doc['verdict'] == 'check' and odom in doc['flagged']
    last = quality.summarize(path, last_s=2.0, reader=reader_of(messages))
    assert last['window']['duration_s'] == 2.0 and last['topics'][cmd]['count'] <= 21


def test_a_session_still_recording_reads_its_closed_segments_or_refuses(data_root):
    path = session(data_root)
    (path / 'bag' / '01').mkdir()                                   # recording: no metadata.yaml yet
    profile = load_profile('crane_full_stack')
    seen = []

    def read(segment, **kw):
        seen.append(segment.name)
        yield from reader_of([(profile.command_topic, S), (profile.command_topic, 2 * S)])(segment, **kw)
    doc = quality.summarize(path, reader=read)
    assert doc['ok'] and seen == ['00'] and doc['recording'] == ['01'] and 'still recording' in doc['warning']
    assert doc['verdict'] == 'check'
    (path / 'bag' / '00' / 'metadata.yaml').unlink()                # only the live segment
    doc = quality.summarize(path, reader=read)
    assert doc['ok'] is False and doc['error'] == 'recording' and 'bag still recording' in doc['hint']


def test_the_verb_reads_a_session_and_refuses_an_escape(data_root, monkeypatch):
    path = session(data_root)

    def run(argv):
        out = io.StringIO()
        code = agent_cli.main(cli.build_parser().parse_args(argv), out=out, settings={'request_s': 5.0})
        return code, json.loads(out.getvalue().splitlines()[-1])

    monkeypatch.setattr(quality.bagread, 'read_raw', reader_of([]))
    code, doc = run(['quality', '--who', 'a', str(path)])
    assert code == 2 and doc['ok'] is False and doc['error'] == 'no_messages'      # nothing read is never ok
    code, doc = run(['quality', '--who', 'a', '../../etc'])
    assert code == 2 and doc['error'] in ('bad_args', 'no_session')


pytestmark = pytest.mark.smoke   # tools/smoke

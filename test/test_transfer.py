"""Transfer worker: parser fixtures for rsync 3.2 and 3.5, the resume formula, and a local rsync."""
import json
import shutil
import subprocess

import pytest

from mhseals_testing.harness import transfer as tr

P32 = '  1,048,576   2%    1.00MB/s    0:00:40 (xfr#1, to-chk=3/5)\r  2,097,152   5%  500.00kB/s    0:01:01\r'
P35 = '      1,048,576B   2%    1.00MB/s    0:00:40 (xfr#1, to-chk=3/5)\r'
ITEM = 'cd+++++++++ 4096\n>f+++++++++ 1000\n>f.st...... 24\n*deleting   old\n'


def test_parse_progress_both_formats_split_on_cr():
    a = tr.parse_progress(P32)
    assert [p['bytes'] for p in a] == [1048576, 2097152]
    assert a[1]['rate'] == 500e3 and a[1]['eta_s'] == 61 and a[0]['pct'] == 2
    assert tr.parse_progress(P35)[0]['bytes'] == 1048576
    assert tr.parse_progress('sending incremental file list\n') == []


def test_parse_itemize_skips_directories():
    assert tr.parse_itemize(ITEM) == [('>f+++++++++', 1000), ('>f.st......', 24)]


def test_fraction_corrects_resume():
    assert tr.fraction(100, 40, 0) == 0.6      # 60 bytes already there
    assert tr.fraction(100, 40, 20) == 0.8
    assert tr.fraction(0, 0, 0) == 1.0 and tr.fraction(100, 40, 999) == 1.0


def test_destination():
    t = {'host': 'h.local', 'user': 'u', 'path': 'p'}
    assert tr.destination(t, 'id') == 'u@h.local:p/id/'
    assert tr.destination({'host': '', 'path': '/x'}, 'id') == '/x/id/'


def make_session(root):
    """A session as data_layout has it: reports at the top, closed bag/00, open bag/01."""
    s = root / 'boat_1'
    (s / 'bag' / '00').mkdir(parents=True)
    (s / 'bag' / '01').mkdir()
    (s / 'session.json').write_text('hi')
    (s / 'bag' / '00' / '00_0.mcap').write_bytes(b'x' * 5000)
    (s / 'bag' / '01' / '01_0.mcap').write_bytes(b'y' * 100)
    return s


def test_command_passes_filters_and_open_segment(tmp_path):
    job = tr.Transfer(make_session(tmp_path), {'host': 'h', 'user': 'u', 'path': 'p'}, ['01'], 500)
    argv = job.command(tr.PASSES['reports'])
    assert '--exclude=/bag/01/' in argv and '--exclude=/bag/' in argv
    assert '--bwlimit=500' in argv and tr.SSH in argv
    assert argv[-1] == 'u@h:p/boat_1/'


def test_total_splits_passes_and_skips_open_segment(tmp_path):
    job = tr.Transfer(make_session(tmp_path), {'host': '', 'path': str(tmp_path / 'out')}, ['01'])
    assert job.total('reports') == 2 and job.total('bags') == 5000 and job.total() == 5002


@pytest.mark.skipif(not shutil.which('rsync'), reason='rsync not installed')
def test_local_send_completes_and_skips_open_segment(tmp_path):
    s = make_session(tmp_path)
    dest = tmp_path / 'out'
    seen = []
    job = tr.Transfer(s, {'host': '', 'path': str(dest)}, ['01'])
    assert job.run(lambda rec, prog: seen.append(rec['bytes'])) == 'complete'
    assert (dest / 'boat_1' / 'bag' / '00' / '00_0.mcap').stat().st_size == 5000
    assert not (dest / 'boat_1' / 'bag' / '01').exists()
    rec = json.loads((s / 'transfer.json').read_text())
    assert rec['status'] == 'complete' and rec['bytes'] == rec['total'] == 5002 and rec['finished']
    assert not (dest / 'boat_1' / 'transfer.json').exists()


@pytest.mark.skipif(not shutil.which('rsync'), reason='rsync not installed')
def test_size_mismatch_is_not_complete(tmp_path):
    s = make_session(tmp_path)
    job = tr.Transfer(s, {'host': '', 'path': str(tmp_path / 'out')}, ['01'])
    assert job.run() == 'complete'
    (tmp_path / 'out' / 'boat_1' / 'bag' / '00' / '00_0.mcap').write_bytes(b'z')
    assert job.pending()          # the check sees the damaged remote file
    job2 = tr.Transfer(s, {'host': '', 'path': str(tmp_path / 'out')}, ['01'])
    assert job2.run() == 'complete'   # a retry repairs it
    assert (tmp_path / 'out' / 'boat_1' / 'bag' / '00' / '00_0.mcap').stat().st_size == 5000


@pytest.mark.skipif(not shutil.which('rsync'), reason='rsync not installed')
def test_cancel_before_start_marks_cancelled(tmp_path):
    s = make_session(tmp_path)
    job = tr.Transfer(s, {'host': '', 'path': str(tmp_path / 'out')})
    job.cancel()
    assert job.run() == 'cancelled'
    assert json.loads((s / 'transfer.json').read_text())['status'] == 'cancelled'


@pytest.mark.skipif(not shutil.which('rsync'), reason='rsync not installed')
def test_failure_records_error(tmp_path):
    s = make_session(tmp_path)
    job = tr.Transfer(s, {'host': '', 'path': '/proc/nope/x'})
    assert job.run() == 'failed'
    assert json.loads((s / 'transfer.json').read_text())['error']


@pytest.mark.skipif(not shutil.which('rsync'), reason='rsync not installed')
def test_pass_one_sends_no_bag_files(tmp_path):
    s = make_session(tmp_path)
    job = tr.Transfer(s, {'host': '', 'path': str(tmp_path / 'out')}, ['01'])
    left = job.pending(tr.PASSES['reports'])
    assert [size for _, size in left] == [2]
    subprocess.run(job.command(tr.PASSES['reports']), check=True, capture_output=True)
    assert (tmp_path / 'out' / 'boat_1' / 'session.json').exists()
    assert not (tmp_path / 'out' / 'boat_1' / 'bag').exists()
    assert [size for _, size in job.pending(tr.PASSES['bags'])] == [5000]


def make_flat(root):
    """A live session: <name>/NN/ segments with sibling report, journal, session file and logs."""
    s = root / 'crane_1'
    for seg, n in (('00', 5000), ('01', 100)):
        (s / seg).mkdir(parents=True)
        (s / seg / f'{seg}_0.mcap').write_bytes(b'x' * n)
    (root / 'logs' / 'crane_1').mkdir(parents=True)
    (root / 'logs' / 'crane_1' / 'a.log').write_text('l')
    for ext in ('report.json', 'events.jsonl', 'session.yaml', 'hardware.yaml'):
        (root / f'crane_1.{ext}').write_text('ab')
    (root / 'crane_1.runs').mkdir()
    (root / 'crane_1.runs' / 'r1.json').write_text('{}')
    return s


def test_flat_totals_split_siblings_reports_bags(tmp_path):
    job = tr.Transfer(make_flat(tmp_path), {'host': '', 'path': str(tmp_path / 'out')}, ['01'])
    assert job.legacy
    assert job.total('siblings') == 10 and job.total('logs') == 1
    assert job.total('reports') == 0 and job.total('bags') == 5000
    assert job.total() == 5011
    assert '--exclude=/01/' in job.command(tr.LEGACY_PASSES['reports'])


@pytest.mark.skipif(not shutil.which('rsync'), reason='rsync not installed')
def test_flat_send_includes_siblings_and_skips_open_segment(tmp_path):
    s = make_flat(tmp_path)
    out = tmp_path / 'out'
    job = tr.Transfer(s, {'host': '', 'path': str(out)}, ['01'])
    assert job.run() == 'complete'
    for ext in ('report.json', 'events.jsonl', 'session.yaml', 'hardware.yaml'):
        assert (out / f'crane_1.{ext}').read_text() == 'ab'
    assert (out / 'crane_1.runs' / 'r1.json').exists()
    assert (out / 'crane_1' / '00' / '00_0.mcap').stat().st_size == 5000
    assert (out / 'logs' / 'crane_1' / 'a.log').read_text() == 'l'
    assert not (out / 'crane_1' / '01').exists()
    assert not job.pending()


@pytest.mark.skipif(not shutil.which('rsync'), reason='rsync not installed')
def test_flat_without_logs_or_siblings_completes(tmp_path):
    s = tmp_path / 'crane_2'
    (s / '00').mkdir(parents=True)
    (s / '00' / 'a.mcap').write_bytes(b'x' * 100)
    out = tmp_path / 'out'
    job = tr.Transfer(s, {'host': '', 'path': str(out)})
    assert job.run() == 'complete', job.record['error']
    assert (out / 'crane_2' / '00' / 'a.mcap').stat().st_size == 100
    assert not job.pending()

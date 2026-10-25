import multiprocessing

import pytest

from mhseals_testing.history import History


def _append(root, n):
    History(root).record('hardware', {'x': n}, source='test')


def test_record_dedupes_and_orders_ids(tmp_path):
    h = History(tmp_path)
    first = h.record('hardware', {'a': 1.0}, 'a: 1\n', source='tui:pins')
    assert first['id'] == 'c0001' and first['before'] is None
    assert h.record('hardware', {'a': 1.0000000000001}, source='tui:pins') is None    # equal at 9 digits
    second = h.record('hardware', {'a': 2}, source='tui:pins')
    assert (second['id'], second['before']) == ('c0002', first['after'])
    assert h.version(first)['raw'] == 'a: 1\n'
    assert h.observe('hardware', {'a': 3})['applied'] == 'observed'


def test_torn_last_line_is_skipped(tmp_path):
    h = History(tmp_path)
    h.record('nav2', {'v': 1}, source='t')
    with open(h.log, 'a') as f:
        f.write('{"schema": "mhseals.config_ch')
    assert [c['id'] for c in h.changes()] == ['c0001']
    assert h.record('nav2', {'v': 2}, source='t')['id'] == 'c0002'


def test_two_processes_get_distinct_ids(tmp_path):
    # spawn, not fork: forking the suite's multi-threaded process can deadlock a child on a
    # lock another thread held, and an unbounded join then hangs the whole run.
    spawn = multiprocessing.get_context('spawn')
    procs = [spawn.Process(target=_append, args=(tmp_path, n)) for n in range(6)]
    [p.start() for p in procs]
    [p.join(timeout=60) for p in procs]
    assert all(p.exitcode == 0 for p in procs)
    ids = [c['id'] for c in History(tmp_path).changes()]
    assert sorted(ids) == [f'c{i:04d}' for i in range(1, 7)]


def test_labels_fold_and_newest_name_wins(tmp_path):
    h = History(tmp_path)
    a = h.record('hardware', {'a': 1}, source='t')
    b = h.record('hardware', {'a': 2}, source='t')
    h.label(a['id'], name='good')
    h.label(b['id'], name='good', note='later')
    assert h.resolve('good')['id'] == b['id']
    assert h.resolve('current')['note'] == 'later'
    assert h.resolve('previous')['id'] == a['id']
    with pytest.raises(KeyError):
        h.resolve('missing')

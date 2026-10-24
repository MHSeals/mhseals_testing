"""harness/watch.py: settle before parsing, keep the last good value, one report per error."""

import os

import yaml

from mhseals_testing.harness.watch import Watched, Watcher


def make(tmp_path, text='a: 1\n'):
    path = tmp_path / 'x.yaml'
    path.write_text(text)
    applied = []
    watcher = Watcher(clock=lambda: 5.0)
    watcher.add(Watched('x', path, lambda p: yaml.safe_load(p.read_text()), applied.append))
    return watcher, path, applied


def test_a_half_written_file_is_not_parsed(tmp_path):
    watcher, path, applied = make(tmp_path)
    with open(path, 'w') as handle:
        handle.write('a: [1,')
        handle.flush()
        assert watcher.poll() == []  # first sight of the new stat key: wait
        handle.write(' 2]\n')
        handle.flush()
        assert watcher.poll() == []  # it moved again: still wait
    changes = watcher.poll()
    assert [c.ok for c in changes] == [True] and applied == [{'a': [1, 2]}]


def test_bad_yaml_keeps_the_old_value_and_reports_once(tmp_path):
    watcher, path, applied = make(tmp_path)
    path.write_text('a: [1\n')
    watcher.poll()
    first = watcher.poll()
    assert first[0].ok is False and first[0].report and 'line' in first[0].error
    path.write_text('a:  [1\n')
    os.utime(path, ns=(1, 1))
    watcher.poll()
    again = watcher.poll()
    assert again[0].ok is False and not again[0].report
    assert applied == [] and watcher.rows()[0][1] is False
    path.write_text('a: 2\n')
    watcher.poll()
    assert watcher.poll()[0].ok and applied == [{'a': 2}] and watcher.rows()[0][1]


def test_reload_skips_the_settle_wait(tmp_path):
    watcher, path, applied = make(tmp_path)
    path.write_text('a: 3\n')
    assert watcher.reload('x').ok and applied == [{'a': 3}]
    assert watcher.poll() == []


def test_a_file_created_later_waits_one_settled_poll(tmp_path):
    path = tmp_path / 'late.yaml'
    applied = []
    watcher = Watcher(clock=lambda: 5.0)
    watcher.add(Watched('late', path, lambda p: yaml.safe_load(p.read_text()), applied.append))
    path.write_text('a: 1\n')
    assert watcher.poll() == [] and applied == []  # first sight: maybe half-written
    assert [c.ok for c in watcher.poll()] == [True] and applied == [{'a': 1}]


def test_callback_failures_are_not_successful_reloads(tmp_path):
    from types import SimpleNamespace
    watcher, path, applied = make(tmp_path)
    watched = watcher.watches['x']
    stamp = watched.loaded_at
    for result in (False, SimpleNamespace(ok=False, error='rejected'),
                   {'ok': False, 'error': 'rejected'}, {'status': 'failed'}):
        watched.apply = lambda value: result
        first = watcher.reload('x')
        assert not first.ok and watched.loaded_at == stamp
        assert not watcher.reload('x').report
    def fail(value):
        raise RuntimeError('transport failed')
    watched.apply = fail
    assert watcher.reload('x').error == 'transport failed'
    watched.apply = applied.append
    assert watcher.reload('x').ok and applied == [{'a': 1}]

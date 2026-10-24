"""Public Nav2File outcomes require explicit complete consumer evidence."""

import pytest

from mhseals_testing.harness.nav2_edit import Nav2File


def surface(tmp_path, applier=None):
    path = tmp_path / 'nav2.yaml'
    path.write_text('node:\n  value: 1\n')
    return Nav2File(path, applier)


def test_initial_file_has_no_consumer_evidence(tmp_path):
    nav = surface(tmp_path)
    assert nav.pending and nav.row()['value'] == 'pending'
    assert 'not wired' in nav.apply()
    assert nav.pending


@pytest.mark.parametrize('receipt', [{}, {'applied': {'node.value': 1}},
                         {'status': 'verified', 'verified': {}},
                         {'status': 'verified', 'verified': {'node.value': 1}, 'error': 'failed'},
                         {'status': 'partial'}, True])
def test_incomplete_receipt_keeps_pending(tmp_path, receipt):
    nav = surface(tmp_path, lambda changes: receipt)
    assert 'unverified' in nav.apply()
    assert nav.pending and nav.applied == {}


def test_verified_receipt_and_removal_pending(tmp_path):
    calls = []
    def apply(changes):
        calls.append(changes)
        return dict(status='verified', verified=changes)
    nav = surface(tmp_path, apply)
    assert 'applied 1' in nav.apply()
    assert not nav.pending and nav.row()['value'] == 'applied'
    nav.path.write_text('{}\n')
    nav.poll()
    nav.poll()
    assert nav.pending and nav.row()['value'] == 'pending'
    assert 'removals pending' in nav.apply()
    assert len(calls) == 1


def test_unreadable_and_transport_error_keep_pending(tmp_path):
    def fail(changes):
        raise RuntimeError('transport failed')
    nav = surface(tmp_path, fail)
    assert 'failed' in nav.apply() and nav.pending
    nav.path.unlink()
    nav.poll()
    nav.poll()
    assert nav.row()['value'] == 'error'
    assert 'unavailable' in nav.apply() and nav.pending

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


@pytest.mark.parametrize('desired,evidence', [
    (1, True), ([1, {'flag': 2}], [True, {'flag': 2}]),
    ([{'flag': 1}], [{'flag': True}]), ([{1: 'a'}], [{True: 'a'}]), (1, [('node.value', 1)]),
])
def test_receipt_requires_typed_values_and_mapping(tmp_path, desired, evidence):
    nav = surface(tmp_path, lambda changes: dict(status='verified', verified=(
        evidence if isinstance(evidence, list) and desired == 1 else {'node.value': evidence})))
    nav.current = {'node': {'value': desired}}
    assert 'unverified' in nav.apply()
    assert nav.pending


def test_bool_edit_cannot_reuse_numeric_receipt(tmp_path):
    nav = surface(tmp_path, lambda changes: dict(status='verified', verified=changes))
    nav.apply()
    nav.path.write_text('node:\n  value: true\n')
    nav.poll()
    nav.poll()
    assert nav.pending and nav.changes() == [('node.value', 1, True)]


@pytest.mark.parametrize('failure', ['partial', 'error', 'unverified', 'raise', 'invalid', 'refused'])
def test_failed_attempt_requires_full_recertification_on_return(tmp_path, failure):
    runtime, calls = {}, []
    def apply(changes):
        calls.append(dict(changes))
        runtime.update(changes)
        if len(calls) == 2:
            if failure == 'raise':
                raise RuntimeError('after mutation')
            if failure == 'invalid':
                return True
            if failure == 'refused':
                return {'refused': 'after mutation'}
            return {'status': failure}
        return dict(status='verified', verified=dict(runtime))
    nav = surface(tmp_path, apply)
    nav.apply()
    assert nav.row()['value'] == 'applied'
    nav.path.write_text('node:\n  value: 2\n')
    nav.poll()
    nav.poll()
    nav.apply()
    assert nav.pending and nav.row()['value'] == 'pending'
    assert runtime == {'node.value': 2}
    nav.path.write_text('node:\n  value: 1\n')
    nav.poll()
    nav.poll()
    assert nav.pending and nav.row()['value'] == 'pending'
    nav.apply()
    assert calls[-1] == {'node.value': 1}
    assert runtime == {'node.value': 1}
    assert not nav.pending and nav.row()['value'] == 'applied'

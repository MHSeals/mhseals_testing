from dataclasses import FrozenInstanceError, replace

import pytest
from mhseals_config import Artifact, BaselineRevision, SessionSelection
from mhseals_testing._configuration_records import (
    EffectiveRevision, ParticipantPlan, ParticipantReceipt, PersistentRevision,
    Plan, Receipt, RequestedRevision, Status, VerifiedRevision,
)


def facts():
    scope = SessionSelection('session', 'baseline', 'overlay')
    artifact = Artifact('snapshot', 'a' * 64)
    requested = RequestedRevision('candidate', scope, (artifact,))
    plan = Plan('plan', requested, (ParticipantPlan('node', 'boot', 'consume', 'v1'),), 10, 30)
    effective = EffectiveRevision('candidate', scope, 'boot', 1, 11)
    verified = VerifiedRevision(effective, 'consume', 'v1', (artifact,), 12)
    row = ParticipantReceipt('node', True, effective, verified, 'observed')
    persistent = PersistentRevision(BaselineRevision('baseline', (artifact,)), 'candidate', scope, artifact)
    return plan, row, persistent


def test_saved_request_and_accepted_are_pending():
    plan, row, _ = facts()
    for row in (ParticipantReceipt('node'), ParticipantReceipt('node', True, detail='accepted')):
        receipt = Receipt(plan, Status.QUEUED, (row,), 13)
        assert receipt.applied == 'pending'
        with pytest.raises(TypeError):
            bool(receipt)
        with pytest.raises(ValueError):
            replace(receipt, status=Status.COMMITTED)


def test_running_verified_and_persistent_are_separate():
    plan, row, persistent = facts()
    receipt = Receipt(plan, Status.VERIFIED_UNPERSISTED, (row,), 13)
    assert receipt.persistent is None and receipt.applied == 'pending'
    committed = replace(receipt, status=Status.COMMITTED, persistent=persistent)
    assert committed.applied == 'confirmed'
    with pytest.raises(FrozenInstanceError):
        committed.status = Status.QUEUED


def test_missing_partial_and_duplicate_rows():
    plan, row, _ = facts()
    plan = replace(plan, participants=plan.participants + (ParticipantPlan('other', 'boot2', 'consume', 'v1'),))
    with pytest.raises(ValueError):
        Receipt(plan, Status.APPLYING, (row,), 13)
    rows = (ParticipantReceipt('other'), row)
    receipt = Receipt(plan, Status.RECOVERY_REQUIRED, rows, 13, detail='partial')
    assert receipt.participants[0].detail == 'missing'
    with pytest.raises(ValueError):
        replace(receipt, status=Status.VERIFIED_UNPERSISTED)
    with pytest.raises(ValueError):
        replace(receipt, participants=(row, row))


@pytest.mark.parametrize('changes', [
    {'candidate': 'other'}, {'identity': 'restarted'}, {'observed_at': 9},
    {'scope': SessionSelection('different', 'baseline', 'overlay')},
])
def test_exact_runtime_association(changes):
    plan, row, _ = facts()
    effective = replace(row.effective, **changes)
    verified = replace(row.verified, effective=effective)
    with pytest.raises(ValueError):
        Receipt(plan, Status.VERIFIED_UNPERSISTED, (replace(row, effective=effective, verified=verified),), 13)


@pytest.mark.parametrize('changes', [{'recipe': 'other'}, {'recipe_version': 'v2'}, {'observed_at': 14}])
def test_recipe_and_observation_association(changes):
    plan, row, _ = facts()
    with pytest.raises(ValueError):
        Receipt(plan, Status.COMMITTED, (replace(row, verified=replace(row.verified, **changes)),), 13)


def test_stale_and_numeric_traps_and_mutable_references():
    plan, row, _ = facts()
    with pytest.raises(ValueError):
        Receipt(plan, Status.VERIFIED_UNPERSISTED, (row,), 31)
    for value in (True, float('nan'), float('inf'), -1):
        with pytest.raises(ValueError):
            replace(row.effective, observed_at=value)
    with pytest.raises(ValueError):
        replace(row.effective, generation=True)
    with pytest.raises(ValueError):
        replace(row, accepted=1)
    with pytest.raises(ValueError):
        replace(plan, participants=list(plan.participants))
    with pytest.raises(ValueError):
        replace(plan.requested, sources=list(plan.requested.sources))
    with pytest.raises(ValueError):
        replace(row.verified, evidence=[])


def test_core_scope_cannot_leak_mutable_optional_fields():
    plan, row, _ = facts()
    scope = replace(plan.requested.scope, profile={})
    with pytest.raises(ValueError):
        replace(plan.requested, scope=scope)
    with pytest.raises(ValueError):
        replace(row.effective, scope=scope)

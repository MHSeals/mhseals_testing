"""Pure transaction facts; constructing records never performs runtime effects."""
from dataclasses import dataclass
from enum import Enum
import math

from mhseals_config import Artifact, BaselineRevision, SessionSelection


def _text(*values):
    if any(type(v) is not str or not v for v in values):
        raise ValueError('nonempty identity required')


def _time(value):
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
        raise ValueError('invalid observation time')


class Status(str, Enum):
    PROPOSED = 'proposed'
    BLOCKED = 'blocked'
    STALE = 'stale'
    QUEUED = 'queued'
    APPLYING = 'applying'
    EFFECTIVE_UNVERIFIED = 'effective_unverified'
    VERIFIED_UNPERSISTED = 'verified_unpersisted'
    COMMITTED = 'committed'
    COMPENSATING = 'compensating'
    ROLLED_BACK_VERIFIED = 'rolled_back_verified'
    ROLLBACK_FAILED = 'rollback_failed'
    RECOVERY_REQUIRED = 'recovery_required'


@dataclass(frozen=True)
class RequestedRevision:
    candidate: str
    scope: SessionSelection
    sources: tuple[Artifact, ...]

    def __post_init__(self):
        _text(self.candidate)
        if type(self.scope) is not SessionSelection or type(self.sources) is not tuple or not all(
                type(a) is Artifact for a in self.sources):
            raise ValueError('immutable scoped source references required')


@dataclass(frozen=True)
class ParticipantPlan:
    name: str
    identity: str
    recipe: str
    recipe_version: str

    def __post_init__(self):
        _text(self.name, self.identity, self.recipe, self.recipe_version)


@dataclass(frozen=True)
class Plan:
    id: str
    requested: RequestedRevision
    participants: tuple[ParticipantPlan, ...]
    created_at: float
    expires_at: float

    def __post_init__(self):
        _text(self.id)
        _time(self.created_at)
        _time(self.expires_at)
        if (type(self.requested) is not RequestedRevision or type(self.participants) is not tuple
                or not self.participants or not all(type(p) is ParticipantPlan for p in self.participants)
                or len({p.name for p in self.participants}) != len(self.participants)
                or self.expires_at <= self.created_at):
            raise ValueError('invalid immutable plan')


@dataclass(frozen=True)
class EffectiveRevision:
    candidate: str
    scope: SessionSelection
    identity: str
    generation: int
    observed_at: float

    def __post_init__(self):
        _text(self.candidate, self.identity)
        _time(self.observed_at)
        if type(self.scope) is not SessionSelection or type(self.generation) is not int or self.generation < 0:
            raise ValueError('invalid scoped runtime generation')


@dataclass(frozen=True)
class VerifiedRevision:
    effective: EffectiveRevision
    recipe: str
    recipe_version: str
    evidence: tuple[Artifact, ...]
    observed_at: float

    def __post_init__(self):
        _text(self.recipe, self.recipe_version)
        _time(self.observed_at)
        if (type(self.effective) is not EffectiveRevision or type(self.evidence) is not tuple
                or not self.evidence or not all(type(a) is Artifact for a in self.evidence)
                or self.observed_at < self.effective.observed_at):
            raise ValueError('independent evidence required')


@dataclass(frozen=True)
class ParticipantReceipt:
    name: str
    accepted: bool = False
    effective: EffectiveRevision | None = None
    verified: VerifiedRevision | None = None
    detail: str = 'missing'

    def __post_init__(self):
        _text(self.name, self.detail)
        if (type(self.accepted) is not bool
                or self.effective is not None and type(self.effective) is not EffectiveRevision
                or self.verified is not None and (type(self.verified) is not VerifiedRevision
                                                 or self.verified.effective != self.effective)):
            raise ValueError('invalid participant facts')


@dataclass(frozen=True)
class PersistentRevision:
    baseline: BaselineRevision
    candidate: str
    scope: SessionSelection
    manifest: Artifact

    def __post_init__(self):
        _text(self.candidate)
        if (type(self.baseline) is not BaselineRevision or type(self.scope) is not SessionSelection
                or type(self.manifest) is not Artifact or self.scope.baseline != self.baseline.revision):
            raise ValueError('persistent baseline association required')


@dataclass(frozen=True)
class Receipt:
    plan: Plan
    status: Status
    participants: tuple[ParticipantReceipt, ...]
    observed_at: float
    persistent: PersistentRevision | None = None
    detail: str = ''

    def __post_init__(self):
        _time(self.observed_at)
        if (type(self.plan) is not Plan or type(self.status) is not Status
                or type(self.participants) is not tuple
                or not all(type(p) is ParticipantReceipt for p in self.participants)):
            raise ValueError('invalid immutable receipt')
        expected = {p.name: p for p in self.plan.participants}
        rows = {p.name: p for p in self.participants}
        if len(rows) != len(self.participants) or set(rows) != set(expected):
            raise ValueError('every participant requires an explicit row, including missing participants')
        requested = self.plan.requested
        fresh = self.plan.created_at <= self.observed_at <= self.plan.expires_at
        complete = fresh
        for row in self.participants:
            target = expected[row.name]
            effective, verified = row.effective, row.verified
            associated = effective is not None and (
                effective.candidate == requested.candidate and effective.scope == requested.scope
                and effective.identity == target.identity
                and self.plan.created_at <= effective.observed_at <= self.observed_at)
            proven = associated and verified is not None and (
                verified.recipe == target.recipe and verified.recipe_version == target.recipe_version
                and effective.observed_at <= verified.observed_at <= self.observed_at)
            if verified is not None and (not proven or not fresh):
                raise ValueError('stale or unrelated verification')
            complete = complete and proven
        if self.status in (Status.VERIFIED_UNPERSISTED, Status.COMMITTED, Status.ROLLED_BACK_VERIFIED) and not complete:
            raise ValueError('complete fresh behavior evidence required')
        if self.persistent is not None and (
                type(self.persistent) is not PersistentRevision or not complete
                or self.persistent.candidate != requested.candidate
                or self.persistent.scope != requested.scope):
            raise ValueError('persistent promotion requires matching verification')
        if self.status is Status.COMMITTED and self.persistent is None:
            raise ValueError('committed manifest required')

    def __bool__(self):
        raise TypeError('inspect receipt.status explicitly')

    @property
    def applied(self):
        """Compatibility history projection; a durable request remains pending."""
        return 'confirmed' if self.status is Status.COMMITTED else 'pending'

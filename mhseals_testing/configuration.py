"""One immutable startup selection owned by the existing testing session.

Receipts cover pure hardware behavior and navigation serialization only. They
neither authorize output activation nor certify ROS plugin consumption.
"""
from dataclasses import dataclass
from pathlib import Path

from mhseals_config import BaselineRevision, digest
from mhseals_config._bootstrap import (
    begin_bootstrap, finish_bootstrap, read_bootstrap_inputs, read_selected_inputs,
)
from mhseals_hardware import startup_config as hardware
from mhseals_nav import startup_snapshot as navigation

DOMAINS = {'hardware': hardware, 'navigation': navigation}


def _evaluators(domains):
    result = {}
    for name in domains:
        for key, value in DOMAINS[name].EVALUATORS.items():
            if key in result:
                raise ValueError('duplicate startup recipe: ' + key)
            result[key] = value
    return result


@dataclass(frozen=True)
class Selection:
    root: str
    scope: str
    candidate: str
    domains: tuple

    def __post_init__(self):
        object.__setattr__(self, 'domains', tuple(self.domains))
        if (not self.root or not Path(self.root).is_absolute() or not self.scope or not self.candidate or not self.domains
                or len(set(self.domains)) != len(self.domains)
                or not set(self.domains) <= DOMAINS.keys()):
            raise ValueError('invalid session configuration reference')

    def record(self):
        return dict(root=self.root, scope=self.scope, candidate=self.candidate,
                    domains=list(self.domains))

    @classmethod
    def from_record(cls, record):
        if not isinstance(record, dict) or set(record) != {'root', 'scope', 'candidate', 'domains'}:
            raise ValueError('missing session configuration reference; legacy unmanaged session')
        return cls(record['root'], record['scope'], record['candidate'], tuple(record['domains']))

    def validate(self):
        """Read-only full closure validation before each managed process start."""
        if not (Path(self.root) / self.scope / 'active.json').is_file():
            raise ValueError('managed startup selection missing')
        selected = read_selected_inputs(self.root, self.scope,
                                        evaluators=_evaluators(self.domains))
        expected = set().union(*(DOMAINS[d].PARTICIPANTS for d in self.domains))
        if selected.ticket.candidate != self.candidate or set(selected.documents) != expected:
            raise ValueError('session configuration selection changed')
        if 'navigation' in self.domains:
            document = selected.documents[navigation.PARTICIPANT]
            if document['compiler'] != navigation.compiler_identity():
                raise ValueError('navigation compiler identity changed')
            # Checking identities never creates the managed BT materializations.
            from mhseals_nav.startup_projection import resource_identity
            paths = {key: value['path'] for key, value in document['resources'].items()}
            current = resource_identity(paths)
            if any(current[k]['sha256'] != v['sha256'] for k, v in document['resources'].items()):
                raise ValueError('navigation resources changed')
        return selected

    def arguments(self):
        return dict(config_root=self.root, config_scope=self.scope, config_candidate=self.candidate)


def bootstrap(root, scope, preparations):
    """Publish one complete union; preparation is pure and precedes reservation."""
    root = str(Path(root).expanduser().absolute())
    domains = tuple(sorted(preparations))
    evaluators = _evaluators(domains)
    documents, recipes, identities, artifacts = {}, {}, {}, []
    for name in domains:
        prepared = preparations[name]
        if set(prepared['documents']) != DOMAINS[name].PARTICIPANTS:
            raise ValueError('incomplete domain closure: ' + name)
        for target, key in ((documents, 'documents'), (recipes, 'recipes'),
                            (identities, 'identities')):
            if target.keys() & prepared[key].keys():
                raise ValueError('duplicate startup participant')
            target.update(prepared[key])
        artifacts.extend(prepared['baseline_artifacts'])
    artifacts = tuple(sorted(artifacts, key=lambda a: a.name))
    if len({a.name for a in artifacts}) != len(artifacts):
        raise ValueError('duplicate baseline artifact')
    baseline = BaselineRevision(digest({a.name: a.sha256 for a in artifacts}), artifacts)
    source = digest({name: preparations[name]['source'] for name in domains})
    ticket = begin_bootstrap(root, scope, baseline, documents, recipes,
                             source_identity=source, participant_identities=identities,
                             closure='inert', evaluators=evaluators)
    pending = read_bootstrap_inputs(root, ticket)
    observations = {}
    for name in domains:
        observations.update(DOMAINS[name].observe_inputs(pending.documents, ticket, source))
    receipt = finish_bootstrap(root, ticket, observations, evaluators=evaluators)
    if receipt['status'] != 'verified_startup':
        raise ValueError('complete startup selection unverified')
    reference = Selection(str(Path(root).expanduser().absolute()), scope, ticket.candidate, domains)
    reference.validate()
    return reference


# Pure records only: no public mutation operation is enabled by these exports.
from ._configuration_records import (  # noqa: E402,F401
    EffectiveRevision, ParticipantPlan, ParticipantReceipt, PersistentRevision,
    Plan, Receipt, RequestedRevision, Status, VerifiedRevision,
)

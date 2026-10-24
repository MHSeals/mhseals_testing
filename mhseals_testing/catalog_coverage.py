"""Bounded reconciliation of supplied resources and historical audit family IDs.

No implicit resource discovery or inferred trace relationships. Historical audit
records are inputs and are never rewritten. No disposition grants write access.
"""
from dataclasses import dataclass
from hashlib import sha256
from mhseals_config.catalog import _yaml


@dataclass(frozen=True)
class CoverageRow:
    identity: str
    owner: str
    revision: str
    disposition: str
    descriptors: tuple
    explanation: str
    blockers: tuple


def _leaves(value, parts=()):
    # Sequences are single typed fields. Empty mappings remain visible leaves.
    if hasattr(value, 'items') and value:
        for key, child in value.items():
            yield from _leaves(child, (*parts, str(key)))
    else:
        yield '/' + '/'.join(p.replace('~', '~0').replace('/', '~1') for p in parts)


def reconcile(catalog, resources, families, *, traces=None, dispositions=None):
    """Resources: {(owner, source_id): (revision, immutable_yaml_bytes)}.

    Families: {(owner, historical_family_id): current_revision}. Traces explicitly
    map those identities to canonical descriptor keys. Partial family coverage
    stays untraced; caller may only supply a complete trace closure. Explicit
    fixed/unsupported dispositions require an explanation. Missing traces stay
    untraced, including families whose representative descendants are cataloged.
    """
    traces, dispositions = traces or {}, dispositions or {}
    if set(traces) - set(families) or set(dispositions) - set(families):
        raise ValueError('trace/disposition outside declared bounded families')
    if set(traces) & set(dispositions):
        raise ValueError('ambiguous family disposition')
    rows = []
    refs = {}
    for row in catalog.rows:
        if row.source:
            key = (row.owner, row.source['source_id'], row.source['yaml_pointer'])
            refs.setdefault(key, []).append(row.key)
            supplied = resources.get((row.owner, row.source['source_id']))
            if supplied is None or sha256(supplied[1]).hexdigest() != row.source['source_hash']:
                raise ValueError('missing or changed descriptor resource')
    for (owner, sid), (revision, raw) in sorted(resources.items()):
        if type(raw) is not bytes or not revision:
            raise ValueError('resource needs immutable bytes and current revision')
        for pointer in sorted(_leaves(_yaml(raw.decode('utf-8')))):
            keys = tuple(sorted(refs.get((owner, sid, pointer), ())))
            rows.append(CoverageRow(sid + ':' + pointer, owner, revision,
                'descriptor-backed' if keys else 'untraced', keys,
                'default reference only; runtime authority absent' if keys else
                'package owner trace/descriptor follow-up required',
                tuple(sorted({blocker for key in keys for blocker in catalog.field(key).blockers}))
                if keys else ('untraced-resource-pointer',)))
    for identity, revision in sorted(families.items()):
        owner, family = identity
        if not revision:
            raise ValueError('family needs current revision')
        keys = tuple(traces.get(identity, ()))
        if identity in traces and not keys:
            raise ValueError('empty family trace closure')
        for key in keys:
            if catalog.field(key).owner != owner:
                raise ValueError('family trace owner mismatch')
        status, reason = ('descriptor-backed', 'explicit complete descriptor closure') if keys else (
            'untraced', 'named family requires package owner follow-up')
        if identity in dispositions:
            status, reason = dispositions[identity]
            if status not in ('fixed', 'unsupported') or not reason:
                raise ValueError('invalid explicit disposition')
        rows.append(CoverageRow(family, owner, revision, status, keys, reason,
            tuple(sorted({blocker for key in keys for blocker in catalog.field(key).blockers}))
            if keys else (status + '-family',)))
    return tuple(rows)


def markdown(rows):
    """Concise read-only documentation from the exact reconciliation rows."""
    lines = ['# Bounded catalog coverage', '',
             'Defaults and descriptor declarations confer no runtime authority.', '',
             '| Owner | Resource pointer / family | Current revision | Disposition | Explanation | Blockers |',
             '| --- | --- | --- | --- | --- | --- |']
    for row in rows:
        fields = (row.owner, row.identity, row.revision, row.disposition, row.explanation, '; '.join(row.blockers))
        lines.append('| ' + ' | '.join(str(v).replace('|', '\\|').replace('\n', ' ') for v in fields) + ' |')
    return '\n'.join(lines) + '\n'

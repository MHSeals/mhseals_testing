"""Pure read-only catalog views; explicit bundles carry their resource defaults.

This module imports no domain package and performs no discovery or resolution.
Requested/effective/runtime values belong to compiler and runtime evidence.
"""
from dataclasses import dataclass
from types import MappingProxyType

from mhseals_config.catalog import DescriptorBundle


@dataclass(frozen=True)
class Row:
    owner: str
    revision: str
    leaf: object

    @property
    def key(self):
        return self.leaf.descriptor['canonical_key']

    @property
    def default(self):
        return self.leaf.default

    @property
    def source(self):
        return self.leaf.descriptor.get('default_ref')

    @property
    def blockers(self):
        return self.leaf.blockers

    @property
    def writable(self):
        return False


@dataclass(frozen=True)
class Catalog:
    rows: tuple
    missing: tuple
    identities: object

    def field(self, identity):
        try:
            return self.identities[identity]
        except KeyError as exc:
            raise ValueError('unknown catalog identity: ' + identity) from exc

    @property
    def blockers(self):
        return tuple('missing-required-bundle:' + owner for owner in self.missing)


def assemble(bundles, *, required=('mhseals_hardware', 'mhseals_nav')):
    """Assemble explicit {expected_owner: DescriptorBundle}; never import loaders.

    Each bundle was validated by its package using explicitly supplied resource
    bytes. The view preserves that immutable default/provenance and never reads
    source files or assumes a default is an effective or live value.
    """
    required = tuple(required)
    if (len(set(required)) != len(required) or
            any(not isinstance(owner, str) or not owner for owner in required)):
        raise ValueError('invalid required owners')
    rows, identities = [], {}
    for owner, bundle in sorted(bundles.items()):
        if not isinstance(bundle, DescriptorBundle) or bundle.descriptor['owner_package'] != owner:
            raise ValueError('unexpected descriptor owner: ' + str(owner))
        if owner not in required:
            raise ValueError('undeclared descriptor owner: ' + owner)
        for leaf in bundle.rows:
            row = Row(owner, bundle.descriptor['revision'], leaf)
            for identity in (row.key, *leaf.descriptor['aliases']):
                if identity in identities:
                    raise ValueError('cross-package catalog identity collision: ' + identity)
                identities[identity] = row
            rows.append(row)
    return Catalog(tuple(sorted(rows, key=lambda r: (r.owner, r.key))),
                   tuple(sorted(set(required) - set(bundles))),
                   MappingProxyType(identities))

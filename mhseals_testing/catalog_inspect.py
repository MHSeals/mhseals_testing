"""Explicit installed providers and a shared read-only inventory representation."""
from collections.abc import Mapping
from importlib import import_module

import yaml

from mhseals_testing.catalog import assemble

PROVIDERS = (
    ('mhseals_hardware', 'mhseals_hardware.catalog', 'load_catalog', False),
    ('mhseals_nav', 'mhseals_nav.owner_catalog', 'load_catalog', False),
    ('astro-run-config', 'astro_run_config', 'catalog_bundle', True),
)


def plain(value):
    if isinstance(value, Mapping):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(item) for item in value]
    return value


def inspection(bundles, availability, *, compiled=None):
    """Serialize immutable metadata; optional explicit compiler evidence is never live."""
    catalog = assemble(bundles, required=tuple(item[0] for item in PROVIDERS))
    rows = []
    for row in catalog.rows:
        descriptor = plain(row.leaf.descriptor)
        sources = {source['id']: plain(source)
                   for source in bundles[row.owner].descriptor['default_sources']}
        reference = descriptor.get('default_ref', {})
        rows.append(dict(owner=row.owner, revision=row.revision, key=row.key,
                         default=plain(row.default), descriptor=descriptor,
                         source=sources.get(reference.get('source_id')),
                         blockers=list(row.blockers), writable=False,
                         numeric_bounds='unknown; see constraint_ref'))
    selection = dict(status='unknown', label='selection unknown; package inventory only', values={})
    if compiled is not None:
        try:
            if not isinstance(compiled, Mapping) or set(compiled) - catalog.identities.keys():
                raise ValueError('unknown compiled selection identity')
            for value in compiled.values():
                if not isinstance(value, Mapping) or set(value) != {'value', 'origins'}:
                    raise ValueError('compiled evidence requires value and origins')
            selection = dict(status='compiled', label='requested/compiled; never live',
                             values=plain(compiled))
        except ValueError as error:
            selection = dict(status='unavailable', label=str(error), values={})
    return dict(schema=1, mode='read-only inventory', selection=selection, rows=rows,
                packages=plain(availability), blockers=list(catalog.blockers),
                complete=not catalog.missing and all(p['status'] == 'available'
                    for p in availability if p['status'] != 'missing-optional'))


def installed(*, compiled=None):
    """Call only named installed providers; isolate absent owners from broken installs."""
    bundles, availability = {}, []
    for owner, module_name, function, optional in PROVIDERS:
        status, detail = 'available', ''
        try:
            module = import_module(module_name)
            bundle = getattr(module, function)()
            # Refuse owner/cross-owner collisions before exposing ambiguous rows.
            assemble({**bundles, owner: bundle}, required=tuple(p[0] for p in PROVIDERS))
            bundles[owner] = bundle
        except ModuleNotFoundError as error:
            absent = error.name in (module_name, module_name.split('.')[0])
            status = ('missing-optional' if optional else 'missing-package') if absent else 'missing-dependency'
            detail = str(error)
        except FileNotFoundError as error:
            status, detail = 'missing-resource', str(error)
        except (ValueError, TypeError, UnicodeError, AttributeError, yaml.YAMLError) as error:
            status, detail = 'invalid-installed', str(error)
        except ImportError as error:
            status, detail = 'missing-dependency', str(error)
        except OSError as error:
            status, detail = 'unreadable-resource', str(error)
        families = ('nav2', 'mavros', 'lidar') if owner == 'mhseals_nav' else ('hardware',) if owner == 'mhseals_hardware' else ('run-config',)
        availability.append(dict(owner=owner, status=status, detail=detail,
                                 families=[dict(name=name, status=status) for name in families]))
    return inspection(bundles, availability, compiled=compiled)


def text(view):
    lines = [view['mode'], view['selection']['label']]
    lines += [f"{p['owner']}: {p['status']} {p['detail']}" for p in view['packages']]
    for row in view['rows']:
        lines.append(f"{row['key']} = {row['default']!r}")
        lines.append(str(row))
    return '\n'.join(lines)

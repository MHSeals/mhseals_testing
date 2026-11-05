"""Each App attribute has one writer (finding #4): ui/app/core.py OWNERS gives each mixin its attribute group.

An AST walk over the mixins (ui/app/actions/**, loop.py, view.py) collects every ``self.<attr> = ...``; a write
to an attribute OWNERS gives another mixin, or to one App builds (APP), is a crossing. SHARED (notice, modal,
focus, ...) is the page's own and anyone's. ALLOWED holds the crossings on 11-04: it only shrinks, and an
entry that no longer crosses must leave it (as the area converting its mixin does). An attribute no table
names may have one writer only.
"""

import ast
from collections import defaultdict
from pathlib import Path

import pytest

from mhseals_testing.ui.app import core
from mhseals_testing.ui.app import state as app_state

APP_DIR = Path(core.__file__).resolve().parent
ALLOWED = {   # (mixin class, attribute): the crossings on 11-04, by owner
    ('Arm', 'command'), ('Procedures', 'command'), ('Hardware', 'draft'), ('Hardware', 'grabbed'),   # Manual's
    ('Loop', 'grabbed'), ('Hardware', 'manual_speed'),
    ('Arm', '_runner_cancelled'),                                                                  # Procedures'
    ('Bags', 'last_tick'), ('Files', 'last_drawn'), ('Loop', '_review_fault'),  # Loop's and View's
    ('Bags', 'layerbox'), ('Bags', 'minimap'), ('Bags', 'review'), ('Bags', 'source'), ('Bags', 'tabs'),  # App's
    ('Loop', 'detaching'), ('Loop', 'source'), ('Quit', 'keys'), ('Quit', 'live'), ('Settings', 'profile'),
}


def owner_of():
    owners = {attr: mixin for mixin, attrs in core.OWNERS.items() for attr in attrs}
    owners.update(dict.fromkeys(core.APP, 'App'))
    return owners


def mixin_files():
    files = sorted(p for p in (APP_DIR / 'actions').rglob('*.py') if '__pycache__' not in p.parts)
    return files + [APP_DIR / 'loop.py', APP_DIR / 'view.py']


def direct(target):
    if isinstance(target, ast.Attribute) and isinstance(target.value, ast.Name) and target.value.id == 'self':
        yield target.attr
    elif isinstance(target, (ast.Tuple, ast.List)):
        for element in target.elts:
            yield from direct(element)
    elif isinstance(target, ast.Starred):
        yield from direct(target.value)


def writes():
    """{(class, attr): [file:line]} of every self.<attr> write in a mixin class."""
    found = defaultdict(list)
    for path in mixin_files():
        for cls in (n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef)):
            for node in ast.walk(cls):
                targets = node.targets if isinstance(node, ast.Assign) else (
                    [node.target] if isinstance(node, (ast.AugAssign, ast.AnnAssign)) else [])
                for target in targets:
                    for attr in direct(target):
                        found[(cls.name, attr)].append(f'{path.relative_to(APP_DIR)}:{node.lineno}')
    return found


def crossings():
    owners = owner_of()
    return {(cls, attr): owners[attr] for cls, attr in writes()
            if attr not in core.SHARED and attr in owners and owners[attr] != cls}


@pytest.mark.smoke
def test_no_mixin_writes_another_ones_state():
    found = writes()
    new = {key: owner for key, owner in crossings().items() if key not in ALLOWED}
    assert not new, 'writes across owners (core.py OWNERS; read through app.services, ask the owner to write):\n' + \
        '\n'.join(f'  {cls}.{attr} (owner {owner}) at {", ".join(found[(cls, attr)])}'
                  for (cls, attr), owner in sorted(new.items()))


@pytest.mark.smoke
def test_the_allowlist_only_shrinks():
    stale = sorted(key for key in ALLOWED if key not in crossings())
    assert not stale, f'no longer crossing, drop from ALLOWED: {stale}'


@pytest.mark.smoke
def test_an_unowned_attribute_has_one_writer():
    owners = owner_of()
    writers = defaultdict(set)
    for cls, attr in writes():
        if attr not in owners and attr not in core.SHARED:
            writers[attr].add(cls)
    shared = {attr: sorted(classes) for attr, classes in writers.items() if len(classes) > 1}
    assert not shared, f'give these an owner in core.py OWNERS: {shared}'


def test_the_tables_name_each_attribute_once():
    named = [attr for attrs in core.OWNERS.values() for attr in attrs] + list(core.APP) + list(core.SHARED)
    assert len(named) == len(set(named))


@pytest.mark.smoke
def test_each_alias_reads_and_writes_its_container():
    """The flat names (app.bag_scan) and the containers (app.state.bags.scan) are one value; services read it."""
    app = core.App.__new__(core.App)
    for name, (group, attr) in app_state.ALIASES.items():
        marker = object()
        setattr(app, name, marker)
        assert getattr(getattr(app.state, group), attr) is marker and getattr(app, name) is marker
        if group in app.services.GROUPS:
            assert getattr(getattr(app.services, group), attr) is marker
    with pytest.raises(AttributeError):
        app.services.bags.scan = None


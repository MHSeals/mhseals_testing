"""Each App attribute has one writer (finding #4, atlas #27): ui/app/core.py OWNERS gives each mixin its group.

An AST walk over every ui/** module collects each write to the App: ``self.<attr> = ...`` in App and its mixins
(keyed by class) and ``app.<attr> = ...`` or ``self.app.<attr> = ...`` anywhere (keyed by module). A mixin's write
to an attribute OWNERS gives another mixin, or to one App builds (APP), is a crossing; a module (a feed, a popup,
a panel) writes no App attribute at all, it calls the owner's method. App.__init__, startup.py and wiring.py build
the App and write what they build. SHARED (modal, focus, ...) is the page's own and anyone's, except the footer
notice: App.say is its one writer. ALLOWED, the 11-04 crossings, is empty and stays so. An attribute no table
names may have one writer only. core.py's own fan-out stays at 25 modules or fewer (it was 40 on 11-04).
"""

import ast
import importlib.util
from collections import defaultdict
from pathlib import Path

import pytest

from mhseals_testing.ui.app import core
from mhseals_testing.ui.app import state as app_state

APP_DIR = Path(core.__file__).resolve().parent
UI_DIR = APP_DIR.parent
FAN_OUT = 25
ALLOWED = set()   # the 11-04 crossings, all converted to owner methods (atlas #27); nothing joins it
BUILD = ('app/startup.py', 'app/wiring.py')   # with App.__init__: they build the App, writing what they build
MIXINS = {cls.__name__ for cls in core.App.__mro__} - {'object'}


def owner_of():
    owners = {attr: mixin for mixin, attrs in core.OWNERS.items() for attr in attrs}
    owners.update(dict.fromkeys(core.APP, 'App'))
    return owners


def ui_files():
    return sorted(p for p in UI_DIR.rglob('*.py') if '__pycache__' not in p.parts)


def app_attrs(target, mixin):
    """The App attributes ``target`` assigns: self.x in a mixin, app.x and self.app.x anywhere."""
    if isinstance(target, (ast.Tuple, ast.List)):
        for element in target.elts:
            yield from app_attrs(element, mixin)
    elif isinstance(target, ast.Starred):
        yield from app_attrs(target.value, mixin)
    elif isinstance(target, ast.Attribute):
        value = target.value
        if isinstance(value, ast.Name) and (value.id == 'app' or (value.id == 'self' and mixin)):
            yield target.attr
        elif isinstance(value, ast.Attribute) and value.attr == 'app' and isinstance(value.value, ast.Name) \
                and value.value.id == 'self':
            yield target.attr


def walk(node, writer, rel, found):
    for child in ast.iter_child_nodes(node):
        if isinstance(child, ast.ClassDef):
            walk(child, child.name if rel.startswith('app/') and child.name in MIXINS else writer, rel, found)
            continue
        if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
            name = {('App', '__init__'): 'build', ('App', 'say'): 'App.say'}.get((writer, child.name), writer)
            walk(child, name, rel, found)
            continue
        targets = child.targets if isinstance(child, ast.Assign) else (
            [child.target] if isinstance(child, (ast.AugAssign, ast.AnnAssign)) else [])
        for target in targets:
            for attr in app_attrs(target, writer in MIXINS or writer in ('build', 'App.say')):
                found[(writer, attr)].append(f'{rel}:{child.lineno}')
        walk(child, writer, rel, found)


def writes():
    """{(writer, attr): [file:line]} of every App write in ui/**: the writer is a mixin class, 'build' or a module."""
    found = defaultdict(list)
    for path in ui_files():
        rel = str(path.relative_to(UI_DIR))
        walk(ast.parse(path.read_text()), 'build' if rel in BUILD else rel, rel, found)
    return found


def crossings():
    owners = owner_of()
    out = {}
    for writer, attr in writes():
        if writer == 'build' or (attr in core.SHARED and attr != 'notice'):
            continue
        if attr == 'notice':
            if writer != 'App.say':
                out[(writer, attr)] = 'App.say'
        elif writer not in MIXINS:
            out[(writer, attr)] = owners.get(attr, 'its owner\'s method')    # a module writes no App state
        elif attr in owners and owners[attr] != writer:
            out[(writer, attr)] = owners[attr]
    return out


@pytest.mark.smoke
def test_the_allowlist_is_empty():
    assert ALLOWED == set(), 'no crossing joins ALLOWED: give the write to its owner (core.py OWNERS)'


@pytest.mark.smoke
def test_nothing_writes_another_owners_state():
    found = writes()
    new = {key: owner for key, owner in crossings().items() if key not in ALLOWED}
    assert not new, 'writes across owners (core.py OWNERS; read through app.services, ask the owner to write):\n' + \
        '\n'.join(f'  {writer} sets {attr} (owner {owner}) at {", ".join(found[(writer, attr)])}'
                  for (writer, attr), owner in sorted(new.items()))


def test_the_walk_sees_mixins_modules_and_the_notice():
    """The walk itself: a mixin's self.x, a module's app.x and self.app.x, and App.say's notice."""
    found = writes()
    assert ('Arm', 'armed_by') in found and ('App.say', 'notice') in found
    assert any(writer.startswith('feeds/') for writer, _ in found)          # channel.py's self.app.activity
    planted = defaultdict(list)
    walk(ast.parse('def f(app):\n    app.x = 1\nclass F:\n    def g(self):\n        self.app.y = self.z = 2\n'),
         'feeds/planted.py', 'feeds/planted.py', planted)
    assert set(planted) == {('feeds/planted.py', 'x'), ('feeds/planted.py', 'y')}


@pytest.mark.smoke
def test_an_unowned_attribute_has_one_writer():
    owners = owner_of()
    writers = defaultdict(set)
    for writer, attr in writes():
        if attr not in owners and attr not in core.SHARED and writer != 'build':
            writers[attr].add(writer)
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
    other = core.App.__new__(core.App)       # every App gets its facades, not only the first
    assert other.services.bags.scan == app_state.BagsState().scan


def imported_modules(path):
    """The modules ``path`` imports at top level, ``from pkg import mod`` counted as pkg.mod."""
    modules = set()
    for node in ast.parse(Path(path).read_text()).body:
        if isinstance(node, ast.Import):
            modules.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            for alias in node.names:
                name = f'{node.module}.{alias.name}'
                try:
                    modules.add(name if importlib.util.find_spec(name) else node.module)
                except (ImportError, ValueError):
                    modules.add(node.module)
    return modules


@pytest.mark.smoke
def test_core_fan_out_stays_small():
    """App builds its feeds and views through wiring.py and takes its mixins from mixins.py (finding #4)."""
    modules = imported_modules(core.__file__)
    assert len(modules) <= FAN_OUT, f'core.py imports {len(modules)} modules > {FAN_OUT}: {sorted(modules)}'

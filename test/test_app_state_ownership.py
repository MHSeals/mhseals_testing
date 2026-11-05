"""Each App attribute has one writer (finding #4, atlas #26 #27): each container in app.state (ui/app/state) is
named for the mixin that owns it.

An AST walk over every ui/** module, and every other package module that receives ``app`` (annotate/live.py),
collects each write to the App: ``self.<attr> = ...`` in App and its mixins (keyed by class), ``app.<attr> = ...``
or ``self.app.<attr> = ...`` anywhere (keyed by module), and ``<...>.state.<group>.<field> = ...``, directly or
through a local name bound to the container, keyed ``state.<group>``. A mixin's write to another mixin's
container, or to an attribute App builds (APP), is a crossing; a module (a feed, a popup, a panel) writes no App
attribute at all, it calls the owner's method. App.__init__, startup.py and wiring.py build the App and write what
they build. SHARED (modal, focus, ...) is the page's own and anyone's, except the footer notice: App.say is its one
writer. ALLOWED, the 11-04 crossings, is empty and stays so. Any other flat attribute may have one writer only.
core.py's own fan-out stays at 25 modules or fewer (it was 40 on 11-04).
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


GROUPS = tuple(app_state.AppState.__dataclass_fields__)


def owner_of():
    """{attr: owner}: ``state.<group>`` belongs to the mixin it is named for, APP to App."""
    mixins = {name.lower(): name for name in MIXINS}
    owners = {f'state.{group}': mixins[group] for group in GROUPS}
    owners.update(dict.fromkeys(core.APP, 'App'))
    return owners


PACKAGE = UI_DIR.parent


def takes_app(path):
    """True when a module outside ui/** has a function taking ``app`` (annotate/live.py, harness/sim_arm.py)."""
    return any(isinstance(node, ast.arg) and node.arg == 'app' for node in ast.walk(ast.parse(path.read_text())))


def ui_files():
    """{rel: path}: every ui/** module (rel to ui/) and every other package module that receives app."""
    files = {str(p.relative_to(UI_DIR)): p for p in UI_DIR.rglob('*.py') if '__pycache__' not in p.parts}
    files.update((str(p.relative_to(PACKAGE)), p) for p in PACKAGE.rglob('*.py')
                 if '__pycache__' not in p.parts and UI_DIR not in p.parents and takes_app(p))
    return dict(sorted(files.items()))


def container(node, mixin):
    """``state.<group>`` when ``node`` is app.state.<group> (self.state.<group> in a mixin, self.app.state.<group>)."""
    if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Attribute) and node.value.attr == 'state' \
            and any(app_attrs(node.value, mixin)):
        return f'state.{node.attr}'
    return None


def app_attrs(target, mixin, local=None):
    """The App attributes ``target`` assigns: self.x in a mixin, app.x and self.app.x anywhere, and a container's
    field (``state.<group>``), directly or through a ``local`` name bound to it."""
    if isinstance(target, (ast.Tuple, ast.List)):
        for element in target.elts:
            yield from app_attrs(element, mixin, local)
    elif isinstance(target, ast.Starred):
        yield from app_attrs(target.value, mixin, local)
    elif isinstance(target, ast.Attribute):
        value = target.value
        group = container(value, mixin) or (local or {}).get(getattr(value, 'id', None))
        if group:
            yield group
        elif isinstance(value, ast.Name) and (value.id == 'app' or (value.id == 'self' and mixin)):
            yield target.attr
        elif isinstance(value, ast.Attribute) and value.attr == 'app' and isinstance(value.value, ast.Name) \
                and value.value.id == 'self':
            yield target.attr


def walk(node, writer, rel, found, local=None):
    """Collect App writes under ``node``; ``local`` maps the names a function bound to a container."""
    local = {} if local is None else local
    for child in ast.iter_child_nodes(node):
        if isinstance(child, ast.ClassDef):
            walk(child, child.name if rel.startswith('app/') and child.name in MIXINS else writer, rel, found)
            continue
        if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
            name = {('App', '__init__'): 'build', ('App', 'say'): 'App.say'}.get((writer, child.name), writer)
            walk(child, name, rel, found, dict(local))
            continue
        mixin = writer in MIXINS or writer in ('build', 'App.say')
        targets = child.targets if isinstance(child, ast.Assign) else (
            [child.target] if isinstance(child, (ast.AugAssign, ast.AnnAssign)) else [])
        for target in targets:
            for attr in app_attrs(target, mixin, local):
                found[(writer, attr)].append(f'{rel}:{child.lineno}')
        for target in targets:           # run = self.state.procedures: run.x = ... writes that container
            values = child.value.elts if isinstance(target, ast.Tuple) and isinstance(child.value, ast.Tuple) \
                else [child.value]
            for name, value in zip(target.elts if len(values) > 1 else [target], values):
                if isinstance(name, ast.Name) and (group := container(value, mixin)):
                    local[name.id] = group
        walk(child, writer, rel, found, local)


def writes():
    """{(writer, attr): [file:line]} of every App write in ui/**: the writer is a mixin class, 'build' or a module."""
    found = defaultdict(list)
    for rel, path in ui_files().items():
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
    assert ALLOWED == set(), 'no crossing joins ALLOWED: give the write to its owner (ui/app/state)'


@pytest.mark.smoke
def test_nothing_writes_another_owners_state():
    found = writes()
    new = {key: owner for key, owner in crossings().items() if key not in ALLOWED}
    assert not new, 'writes across owners (ui/app/state; read through app.services, ask the owner to write):\n' + \
        '\n'.join(f'  {writer} sets {attr} (owner {owner}) at {", ".join(found[(writer, attr)])}'
                  for (writer, attr), owner in sorted(new.items()))


def test_the_walk_sees_mixins_modules_and_the_notice():
    """The walk itself: a mixin's self.x, a module's app.x and self.app.x, and App.say's notice."""
    found = writes()
    assert ('Arm', 'state.arm') in found and ('App.say', 'notice') in found
    assert any(writer.startswith('feeds/') for writer, _ in found)          # channel.py's self.app.activity
    assert 'annotate/live.py' in ui_files()                                  # outside ui/**, but it receives app
    planted = defaultdict(list)
    walk(ast.parse('def f(app):\n    app.x = app.state.arm.w = 1\nclass F:\n    def g(self):\n'
                   '        self.app.y = self.z = self.state.arm.v = 2\n'
                   '    def h(self):\n        run, n = self.app.state.procedures, 0\n        run.paused = True\n'),
         'feeds/planted.py', 'feeds/planted.py', planted)
    assert set(planted) == {('feeds/planted.py', 'x'), ('feeds/planted.py', 'y'), ('feeds/planted.py', 'state.arm'),
                            ('feeds/planted.py', 'state.procedures')}
    assert planted[('feeds/planted.py', 'state.procedures')] == ['feeds/planted.py:8']   # through the local name


@pytest.mark.smoke
def test_an_unowned_attribute_has_one_writer():
    owners = owner_of()
    writers = defaultdict(set)
    for writer, attr in writes():
        if attr not in owners and attr not in core.SHARED and writer != 'build':
            writers[attr].add(writer)
    shared = {attr: sorted(classes) for attr, classes in writers.items() if len(classes) > 1}
    assert not shared, f'give these a container in ui/app/state: {shared}'


def test_each_container_is_named_for_its_owner():
    """The container is the owner: every AppState field names one of App's mixins; the tables never overlap."""
    assert set(GROUPS) <= {name.lower() for name in MIXINS}
    named = list(core.APP) + list(core.SHARED)
    assert len(named) == len(set(named)) and 'state' not in named


@pytest.mark.smoke
def test_services_read_every_container_and_refuse_writes():
    """app.services.<group> is the container App built, read-only: the error names the container and its owner."""
    app = core.App.__new__(core.App)
    app.state = app_state.AppState()
    app.services = app_state.facades(app)
    assert app.services.GROUPS == tuple(app_state.AppState.__dataclass_fields__)
    marker = app.state.bags.scan = object()
    assert app.services.bags.scan is marker
    with pytest.raises(AttributeError, match=r'app\.state\.bags\.scan .* the bags mixin'):
        app.services.bags.scan = None


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

"""Every package module compiles (a bad paste fails here, not at boat_test start), and the pure core imports
without ROS so it runs (and is tested) anywhere. One test per check, naming every offender."""

import ast
from pathlib import Path
import sys

import pytest

PACKAGE = Path(__file__).resolve().parent.parent / 'mhseals_testing'
MODULES = sorted(PACKAGE.rglob('*.py'))
ROS_MODULES = {'rclpy', 'rosbag2_py', 'launch', 'launch_ros', 'tf2_ros', 'geometry_msgs',
               'nav_msgs', 'sensor_msgs', 'std_msgs', 'tf2_msgs', 'vision_msgs', 'nav2_msgs'}
# Only ros/ and a feature package's ros_*.py talk to ROS. characterization.read_rosbag imports lazily inside
# the function.
PURE = [path for path in MODULES
        if 'ros' not in path.relative_to(PACKAGE).parts and not path.name.startswith('ros_')]


def top_level_imports(path):
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Import):
            yield from (alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            yield node.module.split('.')[0]


def test_every_module_compiles():
    broken = []
    for path in MODULES:
        try:
            compile(path.read_text(), str(path), 'exec')
        except SyntaxError as error:
            broken.append(f'{path.relative_to(PACKAGE)}: {error}')
    assert len(MODULES) > 100 and not broken, broken


def test_core_never_imports_ros_at_module_level():
    offenders = {str(path.relative_to(PACKAGE)): sorted(ROS_MODULES & set(top_level_imports(path)))
                 for path in PURE}
    assert len(PURE) > 100 and not {k: v for k, v in offenders.items() if v}


def imported(path, top=True):
    """Every absolute module a file imports (``from a.b import c`` gives ``a.b`` and ``a.b.c``); ``top=False``
    also counts imports inside functions."""
    tree = ast.parse(path.read_text())
    for node in (tree.body if top else ast.walk(tree)):
        if isinstance(node, ast.Import):
            yield from (alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and not node.level:
            yield node.module
            yield from (f'{node.module}.{alias.name}' for alias in node.names)


def under(module, prefixes):
    return any(module == prefix or module.startswith(prefix + '.') for prefix in prefixes)


def files(*folders):
    return [path for folder in folders for path in sorted((PACKAGE / folder).rglob('*.py'))]


# (a) The lower layers never import the UI, at any level (a function-level import counts). The allowlist only
# shrinks: advisories.deploy renders the header's Text until the header's view renders deploy_state() itself.
LOWER = ('ros', 'health', 'procedures', 'model', 'harness')
UI_ALLOWED = {'health/advisories.py': {'mhseals_testing.ui.deploy', 'mhseals_testing.ui.deploy.render'}}


def test_lower_layers_never_import_the_ui():
    offenders = {str(path.relative_to(PACKAGE)): sorted({m for m in imported(path, top=False)
                                                         if under(m, ('mhseals_testing.ui',))})
                 for path in files(*LOWER)}
    offenders = {name: found for name, found in offenders.items() if set(found) - UI_ALLOWED.get(name, set())}
    assert len(files(*LOWER)) > 50 and not offenders, offenders


@pytest.mark.parametrize('name', sorted(UI_ALLOWED))
def test_the_lower_layer_ui_allowlist_only_shrinks(name):
    found = {m for m in imported(PACKAGE / name, top=False) if under(m, ('mhseals_testing.ui',))}
    assert found == UI_ALLOWED[name], f'{name} imports {sorted(found)}: shrink UI_ALLOWED to it (drop it when empty)'


# (b) Panels and popups draw from models: no ROS, harness, subprocess or hardware driver at top level. Today's
# offenders are allowed and the list only shrinks (each entry is what the file imports now).
PANEL_BANNED = ('mhseals_testing.ros', 'mhseals_testing.harness', 'rclpy', 'subprocess', 'mhseals_hardware')
PANEL_ALLOWED = {
    'ui/panels/agent_log.py': {'mhseals_testing.harness'},
    'ui/panels/checklist.py': {'mhseals_testing.harness'},
    'ui/panels/procedures.py': {'mhseals_testing.harness'},
    'ui/panels/thrusters.py': {'mhseals_hardware'},
    'ui/popups/authority.py': {'mhseals_testing.harness'},
    'ui/popups/config.py': {'mhseals_testing.harness'},
    'ui/popups/course.py': {'mhseals_testing.harness'},
    'ui/popups/pins.py': {'mhseals_hardware'},
}


def banned_roots(path):
    return {next(prefix for prefix in PANEL_BANNED if under(m, (prefix,)))
            for m in imported(path) if under(m, PANEL_BANNED)}


def test_panels_and_popups_import_no_ros_harness_or_hardware():
    offenders = {str(path.relative_to(PACKAGE)): sorted(banned_roots(path)) for path in files('ui/panels', 'ui/popups')}
    offenders = {name: found for name, found in offenders.items()
                 if set(found) - PANEL_ALLOWED.get(name, set())}
    assert len(files('ui/panels', 'ui/popups')) > 20 and not offenders, offenders


@pytest.mark.parametrize('name', sorted(PANEL_ALLOWED))
def test_the_panel_allowlist_only_shrinks(name):
    path = PACKAGE / name
    found = banned_roots(path) if path.exists() else set()
    assert found == PANEL_ALLOWED[name], f'{name} imports {sorted(found)}: shrink PANEL_ALLOWED to it (drop it when empty)'


# (c) model/ is the neutral layer: the standard library and model/ itself, nothing else, at any level.
def test_model_imports_only_the_stdlib_and_itself():
    models = files('model')
    offenders = {str(path.relative_to(PACKAGE)): sorted({m for m in imported(path, top=False)
                                                         if m.split('.')[0] not in sys.stdlib_module_names
                                                         and not under(m, ('mhseals_testing.model',))})
                 for path in models}
    assert len(models) >= 4 and not {k: v for k, v in offenders.items() if v}, offenders

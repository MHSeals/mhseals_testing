"""No board names in generic code (CONVENTIONS D-20261104-03): what a profile's outputs can do is a capability
(profiles.OUTPUT_CAPABILITIES, PlantProfile.drives_pwm_pins), never a comparison with a board's name.

Two scans over the package's syntax trees, so comments and docstrings never count:
  - no 'odroid', 'jetson' or 'orin' string constant compared against an ``outputs`` value (==, !=, in);
  - the string constant 'odroid' appears only where the board is the data itself: profiles.py (the outputs
    table that derives the capability) and health/pwm_sysfs.py (the readback's backend label).
ALLOWED holds the files still allowed one more each; it only shrinks.
"""

import ast
from pathlib import Path

import pytest

pytestmark = pytest.mark.smoke

PACKAGE = Path(__file__).resolve().parent.parent / 'mhseals_testing'
BOARDS = {'odroid', 'jetson', 'orin'}
HOMES = {'profiles.py': 2, 'health/pwm_sysfs.py': 1}   # file -> the 'odroid' constants it holds by design
ALLOWED = {}   # file -> 'odroid' constants still allowed (another area's files, as they land): only shrinks


def trees():
    for path in sorted(PACKAGE.rglob('*.py')):
        yield str(path.relative_to(PACKAGE)), ast.parse(path.read_text())


def _names_outputs(node):
    if isinstance(node, ast.Name):
        return node.id == 'outputs'
    if isinstance(node, ast.Attribute):
        return node.attr == 'outputs'
    if isinstance(node, ast.Subscript):
        return isinstance(node.slice, ast.Constant) and node.slice.value == 'outputs'
    if isinstance(node, ast.Call):      # x.get('outputs')
        return (isinstance(node.func, ast.Attribute) and node.func.attr == 'get' and node.args
                and isinstance(node.args[0], ast.Constant) and node.args[0].value == 'outputs')
    return False


def _boards(node):
    values = node.elts if isinstance(node, (ast.Tuple, ast.List, ast.Set)) else [node]
    return {v.value for v in values if isinstance(v, ast.Constant) and v.value in BOARDS}


def test_no_board_name_is_compared_with_outputs():
    found = []
    for name, tree in trees():
        for node in ast.walk(tree):
            if isinstance(node, ast.Compare):
                sides = [node.left, *node.comparators]
                if any(map(_names_outputs, sides)) and set().union(*map(_boards, sides)):
                    found.append(f'{name}:{node.lineno}')
    assert not found, f'ask the capability (profiles.drives_pwm_pins) instead: {found}'


def odroid_counts():
    counts = {}
    for name, tree in trees():
        n = sum(isinstance(node, ast.Constant) and node.value == 'odroid' for node in ast.walk(tree))
        if n:
            counts[name] = n
    return counts


def test_odroid_appears_only_where_the_board_is_the_data():
    extra = {name: n for name, n in odroid_counts().items() if n > HOMES.get(name, 0) + ALLOWED.get(name, 0)}
    assert not extra, f"'odroid' outside profiles.OUTPUTS and pwm_sysfs' readback label: {extra}"


def test_the_homes_and_the_allowlist_only_shrink():
    counts = odroid_counts()
    for name in set(HOMES) | set(ALLOWED):
        assert counts.get(name, 0) == HOMES.get(name, 0) + ALLOWED.get(name, 0), \
            f'{name} holds {counts.get(name, 0)}: lower its entry (drop it from ALLOWED at 0)'


def test_the_scan_sees_a_board_comparison():
    tree = ast.parse("a = profile.outputs == 'odroid'\nb = cfg.get('outputs') in ('orin', 'topic')\n"
                     "c = view['outputs'] != 'jetson'\nd = profile.outputs == 'topic'")
    hits = [node.lineno for node in ast.walk(tree) if isinstance(node, ast.Compare)
            and any(map(_names_outputs, [node.left, *node.comparators]))
            and set().union(*map(_boards, [node.left, *node.comparators]))]
    assert hits == [1, 2, 3]

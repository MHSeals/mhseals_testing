"""No key press is compared against a literal in ui/: keys go through config/keymap.yaml.

An AST scan finds ``key == 'q'``, ``key in ('up', 'k')``, ``fold(key) == 'x'`` and ``ord('x')`` where the
literal is a key name (one character, a KeyReader name, or its ``press:`` event). Code asks the keymap
(``keymap.matches`` / ``keymap.dispatch``); the fixed Space and E are ``keymap.STOP_KEY`` and
``keymap.REVOKE_KEY``. ALLOWED holds the files not converted yet with how many such checks each still has:
a count may only go down, a converted file leaves the table, and the table is to reach empty.
"""

import ast
import functools
from pathlib import Path

import pytest

from mhseals_testing.ui import keymap

UI = Path(__file__).resolve().parents[1] / 'mhseals_testing' / 'ui'
KEY_VARS = {'key', 'raw', 'folded', 'pressed', 'ch'}
EVENTS = ('press:', 'release:', 'repeat:')
ALLOWED = {   # path under ui/ -> literal key checks left (refactor areas 3, 4, 6 and this area's queue)
    'app/actions/settings_rows/status.py': 8, 'app/actions/settings_rows/correction.py': 5,
    'app/actions/settings_rows/hardware.py': 1, 'app/actions/arm.py': 6, 'popups/authority.py': 5,
    'popups/hold.py': 2, 'popups/stop.py': 1, 'feeds/agent.py': 1, 'app/view.py': 1, 'panels/checklist.py': 6,
    'app/loop.py': 35, 'app/actions/bags.py': 26, 'popups/history.py': 15, 'app/actions/labels.py': 13,
    'app/actions/manual.py': 10, 'app/actions/procedures.py': 1, 'app/actions/annotate.py': 1,
}


def key_name(text):
    for event in EVENTS:
        text = text.removeprefix(event)
    return len(text) == 1 or text in keymap.KEY_NAMES


def literal(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return key_name(node.value)
    if isinstance(node, (ast.Tuple, ast.List, ast.Set)):
        return any(literal(e) for e in node.elts)
    return False


def keyish(node):
    return (isinstance(node, ast.Name) and node.id in KEY_VARS
            or isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'fold')


def checks(source):
    """[(line, text)] of the literal key checks in ``source``."""
    found = []
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'ord' and node.args and literal(node.args[0]):
            found.append((node.lineno, ast.unparse(node)))
        if isinstance(node, ast.Compare) and keyish(node.left):
            for op, right in zip(node.ops, node.comparators):
                if isinstance(op, (ast.Eq, ast.NotEq, ast.In, ast.NotIn)) and literal(right):
                    found.append((node.lineno, ast.unparse(node)))
    return found


@functools.lru_cache(maxsize=1)
def scan():
    return {str(p.relative_to(UI)): checks(p.read_text()) for p in sorted(UI.rglob('*.py'))
            if p.name != 'keymap.py'}


def test_the_scan_finds_the_forms():
    assert len(checks("key == 'q'\nraw in ('up', 'k')\nfold(key) != 'x'\nord('a')\nkey == 'press:space'")) == 5
    assert not checks("key == 'fusion_state'\nk != 'schema'\nkey == keymap.STOP_KEY\nname == 'q'")


def test_no_literal_key_checks_outside_the_allowlist():
    over = {path: found for path, found in scan().items() if len(found) > ALLOWED.get(path, 0)}
    assert not over, 'literal key checks (use keymap.matches or keymap.dispatch):\n' + '\n'.join(
        '%s:%d %s' % (path, line, text) for path, found in over.items() for line, text in found)


@pytest.mark.parametrize('path', sorted(ALLOWED))
def test_the_allowlist_only_shrinks(path):
    left = len(scan().get(path, []))
    assert left == ALLOWED[path], '%s has %d literal key checks; set ALLOWED to %d (drop it at 0)' % (
        path, left, left)
pytestmark = pytest.mark.smoke   # tools/smoke

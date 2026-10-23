"""The lifecycle check must list exactly the servers mhseals_nav launches."""

import ast
from pathlib import Path

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parent / 'mhseals_testing'


def assigned_list(path, name):
    """Literal value of a top-level or function-level assignment, read without importing."""
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Assign) and any(getattr(t, 'id', None) == name for t in node.targets):
            return ast.literal_eval(node.value)


def test_servers_match_navigation_launch():
    # fixtures/navigation.launch.py is a copy of mhseals_nav's launch file.
    launched = [executable for _, executable in assigned_list(HERE / 'fixtures' / 'navigation.launch.py', 'servers')]
    assert list(assigned_list(PACKAGE / 'ros' / 'nav2.py', 'SERVERS')) == launched
    assert 'smoother_server' not in launched

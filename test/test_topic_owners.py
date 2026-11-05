"""Each shared topic name is written once, in the module that owns it; everyone else imports it (atlas #32)."""

import ast
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1] / 'mhseals_testing'
OWNERS = {'/command_mux/request': ('ros/plant.py', 'MUX_REQUEST_TOPIC'),
          '/thruster_pwm_node/status': ('ros/arming.py', 'DRIVER_STATUS'),
          '/mhseals_testing/rtk/status': ('ros/rtk.py', 'STATUS_TOPIC')}


def test_a_shared_topic_name_is_written_only_by_its_owner():
    found = {}
    for path in PACKAGE.rglob('*.py'):
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                for name in OWNERS:
                    if name in node.value:
                        found.setdefault(name, []).append(f'{path.relative_to(PACKAGE)}:{node.lineno}')
    for name, (owner, constant) in OWNERS.items():
        assert [where.split(':')[0] for where in found.get(name, [])] == [owner], (name, found.get(name))
        tree = ast.parse((PACKAGE / owner).read_text())
        assert [ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                and [t.id for t in n.targets if isinstance(t, ast.Name)] == [constant]] == [name], owner

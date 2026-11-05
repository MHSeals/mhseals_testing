"""The agent hook reads the device registry with the stdlib only (hook/registry.py)."""

from pathlib import Path

import pytest

from mhseals_testing import agent_hook as hook

BLOCK = '''# a vessel's registry
hosts:
  boat-a: {roles: [boat_computer, time_server], user: roboboat, capabilities: [mavros, thrusters]}
  boat-b:          # block style
    roles:
      - camera_host
    capabilities: [gpu]
  "shore": {roles: [operator], address: $GATEWAY_IP}   # quoted
ping:
- camera_host
'''


def test_the_reader_matches_pyyaml_and_refuses_what_it_cannot_read():
    yaml = pytest.importorskip('yaml')
    package = Path(hook.__file__).resolve().parent.parent / 'config' / 'devices.yaml'
    for text in (BLOCK, package.read_text(), 'hosts: {local: {capabilities: [gpu]}}\n'):
        want = [(k, tuple((v or {}).get('roles', []))) for k, v in (yaml.safe_load(text).get('hosts') or {}).items()]
        assert hook.registry_hosts(text) == want
    for text in ('hosts:\n  a: {roles: [x,\n    y]}\n', 'hosts:\n  a: &x {roles: [x]}\n', 'hosts:\n\ta: {}\n',
                 'hosts: [a]\n', 'hosts:\n  a:\n    roles: x\n', 'hosts:\n  - a\n', 'hosts:\n  a: {}\n  a: {}\n',
                 'hosts:\n  a: {roles: [x]}\n b: {}\n', 'hosts:\n  a: {roles: "x\\"y"}\n'):
        with pytest.raises(ValueError):
            hook.registry_hosts(text)

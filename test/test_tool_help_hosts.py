"""boat_tui and boat_agent name no host: their help fills the TUI host from the vessel registry (tools/registry.sh),
and a vessel's name as BOAT_AGENT_TARGET means the registry's TUI host."""

import os
from pathlib import Path
import subprocess

import pytest

pytestmark = pytest.mark.smoke   # tools/smoke

TOOLS = Path(__file__).resolve().parent.parent / 'tools'


def workspace():
    """The astro_dock workspace whose devices/profile.py the tools read (this checkout's, else its main clone's)."""
    common = subprocess.run(['git', '-C', str(TOOLS), 'rev-parse', '--path-format=absolute', '--git-common-dir'],
                            capture_output=True, text=True).stdout.strip()
    for root in (TOOLS.parent.parent.parent, Path(common).parent.parent.parent if common else None):
        if root and (root / 'devices/profile.py').is_file():
            return root
    pytest.skip('no astro_dock workspace beside this checkout')


def real_hosts(root):
    """Every host named in the workspace's vessel registries (none in a bare checkout)."""
    names = set()
    for registry in (root / 'profiles').glob('*/devices.yaml'):
        names.add(registry.parent.name)
        for line in registry.read_text().splitlines():
            if line.startswith('  ') and not line.startswith('   ') and ':' in line:
                names.add(line.strip().split(':')[0])
    return {n for n in names if n and not n.startswith('#')}


@pytest.fixture
def env(tmp_path):
    vessel = tmp_path / 'vesselq'
    vessel.mkdir()
    (vessel / 'devices.yaml').write_text('hosts:\n  vesselq-brain: {roles: [boat_computer], user: crew}\n'
                                         '  vesselq-eye: {roles: [camera_host], user: crew}\n')
    return dict(os.environ, ASTRO_REGISTRY=str(vessel / 'devices.yaml'), ASTRO_WS=str(workspace()))


def test_boat_tui_help_fills_the_tui_host_from_the_registry_and_names_no_real_host(env):
    out = subprocess.run([str(TOOLS / 'boat_tui'), '--help'], capture_output=True, text=True, env=env).stdout
    assert 'ssh vesselq-brain ' in out and 'TUI_HOST' not in out
    named = {h for h in real_hosts(Path(env['ASTRO_WS'])) if h in out}
    assert not named, named


def test_the_tool_sources_name_no_real_host():
    hosts = real_hosts(workspace())
    for tool in ('boat_tui', 'boat_agent', 'registry.sh'):
        text = (TOOLS / tool).read_text()
        assert not {h for h in hosts if h in text}, tool


def test_a_vessel_name_targets_its_registry_tui_host(env):
    script = f'. {TOOLS}/registry.sh; vessel; tui_host'
    out = subprocess.run(['sh', '-c', script], capture_output=True, text=True, env=env).stdout.split()
    assert out == ['vesselq', 'vesselq-brain']


def test_boat_tui_host_prints_the_registry_tui_host(env):
    out = subprocess.run([str(TOOLS / 'boat_tui'), 'host'], capture_output=True, text=True, env=env)
    assert out.returncode == 0 and out.stdout.strip() == 'vesselq-brain'


def test_ui_and_tools_name_no_registry_host_and_no_ip():
    """Hardware is never hardcoded (D-20261104-03): no workspace registry host or vessel name and no IPv4
    literal (loopback and any-address aside) in the TUI or the tools; config and test fixtures may."""
    import re
    hosts = real_hosts(workspace())
    ip = re.compile(r'\b(?!127\.0\.0\.1\b|0\.0\.0\.0\b)(?:\d{1,3}\.){3}\d{1,3}\b')
    root = TOOLS.parent
    files = [p for p in (root / 'mhseals_testing/ui').rglob('*.py')] + [p for p in TOOLS.iterdir() if p.is_file()]
    found = []
    for path in files:
        text = path.read_text(errors='replace')
        found += [f'{path.relative_to(root)}: {h}' for h in hosts if re.search(rf'(?<![\w-]){re.escape(h)}(?![\w-])', text)]
        found += [f'{path.relative_to(root)}: {m}' for m in ip.findall(text)]
    assert not found, found

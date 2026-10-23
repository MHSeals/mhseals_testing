"""The hold_still command line respects the rig's limits (runs without the rig)."""
from pathlib import Path

import simrig


def test_command_has_no_smoke_gate_and_keeps_domain_in_range(monkeypatch):
    seen = {}
    monkeypatch.setattr(simrig.subprocess, 'run', lambda cmd, **kw: seen.setdefault('cmd', cmd))
    simrig.run_hold_still(Path('/rig'), '/out', domain=210)
    assert '--smoke' not in seen['cmd'] and '--no-drive' in seen['cmd']
    assert seen['cmd'][seen['cmd'].index('--domain') + 1] == '210'


def test_conftest_domain_range():
    import re
    src = (Path(__file__).parent / 'conftest.py').read_text()
    base, mod = map(int, re.search(r'(\d+) \+ uuid\.uuid4\(\)\.int % (\d+)', src).groups())
    assert base >= 200 and base + mod - 1 <= 228

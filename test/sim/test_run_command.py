"""The hold_still command line respects the rig's limits (runs without the rig)."""
import json
from pathlib import Path

import pytest

import simrig


def test_command_has_no_smoke_gate_and_keeps_domain_in_range(monkeypatch, tmp_path):
    seen = {}
    monkeypatch.setattr(simrig.subprocess, 'run', lambda cmd, **kw: seen.setdefault('cmd', cmd))
    simrig.run_hold_still(Path('/rig'), tmp_path / 's', domain=210)
    assert '--smoke' not in seen['cmd'] and '--no-drive' in seen['cmd']
    assert seen['cmd'][seen['cmd'].index('--domain') + 1] == '210'


def test_pick_domain_stays_in_the_network_rule():
    for _ in range(200):
        domain, seed = simrig.pick_domain()
        assert 200 <= domain <= 228 and domain == 200 + seed % 29


def test_rig_run_records_domain_and_seed_even_when_it_fails(monkeypatch, tmp_path):
    def fail(cmd, **kw):
        raise simrig.subprocess.CalledProcessError(1, cmd)
    monkeypatch.setattr(simrig.subprocess, 'run', fail)
    with pytest.raises(simrig.subprocess.CalledProcessError):
        simrig.run_hold_still(Path('/rig'), tmp_path / 's', domain=210, seed=12345)
    assert json.loads((tmp_path / 's' / simrig.RUN_RECORD).read_text()) == {'domain': 210, 'seed': 12345}

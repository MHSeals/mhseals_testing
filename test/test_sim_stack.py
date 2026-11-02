"""tools/boat_sim and boat_tui sim: config/sim/stack.yaml, env overrides, a free sim domain, the dry run."""

import os
from pathlib import Path
import subprocess

import pytest

from mhseals_testing.harness import sim_stack

TOOLS = Path(__file__).resolve().parents[1] / 'tools'


def test_the_shipped_settings_and_env_overrides():
    values = sim_stack.settings(environ={})
    assert values['profile'] == 'crane_full_stack' and values['domains'] == [101, 199]
    assert not values['player'].startswith('~')
    assert 'base' not in values           # built on /opt/ros alone: no stale underlay in the sim or its agent verbs
    assert not any('ctest/base' in (TOOLS / tool).read_text() for tool in ('boat_sim', 'boat_agent'))
    values = sim_stack.settings(environ={'BOAT_SIM_PLAYER': '/opt/CRANE.x86_64', 'BOAT_SIM_DOMAINS': '150 160'})
    assert values['player'] == '/opt/CRANE.x86_64' and values['domains'] == [150, 160]
    assert "SIM_DOMAINS='150 160'" in sim_stack.shell(values)


def test_a_free_domain_skips_held_ones_and_42(tmp_path):
    (tmp_path / 'domain.120').write_text(str(os.getpid()))       # held by a live process
    (tmp_path / 'domain.121').write_text('999999999')            # stale: its pid is gone
    assert sim_stack.held(tmp_path) == {120}
    assert sim_stack.free_domain(120, 121, {120}) == 121
    assert sim_stack.free_domain(42, 42, set()) is None and sim_stack.free_domain(120, 120, {120}) is None


def test_boat_tui_sim_dry_run_names_the_plan_and_starts_nothing(tmp_path):
    env = dict(os.environ, BOAT_SIM_PLAYER='/bin/true', BOAT_SIM_RUN=str(tmp_path / 'run'),
               PYTHONPATH=str(TOOLS.parent))
    done = subprocess.run([str(TOOLS / 'boat_tui'), 'sim', '--seed', '7', '--dry-run'], capture_output=True,
                          text=True, env=env, timeout=30)
    assert done.returncode == 0, done.stderr
    assert 'crane_full_stack' in done.stdout and 'seed 7' in done.stdout and 'tmux -L boat' in done.stdout
    missing = subprocess.run([str(TOOLS / 'boat_sim'), 'up'], capture_output=True, text=True, timeout=30,
                             env=dict(env, BOAT_SIM_PLAYER=str(tmp_path / 'none')))
    assert missing.returncode == 2 and 'no player' in missing.stderr


pytestmark = pytest.mark.smoke   # tools/smoke

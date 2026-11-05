"""tools/boat_sim and boat_tui sim: config/sim/stack.yaml, env overrides, a free sim domain, the dry run."""

import os
from pathlib import Path
import re
import subprocess

import pytest

from mhseals_testing.harness import sim_stack
from mhseals_testing.hook import registry

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


def test_a_free_domain_skips_held_ones(tmp_path):
    (tmp_path / 'domain.120').write_text(str(os.getpid()))       # held by a live process
    (tmp_path / 'domain.121').write_text('999999999')            # stale: its pid is gone
    assert sim_stack.held(tmp_path) == {120}
    assert sim_stack.free_domain(120, 121, {120}) == 121
    assert sim_stack.free_domain(120, 120, {120}) is None


def test_two_domains_give_two_endpoint_ports_and_no_file_pins_one():
    assert sim_stack.endpoint_port(101) == 10101 and sim_stack.endpoint_port('102') == 10102
    assert sim_stack.endpoint_port(101) != sim_stack.endpoint_port(102)    # two agent sims never share a port
    assert 'port' not in sim_stack.settings(environ={})
    for name in ('sim/stack.yaml', 'preview.yaml'):
        assert not re.search(r'\bport:', (TOOLS.parent / 'config' / name).read_text())


def test_the_vessels_domains_come_from_the_profile_set(tmp_path, monkeypatch):
    for name, kind, domain in (('boat_a', 'vessel', 42), ('boat_b', 'vessel', 142), ('station', 'station', 7)):
        (tmp_path / name).mkdir()
        (tmp_path / name / 'profile.yaml').write_text(f'kind: {kind}\n')
        (tmp_path / name / 'network.env').write_text(f'# a comment\nROS_DOMAIN_ID={domain}\n')
    monkeypatch.setenv('ASTRO_PROFILES', str(tmp_path))
    assert registry.boat_domains() == (42, 142)
    monkeypatch.setenv('ASTRO_PROFILES', str(tmp_path / 'none'))
    assert registry.boat_domains() == ()
    assert registry.dev_container()          # the package registry's defaults.container, never a literal in code


def test_boat_tui_sim_dry_run_names_the_plan_and_starts_nothing(tmp_path, ws):
    # the fixture workspace (stub hw.sh, scratch repos): CI's standalone checkout has no astro_dock around it
    env = dict(os.environ, BOAT_SIM_PLAYER='/bin/true', BOAT_SIM_RUN=str(tmp_path / 'run'),
               BOAT_SIM_SRC=str(ws[0] / 'src'), PYTHONPATH=str(TOOLS.parent))
    done = subprocess.run([str(TOOLS / 'boat_tui'), 'sim', '--seed', '7', '--dry-run'], capture_output=True,
                          text=True, env=env, timeout=30)
    assert done.returncode == 0, done.stderr
    assert 'crane_full_stack' in done.stdout and 'seed 7' in done.stdout and 'tmux -L boat' in done.stdout
    missing = subprocess.run([str(TOOLS / 'boat_sim'), 'up'], capture_output=True, text=True, timeout=30,
                             env=dict(env, BOAT_SIM_PLAYER=str(tmp_path / 'none')))
    assert missing.returncode == 2 and 'no player' in missing.stderr
    nowhere = subprocess.run([str(TOOLS / 'boat_tui'), 'sim', '--dry-run'], capture_output=True, text=True,
                             timeout=30, env=dict(env, BOAT_SIM_SRC=str(tmp_path / 'none' / 'src')))
    assert nowhere.returncode != 0 and 'no astro_dock workspace' in nowhere.stderr   # a real user is told, loudly


pytestmark = pytest.mark.smoke   # tools/smoke


def test_the_image_and_player_are_found_on_this_machine_never_written_down(tmp_path):
    shipped = (TOOLS.parent / 'config/sim/stack.yaml').read_text()
    assert 'image: auto' in shipped and 'uid:latest' not in shipped and 'image_match' not in shipped
    values = sim_stack.settings(environ={})
    assert values['image'] == 'auto'                       # resolved by the workspace (hw.sh image), not here
    seen = []
    found = sim_stack.resolve(values, tmp_path, image=lambda root, pull: seen.append((root, pull)) or 'img:x',
                              repos=lambda root, roles: [('repo_a', 'origin/main')])
    assert found['image'] == 'img:x' and seen == [(tmp_path, False)] and found['repos'] == [('repo_a', 'origin/main')]
    assert "SIM_REPOS=repo_a=origin/main" in sim_stack.shell(found)
    cluster = tmp_path / 'players/CRANE-Worker-Linux'
    cluster.mkdir(parents=True)
    (cluster / 'CRANE-Worker-Linux.x86_64').write_text('#!/bin/sh\n')
    (cluster / 'CRANE-Worker-Linux.x86_64').chmod(0o755)
    gone = str(tmp_path / 'phase1/CRANE.x86_64')
    assert sim_stack.first_player([gone, str(cluster)]) == str(cluster / 'CRANE-Worker-Linux.x86_64')
    assert sim_stack.first_player([gone]) == gone                  # nothing here: the message names the first
    values = sim_stack.settings(environ={'BOAT_SIM_PLAYER': f'{gone}:{cluster}'})
    assert values['player'].endswith('CRANE-Worker-Linux.x86_64')

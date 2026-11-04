"""boat_tui preview: settings, the checklist (commits by feature, runbook, checks), its panel, the dry run, the hook."""

import json
import os
from pathlib import Path
import subprocess

import pytest

from mhseals_testing import agent_hook
from mhseals_testing.harness import preview
from mhseals_testing.ui.panels.checklist import Checklist

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
RUNBOOK = """# Field runbook
## Before a boat test
1. Rebuild the dev workspace
   with symlink-install.
2. `./hw.sh station take`.

## Before launch
1. Not this one.
"""


def test_settings_share_the_sim_and_test_has_its_own_everything():
    values = preview.settings(environ={})
    assert values['profile'] == 'crane_full_stack' and values['container'] == 'roboboat_preview'
    assert values['socket'] == 'preview' and not values['cache'].startswith('~') and values['results']
    test = preview.settings(test=True, environ={'BOAT_PREVIEW_PORT': '7000'})
    assert test['container'] != values['container'] and test['socket'] != values['socket']
    assert test['domain'] != values['domain'] and test['results'] == '' and test['port'] == '7000'
    assert 'repos' not in values and values['ref'] == ''    # the workspace manifest (test_workspace_derivation)
    assert not values['runbook'].startswith('{src}') and values['runbook'].startswith(values['src'])
    assert 'PV_BASE' not in preview.shell(values)   # built on /opt/ros alone, no stale underlay
    assert 'PV_CHECKS' not in preview.shell(values)


@pytest.mark.parametrize('test, env, refused', [
    (False, {}, None), (True, {}, None), (False, {'BOAT_PREVIEW_CONTAINER': 'mine'}, None),
    (False, {'BOAT_PREVIEW_DOMAIN': '42'}, "domain 42 is the boat's"),
    (False, {'BOAT_PREVIEW_CONTAINER': 'roboboat_dev'}, 'container roboboat_dev'),
    (True, {'BOAT_PREVIEW_SOCKET': 'boat'}, 'tmux -L boat'),
    (True, {'BOAT_PREVIEW_CONTAINER': 'roboboat_preview'}, 'container shared'),
])
def test_a_preview_never_takes_the_boats_or_the_other_previews(test, env, refused):
    found = preview.refusal(preview.settings(test, environ=env), preview.settings(not test, environ={}))
    assert (found is None) if refused is None else refused in found


def test_slug_and_feature():
    assert preview.slug('origin/main') == 'origin-main' and preview.slug('field/2026-11-02') == 'field-2026-11-02'
    assert preview.feature('nav2_live: refuse a key') == 'nav2_live'
    assert preview.feature('Fusion tab: i turns the camera') == 'fusion'
    assert preview.feature('The hold watchdogs count an auto command') == 'other'
    assert preview.feature('boat_sim up leaves the stop file: x') == 'boat_sim'
    assert preview.feature('Tests for tools/boat_tui and hw.sh', 'testing') == 'boat_tui'


def test_runbook_items_are_the_before_a_test_section():
    items = preview.runbook_items(RUNBOOK)
    assert [i['text'] for i in items] == ['Rebuild the dev workspace with symlink-install.', '`./hw.sh station take`.']


def test_checklist_groups_commits_since_a_date(tmp_path):
    repo = tmp_path / 'ws' / 'src' / 'mhseals_testing'
    git = ['git', '-C', str(repo), '-c', 'user.name=t', '-c', 'user.email=t@t']
    subprocess.run(['git', 'init', '-q', str(repo)], check=True)
    for subject in ('boat_tui: preview', 'boat_tui: attach', 'fence_watchdog alerts'):
        subprocess.run([*git, 'commit', '-q', '--allow-empty', '-m', subject], check=True)
    (tmp_path / 'runbook.md').write_text(RUNBOOK)
    values = dict(preview.settings(environ={}), repos=['mhseals_testing'], since='2000-01-01',
                  runbook=str(tmp_path / 'runbook.md'))
    made = preview.checklist(values, tmp_path / 'ws', 'origin/main')
    changes = [i['text'] for i in made['items'] if i['group'] == 'Changes']
    assert changes == ['fence_watchdog (1)', 'boat_tui (2)']
    groups = [i['group'] for i in made['items']]
    assert groups.count('Before a test') == 2 and groups.count('Checks') == len(values['checks'])


class App:
    notice = ''

    def ask_text(self, title, message):
        return 'slow on the link'


def test_the_panel_marks_and_appends(tmp_path):
    items = [dict(id='a', group='Checks', text='Arming countdown'), dict(id='b', group='Checks', text='Fence')]
    checklist = Checklist(dict(ref='origin/main', items=items), tmp_path)
    app = App()
    assert not checklist.handle(app, 'p')                 # off: p stays the TUI's
    assert checklist.handle(app, 'W') and checklist.active
    checklist.handle(app, 'p')
    checklist.handle(app, 'n')
    assert not checklist.handle(app, 'space')             # Space still stops
    checklist.handle(app, 'escape')
    assert not checklist.active
    text = preview.results_path(tmp_path).read_text()
    assert text.count('## ') == 1 and 'PASS Checks: Arming countdown' in text
    assert 'NOTE Checks: Fence — slow on the link' in text
    again = Checklist(dict(ref='origin/main', items=items), tmp_path)
    again.mark('fail')
    assert preview.results_path(tmp_path).read_text().startswith(text)      # appended, never rewritten


def test_the_panel_comes_only_with_boat_tui_preview(tmp_path):
    assert Checklist.from_env({}) is None
    path = tmp_path / 'checklist.json'
    path.write_text(json.dumps(dict(ref='r', items=[])))
    found = Checklist.from_env({'BOAT_PREVIEW_CHECKLIST': str(path)})
    assert found is not None and found.results == tmp_path and 'W' in found.toggles


def test_boat_tui_preview_dry_run_and_test_needs_headless(tmp_path):
    env = dict(os.environ, BOAT_PREVIEW_PLAYER='/bin/true', BOAT_PREVIEW_CACHE=str(tmp_path),
               PYTHONPATH=str(TOOLS.parent))
    done = subprocess.run([str(TOOLS / 'boat_tui'), 'preview', 'field/x', '--dry-run'], capture_output=True,
                          text=True, env=env, timeout=30)
    assert done.returncode == 0, done.stderr
    assert 'heavy preview' in done.stdout and 'field-x' in done.stdout and 'windowed' in done.stdout
    test = subprocess.run([str(TOOLS / 'boat_tui'), 'preview', '--test', '--dry-run'], capture_output=True,
                          text=True, env=env, timeout=30)
    assert test.returncode == 2 and '--headless' in test.stderr


@pytest.mark.parametrize('command, denied', [
    ('tools/boat_tui preview', True), ('boat_tui preview origin/main --headless', True),
    ('boat_tui preview down', True), ('tools/boat_preview up', True), ('tmux -L preview send-keys q', True),
    ('docker stop roboboat_preview', True), ('boat_tui preview status', False),
    ('boat_tui preview --headless --test', False), ('boat_tui preview down --test', False),
    ('tmux -L preview capture-pane -p', False), ('grep boat_tui preview README.md', False)])
def test_the_hook_keeps_agents_off_the_owners_preview(command, denied):
    config = agent_hook.hook_config()
    assert bool(agent_hook.denied(command, config)) is denied



@pytest.mark.parametrize('width, active, shown', [(150, False, True), (100, False, False), (100, True, True)])
def test_the_layout_shows_the_checklist_beside_wide_bodies_and_over_narrow_ones_while_checking(width, active, shown):
    import io
    from rich.console import Console
    from mhseals_testing.profiles import load_profile
    from mhseals_testing.ui.layout import render
    from test_dashboard import monitor, view
    checklist = Checklist(dict(ref='r', items=[dict(id='a', group='Checks', text='Arming countdown')]))
    checklist.active = active
    console = Console(width=width, height=40, file=io.StringIO(), force_terminal=False)
    console.print(render(monitor().snapshot(), view(load_profile('boat'), checklist=checklist), width, 40))
    out = console.file.getvalue()
    assert ('Arming countdown' in out) is shown and ('Thrusters' in out) is (width > 120 or not active)


pytestmark = pytest.mark.smoke   # tools/smoke

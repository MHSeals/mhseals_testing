"""boat_tui preview and sim take the workspace's own sources of truth (docs/preview.md): the repos and commits of its
manifest (hw.sh hosts repos, astro_dock's submodule pins), its dev image (hw.sh image) and its build (hw.sh build).
Fixture workspaces (a stub hw.sh, scratch git repos) check the resolution; the real workspace, when this checkout
sits in one, must resolve as hw.sh and git say. No container, image pull or build runs."""

import os
from pathlib import Path
import re
import subprocess

import pytest

from mhseals_testing.harness import preview, sim_stack

pytestmark = pytest.mark.smoke   # tools/smoke

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
GIT_ENV = {**os.environ, 'GIT_CONFIG_GLOBAL': os.devnull, 'GIT_CONFIG_NOSYSTEM': '1', 'GIT_AUTHOR_NAME': 't',
           'GIT_AUTHOR_EMAIL': 't@t', 'GIT_COMMITTER_NAME': 't', 'GIT_COMMITTER_EMAIL': 't@t',
           'GIT_ALLOW_PROTOCOL': 'file', 'GIT_PROTOCOL_FROM_USER': '1'}


def git(repo, *args):
    return subprocess.run(['git', '-C', str(repo), *args], check=True, capture_output=True, text=True,
                          env=GIT_ENV).stdout.strip()


def repo(path, *commits):
    path.mkdir(parents=True)
    git(path, 'init', '-q', '-b', 'main')
    for name in commits or ('base',):
        (path / name).write_text(name)
        git(path, 'add', '-A')
        git(path, 'commit', '-qm', name)
    return git(path, 'rev-parse', 'HEAD')


@pytest.fixture
def ws(tmp_path):
    """A workspace: a stub hw.sh (hosts repos per --role, image), team repos with origin/main, a pinned submodule."""
    root = tmp_path / 'astro'
    root.mkdir()
    (root / 'hw.sh').write_text('''#!/bin/sh
case "$1 $2" in
  "hosts repos") echo "$*" > "$(dirname "$0")/repos.args"
                 echo "team_a https://x/team_a.git main"; echo "team_b https://x/team_b.git field" ;;
  "image --local") echo "img:built-on-tag" ;;
  *) echo "stub: $*" >&2; exit 1 ;;
esac
''')
    (root / 'hw.sh').chmod(0o755)
    git(root, 'init', '-q', '-b', 'main')
    for name, branch in (('team_a', 'main'), ('team_b', 'field')):
        repo(root / 'src' / name, 'one')
        git(root / 'src' / name, 'update-ref', f'refs/remotes/origin/{branch}', 'HEAD')
    upstream = tmp_path / 'upstream'
    pin = repo(upstream, 'ros2')
    git(upstream, 'checkout', '-q', '--orphan', 'ros1')
    (upstream / 'ros1').write_text('ros1')
    git(upstream, 'add', '-A')
    git(upstream, 'commit', '-qm', 'ros1')                            # the upstream's own unrelated history
    git(upstream, 'checkout', '-q', 'main')
    subprocess.run(['git', '-C', str(root), '-c', 'protocol.file.allow=always', 'submodule', 'add', '-q',
                    str(upstream), 'src/pinned'], check=True, capture_output=True, env=GIT_ENV)
    git(root, 'commit', '-qm', 'pin')
    git(root / 'src/pinned', 'fetch', '-q', 'origin', 'ros1:refs/remotes/origin/ros1')
    return root, pin


def test_the_manifest_is_hosts_repos_of_the_roles_and_the_submodule_pins(ws):
    root, pin = ws
    found = sim_stack.manifest(root, ['boat_computer'])
    assert found == [('team_a', 'origin/main'), ('team_b', 'origin/field'), ('pinned', pin)]
    assert (root / 'repos.args').read_text().split() == ['hosts', 'repos', '--role', 'boat_computer']


def test_the_plan_takes_an_explicit_ref_only_where_it_applies(ws):
    root, pin = ws
    repos = sim_stack.manifest(root, [])
    src = root / 'src'
    heads = {name: git(src / name, 'rev-parse', 'HEAD') for name in ('team_a', 'team_b')}
    assert dict(preview.plan('', repos, src)) == {**heads, 'pinned': pin}
    git(src / 'team_a', 'checkout', '-q', '-b', 'field/x')
    (src / 'team_a/new').write_text('new')
    git(src / 'team_a', 'add', '-A')
    git(src / 'team_a', 'commit', '-qm', 'new')
    planned = dict(preview.plan('field/x', repos, src))
    assert planned['team_a'] == git(src / 'team_a', 'rev-parse', 'HEAD') and planned['team_b'] == heads['team_b']
    assert dict(preview.plan('origin/ros1', repos, src))['pinned'] == pin   # never an upstream's unrelated branch


def test_missing_pieces_name_the_command_that_fixes_them(ws, tmp_path):
    root, _ = ws
    with pytest.raises(sim_stack.Missing, match='setup.linux.sh --components repos'):
        os.rename(root / 'src/team_b', tmp_path / 'away')
        sim_stack.manifest(root, [])
    with pytest.raises(sim_stack.Missing, match='hw.sh profile sync'):
        (root / 'hw.sh').write_text('#!/bin/sh\necho "No profiles" >&2; exit 1\n')
        sim_stack.manifest(root, [])
    with pytest.raises(sim_stack.Missing, match='BOAT_SIM_SRC'):
        sim_stack.workspace(tmp_path / 'nowhere/src')
    with pytest.raises(sim_stack.Missing, match='no dev image: run prebuild'):
        sim_stack.dev_image(root, run=lambda *a, **k: subprocess.CompletedProcess(a, 2, '', 'run prebuild.sh\n'))


def test_the_image_is_hw_sh_image_and_the_build_is_hw_sh_build(ws):
    root, _ = ws
    assert sim_stack.dev_image(root) == 'img:built-on-tag'
    for tool in ('boat_preview', 'boat_sim'):
        text = (TOOLS / tool).read_text()
        assert '/hw.sh" build --ws' in text and 'colcon build' not in text, tool
        assert '--resolve' in text, tool                      # image and repos from the workspace


def test_the_preview_resolves_as_the_real_workspace_says():
    """This checkout's workspace: the preview's repos are hw.sh hosts repos and git's gitlinks, its image hw.sh's."""
    src = sim_stack.src_dir({})
    try:
        root = sim_stack.workspace(src)
    except sim_stack.Missing:
        pytest.skip('no astro_dock workspace around this checkout')
    roles = preview.settings(environ={})['roles']
    listed = subprocess.run([str(root / 'hw.sh'), 'hosts', 'repos', *[a for r in roles for a in ('--role', r)]],
                            capture_output=True, text=True, timeout=60)
    if listed.returncode:
        pytest.skip('no profiles in this workspace (hw.sh profile sync)')
    pins = re.findall(r'^160000 commit (\w+)\tsrc/(\S+)$', git(root, 'ls-tree', 'HEAD', 'src/'), re.M)
    want = [(line.split()[0], 'origin/' + line.split()[2]) for line in listed.stdout.splitlines()]
    want += [(Path(path).name, sha) for sha, path in pins]
    try:
        assert sim_stack.manifest(root, roles) == want
    except sim_stack.Missing as error:
        pytest.skip(f'the workspace lacks a manifest repo: {error}')

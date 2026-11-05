"""A fixture astro_dock workspace (a stub hw.sh, scratch team repos, a pinned submodule), shared by the
derivation tests and the preview and sim dry runs, so they run in CI's standalone checkout as they do here."""

import os
import subprocess

import pytest

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

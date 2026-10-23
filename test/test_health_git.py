"""git probe, git_state kind and header.deploy over real throwaway repos."""

import importlib
import subprocess
from pathlib import Path

import pytest

from mhseals_testing.health.config import load, lint
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.kinds import git_state
from mhseals_testing.health.probes.git import GitProbe, from_specs, parse_status
from mhseals_testing.ui import theme
from mhseals_testing.ui.deploy import deploy_text

ENV = ['-c', 'user.name=t', '-c', 'user.email=t@t', '-c', 'commit.gpgsign=false']


def git(path, *args):
    subprocess.run(['git', *ENV, '-C', str(path), *args], check=True, capture_output=True)


def commit(path, name):
    (Path(path) / name).write_text(name)
    git(path, 'add', name)
    git(path, 'commit', '-m', name)


@pytest.fixture
def repos(tmp_path):
    """origin plus a clone; ``clone`` starts matching origin."""
    origin = tmp_path / 'origin'
    git(tmp_path, 'init', '-b', 'main', str(origin))
    commit(origin, 'a')
    git(tmp_path, 'clone', str(origin), str(tmp_path / 'clone'))
    return origin, tmp_path / 'clone'


def state(clone, **kw):
    return GitProbe({'r': clone}, **kw).sample()['git.r'].value


def test_parse_status_counts():
    text = ('# branch.oid 1234567890\n# branch.head main\n# branch.upstream origin/main\n'
            '# branch.ab +2 -3\n1 .M N... 100644 100644 100644 a b x\n? new\n? new2\n')
    assert parse_status(text) == dict(branch='main', upstream='origin/main', ahead=2, behind=3,
                                      dirty=1, untracked=2, hash='1234567')


def test_clean_clone_matches_origin(repos):
    value = state(repos[1])
    assert (value['ahead'], value['behind'], value['dirty'], value['untracked']) == (0, 0, 0, 0)
    assert value['branch'] == 'main' and value['fetch_age'] is None   # never fetched


def test_ahead_dirty_untracked_and_behind_after_fetch(repos):
    origin, clone = repos
    commit(clone, 'local')
    (clone / 'a').write_text('changed')
    (clone / 'scratch').write_text('x')
    value = state(clone)
    assert (value['ahead'], value['dirty'], value['untracked']) == (1, 1, 1)
    commit(origin, 'remote')
    assert state(clone)['behind'] == 0                       # fetch off: origin unseen
    value = GitProbe({'r': clone}, fetch=1e-9).sample()['git.r'].value
    assert (value['ahead'], value['behind']) == (1, 1)


def test_unreadable_repo_is_an_error_fact(tmp_path):
    assert 'error' in state(tmp_path / 'missing')


def test_fetch_failure_does_not_fail_the_sample(repos):
    origin, clone = repos
    git(clone, 'remote', 'set-url', 'origin', str(origin) + '-gone')
    assert state(clone, fetch=1e-9)['behind'] == 0


def result(clone, **params):
    spec_params = git_state.Params('r', str(clone), **params)
    return git_state.evaluate(spec_params, GitProbe({'r': clone}).sample())


def test_kind_bands(repos):
    origin, clone = repos
    assert result(clone).status == 'ok'
    (clone / 'new').write_text('x')
    assert result(clone).status == 'warn'
    assert result(clone, untracked={'info': 1}).status == 'info'
    (clone / 'new').unlink()
    commit(clone, 'local')
    assert result(clone).status == 'info'                    # ahead is info by default
    assert result(clone, ahead={'fail': 1}).status == 'fail'


def test_diverged_is_warn_with_flag(repos):
    origin, clone = repos
    commit(origin, 'remote')
    commit(clone, 'local')
    git(clone, 'fetch', '-q')
    got = result(clone)
    assert got.status == 'warn' and got.data['diverged']


def test_kind_unknown_without_fact():
    got = git_state.evaluate(git_state.Params('r', '/x'), {})
    assert got.status == 'unknown'


@pytest.fixture
def shipped_kinds():
    """Register the shipped kinds even if an earlier test patched the registry."""
    from mhseals_testing.health import registry
    from mhseals_testing.health.kinds import threshold
    if 'threshold' not in registry.KINDS:
        importlib.reload(threshold)
    if 'git_state' not in registry.KINDS:
        importlib.reload(git_state)


def test_shipped_config_lints_and_skips_without_probe(shipped_kinds):
    config = load(user=False)
    specs = [s for s in config.specs if s.kind == 'git_state']
    assert {s.id for s in specs} == {'git.astro_dock', 'git.hardware', 'git.nav', 'git.testing'}
    assert lint(specs) == []
    assert config.probes['git'] == {'period': 30, 'fetch': 0}
    assert not Engine(specs).tick({}, {}).results            # missing: skip, hidden
    probe = from_specs(specs, config.probes['git'])
    assert set(probe.repos) == {'astro_dock', 'hardware', 'nav', 'testing'}


def test_deploy_text_glyphs(monkeypatch):
    clean = dict(ahead=0, behind=0, dirty=0, untracked=0, diverged=False, fetch_age=None)
    assert deploy_text([clean, clean]).plain == '≡'
    assert deploy_text([]) is None
    busy = [dict(clean, ahead=1, dirty=2), dict(clean, behind=3, untracked=4, fetch_age=7300)]
    assert deploy_text(busy).plain == '↑1 ↓3 ±2 ?4 ↻2h'
    diverged = deploy_text([dict(clean, ahead=1, behind=1, diverged=True)])
    assert diverged.plain == '↕ ↑1 ↓1' and diverged.spans[0].style == theme.BAD
    monkeypatch.setattr(theme, 'ASCII', True)
    assert deploy_text(busy).plain == '^1 v3 *2 ?4 ~2h'
    assert deploy_text([clean]).plain == '='

"""model/deploy.py: the header's deploy item as data; ui/deploy.render draws it exactly as deploy_text did."""

import pytest

from mhseals_testing.model import deploy

pytestmark = pytest.mark.smoke


def test_summary_folds_the_repos():
    clean = {'ahead': 0, 'behind': 0, 'dirty': 0, 'untracked': 0, 'fetch_age': 10}
    busy = [dict(clean, ahead=1, fetch_age=7200), dict(clean, behind=3, dirty=2, untracked=4, diverged=True)]
    assert deploy.summary([]) is None and deploy.summary([None, {}]) is None
    state = deploy.summary(busy, code_newer=1)
    assert state == deploy.Deploy({'ahead': 1, 'behind': 3, 'dirty': 2, 'untracked': 4}, True, 7200, True)
    assert deploy.summary([clean]).stale_age is None
    assert deploy.summary([clean], stale=5).stale_age == 10


@pytest.mark.parametrize('seconds, text', [(0, '0s'), (59, '59s'), (60, '1m'), (7200, '2h'), (90000, '1d')])
def test_age(seconds, text):
    assert deploy.age(seconds) == text


def test_render_is_deploy_text():
    from mhseals_testing.ui import deploy as ui_deploy
    states = [{'ahead': 2, 'fetch_age': 4000, 'diverged': True}]
    assert ui_deploy.render(deploy.summary(states, code_newer=True)).plain == \
        ui_deploy.deploy_text(states, code_newer=True).plain
    assert ui_deploy.render(None) is None and ui_deploy.STALE == deploy.STALE == 3600.0

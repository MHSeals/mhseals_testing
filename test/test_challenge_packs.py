"""Every shipped challenge pack loads; typos and unknown kinds fail at load."""
import os
from pathlib import Path

import pytest
import yaml

from mhseals_testing.referee import pack

NAV_TASKS = Path(os.environ.get('MHSEALS_NAV', Path(__file__).resolve().parents[2] / 'mhseals_nav')) / 'config' / 'tasks'


def test_every_challenge_file_loads_with_a_kind_that_exists():
    files = sorted(pack.CHALLENGE_DIR.glob('*/*.yaml'))
    specs = [pack.load_spec(f) for f in files if f.name != 'pack.yaml']
    assert specs and all(s.kind in pack.kinds() for s in specs)


def bad(tmp_path, **changes):
    data = dict(schema='mhseals.challenge/1', name='x', kind='gates', judge={})
    path = tmp_path / 'x.yaml'
    path.write_text(yaml.safe_dump({**data, **changes}))
    return path


def test_unknown_kind_lists_the_available_ones(tmp_path):
    with pytest.raises(ValueError, match='available: .*gates'):
        pack.load_spec(bad(tmp_path, kind='tiddlywinks'))


def test_a_misspelt_judge_key_is_refused(tmp_path):
    with pytest.raises(ValueError, match='unknown judge keys'):
        pack.load_spec(bad(tmp_path, judge=dict(max_widht=4)))


def nav_view(spec):
    """The facts of a challenge that the nav task file also states, under the nav file's key names."""
    p = spec.params
    gate = p.get('gate', p)
    view = {k: gate[k] for k in ('red', 'green') if k in gate}
    if 'max_width' in gate:
        view['max_gate_width'] = gate['max_width']
    if 'mark' in p:
        view['mark'] = p['mark']
    if 'count' in p:
        view['count'] = p['count']['label']
    if 'banner' in p:
        view['banner'] = p['banner']
    return view


CHALLENGES = [(f.parent.name, pack.load_spec(f)) for f in sorted(pack.CHALLENGE_DIR.glob('*/*.yaml'))
              if f.name != 'pack.yaml']
CHALLENGES = [(n, s) for n, s in CHALLENGES if s.tree]


@pytest.mark.parametrize('name,spec', CHALLENGES, ids=[s.name for _, s in CHALLENGES])
def test_labels_and_widths_match_the_nav_task_files(name, spec):
    nav = NAV_TASKS / f'{spec.tree}.yaml'
    if not nav.exists():
        pytest.skip('mhseals_nav is not beside this checkout')
    task = yaml.safe_load(nav.read_text())
    if spec.name == 'return_home':  # the nav task names one marker for both sides of the gate
        task = {**task, 'red': task['marker'], 'green': task['marker']}
    view = nav_view(spec)
    assert view, f'{spec.name} states nothing to compare'
    assert view == {k: task[k] for k in view}

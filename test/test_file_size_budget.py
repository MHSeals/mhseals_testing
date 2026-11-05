"""Size budgets (CONVENTIONS "Modular, abstracted, configurable"): a file of about 500 lines, a feature in about 4 places.

Files: every module of the package stays at or below max(500, its ceiling in CEILINGS). CEILINGS holds the files
over 500 lines when this test landed (11-04), at their size then; a new file gets 500. A file that shrinks needs no
edit here: the test only fails on growth past the ceiling, and lists every offender. The ratchet commit lowers the
ceilings once the refactor areas have landed; a ceiling only ever goes down. Split a file that would grow past it.

Features: the modules of one feature (its name is a path segment or a module stem, e.g. ``rtk``) sit in at most 4
places; a package named for the feature counts as one place (``ui/feeds/agent_desk/``). SPREAD allows today's
wider ones at their count, which may only go down.
"""

import re
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[1] / 'mhseals_testing'
LIMIT = 500
FEATURE_LIMIT = 4
CEILINGS = {   # path under mhseals_testing/ -> its lines on 11-04 (only ever lowered; agent/cli.py: d591fd73's 729)
    'agent_hook.py': 1290, 'analysis/score.py': 964, 'ui/imaging/__init__.py': 886, 'ntrip.py': 778,
    'nav2/live.py': 773, 'harness/components.py': 735, 'agent/cli.py': 729,
    'fusion/campaign.py': 667, 'ui/panels/conditions.py': 661, 'rtk.py': 660, 'ros/arm_link.py': 649,
    'procedures/physics.py': 647, 'harness/score_analyze.py': 560,
    'conditions/estimator.py': 551, 'fusion/ros_feed.py': 518,
}
# feature -> the path segments or module stems that are it (matched at the start of a segment)
FEATURES = {
    'rtk': r'rtk|ntrip', 'bags': r'bags?\b|bagread|bag_', 'labels': r'labels?\b', 'annotate': r'annotat',
    'conditions': r'conditions?\b', 'fusion': r'fusion', 'perception': r'perception', 'authority': r'authority',
    'fcu': r'fcu\b', 'minimap': r'minimap', 'camera': r'camera', 'agent_desk': r'agent_desk',
    'calibration': r'calibrat|magcal', 'thrusters': r'thruster', 'nav2': r'nav2', 'holds': r'holds?\b|hold_',
}
SPREAD = {'rtk': 8, 'bags': 5, 'authority': 6}   # feature -> places today (only ever lowered; RTK: area 10's ui/rtk/)


def modules():
    return {p.relative_to(PACKAGE).as_posix(): p for p in PACKAGE.rglob('*.py') if '__pycache__' not in p.parts}


def lines(path):
    return len(path.read_text(encoding='utf-8').splitlines())


def places(pattern):
    """The places a feature's modules sit: each module, or the package named for it, once."""
    found = set()
    for rel in modules():
        parts = rel[:-3].split('/')
        for i, segment in enumerate(parts):
            if re.match(f'(?:{pattern})', segment):
                found.add('/'.join(parts[:i + 1]) + ('/' if i < len(parts) - 1 else '.py'))
                break
    return found


@pytest.mark.smoke
def test_no_file_grows_past_its_budget():
    over = {rel: (n, max(LIMIT, CEILINGS.get(rel, LIMIT))) for rel, path in modules().items()
            if (n := lines(path)) > max(LIMIT, CEILINGS.get(rel, LIMIT))}
    assert not over, 'files past their line budget (split them; CEILINGS only goes down):\n' + '\n'.join(
        f'  {rel}: {n} lines > {budget}' for rel, (n, budget) in sorted(over.items()))


@pytest.mark.smoke
def test_ceilings_name_real_files_over_the_limit():
    """A ceiling names a file that exists and is over 500; one that shrank to 500 or less can leave the table."""
    present = modules()
    assert not [rel for rel in CEILINGS if rel not in present], 'CEILINGS names files that moved or went away'
    assert all(ceiling > LIMIT for ceiling in CEILINGS.values())


@pytest.mark.smoke
@pytest.mark.parametrize('feature', sorted(FEATURES))
def test_a_feature_sits_in_few_places(feature):
    found = places(FEATURES[feature])
    budget = SPREAD.get(feature, FEATURE_LIMIT)
    assert len(found) <= budget, f'{feature} sits in {len(found)} places > {budget}: {sorted(found)}'


def test_the_spread_allowlist_only_names_wide_features():
    """An allowed spread is above the limit and is the count today or more: lower it as a feature gathers."""
    for feature, allowed in SPREAD.items():
        assert allowed > FEATURE_LIMIT and feature in FEATURES
        assert len(places(FEATURES[feature])) <= allowed

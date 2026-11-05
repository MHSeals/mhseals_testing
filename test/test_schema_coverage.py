"""Every key of every shipped config file is described in its schema fragment, and every entry names a shipped key.

UNCOVERED lists the files not yet fully described; it only shrinks: a file in it that has become fully described
fails here until it is removed, and the list may not grow past its size. A fully described file's fragment is
closed (``<file>: {closed: true}``), so an unknown key in it is a load error, not only a test failure.
"""
from pathlib import Path

import pytest

from mhseals_testing import schema
from mhseals_testing.strict_yaml import load_strict

CONFIG = Path(__file__).resolve().parents[1] / 'config'
# subdirectories by pattern: one fragment would describe every file the pattern matches (none yet)
PATTERNS = ('*.yaml', 'profiles/*.yaml', 'courses/*.yaml', 'missions/*.yaml')
UNCOVERED = {
    'agent.yaml', 'annotate.yaml', 'backend.yaml', 'bags.yaml', 'conditions.yaml', 'correction.yaml',
    'devices.yaml', 'fastsim_replay.yaml', 'fastsim_roboboat.yaml', 'fusion_calib.yaml', 'fusion_campaign.yaml',
    'input.yaml', 'keymap.yaml', 'minimap.yaml', 'nav2_classes.yaml', 'ntrip.yaml', 'physics.yaml',
    'preview.yaml', 'reload.yaml', 'theme.yaml',
    'profiles/*.yaml', 'courses/*.yaml', 'missions/*.yaml',
}
CEILING = 23      # len(UNCOVERED) when it was written; lower it as files are covered, never raise it


def shipped():
    for pattern in PATTERNS:
        for path in sorted(CONFIG.glob(pattern)):
            if path.name != 'schema.yaml':
                yield path, (pattern if '/' in pattern else path.name)


def test_the_uncovered_list_only_shrinks():
    assert len(UNCOVERED) <= CEILING
    assert UNCOVERED <= {label for _, label in shipped()}, 'an allowlisted file is gone: drop it'


@pytest.mark.smoke
@pytest.mark.parametrize('path, label', list(shipped()), ids=[str(p.relative_to(CONFIG)) for p, _ in shipped()])
def test_every_key_is_described_and_every_entry_is_shipped(path, label):
    doc = load_strict(path.read_text())
    uncovered, unused = schema.coverage(path, doc)
    assert unused == [], f'{path.name}: entries naming no shipped key: mark them optional: true or drop them'
    if label in UNCOVERED:
        assert uncovered or label.endswith('/*.yaml') and any(
            schema.coverage(p, load_strict(p.read_text()))[0] for p, other in shipped() if other == label), \
            f'{label} is fully described now: remove it from UNCOVERED and lower CEILING'
        return
    assert uncovered == []
    assert schema.closed(path), f'{path.name}: fully described: close its fragment ({path.stem}: {{closed: true}})'

import os
from pathlib import Path

import pytest
import yaml

from mhseals_testing.analysis.classes import ClassTable, load_classes

CONFIG = 'config/custom_object_detection.yaml'


def find_detector(env=None, sibling=None):
    """Locate the detector YAML: $MHSEALS_NAV, then the ament share dir, then a sibling checkout."""
    env = os.environ if env is None else env
    sibling = sibling or Path(__file__).resolve().parents[2] / 'mhseals_nav'
    candidates = []
    if env.get('MHSEALS_NAV'):
        candidates.append(Path(env['MHSEALS_NAV']))
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory('mhseals_nav')))
    except Exception:
        pass
    candidates.append(sibling)
    return next((c / CONFIG for c in candidates if (c / CONFIG).exists()), sibling / CONFIG)


DETECTOR = find_detector()
TRUTH_LABELS = ['red_pole_buoy', 'red_buoy', 'green_pole_buoy', 'green_buoy',
                'yellow_buoy', 'black_buoy', 'blue_buoy']


def test_thirteen_classes_with_unique_glyphs_letters_and_pairs():
    classes = list(load_classes())
    assert len(classes) == 13
    for field in ('glyph', 'ascii'):
        values = [getattr(c, field) for c in classes]
        assert len(set(values)) == len(values)
    assert len({(c.glyph, c.ascii) for c in classes}) == 13


def test_unknown_fallback_is_distinct_and_used_for_unlisted_labels():
    table = load_classes()
    assert table.symbol('never_heard_of_it') == table.unknown
    assert table.unknown.glyph == '?' and table.unknown.ascii == '?'
    assert table.unknown not in list(table)


def test_truth_labels_resolve_to_their_own_symbol():
    table = load_classes()
    for label in TRUTH_LABELS:
        assert label in table and table.symbol(label).label == label


@pytest.mark.skipif(not DETECTOR.exists(), reason='mhseals_nav not checked out beside')
def test_every_detector_label_has_a_class():
    table = load_classes()
    params = yaml.safe_load(DETECTOR.read_text())
    labels = {entry['label'] for entry in _walk(params) if 'label' in entry}
    assert labels and all(label in table for label in labels)


def _walk(node):
    if isinstance(node, dict):
        yield node
        for value in node.values():
            yield from _walk(value)


def test_duplicate_glyph_or_ascii_is_rejected():
    entry = dict(glyph='A', ascii='a', colour='obj.red', radius=1, name='a')
    unknown = dict(glyph='?', ascii='?', colour='obj.unknown', radius=1, name='u')
    with pytest.raises(ValueError):
        ClassTable({'one': entry, 'two': dict(entry, ascii='b')}, unknown)
    with pytest.raises(ValueError):
        ClassTable({'one': entry, 'two': dict(entry, glyph='B')}, unknown)


def test_find_detector_prefers_env_over_sibling(tmp_path):
    (tmp_path / 'config').mkdir()
    (tmp_path / CONFIG).write_text('labels: []\n')
    assert find_detector({'MHSEALS_NAV': str(tmp_path)}, tmp_path / 'missing') == tmp_path / CONFIG


def _table(**changes):
    entry = {'glyph': 'o', 'ascii': 'a', 'colour': 'red', 'radius': 1.0, 'name': 'A'}
    return ClassTable({'a': {**entry, **changes}}, {**entry, 'glyph': '?', 'ascii': '?'})


@pytest.mark.parametrize('changes', [
    {'ascii': True}, {'glyph': False}, {'ascii': 'ab'}, {'ascii': '\u00e9'}, {'ascii': ''},
    {'glyph': 'AB'}, {'glyph': '\U0001f534'}, {'glyph': '\uff21'}, {'radius': 0}, {'radius': -1}])
def test_malformed_class_entries_are_rejected(changes):
    with pytest.raises(ValueError):
        _table(**changes)


def test_well_formed_entry_is_accepted():
    assert _table().symbol('a').ascii == 'a'

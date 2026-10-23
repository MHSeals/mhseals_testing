from pathlib import Path

import pytest
import yaml

from mhseals_testing.analysis.classes import ClassTable, load_classes

DETECTOR = Path(__file__).resolve().parents[2] / 'mhseals_nav/config/custom_object_detection.yaml'
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

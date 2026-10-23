import pytest
import yaml

from conftest import find_nav
from mhseals_testing.analysis.classes import ClassTable, load_classes

CONFIG = 'config/custom_object_detection.yaml'


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


def test_every_detector_label_has_a_class(nav_root):
    table = load_classes()
    params = yaml.safe_load((nav_root / CONFIG).read_text())
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


def test_find_nav_prefers_env_then_plain_then_integ_sibling(tmp_path):
    env, plain, integ = (tmp_path / n for n in ('env', 'mhseals_nav', 'integ-mhseals_nav'))
    integ.mkdir()
    assert find_nav({}, tmp_path) == integ
    plain.mkdir()
    assert find_nav({}, tmp_path) == plain
    env.mkdir()
    assert find_nav({'MHSEALS_NAV': str(env)}, tmp_path) == env


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


GOOD = {'glyph': 'a', 'ascii': 'A', 'colour': 'red', 'radius': 1, 'name': 'a'}


@pytest.mark.parametrize('text, needle', [
    ('', 'expected a mapping'),
    ('classes: {x: {glyph: a}}\n', 'missing unknown'),
    ('classes: {x: {glyph: a}}\nunknown: {glyph: u}\n', "class 'x' is missing ascii"),
    ('classes: {x: {glyph: a, ascii: A, colour: c, name: n}}\nunknown: {}\n', 'missing unknown'),
    ('classes: {x: {glyph: a, ascii: A, colour: c, name: n}}\n'
     'unknown: {glyph: u, ascii: U, colour: c, radius: 1, name: n}\n', "'x' is missing radius"),
    ('classes:\n  x: {glyph: a}\n  x: {glyph: b}\nunknown: {glyph: u}\n', "duplicate key 'x'"),
])
def test_malformed_file_raises_one_value_error_naming_path(tmp_path, text, needle):
    path = tmp_path / 'minimap.yaml'
    path.write_text(text)
    with pytest.raises(ValueError) as error:
        load_classes(path)
    assert str(path) in str(error.value) and needle in str(error.value)

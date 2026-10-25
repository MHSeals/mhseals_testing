from mhseals_testing.strict_yaml import load_strict

import pytest

AMBIGUOUS = ['a: 1\na: 2\n', 'a: &x 1\nb: *x\n', 'a: {b: 1}\nc:\n  <<: {b: 2}\n', '1: a\n']


@pytest.mark.parametrize('text', AMBIGUOUS)
def test_ambiguous_documents_are_refused(text):
    with pytest.raises(ValueError, match='line|aliases'):
        load_strict(text)


def test_plain_document_loads():
    assert load_strict('a: [1, {b: x}]\n') == {'a': [1, {'b': 'x'}]}


@pytest.mark.parametrize('text', ['a: .inf\n', 'a: [1, .nan]\n', 'a: -.inf\n'])
def test_non_finite_numbers_are_refused(text):
    with pytest.raises(ValueError, match='non-finite'):
        load_strict(text)


def test_errors_name_the_file_and_line():
    with pytest.raises(ValueError, match=r'x\.yaml, line 3'):
        load_strict('a: 1\nb: 2\na: 3\n', 'x.yaml')


def test_syntax_errors_name_the_file():
    with pytest.raises(Exception, match=r'x\.yaml') as raised:
        load_strict('a: [1,\n', 'x.yaml')
    assert 'unicode string' not in str(raised.value)


def test_operator_overlay_with_a_duplicate_key_is_ignored(tmp_path):
    from mhseals_testing.ui import palette
    overlay = tmp_path / 'theme.yaml'
    overlay.write_text('theme: dark\ntheme: light\n')
    theme, warning = palette.load_theme(overlay=overlay)
    assert warning and 'line 2' in warning

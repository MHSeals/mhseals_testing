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

from mhseals_testing.strict_yaml import load_strict

import pytest

AMBIGUOUS = ['a: 1\na: 2\n', 'a: &x 1\nb: *x\n', 'a: {b: 1}\nc:\n  <<: {b: 2}\n', '1: a\n']


@pytest.mark.parametrize('text', AMBIGUOUS)
def test_ambiguous_documents_are_refused(text):
    with pytest.raises(ValueError, match='line|aliases'):
        load_strict(text)


def test_plain_document_loads():
    assert load_strict('a: [1, {b: x}]\n') == {'a': [1, {'b': 'x'}]}

"""The authority chip's ages read through theme.age's chip style (audit finding 12), with the same text."""
import inspect

import pytest

from mhseals_testing.ui import authority_chip, theme


@pytest.mark.parametrize('seconds, text', [(-3, '0 s'), (0, '0 s'), (40.9, '40 s'), (60, '1m'), (3599, '59m'),
                                           (3600, '1h'), (7300, '2h')])
def test_chip_ages_keep_their_text(seconds, text):
    assert authority_chip.age(seconds) == text == theme.age(seconds, 'chip')


def test_the_chip_has_no_formatter_of_its_own():
    source = inspect.getsource(authority_chip)
    assert '// 60' not in source and '// 3600' not in source

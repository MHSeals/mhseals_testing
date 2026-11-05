"""theme.age: one age formatter whose styles print what each call site printed before it (finding 12)."""
import datetime

import pytest

from mhseals_testing.ui import theme

SECONDS = (0, 0.4, 5, 9.94, 9.96, 10, 59, 59.9, 60, 61, 99, 99.4, 100, 3599, 3600, 7322, 86399, 86400, 200000)


@pytest.mark.parametrize('style, seconds, text', [
    ('compact', 42, '42s'), ('compact', 240, '4m'), ('compact', 7200, '2h'), ('compact', 3 * 86400, '3d'),
    ('chip', 40, '40 s'), ('chip', 240, '4m'), ('chip', 7200, '2h'), ('chip', -3, '0 s'),
    ('words', 30, 'just now'), ('words', 240, '4 min ago'), ('words', 7200, '2 h ago'), ('words', 86400, '1 d ago'),
    ('precise', 2.44, '2.4s'), ('precise', 12.3, '12s'), ('precise', 120, '>99s'),
])
def test_each_style_reads_as_pinned(style, seconds, text):
    assert theme.age(seconds, style) == text


def test_precise_narrow_drops_the_tenth():
    assert theme.age(2.44, 'precise', narrow=True) == '2s'


def test_an_unknown_style_is_refused():
    with pytest.raises(ValueError, match='age style'):
        theme.age(1, 'fancy')


def _old_precise(age, narrow):   # rtk_row._age before theme.age
    if narrow or age >= 10:
        return '>99s' if age > 99 else f'{age:.0f}s'
    return f'{age:.1f}s'


@pytest.mark.parametrize('seconds', SECONDS)
def test_the_styles_match_the_formatters_they_replace(seconds):
    from mhseals_testing.ui import authority_chip, deploy
    from mhseals_testing.ui.popups import history
    now = datetime.datetime(2026, 11, 4, 12, tzinfo=datetime.timezone.utc)
    assert theme.age(seconds, 'compact') == deploy._age(seconds)
    assert theme.age(seconds, 'chip') == authority_chip.age(seconds)
    assert theme.age(seconds, 'words') == history.age(now - datetime.timedelta(seconds=seconds), now)
    for narrow in (False, True):
        assert theme.age(seconds, 'precise', narrow=narrow) == _old_precise(seconds, narrow)

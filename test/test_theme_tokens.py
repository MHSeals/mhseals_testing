import pytest

from mhseals_testing.analysis.classes import load_classes
from mhseals_testing.ui import palette


@pytest.fixture(scope='module')
def theme():
    return palette.load_theme(overlay=False)[0]


def test_status_tokens_map_onto_the_theme_constants():
    from mhseals_testing.ui import theme as ui
    assert (ui.GOOD, ui.WARN, ui.BAD, ui.BUSY) == tuple(
        ui.PALETTE.style(t) for t in ('ok', 'warn', 'fail', 'busy'))


def test_every_map_class_colour_is_a_theme_token(theme):
    assert all(symbol.colour in theme.tokens for symbol in load_classes())
    assert all(token in theme.tokens for token in ('ok', 'warn', 'fail', 'busy', 'muted'))


def test_class_with_an_unknown_colour_token_is_rejected(tmp_path):
    text = (palette.default_path().parent / 'minimap.yaml').read_text().replace('obj.blue', 'obj.bleu')
    path = tmp_path / 'minimap.yaml'
    path.write_text(text)
    with pytest.raises(ValueError, match='obj.bleu.*not a theme.yaml token'):
        load_classes(path)

"""The other dot widgets through ui/pixels.py: Motion plot, popup spark and dial, and the minimap
draw as image cells with labels as text in kitty and sixel, and as braille in the text tier."""
import pytest
from rich.cells import cell_len
from rich.text import Text

from mhseals_testing.ui import braille, pixels, sparks, theme
from test_pixels import braille_chars, image_chars


@pytest.fixture(autouse=True)
def text_after():
    yield
    pixels.use('text')


def widgets():
    from mhseals_testing.ui.popups import inspect
    plot = braille.canvas(30, 4, name='t.plot')
    plot.hline(0.0, -1, 1)
    plot.line([(t / 10, (t % 7) / 7 - 0.5) for t in range(100)], 0, 10, -1, 1)
    spark = sparks.spark([((i * 13) % 11) / 10 for i in range(40)], 20, 0, 1, height=2)
    dial = inspect.dial([(40.0, theme.ACCENT, False), (80.0, theme.MUTED, True)], 21, 9)
    return dict(plot=plot.rows(), spark=spark, dial=dial)


@pytest.mark.parametrize('tier', ['kitty', 'sixel'])
def test_each_widget_draws_as_image_cells_with_its_labels_as_text(tier):
    pixels.use(tier)
    out = widgets()
    for name, rows in out.items():
        assert len({cell_len(r.plain) for r in rows}) == 1, name
        assert image_chars(rows) and not braille_chars(rows), name
    assert {'N', 'E', 'S', 'W'} <= set(''.join(r.plain for r in out['dial']))


@pytest.mark.parametrize('tier', ['kitty', 'sixel'])
def test_the_minimap_draws_as_image_lines_with_glyphs_as_text(tier):
    from test_minimap_frame import TABLE, box, scene
    from mhseals_testing.ui.minimap import panel
    from mhseals_testing.ui.minimap.emit import emit_image
    pixels.use(tier)
    s = scene()
    canvas = panel.draw(s, 60, 20, 10.0, TABLE, False, box(s), full=True)
    assert canvas.k == pixels.K > 1 and canvas.dots_wide == 60 * 2 * pixels.K
    lines = emit_image(canvas, lambda layer: panel.style(layer, 'truecolor'))
    plain = [Text.from_ansi(line).plain for line in lines]
    assert len(plain) == 20 and {cell_len(p) for p in plain} == {60}
    assert '5 m' in ''.join(plain)          # the scale bar stays text


def test_the_text_tier_draws_braille_as_before():
    pixels.use('text')
    for name, rows in widgets().items():
        assert braille_chars(rows) and not image_chars(rows), name

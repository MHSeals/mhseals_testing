"""Sixel parts join into as few images as tmux keeps (config/perf.yaml image_cap), so a tmux redraw never drops
part of a picture: a picture of many tiles that changed is one image, a hole splits it round the letters only,
and an unchanged frame paints nothing."""
import pytest
from rich.text import Text

from mhseals_testing.ui import imaging as pixels
from mhseals_testing.ui.imaging import sixel_merge
from test_pixels import paint_box, sixel_frame, text_after  # noqa: F401  (text_after: the autouse reset)


@pytest.fixture
def cap(monkeypatch):
    def set_cap(n):
        monkeypatch.setattr(sixel_merge, 'CAP', n)
    return set_cap


def picture(colour, holes=()):
    grid = pixels.surface('map', paint_box(colour), 24, 8, holes=holes)   # 3 x 2 tiles of 8 x 4 cells
    return [Text(' ') + row for row in pixels.text_rows(grid, lambda col, row: ('N', 'bold'))]


def test_a_changed_picture_of_six_tiles_is_one_image_and_an_unchanged_one_none(cap):
    cap(16)
    pixels.use('sixel')
    _, drawn = sixel_frame(picture((0, 255, 0)))
    assert drawn.count('\x1bP') == 1 and '"1;1;240;160' in drawn          # 24 x 8 cells of 10 x 20 px
    assert sixel_frame(picture((0, 255, 0)))[1] == ''
    _, moved = sixel_frame(picture((255, 0, 0)))                          # it moved: every tile is new
    assert moved.count('\x1bP') == 1


def test_each_part_alone_with_cap_zero_and_round_a_hole_never_over_its_letter(cap):
    cap(0)
    pixels.use('sixel')
    assert sixel_frame(picture((0, 255, 0)))[1].count('\x1bP') == 6
    cap(16)
    pixels.use('sixel')
    text, drawn = sixel_frame(picture((0, 0, 255), holes={(5, 2)}))
    assert 'N' in text and drawn.count('\x1bP') == 4                     # above, left, right and below the hole
    assert '\x1b[3;7H\x1bP' not in drawn


def test_past_the_cap_every_visible_part_is_painted_in_its_fewer_rectangles(cap):
    cap(1)
    pixels.use('sixel')
    sixel_frame(picture((0, 255, 0)))
    grid = pixels.surface('map', paint_box((0, 255, 0)), 24, 8)
    # a checkerboard of changes would be several rectangles: past the cap the whole picture goes as one
    changed = [(r, c, n, crop) for (r, c), (n, crop) in pixels._drawn.items()][::2]
    visible = [(r, c, n, crop) for (r, c), (n, crop) in pixels._drawn.items()]
    out = sixel_merge.compose(changed, visible, pixels._by_id, pixels._rects, 1)
    assert len(out) == 1 and out[0][0] == (0, 1, 8, 25) and grid

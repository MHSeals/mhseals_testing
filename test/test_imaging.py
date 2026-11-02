"""The image protocol interface: every backend is one file of ui/imaging with the four calls."""

import numpy as np
import pytest

from mhseals_testing.ui import imaging
from mhseals_testing.ui.imaging import protocol

pytestmark = pytest.mark.smoke


def test_the_backends_are_found_by_rank_and_text_is_the_last_resort():
    found = protocol.backends()
    assert [b.name for b in found] == ['kitty', 'sixel', 'text']
    for backend in found:
        assert all(callable(getattr(backend, call)) for call in ('probe', 'place', 'clear', 'cost'))
    assert found[-1].probe('off', {}, {}, 'standard')


def test_a_tile_places_and_costs_through_its_backend():
    imaging.use('kitty')
    tile = imaging.Tile(b'd', 1, 1, np.ones((20, 10), np.uint8), [(0, 0, 0), (255, 0, 0)])
    tile.id = 1
    kitty = protocol.by_name('kitty')
    assert imaging.cell(tile, 0, 0) == kitty.place(tile, 0, 0)
    assert kitty.cost(tile).startswith('\x1b_Ga=T,f=32,s=10,v=20') and kitty.clear([1]).startswith('\x1b_Ga=d')
    imaging.use('sixel', cell=(10, 20))
    sixel = protocol.by_name('sixel')
    assert imaging.cell(tile, 0, 0) == sixel.place(tile, 0, 0) and sixel.cost(tile).startswith('\x1bP0;1;0q')
    assert sixel.clear([1]) == ''
    imaging.use('text')

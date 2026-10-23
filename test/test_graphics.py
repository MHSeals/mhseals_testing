"""Terminal images: exact kitty escape bytes, placeholders, delete on close and the half-block fallback."""
import base64
import io

import pytest
from rich.cells import cell_len
from rich.color import ColorType
from rich.console import Console

from mhseals_testing.ui import graphics


def test_small_png_uploads_in_one_escape_with_exact_bytes():
    out = graphics.upload(b'abc', 10, 4)
    assert out == '\x1b_Ga=T,f=100,U=1,i=31,c=10,r=4,q=2,m=0;YWJj\x1b\\'


def test_large_png_is_chunked_at_4096_with_m_flags():
    png = bytes(range(256)) * 40  # 10240 bytes -> 13656 base64 characters
    out = graphics.upload(png, 20, 8)
    parts = out.split('\x1b\\')[:-1]
    assert [len(p.split(';', 1)[1]) for p in parts] == [4096, 4096, 4096, 1368]
    assert parts[0].startswith('\x1b_Ga=T,f=100,U=1,i=31,c=20,r=8,q=2,m=1;')
    assert parts[1].startswith('\x1b_Gq=2,m=1;') and parts[3].startswith('\x1b_Gq=2,m=0;')
    assert base64.b64decode(''.join(p.split(';', 1)[1] for p in parts)) == png


def test_delete_on_close_frees_the_fixed_id():
    assert graphics.delete() == '\x1b_Ga=d,d=I,i=31,q=2\x1b\\'
    assert graphics.delete(7) == '\x1b_Ga=d,d=I,i=7,q=2\x1b\\'


def test_placeholder_rows_carry_row_and_column_diacritics():
    rows = graphics.placeholder_rows(3, 2)
    assert rows[0].plain == '\U0010EEEE̅̅\U0010EEEE\U0010EEEE'
    assert rows[1].plain == '\U0010EEEE̍̅\U0010EEEE\U0010EEEE'
    assert rows[0].style.color.number == 31


def test_placeholder_rows_measure_one_cell_per_column_even_when_wide():
    for cols, n in ((3, 1), (64, 18)):
        rows = graphics.placeholder_rows(cols, n)
        assert len(rows) == n and all(cell_len(r.plain) == cols for r in rows)


def test_placeholders_refuse_sizes_the_diacritics_cannot_address():
    with pytest.raises(ValueError):
        graphics.placeholder_rows(1, graphics.MAX_ROWS + 1)
    with pytest.raises(ValueError):
        graphics.placeholder_rows(1, 1, image_id=256)


def test_kitty_only_on_graphics_ok_and_a_kitty_or_ghostty_version():
    assert graphics.kitty_enabled(True, 'kitty(0.35.2)')
    assert graphics.kitty_enabled(True, 'ghostty 1.0')
    assert not graphics.kitty_enabled(False, 'kitty(0.35.2)')
    assert not graphics.kitty_enabled(True, 'tmux 3.8')
    assert not graphics.kitty_enabled(True, None)


def test_half_blocks_pair_rows_pad_an_odd_row_and_use_256_colours():
    px = [[(255, 0, 0), (0, 255, 0)], [(0, 0, 255), (10, 11, 12)], [(255, 255, 255), (16, 17, 18)]]
    lines = graphics.half_blocks(px)
    assert [r.plain for r in lines] == ['▀▀', '▀▀'] and all(cell_len(r.plain) == 2 for r in lines)
    spans = [s.style for s in lines[0].spans]
    assert spans[0].color.number == 9 or spans[0].color.number == 196
    assert all(c.type == ColorType.EIGHT_BIT for st in spans for c in (st.color, st.bgcolor))
    console = Console(color_system='256', force_terminal=True, file=io.StringIO(), width=10)
    console.print(lines[1], end='')
    out = console.file.getvalue()
    assert '38;5;' in out and '48;5;' in out and ';2;' not in out

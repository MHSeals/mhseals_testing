"""Image tiers (ui/pixels.py): probe parsing, tier choice, tile ids, kitty placeholders and upload,
the sixel encoder and frame flow, the widgets in each tier, and the text tier left as it was."""
import base64
import io
import os
import pty
import tty
import zlib
from types import SimpleNamespace

import numpy as np
import pytest
from rich.cells import cell_len
from rich.console import Console, Group
from rich.text import Text

from mhseals_testing.ui import palette, pixels, theme
from mhseals_testing.ui.panels import conditions

OK = b'\x1b_Gi=4242;OK\x1b\\'
XT = b'\x1bP>|kitty(0.39.1)\x1b\\'
CELL = b'\x1b[6;20;10t'
DA1_SIXEL = b'\x1b[?62;4;22c'
DA1_PLAIN = b'\x1b[?62;22c'


@pytest.fixture(autouse=True)
def text_after():
    pixels.forget()
    yield
    pixels.use('text')
    pixels.forget()


def render(rows, width=80):
    console = Console(file=io.StringIO(), width=width, height=40, force_terminal=True, color_system='truecolor')
    return list(console.render(Group(*rows))), console


# --- detection ---------------------------------------------------------------------------------

def test_parse_reads_every_reply_and_keeps_typed_keys():
    caps, rest = pixels.parse(b'j' + OK + XT + CELL + b'k' + DA1_SIXEL)
    assert caps == dict(done=True, kitty=True, sixel=True, cell=(10, 20), xtversion='kitty(0.39.1)')
    assert rest == b'jk'


def test_parse_before_da1_is_not_done_and_an_error_reply_is_no_kitty():
    caps, _ = pixels.parse(b'\x1b_Gi=4242;ENOTSUPPORTED:x\x1b\\' + CELL)
    assert not caps['done'] and not caps['kitty'] and caps['cell'] == (10, 20)
    caps, _ = pixels.parse(DA1_PLAIN)
    assert caps['done'] and not caps['sixel']
    assert not pixels.parse(b'\x1b[?4;6c')[0]['sixel']     # 4 as the class (VT132) is not the attribute


@pytest.mark.parametrize('setting,caps,env,want', [
    ('auto', dict(kitty=True, xtversion='kitty(0.39)'), {}, 'kitty'),
    ('auto', dict(kitty=True, xtversion='ghostty 1.2'), {}, 'kitty'),
    ('auto', dict(kitty=True, xtversion='WezTerm 2024', sixel=True), {}, 'sixel'),   # no placeholders
    ('auto', dict(kitty=True), {'TERM': 'xterm-kitty'}, 'kitty'),                    # no XTVERSION: env hint
    ('auto', dict(kitty=True, xtversion='kitty'), {'TMUX': '/tmp/t'}, 'text'),
    ('auto', dict(sixel=True, xtversion='tmux 3.8'), {}, 'text'),      # tmux drops old sixel tiles
    ('sixel', dict(sixel=True, xtversion='tmux 3.8'), {}, 'sixel'),
    ('auto', dict(sixel=True, xtversion='foot(1.28.0)'), {}, 'sixel'),
    ('auto', {}, {}, 'text'),
    ('off', dict(kitty=True, sixel=True, xtversion='kitty'), {}, 'text'),
    ('sixel', {}, {}, 'sixel'),
    ('kitty', {}, {}, 'kitty'),
    ('kitty', dict(kitty=True), {'NO_COLOR': '1'}, 'text'),
    ('sixel', dict(sixel=True), {'MHS_ASCII': '1'}, 'text'),
])
def test_choose_follows_the_setting_the_replies_and_the_environment(setting, caps, env, want):
    assert pixels.choose(setting, caps, env) == want


def test_kitty_needs_a_256_colour_console():
    assert pixels.choose('auto', dict(kitty=True, xtversion='kitty'), {}, 'standard') == 'text'


def raw_pty():
    """A pty whose slave is non-canonical, as KeyReader leaves the terminal before the probe."""
    master, slave = pty.openpty()
    tty.setraw(slave)
    return master, slave


def answering(master, replies):
    """A write() that answers each query with the next reply, as a terminal on the pty would."""
    sent = []

    def write(text):
        sent.append(text)
        os.write(master, replies[len(sent) - 1])
    return sent, write


def test_probe_asks_kitty_for_graphics_only_after_it_names_itself():
    master, slave = raw_pty()
    try:
        sent, write = answering(master, [XT + CELL + DA1_PLAIN, OK + DA1_PLAIN])
        caps, rest = pixels.probe(slave, write, timeout=2.0, env={})
        assert caps['done'] and caps['kitty'] and caps['cell'] == (10, 20) and rest == b''
        assert sent == [pixels.PROBE, pixels.PROBE_KITTY]
    finally:
        os.close(master)
        os.close(slave)


def test_probe_never_sends_the_graphics_apc_to_tmux_or_foot():
    master, slave = raw_pty()
    try:
        for name in (b'tmux 3.8', b'foot(1.28.0)'):
            sent, write = answering(master, [b'\x1bP>|' + name + b'\x1b\\' + CELL + DA1_SIXEL])
            caps, _ = pixels.probe(slave, write, timeout=2.0, env={})
            assert sent == [pixels.PROBE] and caps['sixel'] and not caps['kitty']
            assert '\x1b_G' not in pixels.PROBE
    finally:
        os.close(master)
        os.close(slave)


def test_probe_gives_up_after_its_timeout_and_start_falls_back_to_text():
    master, slave = raw_pty()
    try:
        keys = SimpleNamespace(fd=slave, buffer=bytearray(), escape_started=None)
        os.write(master, b'q')       # a key typed while the probe waits: handed back to the reader
        tier = pixels.start(keys, None, lambda text: None, 'auto', env={})
        assert tier == 'text' and keys.buffer == b'q'
    finally:
        os.close(master)
        os.close(slave)


def test_theme_graphics_setting_is_off_by_default_validated_and_may_be_overlaid(tmp_path):
    base, _ = palette.load_theme(overlay=False)
    assert base.graphics == 'off'
    overlay = tmp_path / 'theme.yaml'
    overlay.write_text('graphics: sixel\n')
    assert palette.load_theme(overlay=overlay)[0].graphics == 'sixel'
    overlay.write_text('graphics: off\n')            # YAML 1.1 reads it as false
    assert palette.load_theme(overlay=overlay)[0].graphics == 'off'
    overlay.write_text('graphics: text\n')
    themed, warning = palette.load_theme(overlay=overlay)
    assert themed.graphics == 'off' and 'graphics' in warning


def test_save_graphics_sets_one_line_and_keeps_the_rest_of_the_overlay(tmp_path):
    overlay = tmp_path / 'mhseals' / 'theme.yaml'
    assert palette.save_graphics('auto', overlay) and overlay.read_text() == 'graphics: auto\n'
    overlay.write_text('# mine\ntheme: dark\ngraphics: auto   # was auto\ncvd: true')
    assert palette.save_graphics('sixel', overlay)
    assert overlay.read_text() == '# mine\ntheme: dark\ngraphics: sixel   # was auto\ncvd: true'
    assert palette.load_theme(overlay=overlay)[0].graphics == 'sixel'
    overlay.write_text('theme: light')
    palette.save_graphics('off', overlay)
    assert overlay.read_text() == 'theme: light\ngraphics: off\n'
    with pytest.raises(ValueError):
        palette.save_graphics('text', overlay)


def test_off_probes_nothing_and_a_session_probes_once(monkeypatch):
    asked = []
    monkeypatch.setattr(pixels, 'probe', lambda fd, write, env=None: asked.append(fd) or (
        dict(done=True, sixel=True, kitty=False, cell=(9, 18), xtversion='foot(1.28)'), b''))
    monkeypatch.setattr(os, 'isatty', lambda fd: True)
    keys = SimpleNamespace(fd=5, buffer=bytearray())
    assert pixels.start(keys, None, lambda text: None, 'off', env={}) == 'text' and asked == []
    assert pixels.start(keys, None, lambda text: None, 'auto', env={}) == 'sixel' and asked == [5]
    sent = []
    assert pixels.restart(keys, None, sent.append, 'off', env={}) == 'text' and sent == ['\x1b[2J']
    assert pixels.restart(keys, None, sent.append, 'sixel', env={}) == 'sixel' and asked == [5]
    assert pixels.CELL == (9, 18) and pixels.SETTING == 'sixel'


# --- tiles, ids and kitty placeholders -----------------------------------------------------------

def paint_box(colour):
    def paint(r):
        r.rect(2, 2, r.w - 2, r.h - 2, r.ink(colour))
    return paint


def test_same_content_reuses_its_tile_id_and_a_key_skips_the_paint():
    pixels.use('kitty')
    a = pixels.surface('a', paint_box((200, 0, 0)), 4, 2)
    b = pixels.surface('b', paint_box((200, 0, 0)), 4, 2)
    assert a == b and pixels.STATS['tiles'] == 1
    calls = []
    pixels.surface('k', lambda r: calls.append(1), 4, 2, key=1)
    pixels.surface('k', lambda r: calls.append(1), 4, 2, key=1)
    assert calls == [1]
    c = pixels.surface('a', paint_box((0, 200, 0)), 4, 2)
    assert c != a and pixels.STATS['tiles'] == 3


def test_kitty_ids_carry_a_high_byte_and_a_256_colour_low_byte():
    assert pixels._kitty_id(1) == (1 << 24) | 1
    assert pixels._kitty_id(255) == (1 << 24) | 255
    assert pixels._kitty_id(256) == (2 << 24) | 1


def test_placeholder_cells_name_row_column_and_id_byte():
    pixels.use('kitty')
    grid = pixels.surface('p', paint_box((1, 2, 3)), 3, 2, holes={(2, 1)})
    text, style = grid[1][1]
    kid = pixels._kitty_id(pixels._pool[next(iter(pixels._pool))].id)
    assert text == pixels.PLACEHOLDER + pixels.DIACRITICS[1] + pixels.DIACRITICS[1] + pixels.DIACRITICS[kid >> 24]
    assert style == f'color({kid & 0xFF})' and grid[1][2] is None
    rows = pixels.text_rows(grid, lambda col, row: ('x', None))
    assert [cell_len(r.plain) for r in rows] == [3, 3]


def test_a_run_of_placeholders_goes_out_bare_after_its_first_cell():
    pixels.use('kitty')
    rows = pixels.text_rows(pixels.surface('r', paint_box((9, 9, 9)), 6, 1))
    segments, _ = render(rows)
    found, out = pixels._scan(segments)
    text = ''.join(s.text for s in out)
    assert text.count(pixels.PLACEHOLDER) == 6
    assert sum(ch in pixels.DIACRITICS for ch in text) == 3          # only the first cell keeps them
    assert len(found) == 1


def test_a_run_cut_by_other_text_keeps_the_diacritics_after_the_cut():
    pixels.use('kitty')
    grid = pixels.surface('r', paint_box((9, 9, 9)), 6, 1)
    row = Text()
    for col, spec in enumerate(grid[0]):
        row.append(*(('|', None) if col == 3 else spec))    # a popup border over cell 3
    _, out = pixels._scan(render([row])[0])
    text = ''.join(s.text for s in out)
    assert sum(ch in pixels.DIACRITICS for ch in text) == 6           # cells 0 and 4 carry them


def test_kitty_upload_is_zlib_rgba_with_a_virtual_placement():
    pixels.use('kitty')
    pixels.surface('u', paint_box((255, 0, 0)), 2, 1)
    tile = next(iter(pixels._pool.values()))
    out = pixels.kitty_upload(77, tile)
    head, payload = out[len('\x1b_G'):-2].split(';', 1)
    assert head == 'a=T,f=32,s=20,v=20,o=z,U=1,i=77,c=2,r=1,q=2,m=0'
    rgba = np.frombuffer(zlib.decompress(base64.b64decode(payload)), np.uint8).reshape(20, 20, 4)
    assert tuple(rgba[10, 10]) == (255, 0, 0, 255) and rgba[0, 0, 3] == 0


def test_kitty_sends_new_tiles_once_and_deletes_evicted_ones(monkeypatch):
    pixels.use('kitty')
    monkeypatch.setattr(pixels, 'POOL', 2)
    pixels._free[:] = [2, 1]
    sent = []
    for shade in (10, 20, 30):
        pixels.surface('e', paint_box((shade, 0, 0)), 1, 1)
        pixels.before(sent.append)
    pixels.before(sent.append)
    data = ''.join(sent)
    assert data.count('a=T') == 3 and data.count('a=d,d=I') == 1


# --- sixel ---------------------------------------------------------------------------------------

def test_sixel_encodes_bands_colours_and_runs():
    px = np.zeros((6, 2), np.uint8)
    px[:, 0] = 1
    px[0, 1] = 1
    assert pixels.sixel(px, [(0, 0, 0), (255, 0, 0)]) == '\x1bP0;1;0q"1;1;2;6#1;2;100;0;0#1~@\x1b\\'
    wide = np.zeros((14, 5), np.uint8)
    wide[:6] = 1
    wide[12] = 1
    # three bands, a line feed between them and none after the last: tmux sizes images by it
    assert pixels.sixel(wide, [(0, 0, 0), (0, 0, 255)]).endswith('#1!5~--#1!5@\x1b\\')
    assert pixels.sixel(np.zeros((12, 3), np.uint8), [(0, 0, 0)]) == '\x1bP0;1;0q"1;1;3;12\x1b\\'


def test_sixel_two_colours_share_a_band():
    px = np.array([[1, 2]] * 6, np.uint8)
    out = pixels.sixel(px, [(0, 0, 0), (255, 255, 255), (0, 0, 0)])
    assert out.endswith('#1~$#2?~\x1b\\')


def sixel_frame(rows, size=(80, 40)):
    segments, console = render(rows)
    out = pixels.frame(segments, size)
    stream = pixels._Stream(io.StringIO())
    stream.live = frozenset(n for n, _ in pixels._live.values())
    stream.write(console._render_buffer(out))
    sent = []
    pixels.after(sent.append)
    return stream.inner.getvalue(), ''.join(sent)


def test_sixel_draws_a_tile_once_at_its_place_and_skips_its_cells():
    pixels.use('sixel')
    rows = [Text('ab ') + row for row in pixels.text_rows(pixels.surface('s', paint_box((0, 255, 0)), 4, 2))]
    text, drawn = sixel_frame(rows)
    assert '\x1b[4C' in text and chr(pixels.MARK0) not in text
    assert drawn.startswith('\x1b7') and '\x1b[1;4H\x1bP0;1;0q"1;1;40;40' in drawn
    assert sixel_frame(rows)[1] == ''                                   # already there: nothing sent
    assert sixel_frame(rows, size=(100, 40))[1] != ''                   # a resize paints it again


def test_sixel_crops_a_tile_cut_by_a_popup_and_splits_an_l_shape():
    pixels.use('sixel')
    grid = pixels.surface('s', paint_box((0, 255, 0)), 4, 2)
    cut = [Text().join(Text(*spec) for spec in row[:2]) + Text('||') for row in grid]
    text, drawn = sixel_frame(cut)
    assert '"1;1;20;40' in drawn                                        # two columns of four
    corner = [Text().join(Text(*spec) for spec in grid[0]), Text().join(Text(*spec) for spec in grid[1][:2])]
    text, drawn = sixel_frame(corner)                                   # an L: two rectangles
    assert '\x1b[1;1H\x1bP0;1;0q"1;1;40;20' in drawn and '\x1b[2;1H\x1bP0;1;0q"1;1;20;20' in drawn
    assert drawn.count('\x1bP') == 2


def test_sixel_draws_round_a_label_hole_and_a_tile_shown_twice():
    pixels.use('sixel')
    grid = pixels.surface('s', paint_box((0, 255, 0)), 4, 2, holes={(1, 0)})
    rows = [pixels.text_rows(grid, lambda col, row: ('N', 'bold'))[r] + Text(' ') +
            pixels.text_rows(grid, lambda col, row: ('N', 'bold'))[r] for r in range(2)]
    text, drawn = sixel_frame(rows)
    assert 'N' in text and drawn.count('\x1bP') == 6                   # 3 parts round the hole, twice
    assert '\x1b[1;2H' not in drawn and '\x1b[1;7H' not in drawn         # never over the letters


def test_rects_cover_cells_with_whole_row_runs():
    cells = {(0, 0), (0, 2), (0, 3), (1, 0), (1, 1), (1, 2), (1, 3), (2, 0), (2, 1), (2, 2), (2, 3)}
    assert pixels._rects(cells) == [(0, 0, 1, 1), (0, 2, 1, 4), (1, 0, 3, 4)]


# --- widgets ------------------------------------------------------------------------------------

def image_chars(rows):
    return sum(ch == pixels.PLACEHOLDER or 0xF0000 <= ord(ch) <= 0xFFFFD for r in rows for ch in r.plain)


def braille_chars(rows):
    return sum(0x2801 <= ord(ch) <= 0x28FF for r in rows for ch in r.plain)


def rose(heading=30.0, command=(0.6, 0.2, 0.3)):
    return conditions.rose((0.05, 0.08), heading, True, 0.0, command, 1.0, theme.ACCENT)


@pytest.mark.parametrize('tier', ['kitty', 'sixel'])
def test_the_rose_is_image_cells_with_its_letters_as_text(tier):
    pixels.use(tier)
    rows = rose()
    assert len(rows) == conditions.ROSE_ROWS and {cell_len(r.plain) for r in rows} == {conditions.ROSE_COLS}
    assert image_chars(rows) and not braille_chars(rows)
    assert {'N', 'E', 'S', 'W'} <= set(''.join(r.plain for r in rows))


def test_the_rose_repaints_only_when_a_quantized_input_changes():
    pixels.use('kitty')
    turn = lambda heading: conditions.rose(None, heading, True, 0.0, (0.0, 0.0, 0.0), 1.0, theme.ACCENT)
    turn(30.0)
    tiles = pixels.STATS['tiles']
    turn(30.6)                       # the same 2 deg step: the cached cells, no paint
    assert pixels.STATS['tiles'] == tiles
    turn(40.0)                       # the hull turns: only its (centre) tile is new
    assert pixels.STATS['tiles'] == tiles + 1
    turn(30.0)                       # back: every tile is already on the terminal
    assert pixels.STATS['tiles'] == tiles + 1


def test_the_text_tier_draws_the_braille_rose_as_before():
    pixels.use('text')
    assert pixels.K == 1 and not pixels.active()
    rows = rose()
    assert braille_chars(rows) and not image_chars(rows)
    assert pixels.frame(['x'], (1, 1)) == ['x'] and pixels.STATS['tiles'] == 0

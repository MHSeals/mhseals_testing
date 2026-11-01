"""Image tiers (ui/pixels.py): probe parsing, tier choice, tile ids, kitty placeholders and upload,
the sixel encoder and frame flow, the widgets in each tier, and the text tier left as it was."""
import base64
import io
import os
import pty
import re
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
    ('auto', dict(kitty=True, xtversion='WezTerm 2024', sixel=True, cell=(9, 18)), {}, 'sixel'),   # no placeholders
    ('auto', dict(kitty=True), {'TERM': 'xterm-kitty'}, 'kitty'),                    # no XTVERSION: env hint
    ('auto', dict(kitty=True, xtversion='kitty'), {'TMUX': '/tmp/t'}, 'text'),
    ('auto', dict(sixel=True, xtversion='tmux 3.8'), {}, 'text'),      # tmux drops old sixel tiles
    ('auto', dict(sixel=True, cell=(9, 18), xtversion='foot(1.28.0)'), {}, 'sixel'),
    ('auto', dict(sixel=True, xtversion='foot(1.28.0)'), {}, 'text'),    # no cell size: tiles could spill
    ('auto', {}, {}, 'text'),
    ('off', dict(kitty=True, sixel=True, xtversion='kitty'), {}, 'text'),
    ('sixel', dict(cell=(9, 18)), {}, 'sixel'),
    ('sixel', {}, {}, 'text'),                                            # forced, still no guessed cell
    ('kitty', {}, {}, 'kitty'),
    # a forced tier keeps the guards: tmux, screen, a dumb terminal
    ('sixel', dict(sixel=True, cell=(9, 18), xtversion='tmux 3.8'), {}, 'text'),
    ('sixel', dict(sixel=True, cell=(9, 18)), {'TMUX': '/tmp/t'}, 'text'),
    ('kitty', dict(kitty=True), {'STY': '1.pts-0'}, 'text'),
    ('kitty', dict(kitty=True), {'TERM': 'dumb'}, 'text'),
    ('sixel', dict(sixel=True, cell=(9, 18)), {'TERM': 'dumb'}, 'text'),
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


def test_a_forced_tier_needs_a_tty(monkeypatch):
    monkeypatch.setattr(pixels, 'probe', lambda fd, write, env=None: (
        dict(done=True, sixel=True, kitty=True, cell=(9, 18), xtversion='kitty'), b''))
    monkeypatch.setattr(os, 'isatty', lambda fd: False)
    for setting in ('kitty', 'sixel'):
        assert pixels.start(SimpleNamespace(fd=5, buffer=bytearray()), None, lambda text: None, setting,
                            env={}) == 'text'
    assert pixels.start(SimpleNamespace(), None, lambda text: None, 'kitty', env={}) == 'text'


def test_sixel_without_a_cell_size_reply_stays_text(monkeypatch):
    monkeypatch.setattr(pixels, 'probe', lambda fd, write, env=None: (
        dict(done=True, sixel=True, kitty=False, cell=None, xtversion='foot(1.28)'), b''))
    monkeypatch.setattr(os, 'isatty', lambda fd: True)
    keys = SimpleNamespace(fd=5, buffer=bytearray())
    assert pixels.start(keys, None, lambda text: None, 'auto', env={}) == 'text'
    assert pixels.restart(keys, None, lambda text: None, 'sixel', env={}) == 'text'


# --- tiles, ids and kitty placeholders -----------------------------------------------------------

def paint_box(colour):
    def paint(r):
        r.rect(2, 2, r.w - 2, r.h - 2, r.ink(colour))
    return paint


def test_over_the_link_budget_images_repaint_at_the_throttle_rate(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(pixels, 'clock', lambda: now[0])
    pixels.use('sixel', budget=1000)
    paint = lambda colour: paint_box((colour, 0, 0))
    first = pixels.surface('s', paint(10), 4, 2, key=1)
    pixels._meter(5000)                                    # 2500 B/s over the last METER_S
    now[0] += 0.1
    assert pixels.surface('s', paint(20), 4, 2, key=2) is first     # held: over budget
    now[0] += 1.0                                          # the throttle's 0.5 s, and the second's bytes aged
    assert pixels.surface('s', paint(30), 4, 2, key=3) is not first
    now[0] += pixels.METER_S + 1                           # the burst ages out: back to every change
    held = pixels.surface('s', paint(40), 4, 2, key=4)
    now[0] += 0.1
    assert pixels.surface('s', paint(50), 4, 2, key=5) is not held


def test_the_byte_meter_stays_bounded_without_a_budget(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(pixels, 'clock', lambda: now[0])
    pixels.use('sixel')                                    # a local terminal: rate() never runs
    for _ in range(1000):
        pixels._meter(100)
        now[0] += 0.1
    assert len(pixels._sent) <= pixels.METER_S / 0.1 + 1


def test_ssh_sets_the_budget(monkeypatch):
    monkeypatch.setattr(pixels, 'probe', lambda fd, write, env=None: (dict(done=True, sixel=True, cell=(9, 18)), b''))
    monkeypatch.setattr(os, 'isatty', lambda fd: True)
    keys = SimpleNamespace(fd=5, buffer=bytearray())
    pixels.start(keys, None, lambda text: None, 'sixel', env={'SSH_CONNECTION': '10 1 10 22'})
    assert pixels.BUDGET == pixels.SSH_BUDGET
    pixels.restart(keys, None, lambda text: None, 'sixel', env={})
    assert pixels.BUDGET is None


def test_an_empty_sixel_tile_clears_its_cells_once_and_draws_nothing():
    pixels.use('sixel')
    rows = pixels.text_rows(pixels.surface('s', lambda r: None, 4, 2))
    text, drawn = sixel_frame(rows)
    assert '\x1b[4C' in text and '\x1bP' not in drawn and '\x1b[1;1H    ' in drawn
    assert sixel_frame(rows)[1] == ''


def test_same_content_reuses_its_tile_id_and_a_key_skips_the_paint():
    pixels.use('kitty')
    a = pixels.surface('a', paint_box((200, 0, 0)), 4, 2)
    b = pixels.surface('b', paint_box((200, 0, 0)), 4, 2)
    assert a == b and pixels.STATS['tiles'] == 1
    calls = []
    pixels.surface('k', lambda r: calls.append(1), 4, 2, key=1)
    pixels.surface('k', lambda r: calls.append(1), 4, 2, key=1)
    assert calls == [1] and pixels.STATS['tiles'] == 1   # nothing painted: plain spaces, no tile
    assert pixels.surface('e', lambda r: None, 4, 2, key=1) == [[pixels.BLANK] * 4] * 2
    c = pixels.surface('a', paint_box((0, 200, 0)), 4, 2)
    assert c != a and pixels.STATS['tiles'] == 2


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


@pytest.mark.parametrize('tier', ['sixel', 'kitty'])
def test_a_tile_under_a_modal_backdrop_is_drawn_dimmed_like_the_text(tier):
    pixels.use(tier)
    grid = pixels.surface('s', paint_box((200, 100, 0)), 4, 2)
    (bright,) = pixels._by_id.values()
    dimmed = [row.copy() for row in pixels.text_rows(grid)]
    for row in dimmed:
        row.stylize('dim')
    segments, _ = render(dimmed)
    pixels.frame(segments, (80, 40))
    copies = [t for t in pixels._by_id.values() if t is not bright]
    assert len(copies) == 1 and copies[0].palette[1] == tuple((c + b) // 2 for c, b in zip((200, 100, 0),
                                                                                       pixels.token('bg', (0, 0, 0))))
    if tier == 'sixel':
        assert {n for n, _ in pixels._live.values()} == {copies[0].id}
        assert pixels._stream is None or bright.id in pixels._stream.live
    else:
        assert copies[0] in pixels._pending       # uploaded before the frame that names it


def test_the_image_rose_paints_the_text_tiers_tokens():
    pixels.use('sixel')
    seen = {}
    real = pixels.surface
    try:
        pixels.surface = lambda name, paint, cols, rows, **kw: seen.setdefault('r', paint) and real(
            name, paint, cols, rows, **kw)
        conditions.rose(None, 30.0, True, 0.0, (0.6, 0.2, 0.0), 1.0, '#ff00ff')
    finally:
        pixels.surface = real
    raster = pixels.Raster(conditions.ROSE_COLS, conditions.ROSE_ROWS)
    seen['r'](raster)
    used = {raster.palette[i] for i in np.unique(raster.px) if i}
    assert used == {pixels.rgb(theme.PALETTE.style('slate')), pixels.rgb(theme.PALETTE.style('bow')),
                    pixels.rgb(theme.MUTED), (255, 0, 255)}


def test_rects_cover_cells_with_whole_row_runs():
    spans = [(0, 0, 1), (0, 2, 4), (1, 0, 2), (1, 2, 4), (2, 0, 4)]     # (1, ...) split by a style change
    assert pixels._rects(spans) == [(0, 0, 1, 1), (0, 2, 1, 4), (1, 0, 3, 4)]
    assert pixels._rects([(1, 2, 5), (2, 2, 5)]) == [(1, 2, 3, 5)]
    assert pixels._rects([(0, 2, 5), (2, 2, 5)]) == [(0, 2, 1, 5), (2, 2, 3, 5)]   # a row hidden between


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


@pytest.mark.skipif(theme.ASCII, reason='ASCII draws no braille')
def test_the_text_tier_draws_the_braille_rose_as_before():
    pixels.use('text')
    assert pixels.K == 1 and not pixels.active()
    rows = rose()
    assert braille_chars(rows) and not image_chars(rows)
    assert pixels.frame(['x'], (1, 1)) == ['x'] and pixels.STATS['tiles'] == 0


# --- under tmux --------------------------------------------------------------------------------

TMUX_ENV = {'TMUX': '/tmp/tmux-1/default,1,0', 'TMUX_PANE': '%3'}
FOOT = ('1', 'foot(1.28.0)', frozenset({'RGB', 'sixel'}), (12, 24))
KITTY = ('2', 'kitty(0.42.0)', frozenset({'RGB'}), (12, 24))
XTERM = ('3', 'xterm(390)', frozenset({'RGB'}), (12, 24))
TMUX_IN = ('4', 'tmux 3.7c', frozenset({'RGB', 'sixel'}), (12, 24))   # a nested tmux: its client is a tmux
TMUX_CAPS = dict(done=True, sixel=True, kitty=False, cell=(12, 24), xtversion='tmux 3.8')


def view(*clients, passthrough=False):
    return dict(window='@1', passthrough=passthrough, clients=tuple(sorted(clients)))


def test_tmux_view_reads_the_clients_viewing_the_panes_window():
    out = ('@1\ton\n'
           '@1\t17\tfoot(1.28.0)\t256,RGB,sixel\t12\t24\n'
           '@2\t18\txterm(390)\tRGB\t9\t18\n'           # on another window: tmux does not draw the pane there
           '@1\t19\tkitty(0.42.0)\tRGB\t0\t0\n')
    calls = []
    run = lambda cmd, **kw: calls.append(cmd) or SimpleNamespace(stdout=out)
    got = pixels.tmux_view(TMUX_ENV, run)
    assert calls[0][:5] == ['tmux', 'display', '-p', '-t', '%3'] and 'list-clients' in calls[0]
    assert got == dict(window='@1', passthrough=True, clients=(
        ('17', 'foot(1.28.0)', frozenset({'256', 'RGB', 'sixel'}), (12, 24)),
        ('19', 'kitty(0.42.0)', frozenset({'RGB'}), (0, 0))))
    assert pixels.tmux_view({'TMUX': '/x'}, run) is None                       # no pane to ask about
    assert pixels.tmux_view(TMUX_ENV, lambda cmd, **kw: SimpleNamespace(stdout='')) is None

    def gone(cmd, **kw):
        raise FileNotFoundError('tmux')                # $TMUX inside a container without tmux
    assert pixels.tmux_view(TMUX_ENV, gone) is None


@pytest.mark.parametrize('setting, caps, seen, want', [
    ('auto', TMUX_CAPS, view(FOOT), 'sixel'),
    ('sixel', TMUX_CAPS, view(FOOT), 'sixel'),
    ('auto', TMUX_CAPS, view(FOOT, XTERM), 'text'),               # tmux draws 'SIXEL IMAGE' text for xterm
    ('auto', dict(TMUX_CAPS, sixel=False), view(FOOT), 'text'),   # tmux built without sixel
    ('auto', TMUX_CAPS, view(FOOT, ('5', 'foot', FOOT[2], (10, 20))), 'text'),   # two cell sizes
    ('auto', TMUX_CAPS, view(('1', 'foot', FOOT[2], (0, 0))), 'text'),           # no pixel size
    ('auto', TMUX_CAPS, view(TMUX_IN), 'text'),                   # nested: the outer clients are unknown
    ('auto', TMUX_CAPS, view(), 'text'),                          # detached
    ('auto', TMUX_CAPS, None, 'text'),                            # tmux did not answer
    ('off', TMUX_CAPS, view(FOOT), 'text'),
    ('auto', TMUX_CAPS, view(KITTY, passthrough=True), 'kitty'),
    ('kitty', TMUX_CAPS, view(KITTY, passthrough=True), 'kitty'),
    ('auto', TMUX_CAPS, view(KITTY), 'text'),                     # allow-passthrough off
    ('kitty', TMUX_CAPS, view(KITTY), 'text'),
    ('kitty', TMUX_CAPS, view(KITTY, XTERM, passthrough=True), 'text'),   # placeholders would show on xterm
    ('kitty', TMUX_CAPS, view(FOOT, passthrough=True), 'text'),   # a forced tier needs a capable chain
    ('sixel', TMUX_CAPS, view(KITTY, passthrough=True), 'text'),
])
def test_under_tmux_only_what_tmux_reports_for_every_client_picks_an_image(setting, caps, seen, want):
    assert pixels.choose(setting, caps, TMUX_ENV, view=seen) == want


def test_tmux_without_a_view_and_screen_stay_text():
    assert pixels.choose('auto', TMUX_CAPS, {}) == 'text'           # tmux over ssh: XTVERSION only
    assert pixels.choose('sixel', dict(TMUX_CAPS, xtversion='screen'), {}, view=view(FOOT)) == 'text'
    assert pixels.choose('sixel', dict(TMUX_CAPS, xtversion='foot'), dict(TMUX_ENV, NO_COLOR='1'),
                         view=view(FOOT)) == 'text'


def test_kitty_escapes_under_tmux_go_out_only_inside_passthrough():
    pixels.use('kitty')
    pixels.PASSTHROUGH = True
    pixels.surface('s', paint_box((200, 0, 0)), 8, 4, key=1)
    sent = []
    pixels.before(sent.append)
    data = ''.join(sent)
    wrapped = re.findall(r'\x1bPtmux;((?:[^\x1b]|\x1b\x1b)*)\x1b\\', data)
    assert wrapped and re.sub(r'\x1bPtmux;(?:[^\x1b]|\x1b\x1b)*\x1b\\', '', data) == ''
    assert all(w.replace('\x1b\x1b', '\x1b').startswith('\x1b_G') for w in wrapped)
    pixels.PASSTHROUGH = False
    pixels.repaint()
    sent.clear()
    pixels.before(sent.append)
    assert ''.join(sent).startswith('\x1b_G') and 'tmux;' not in ''.join(sent)


def test_watch_follows_tmux_clients_and_repaints(monkeypatch):
    now, seen = [100.0], [view()]
    monkeypatch.setattr(pixels, 'clock', lambda: now[0])
    monkeypatch.setattr(pixels, 'tmux_view', lambda env: seen[0])
    monkeypatch.setattr(pixels, '_spawn', lambda run: run())      # the background query, inline
    monkeypatch.setattr(pixels, 'probe', lambda fd, write, env=None: (dict(TMUX_CAPS, cell=(10, 20)), b''))
    monkeypatch.setattr(os, 'isatty', lambda fd: True)
    keys, sent = SimpleNamespace(fd=5, buffer=bytearray()), []
    console = Console(file=io.StringIO(), color_system='truecolor')
    assert pixels.start(keys, console, sent.append, 'auto', env=TMUX_ENV) == 'text'   # started detached
    seen[0] = view(FOOT)
    assert pixels.watch() is False and pixels.TIER == 'text'      # polls at most every TMUX_POLL_S
    now[0] += pixels.TMUX_POLL_S
    assert pixels.watch() is True and pixels.TIER == 'sixel'
    assert pixels.CELL == (12, 24)                                # the client's cell, not a stale 16t
    pixels._drawn[(0, 0)] = (1, (0, 0, 1, 1))
    now[0] += pixels.TMUX_POLL_S
    assert pixels.watch() is False and pixels._drawn                # nothing changed
    seen[0] = view(FOOT, ('9', FOOT[1], FOOT[2], FOOT[3]))           # a second sixel client attached
    now[0] += pixels.TMUX_POLL_S
    assert pixels.watch() is True and pixels.TIER == 'sixel' and not pixels._drawn
    pixels._drawn[(0, 0)] = (1, (0, 0, 1, 1))
    now[0] += pixels.TMUX_REPAINT_S
    assert pixels.watch() is True and not pixels._drawn             # the periodic repaint
    seen[0] = view(FOOT, XTERM)                                    # a viewer without sixel
    now[0] += pixels.TMUX_POLL_S
    sent.clear()
    assert pixels.watch() is True and pixels.TIER == 'text' and sent == ['\x1b[2J']
    assert not isinstance(console.file, pixels._Stream)
    seen[0] = None                                                 # tmux stopped answering
    now[0] += pixels.TMUX_POLL_S
    assert pixels.watch() is False and pixels.TIER == 'text'
    pixels.stop(console, sent.append)
    now[0] += pixels.TMUX_POLL_S
    assert pixels.watch() is False


def test_watch_never_waits_on_tmux(monkeypatch):
    """tmux answers on a background thread: watch() returns at once while it is asked, asks once at a
    time, and applies the answer on a later frame; an answer for a stopped session is dropped."""
    import threading
    import time as _time
    now, asked, release = [100.0], [], threading.Event()

    def slow(env):
        asked.append(env)
        release.wait(5)
        return view(FOOT)
    monkeypatch.setattr(pixels, 'clock', lambda: now[0])
    monkeypatch.setattr(pixels, 'tmux_view', lambda env: view())
    monkeypatch.setattr(pixels, 'probe', lambda fd, write, env=None: (dict(TMUX_CAPS, cell=(10, 20)), b''))
    monkeypatch.setattr(os, 'isatty', lambda fd: True)
    keys, sent = SimpleNamespace(fd=5, buffer=bytearray()), []
    console = Console(file=io.StringIO(), color_system='truecolor')
    assert pixels.start(keys, console, sent.append, 'auto', env=TMUX_ENV) == 'text'
    monkeypatch.setattr(pixels, 'tmux_view', slow)
    now[0] += pixels.TMUX_POLL_S
    began = _time.monotonic()
    assert pixels.watch() is False and _time.monotonic() - began < 0.1      # did not wait on tmux
    now[0] += pixels.TMUX_POLL_S
    assert pixels.watch() is False and len(asked) == 1                      # one query in flight
    release.set()
    deadline = _time.monotonic() + 5
    while pixels._mux.get('busy') and _time.monotonic() < deadline:
        _time.sleep(0.01)
    assert pixels.watch() is True and pixels.TIER == 'sixel'               # its answer, on the next frame
    release.clear()
    now[0] += pixels.TMUX_POLL_S
    pixels.watch()
    pixels.stop(console, sent.append)                                      # the session ends mid-query
    release.set()
    _time.sleep(0.05)
    assert 'answer' not in pixels._mux and pixels.watch() is False


def test_outside_tmux_watch_asks_nothing(monkeypatch):
    monkeypatch.setattr(pixels, 'tmux_view', lambda env: pytest.fail('asked tmux'))
    monkeypatch.setattr(pixels, 'probe', lambda fd, write, env=None: (
        dict(done=True, sixel=True, kitty=False, cell=(9, 18), xtversion='foot(1.28)'), b''))
    monkeypatch.setattr(os, 'isatty', lambda fd: True)
    assert pixels.start(SimpleNamespace(fd=5, buffer=bytearray()), None, lambda t: None, 'auto', env={}) == 'sixel'
    monkeypatch.setattr(pixels, 'clock', lambda: 1e9)
    assert pixels.watch() is False


def test_over_the_link_budget_repaints_are_spaced_so_a_second_stays_in_it(monkeypatch):
    """A repaint of 60% of the budget is followed by none for a second."""
    now = [10.0]
    monkeypatch.setattr(pixels, 'clock', lambda: now[0])
    pixels.use('sixel', budget=1000)
    paint = lambda colour: paint_box((colour, 0, 0))
    a = pixels.surface('a', paint(10), 4, 2, key=1)
    pixels.STATS['sixel'] += 600                           # what the repaint sent
    pixels._meter(600)
    now[0] += 0.1
    assert pixels.surface('a', paint(20), 4, 2, key=2) is a             # 600 + 600 would pass the budget
    now[0] += 1.0
    assert pixels.surface('a', paint(30), 4, 2, key=3) is not a


def test_budget_spacing_never_starves_a_surface(monkeypatch):
    """An oversize repaint is only spaced (it draws again once the last second is quiet), a fresh surface always draws,
    and a small surface is not charged another's big repaint as its own size."""
    now = [10.0]
    monkeypatch.setattr(pixels, 'clock', lambda: now[0])
    pixels.use('sixel', budget=1000)
    paint = lambda colour: paint_box((colour, 0, 0))
    a = pixels.surface('a', paint(10), 4, 2, key=1)
    pixels.STATS['sixel'] += 900                           # 90% of the budget in one repaint
    pixels._meter(900)
    now[0] += 0.1
    assert pixels.surface('a', paint(20), 4, 2, key=2) is a             # spaced
    assert pixels.surface('fresh', paint(10), 4, 2, key=1) != [[pixels.BLANK] * 4] * 2   # a new tile draws at once
    now[0] += 1.0
    assert pixels.surface('a', paint(30), 4, 2, key=3) is not a         # the second is quiet: repaints
    pixels.STATS['sixel'] += 100                           # b's own small repaint
    b = pixels.surface('b', paint(10), 4, 2, key=1)
    pixels.STATS['sixel'] += 50
    pixels._meter(50)
    now[0] += 0.3
    pixels.surface('a', paint(40), 4, 2, key=4)
    assert pixels._sizes.get('b', 0) <= 150                # not a's bytes


def test_budget_spacing_lets_a_big_surface_through_beside_a_busy_small_one(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(pixels, 'clock', lambda: now[0])
    pixels.use('sixel', budget=1000)
    paint = lambda colour: paint_box((colour, 0, 0))
    big = pixels.surface('big', paint(1), 4, 2, key=1)
    pixels.STATS['sixel'] += 850
    pixels._meter(850)
    repainted = None
    for i in range(1, 60):                                 # a small surface keeps the last second from going quiet
        now[0] += 0.1
        if i % 5 == 0:
            pixels.surface('small', paint(i), 4, 2, key=i)
            pixels.STATS['sixel'] += 20
            pixels._meter(20)
        if pixels.surface('big', paint(100 + i), 4, 2, key=100 + i) is not big:
            repainted = now[0] - 10.0
            break
    assert repainted is not None and repainted <= pixels.PAINT_MAX_WAIT_S + 0.2

"""Cell resolution of the map canvas: braille OR, priority, glyphs, chips and gap marks."""
from mhseals_testing.ui.minimap import canvas


def test_dots_in_one_cell_or_into_one_braille_character():
    c = canvas.Canvas(2, 1)
    c.dot(0, 0, 'trail')
    c.dot(1, 3, 'trail')
    assert c.text() == [chr(0x2800 + 0x01 + 0x80) + canvas.BLANK]


def test_cell_colour_is_the_highest_priority_dot_and_off_canvas_dots_are_dropped():
    c = canvas.Canvas(1, 1)
    c.dot(0, 0, 'plan')
    c.dot(1, 0, 'boat')
    c.dot(0, 1, 'trail')
    c.dot(5, 5, 'boat')
    assert c.cells()[0][0].fg == 'boat'


def test_a_glyph_replaces_the_cell_and_the_higher_priority_glyph_wins():
    c = canvas.Canvas(1, 1)
    c.dot(0, 0, 'trail')
    c.glyph(0, 0, 'o', 'object')
    c.glyph(0, 0, 'x', 'contact', chip=True)
    c.glyph(0, 0, 'y', 'object')
    cell = c.cells()[0][0]
    assert (cell.text, cell.fg, cell.chip) == ('x', 'contact', True)


def test_a_chip_drops_the_costmap_tint_but_a_plain_glyph_keeps_it():
    c = canvas.Canvas(2, 1)
    c.tint(0, 0, 'lethal')
    c.tint(1, 0, 'lethal')
    c.glyph(0, 0, 'x', 'contact', chip=True)
    c.glyph(1, 0, 'o', 'object')
    cells = c.cells()[0]
    assert cells[0].bg is None and cells[1].bg == 'lethal'


def test_gap_mark_yields_to_glyphs_and_to_dots_above_trail_priority():
    c = canvas.Canvas(4, 1)
    for col in range(4):
        c.gap(col, 0)
    c.glyph(0, 0, 'o', 'object')
    c.dot(2, 0, 'boat')   # col 1
    c.dot(4, 0, 'plan')   # col 2, below trail
    assert c.text() == ['o' + chr(0x2801) + canvas.GAP + canvas.GAP]
    assert c.cells()[0][1].fg == 'boat' and c.cells()[0][2].fg == 'trail'


def test_a_layer_colour_suffix_does_not_change_its_rank():
    assert canvas.rank('object:obj.red') == canvas.rank('object')
    assert canvas.rank('contact:x') < canvas.rank('object:obj.red')

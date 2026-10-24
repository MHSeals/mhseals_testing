"""Objects and truth on the map canvas: contact chips, confidence, labels, rings."""
from mhseals_testing.analysis.classes import load_classes
from mhseals_testing.ui.minimap import canvas
from mhseals_testing.ui.minimap.marks import Obj, Projection, draw_objects, draw_truth

TABLE = load_classes()


def make(half=8.0, cols=20, rows=5):
    c = canvas.Canvas(cols, rows)
    return c, Projection(c, (0.0, 0.0), half)


def test_projection_puts_the_centre_mid_canvas_and_north_up():
    c, p = make()
    assert p.cell((0, 0)) == (10, 2)
    assert p.cell((0, 3))[1] < 2 and p.cell((3, 0))[0] > 10


def test_contact_is_a_chip_with_the_class_glyph_and_x_in_ascii():
    c, p = make(half=20)
    draw_objects(p, [Obj(0, 0, 'red_buoy', 0.9, True)], TABLE)
    cell = c.cells()[2][10]
    assert (cell.text, cell.fg, cell.chip) == ('▴', 'contact', True)
    c, p = make(half=20)
    draw_objects(p, [Obj(0, 0, 'red_buoy', 0.9, True)], TABLE, ascii_=True)
    assert c.cells()[2][10].text == 'X'


def test_plain_object_carries_its_colour_token_and_low_scores_are_hidden():
    c, p = make(half=20)
    draw_objects(p, [Obj(0, 0, 'red_buoy', 0.9), Obj(5, 0, 'red_buoy', 0.2)], TABLE)
    assert c.cells()[2][10].fg == 'object:obj.red' and not any('▴' in r for r in c.text()[:2] + c.text()[3:])
    assert c.text()[2].count('▴') == 1


def test_higher_score_wins_a_shared_cell():
    c, p = make(half=20)
    draw_objects(p, [Obj(0, 0, 'red_buoy', 0.5), Obj(0.1, 0, 'green_buoy', 0.8)], TABLE)
    assert c.cells()[2][10].text == '▪'


def test_label_goes_right_else_left_else_is_skipped_only_inside_ten_m():
    c, p = make(half=8)
    draw_objects(p, [Obj(0, 0, 'red_buoy', 0.89)], TABLE)
    assert c.text()[2][10:14] == '▴r89'
    c, p = make(half=8)
    draw_objects(p, [Obj(7.5, 0, 'red_buoy', 0.89)], TABLE)
    assert 'r89▴' in c.text()[2]
    c, p = make(half=8, cols=2)
    draw_objects(p, [Obj(0, 0, 'red_buoy', 0.89)], TABLE)
    assert 'r' not in ''.join(c.text())
    c, p = make(half=12)
    draw_objects(p, [Obj(0, 0, 'red_buoy', 0.89)], TABLE)
    assert 'r89' not in c.text()[2]


def test_rings_only_at_five_m_or_less():
    c, p = make(half=4, cols=40, rows=10)
    draw_objects(p, [Obj(0, 0, 'blue_buoy', 0.9)], TABLE)
    assert any(cell.fg == 'ring' for row in c.cells() for cell in row)
    c, p = make(half=8, cols=40, rows=10)
    draw_objects(p, [Obj(0, 0, 'blue_buoy', 0.9)], TABLE)
    assert not any(cell.fg == 'ring' for row in c.cells() for cell in row)


def test_truth_is_a_dot_plus_a_line_to_the_match_and_nothing_glyph_sized():
    c, p = make(half=10, cols=20, rows=1)
    draw_truth(p, [((-8, 0), (0, 0)), ((8, 0), None)])
    cells = c.cells()[0]
    assert cells[2].fg == "truth_dot" and cells[18].fg == 'truth_dot'
    assert cells[5].fg == 'truth_line' and cells[14].fg is None
    assert all(0x2800 <= ord(x.text) < 0x2900 for x in cells)


def test_confidence_fades_the_colour_token_and_the_floor_is_a_parameter():
    c, p = make(half=20)
    draw_objects(p, [Obj(0, 0, 'red_buoy', 0.9), Obj(5, 0, 'red_buoy', 0.6), Obj(-5, 0, 'red_buoy', 0.4)], TABLE)
    fgs = [x.fg for x in c.cells()[2] if x.fg]
    assert sorted(fgs) == ['object:obj.red', 'object:obj.red@50', 'object:obj.red@78']
    for floor, n in ((0, 3), (0.5, 2), (0.7, 1)):
        c, p = make(half=20)
        draw_objects(p, [Obj(0, 0, 'red_buoy', 0.9), Obj(5, 0, 'red_buoy', 0.6), Obj(-5, 0, 'red_buoy', 0.4)],
                     TABLE, floor=floor)
        assert sum(x.fg is not None for x in c.cells()[2]) == n


def test_non_finite_positions_are_skipped_not_raised():
    c, p = make()
    bad = [Obj(float('nan'), 1, 'red_buoy', .9), Obj(1, float('inf'), 'red_buoy', .9)]
    draw_objects(p, bad + [Obj(0, 0, 'red_buoy', .9)], TABLE)
    assert c.cells()[2][10].fg == 'object:obj.red'


def test_object_marks_drops_non_finite_positions():
    from types import SimpleNamespace as N
    from mhseals_testing.ros.minimap_feeds import object_marks

    def det(x, y):
        return N(results=[N(hypothesis=N(class_id='red_buoy', score=.9))],
                 bbox=N(center=N(position=N(x=x, y=y))))
    out = object_marks(N(detections=[det(float('nan'), 1), det(2, float('inf')), det(1, 2)]))
    assert [(o.x, o.y) for o in out] == [(1, 2)]

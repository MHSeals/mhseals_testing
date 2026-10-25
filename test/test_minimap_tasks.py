"""The map's task layer (ui/minimap/tasks.py): hypotheses parsed from the contract and drawn by state."""
import json
from pathlib import Path

from mhseals_testing.ui.minimap import canvas as canvas_module, tasks
from mhseals_testing.ui.minimap.canvas import Canvas
from mhseals_testing.ui.minimap.marks import Projection

HYPOTHESES = json.loads((Path(__file__).parent / 'fixtures' / 'task_hypotheses.json').read_text())


def by_id():
    return {t.id: t for t in tasks.parse(HYPOTHESES)}


def painted(found, goal=None, ascii_=False, labels=True):
    canvas = Canvas(60, 30)
    tasks.draw(Projection(canvas, (0.0, 0.0), 20.0), found, goal, ascii_, labels)
    return canvas


def dots_of(canvas, token):
    return sum(bin(bit).count('1') for bits in canvas._dots.values() for bit, layer in bits.items()
               if token in layer)


def test_parse_reads_the_contract_and_drops_what_it_cannot_place():
    found = by_id()
    assert found['t3'] == tasks.Task('t3', 'speed_gate', 'hypothesis', 0.45, (12.0, -6.0, 0.0),
                                     ((10.0, -8.0), (14.0, -8.0), (14.0, -4.0), (10.0, -4.0), (10.0, -8.0)),
                                     ((12, 'red_buoy', 12.0, -8.0), (13, 'green_buoy', 12.0, -4.0)))
    odd = dict(HYPOTHESES, hypotheses=[{'id': 'a', 'pose': ['x', 0]}, {'id': 'b', 'pose': [1, 2], 'state': 'weird'}])
    assert [(t.id, t.state) for t in tasks.parse(odd)] == [('b', 'hypothesis')]
    assert tasks.parse(dict(HYPOTHESES, schema='other/1')) == [] and tasks.parse(dict(HYPOTHESES, frame='odom')) == []


def test_confirmed_is_solid_a_hypothesis_dashed_and_faded_by_confidence():
    found = by_id()
    square = found['t4']._replace(extent=found['t3'].extent, pose=found['t3'].pose)
    solid = dots_of(painted([square]), 'task.confirmed')
    dashed = dots_of(painted([found['t3']]), 'task.hypothesis@50')
    assert solid > 2 * dashed > 0


def test_labels_carry_the_state_without_colour_and_stay_ascii():
    found = list(by_id().values())
    text = '\n'.join(painted(found).text())
    assert 'channel ✓' in text and 'dock ✗' in text and 'speed ?45' in text and 'path' in text
    text = '\n'.join(painted(found, ascii_=True).text())
    assert 'channel +' in text and 'dock x' in text
    assert 'speed' not in '\n'.join(painted(found, labels=False).text())


def test_the_goal_and_its_why_draw_and_the_layer_ranks_between_object_and_wake():
    canvas = painted([], goal=(0.0, 0.0, 0.0, 'waypoint between the gate buoys'))
    text = '\n'.join(canvas.text())
    assert '◎' in text and 'waypoint between the gate b…' in text
    assert '@' in '\n'.join(painted([], goal=(0.0, 0.0, 0.0, ''), ascii_=True).text())
    rank = canvas_module.rank
    assert rank('object') < rank('task:task.done@50') < rank('wake')


def test_a_far_off_extent_is_not_walked():
    far = tasks.Task('f', 'dock', 'confirmed', 1.0, (5e4, 0.0, 0.0), ((-5e4, 0.0), (5e4, 0.0)), ())
    assert dots_of(painted([far], labels=False), 'task') == 0

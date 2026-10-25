"""The map's task layer (ui/minimap/tasks.py): hypotheses parsed from the contract and drawn by state."""
import json
from pathlib import Path

from mhseals_testing.ui.minimap import canvas as canvas_module, tasks
from mhseals_testing.ui.minimap.canvas import Canvas
from mhseals_testing.ui.minimap.marks import Projection

HYPOTHESES = json.loads((Path(__file__).parent / 'fixtures' / 'task_hypotheses.json').read_text())


def by_id():
    return {t.id: t for t in tasks.parse(HYPOTHESES)}


def painted(found, goal=None, ascii_=False, labels=True, half=20.0):
    canvas = Canvas(60, 30)
    tasks.draw(Projection(canvas, (0.0, 0.0), half), found, goal, ascii_, labels)
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


def test_confirmed_is_solid_a_hypothesis_dashed_and_close_in_every_task_is_its_corners():
    found = by_id()
    square = found['t4']._replace(extent=found['t3'].extent, pose=found['t3'].pose)
    solid = dots_of(painted([square]), 'task.confirmed')
    dashed = dots_of(painted([found['t3']]), 'task:task.hypothesis')
    assert solid > 2 * dashed > 0 and dots_of(painted([found['t3']]), '@') == 0   # no confidence fade
    box = ((-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0), (-2.0, -2.0))
    for half in (5.0, 10.0):   # a closed 4-corner box in view: its 4 corners; a hypothesis its label alone
        for task in (square, found['t2'], found['t1']):
            assert dots_of(painted([task._replace(extent=box)], labels=False, half=half), 'task') == 4
        hypothesis = painted([found['t3']._replace(extent=box, pose=(0.0, 0.0, 0.0))], half=half)
        assert dots_of(hypothesis, 'task') == 0 and 'speed ?45' in '\n'.join(hypothesis.text())


def test_parse_survives_fuzzed_hypotheses():
    good = HYPOTHESES['hypotheses'][2]
    bad = [5, 'x', None, [], {'pose': None}, {'pose': [1]}, {'pose': {'x': 1}}, dict(good, confidence=float('nan')),
           dict(good, confidence=float('inf')), dict(good, confidence=1.5), dict(good, confidence='high'),
           dict(good, pose=[1e308, -1e308]), dict(good, pose=[float('nan'), 0]), dict(good, state=['x'])]
    for message in ({'hypotheses': 5}, {'hypotheses': {'a': 1}}, {'hypotheses': 'abc'}, [], 7,
                    dict(HYPOTHESES, hypotheses=bad + [dict(good, extent=[[1e308, 0], [0, -1e308], [1], 3, [2, 2]])])):
        found = tasks.parse(message)
        assert all(abs(c) <= tasks.REACH for t in found for p in (t.pose[:2], *t.extent) for c in p)
        painted(found)
    found = tasks.parse(dict(HYPOTHESES, hypotheses=bad + [dict(good, extent=[[1e308, 0], [2, 2]])]))
    assert [t.state for t in found] == ['hypothesis', 'hypothesis'] and found[-1].extent == ((2.0, 2.0),)


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
    assert rank('object') < rank('task:task.done') < rank('wake')


def test_a_far_off_extent_is_not_walked():
    far = tasks.Task('f', 'dock', 'confirmed', 1.0, (5e4, 0.0, 0.0), ((-5e4, 0.0), (5e4, 0.0)), ())
    assert dots_of(painted([far], labels=False), 'task') == 0
